// ============================================================================
// 📡 DRON BUSCADOR TÉRMICO – TELEMETRÍA LoRa + GPS + BMP280
// Organización modular y comentarios por bloques funcionales
// ============================================================================

#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>
#include <Adafruit_BMP280.h>
#include <Preferences.h>

// ============================================================================
// 🛰️ CONFIGURACIÓN DE HARDWARE
// ============================================================================

// --- GPS ---
#define GPS_RX 16
#define GPS_TX 17
HardwareSerial SerialGPS(2);
TinyGPSPlus gps;
Preferences gpsPrefs;

// --- BMP280 ---
Adafruit_BMP280 bmp;
bool bmp_ok = false;

// --- LoRa ---
#define LORA_CS 5
#define LORA_RST 14
#define LORA_IRQ 27
#define LORA_BAND 433E6

static const char* GS_HDR  = "GS#";
static const char* UAV_HDR = "UAV#";
static const char* SFX     = "#END";

String loraRxBuf;

// --- UART Raspberry ---
#define RPI_RX  32
#define RPI_TX  33
HardwareSerial SerialRPI(1);

// ============================================================================
// ⚙️ CONFIGURACIÓN DE TRIMS (guardado en flash)
// ============================================================================
Preferences prefs;
struct TrimValues {
  float accel   = 128.0;
  float roll_lr = 128.0;
  float roll_fb = 128.0;
  float rudder  = 128.0;
  float sw      = 128.0;
} trims;

void loadTrims() {
  prefs.begin("drone", true);
  trims.accel   = prefs.getFloat("accel", 128.0);
  trims.roll_lr = prefs.getFloat("roll_lr", 128.0);
  trims.roll_fb = prefs.getFloat("roll_fb", 128.0);
  trims.rudder  = prefs.getFloat("rudder", 128.0);
  trims.sw      = prefs.getFloat("switch", 128.0);
  prefs.end();
}

void saveTrims() {
  prefs.begin("drone", false);
  prefs.putFloat("accel", trims.accel);
  prefs.putFloat("roll_lr", trims.roll_lr);
  prefs.putFloat("roll_fb", trims.roll_fb);
  prefs.putFloat("rudder", trims.rudder);
  prefs.putFloat("switch", trims.sw);
  prefs.end();
}

// ============================================================================
// 🔹 FUNCIONES DE UTILIDAD
// ============================================================================
unsigned long toUnixTime(int y,int m,int d,int h,int min,int s){
  if(m<=2){y-=1; m+=12;}
  long a=y/100, b=2-a+a/4;
  long days=(long)(365.25*(y+4716))+(long)(30.6001*(m+1))+d+b-1524.5;
  unsigned long ts=(days-2440588)*86400UL + h*3600UL + min*60UL + s;
  return ts;
}

// ============================================================================
// 🛰️ FILTROS GPS con fase de estabilización inicial
// ============================================================================
double lat_window[4] = {0}, lon_window[4] = {0}, alt_window[4] = {0};
int win_idx = 0;

double lat_f = 0.0, lon_f = 0.0, alt_f = 0.0;
bool have_filter = false;
int gpsWarmup = 0;                 // contador de estabilización
const int GPS_WARMUP_COUNT = 10;   // lecturas antes de activar rechazo de saltos
const double MAX_JUMP_DEG = 0.00030; // ≈33 m

// --- Mediana de 4 valores ---
double median4(double a, double b, double c, double d) {
  double arr[4] = {a,b,c,d};
  for (int i = 0; i < 3; i++)
    for (int j = i + 1; j < 4; j++)
      if (arr[j] < arr[i]) {
        double t = arr[i]; arr[i] = arr[j]; arr[j] = t;
      }
  return (arr[1] + arr[2]) / 2.0;
}

// --- Filtro GPS con warm-up adaptativo ---
void filterGPS(double lat, double lon, double alt,
               double &lat_out, double &lon_out, double &alt_out) {

  if (fabs(lat) < 0.0001 && fabs(lon) < 0.0001)
    return;  // coordenadas nulas, sin fix válido

  // Ventana circular
  lat_window[win_idx] = lat;
  lon_window[win_idx] = lon;
  alt_window[win_idx] = alt;
  win_idx = (win_idx + 1) % 4;

  double lat_m = median4(lat_window[0], lat_window[1], lat_window[2], lat_window[3]);
  double lon_m = median4(lon_window[0], lon_window[1], lon_window[2], lon_window[3]);

  if (!have_filter) {
    lat_f = lat_m; lon_f = lon_m; alt_f = alt;
    have_filter = true;
    gpsWarmup = 0;
    Serial.println("[GPS] 🆗 Filtro inicializado (modo warm-up)");
  }
  else {
    gpsWarmup++;

    // ► Warm-up adaptativo
    if (gpsWarmup < 200) {  // máximo 200 lecturas (~200 s)
      double dLat = fabs(lat_m - lat_f);
      double dLon = fabs(lon_m - lon_f);

      lat_f = 0.7 * lat_m + 0.3 * lat_f;
      lon_f = 0.7 * lon_m + 0.3 * lon_f;
      alt_f = 0.5 * alt + 0.5 * alt_f;

      // cuando los saltos son pequeños por varias lecturas, termina warm-up
      if (dLat < 0.00002 && dLon < 0.00002) {
        static int stableCount = 0;
        stableCount++;
        if (stableCount >= 8) {  // 8 lecturas estables consecutivas
          gpsWarmup = 9999;  // marca como completado
          Serial.println("[GPS] ✅ Estabilización completada (automática)");
        }
      }
    } else {
      // ► Filtro normal con rechazo de saltos
      if (fabs(lat_m - lat_f) > MAX_JUMP_DEG || fabs(lon_m - lon_f) > MAX_JUMP_DEG) {
        Serial.println("[GPS] ⚠️ Salto grande descartado");
        lat_out = lat_f; lon_out = lon_f; alt_out = alt_f;
        return;
      }

      float spd_kmh = gps.speed.kmph();
      double alpha = (spd_kmh < 5.0) ? 0.25 : 0.6;
      lat_f = alpha * lat_m + (1.0 - alpha) * lat_f;
      lon_f = alpha * lon_m + (1.0 - alpha) * lon_f;
      alt_f = alpha * alt + (1.0 - alpha) * alt_f;
    }
  }

  lat_out = lat_f;
  lon_out = lon_f;
  alt_out = alt_f;
}


// ============================================================================
// 🌡️ ALTITUD BAROMÉTRICA FILTRADA (BMP280)
// ============================================================================
static float QNH_hPa = 1013.25;
static const float ALT_EMA_ALPHA = 0.25f;
static const float ALT_MAX_STEP = 3.0f;
static bool  alt_zero_set=false;
static float alt_zero=0.0f, alt_baro_f=0.0f;
static unsigned long lastAltMs=0;
static float climb_mps=0.0f;

void calibrateAltZero(int samples=60,int delay_ms=20){
  float acc=0.0f; int ok=0;
  for(int i=0;i<samples;i++){
    if(!bmp_ok) break;
    float v=bmp.readAltitude(QNH_hPa);
    if(isfinite(v)){acc+=v;ok++;}
    delay(delay_ms);
  }
  if(ok>0){
    alt_zero=acc/ok; alt_baro_f=0.0f; alt_zero_set=true;
    Serial.printf("[BARO] 🔧 Cero calibrado: %.2f m (%d muestras)\n",alt_zero,ok);
  } else Serial.println("[BARO] ⚠️ No se pudo calibrar el cero");
}

void updateAltitudeBaro(){
  if(!bmp_ok||!alt_zero_set) return;
  float newAlt=bmp.readAltitude(QNH_hPa);
  if(!isfinite(newAlt)) return;
  float rel=newAlt-alt_zero;
  float step=fabsf(rel-alt_baro_f);
  if(step>ALT_MAX_STEP){Serial.printf("[BARO] ⚠️ Pico descartado (Δ=%.2f)\n",step);return;}
  alt_baro_f = ALT_EMA_ALPHA*rel + (1.0f-ALT_EMA_ALPHA)*alt_baro_f;
  static float prev=0.0f;
  unsigned long now=millis();
  if(lastAltMs!=0){
    float dt=(now-lastAltMs)/1000.0f;
    if(dt>0.001f) climb_mps=(alt_baro_f-prev)/dt;
  }
  prev=alt_baro_f; lastAltMs=now;
}

// ============================================================================
// 📤 COMUNICACIÓN LoRa – ACK Y REENVÍO
// ============================================================================
struct PendingMsg {
  String payload;
  unsigned long lastSend;
  int retries;
  bool waitingAck;
  String msgID;
};
#define MAX_PENDING 5
PendingMsg pendingMsgs[MAX_PENDING];
unsigned long ackTimeout=1500;
int maxRetries=4;
unsigned long nextMsgCounter=0;

String generateMsgID(){nextMsgCounter++;return String(nextMsgCounter);}

void sendWithAck(const String &jsonPayload,const String &id){
  String full=jsonPayload;
  if(!jsonPayload.startsWith("UAV#")) full="UAV#"+jsonPayload;
  if(!full.endsWith("#END")) full+="#END";
  LoRa.beginPacket();
  LoRa.print(full);
  LoRa.endPacket();
  Serial.println("📤 [sendWithAck] "+full);
  for(int i=0;i<MAX_PENDING;i++){
    if(!pendingMsgs[i].waitingAck){
      pendingMsgs[i].payload=full;pendingMsgs[i].lastSend=millis();
      pendingMsgs[i].retries=maxRetries;pendingMsgs[i].waitingAck=true;
      pendingMsgs[i].msgID=id;return;
    }
  }
  Serial.println("⚠️ Cola ACK llena");
}

// ==========================================================
// 🔹 Manejar recepción de ACK desde la Ground Station
// ==========================================================
void handleAck(const String &ackID) {
  for (int i = 0; i < MAX_PENDING; i++) {
    if (pendingMsgs[i].waitingAck && pendingMsgs[i].msgID == ackID) {
      
      // Extraer tipo del mensaje confirmado (por legibilidad)
      String tipo = "Desconocido";
      int posT = pendingMsgs[i].payload.indexOf("\"t\":\"");
      if (posT >= 0) {
        int posEnd = pendingMsgs[i].payload.indexOf("\"", posT + 5);
        if (posEnd > posT) tipo = pendingMsgs[i].payload.substring(posT + 5, posEnd);
      }

      Serial.print("✅ [ACK recibido] ID=");
      Serial.print(ackID);
      Serial.print(" (Tipo=");
      Serial.print(tipo);
      Serial.println(")");

      // Liberar el slot del mensaje
      pendingMsgs[i].waitingAck = false;
      pendingMsgs[i].payload = "";
      pendingMsgs[i].msgID = "";
      return;
    }
  }

  Serial.print("⚠️ [ACK desconocido] No se encontró ID=");
  Serial.println(ackID);
}


// ==========================================================
// 🔹 Reenvío automático de mensajes pendientes de ACK
// ==========================================================
void checkPendingAcks() {
  unsigned long now = millis();

  for (int i = 0; i < MAX_PENDING; i++) {
    if (!pendingMsgs[i].waitingAck) continue;

    // Verificar tiempo transcurrido desde último envío
    if (now - pendingMsgs[i].lastSend > ackTimeout) {

      if (pendingMsgs[i].retries > 0) {
        // Intento de reenvío
        LoRa.beginPacket();
        LoRa.print(pendingMsgs[i].payload);
        LoRa.endPacket();

        pendingMsgs[i].lastSend = now;
        pendingMsgs[i].retries--;

        // Obtener tipo de mensaje para log
        String tipo = "Desconocido";
        int posT = pendingMsgs[i].payload.indexOf("\"t\":\"");
        if (posT >= 0) {
          int posEnd = pendingMsgs[i].payload.indexOf("\"", posT + 5);
          if (posEnd > posT) tipo = pendingMsgs[i].payload.substring(posT + 5, posEnd);
        }

        Serial.print("🔁 [Reintento ACK] ID=");
        Serial.print(pendingMsgs[i].msgID);
        Serial.print(" (Tipo=");
        Serial.print(tipo);
        Serial.print(", quedan ");
        Serial.print(pendingMsgs[i].retries);
        Serial.println(" intentos)");
      }

      else {
        // Si ya agotó los intentos, se descarta
        Serial.print("❌ [ACK perdido] ID=");
        Serial.print(pendingMsgs[i].msgID);
        Serial.print(" descartado. Último payload: ");
        Serial.println(pendingMsgs[i].payload);

        pendingMsgs[i].waitingAck = false;
        pendingMsgs[i].payload = "";
        pendingMsgs[i].msgID = "";
      }
    }
  }
}


void sendAckToGS(const String &id){
  if(id==""){Serial.println("⚠️ ACK sin ID");return;}
  StaticJsonDocument<128> doc;
  doc["t"]="ACK";
  JsonObject d=doc.createNestedObject("d");d["id"]=id;
  doc["ts"]=millis();
  String p;serializeJson(doc,p);
  String msg=String(UAV_HDR)+p+String(SFX);
  LoRa.beginPacket();LoRa.print(msg);LoRa.endPacket();
  Serial.println("📤 [sendAckToGS] "+msg);
}

// ============================================================================
// 📡 TELEMETRÍA Y EVENTOS
// ============================================================================
void sendTelemetry(double lat,double lon,double alt,double speed,double heading,unsigned long ts){
  StaticJsonDocument<256> doc;
  doc["t"]="TELEMETRY";
  JsonObject d=doc.createNestedObject("d");
  d["lat"]=lat;
  d["lon"]=lon;
  d["alt"]=alt;
  d["speed"]=speed;
  d["heading"]=heading;
  d["battery"]=0;
  d["status"]=0;
  doc["ts"]=ts;
  String p;serializeJson(doc,p);
  String msg=String(UAV_HDR)+p+String(SFX);
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();
  Serial.println("📤 [sendTelemetry] "+msg);
}

void sendTrims(){
  StaticJsonDocument<256> doc;
  doc["t"]="TRIM_DATA";
  JsonObject d=doc.createNestedObject("d");
  d["accel"]=trims.accel;
  d["roll_lr"]=trims.roll_lr;
  d["roll_fb"]=trims.roll_fb;
  d["rudder"]=trims.rudder;
  d["switch"]=trims.sw;
  String id=generateMsgID();
  doc["id"]=id;
  doc["ts"]=millis();
  String p;
  serializeJson(doc,p);
  sendWithAck(p,id);
}

// ==========================================================
// 🔹 Enviar eventos desde RPi (requieren ACK confiable)
// ==========================================================
void sendEventToGS(const String &topic, double lat, double lon, double alt, unsigned long ts) {
  StaticJsonDocument<256> doc;
  doc["t"] = topic;

  // Datos de posición del dron
  JsonObject d = doc.createNestedObject("d");
  d["lat"] = lat;
  d["lon"] = lon;
  d["alt"] = alt;

  // Generar ID único
  String msgID = generateMsgID();
  doc["id"] = msgID;
  doc["ts"] = ts;

  // Serializar JSON
  String payload;
  serializeJson(doc, payload);

  // Enviar con ACK automático (agrega header/sufijo internamente)
  sendWithAck(payload, msgID);

  // Log local
  Serial.print("📤 [TX → GS] Evento ");
  Serial.print(topic);
  Serial.print(" enviado con ACK (ID=");
  Serial.print(msgID);
  Serial.println(")");
}


// ============================================================================
// 📥 PROCESAMIENTO DE MENSAJES LoRa ENTRANTES
// ============================================================================
bool extractNextFrame(String& buf,String& jsonOut,const char* wantedHdr){
  int h=buf.indexOf(wantedHdr);
  int hU=buf.indexOf(UAV_HDR);
  if(hU>=0&&(hU<h||h<0)){
    int endU=buf.indexOf(SFX,hU);
    if(endU<0)return false;
    buf.remove(0,endU+strlen(SFX));
    return false;}
  if(h<0){if(buf.length()>2048)buf.remove(0,buf.length()-256);return false;}
  int lbrace=buf.indexOf('{',h);
  if(lbrace<0)return false;
  int end=buf.indexOf(SFX,lbrace);
  if(end<0)return false;
  jsonOut=buf.substring(lbrace,end);
  buf.remove(0,end+strlen(SFX));
  jsonOut.trim();
  return true;
}

void processIncomingJSON(const String &jsonIn, bool fromGS) {
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, jsonIn)) {
    Serial.println("❌ JSON inválido");
    return;
  }

  const char* type  = doc["t"]  | "";
  const char* msgId = doc["id"] | "";

  Serial.println("📥 [processIncomingJSON] " + jsonIn);

  // ==========================================================
  // 🔹 1. Mensajes que provienen de la Ground Station (GS)
  // ==========================================================
  if (fromGS) {
    if (strcmp(type, "ACK") == 0) {
      String id = "";
      if (doc.containsKey("d")) id = doc["d"]["id"] | "";
      if (id != "") handleAck(id);
      return;
    }

    if (strcmp(type, "GET_TRIMS") == 0) {
      sendTrims();
      sendAckToGS(msgId);
      return;
    }

    if (strcmp(type, "ARM") == 0) {
      Serial.println("🚁 ARM recibido");
      if (bmp_ok) { calibrateAltZero(60, 20); }
      sendAckToGS(msgId);
      return;
    }

    if (strcmp(type, "DISARM") == 0) {
      Serial.println("🛑 DISARM recibido");
      sendAckToGS(msgId);
      return;
    }

    if (strcmp(type, "MISSION_COMPACT") == 0) {
      Serial.println("📦 Misión recibida");
      sendAckToGS(msgId);
      return;
    }

    sendAckToGS(msgId);
    return;
  }

  // ==========================================================
  // 🔹 2. Mensajes internos del dron (desde RPi / simulador)
  // ==========================================================
  if (strcmp(type, "FIRE") == 0 || strcmp(type, "PERSON") == 0) {
    Serial.printf("🔥 [RPI] Evento recibido: %s\n", type);

    unsigned long ts = 0;
    if (gps.date.isValid() && gps.time.isValid()) {
      ts = toUnixTime(gps.date.year(), gps.date.month(), gps.date.day(),
                      gps.time.hour(), gps.time.minute(), gps.time.second());
    }

    // Enviar evento completo (GPS, altitud, timestamp) a la Ground Station
    sendEventToGS(type, lat_f, lon_f, alt_f, ts);
    return;
  }

  Serial.printf("⚠️ [RPI] Evento desconocido: %s\n", type);
}

// ============================================================================
// 🔄 MANEJO DE SENSORES Y COMUNICACIONES
// ============================================================================
void updateGPS(){
  while(SerialGPS.available()){gps.encode(SerialGPS.read());}
}

void handleTelemetry(){
  static unsigned long lastTelemetryTime=0;
  if(millis()-lastTelemetryTime>=1000){
    updateAltitudeBaro();
    if(gps.location.isValid()){
      double lat,lon,alt_dum;
      filterGPS(gps.location.lat(),gps.location.lng(),gps.altitude.meters(),lat,lon,alt_dum);
      unsigned long ts=0;
      if(gps.date.isValid()&&gps.time.isValid()){
        ts=toUnixTime(gps.date.year(),gps.date.month(),gps.date.day(),gps.time.hour(),gps.time.minute(),gps.time.second());
      }
      sendTelemetry(lat,lon,alt_baro_f,gps.speed.kmph(),gps.course.deg(),ts);
      lastTelemetryTime=millis();
    }
  }
}

// ============================================================================
// 🔄 Comunicación UART1 (Raspberry simulada)
// ============================================================================
void handleSerialRPI() {
  // Verificar si hay bytes disponibles
  while (SerialRPI.available()) {
    String jsonIn = SerialRPI.readStringUntil('\n');  // lee hasta fin de JSON
    jsonIn.trim();

    // --- Depuración serial ---
    if (jsonIn.length() > 0) {
      Serial.print("[RPI] 📥 Mensaje recibido UART1 → ");
      Serial.println(jsonIn);
    }

    // Validar si tiene formato JSON
    if (jsonIn.startsWith("{") && jsonIn.endsWith("}")) {
      processIncomingJSON(jsonIn, false);
    } else {
      Serial.println("[RPI] ⚠️ Mensaje inválido o incompleto");
    }
  }
}


void handleLoRa(){
  int size=LoRa.parsePacket();
  if(size){while(LoRa.available())loraRxBuf+=(char)LoRa.read();}
  String js;
  while(extractNextFrame(loraRxBuf,js,GS_HDR)){processIncomingJSON(js,true);}
}

// ============================================================================
// 🚀 SETUP Y LOOP PRINCIPALES
// ============================================================================
void setup(){
  Serial.begin(115200);
  SerialGPS.begin(9600,SERIAL_8N1,GPS_RX,GPS_TX);
  SerialRPI.begin(9600, SERIAL_8N1, RPI_RX, RPI_TX);
  Serial.println("✅ UART0 (USB) OK");
  Serial.println("✅ UART2 (GPS) inicializado 9600 bps");
  Serial.println("✅ UART1 (RPI) inicializado 9600 bps");


  LoRa.setPins(LORA_CS,LORA_RST,LORA_IRQ);
  if(!LoRa.begin(LORA_BAND)){Serial.println("❌ LoRa fallo");while(1)delay(1000);}
  Serial.println("✅ LoRa listo");

  if(bmp.begin(0x76)){
    bmp_ok=true;
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_63);
    Serial.println("✅ BMP280 detectado");
  } else Serial.println("⚠️ BMP280 no encontrado");

  loadTrims();
  delay(1500);
}

void loop(){
  updateGPS();
  handleTelemetry();
  handleSerialRPI();
  handleLoRa();
  checkPendingAcks();
}
