
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
#include <math.h>
//#include <Vector.h>
#include <units.h>
#include <mpu9250.h>
#include <eigen.h>
#include <Wire.h>


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

// --- IMU MPU9250 ---
bfs::Mpu9250 mpu(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
bool mpuReady = false;

// Estado angular
float roll = 0, pitch = 0, yaw = 0;
static float fusedYaw = 0;
unsigned long lastIMUUpdate = 0;
const float ALPHA_YAW = 0.02f;  // fusión yaw lenta

// Filtros EMA para acelerómetro y giroscopio
const float ALPHA_ACC  = 0.2f;
const float ALPHA_GYRO = 0.1f;
float ax_f = 0, ay_f = 0, az_f = 0;
float gx_f = 0, gy_f = 0, gz_f = 0;
float yaw_f = 0;
const float ALPHA_FILT = 0.15f;

// ============================================================
// ESTRUCTURAS BÁSICAS
// ============================================================
struct Coordinate {
  double lat;
  double lon;
};

struct Mission {
  bool loaded = false;
  std::vector<Coordinate> polygon;
  Coordinate home;
  double altitude = 0;
  double spacing  = 0;
  String event_action;
} mission;

struct {
  float accelBias[3] = {0,0,0};
  float gyroBias[3]  = {0,0,0};
  float magBias[3]   = {0,0,0};  // hard-iron
  float magScale[3]  = {1,1,1};  // soft-iron
} imuCal;

struct {
  bool active = false;
  float minv[3] = {  1e9,  1e9,  1e9};
  float maxv[3] = { -1e9, -1e9, -1e9};
  unsigned long startMs = 0;
  unsigned long minDurationMs = 10000;
  int samples = 0;
} magCal;

String magQuality = "GOOD";

// ============================================================
// 🧭 Máquina de estados del dron
// ============================================================

enum DroneState {
  IDLE,            // Esperando misión
  TAKEOFF,         // Ascendiendo hasta altitud deseada
  NAVIGATE,        // Avanzando a waypoint
  STABILIZE,       // Estabilizando cámaras y enviando STABLE
  WAIT_ANALYSIS,   // Esperando respuesta del RPi
  RETURN_HOME,     // Retorno al punto HOME
  LAND,            // Descendiendo
  COMPLETE         // Misión finalizada
};

DroneState state = IDLE;
unsigned long stateEntryTime = 0;

// Resultado de análisis de la RPi
enum AnalysisResult { NONE, GO, FIRE, PERSON };
AnalysisResult analysisResult = NONE;

// Misión y recorrido
int currentWaypoint = 0;
std::vector<Coordinate> pathPoints;

// Timeout de análisis (90 s)
const unsigned long ANALYSIS_TIMEOUT = 90000;
unsigned long analysisStartTime = 0;

// Interrupciones por LoRa
bool loraReturnCommand = false;
bool loraDisarmCommand = false;

// Para telemetría / misión
unsigned long missionStartTime = 0;
// --- SIMULACIÓN ---
bool simulationMode = false;

// ============================================================================
// ⚙️ CONFIGURACIÓN DE PREFS
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

void saveIMUCalibration() {
  prefs.begin("mpu9250", false);
  prefs.putBytes("accelBias", imuCal.accelBias, sizeof(imuCal.accelBias));
  prefs.putBytes("gyroBias",  imuCal.gyroBias,  sizeof(imuCal.gyroBias));
  prefs.putBytes("magBias",   imuCal.magBias,   sizeof(imuCal.magBias));
  prefs.putBytes("magScale",  imuCal.magScale,  sizeof(imuCal.magScale));
  prefs.end();
  Serial.println("💾 Calibraciones IMU guardadas");
}

void loadIMUCalibration() {
  prefs.begin("mpu9250", true);
  prefs.getBytes("accelBias", imuCal.accelBias, sizeof(imuCal.accelBias));
  prefs.getBytes("gyroBias",  imuCal.gyroBias,  sizeof(imuCal.gyroBias));
  prefs.getBytes("magBias",   imuCal.magBias,   sizeof(imuCal.magBias));
  prefs.getBytes("magScale",  imuCal.magScale,  sizeof(imuCal.magScale));
  prefs.end();
  Serial.println("📥 Calibraciones IMU cargadas");
}

// ============================================================================
// 🔹 FUNCIONES DE UTILIDAD y geodesicas
// ============================================================================
unsigned long toUnixTime(int y,int m,int d,int h,int min,int s){
  if(m<=2){y-=1; m+=12;}
  long a=y/100, b=2-a+a/4;
  long days=(long)(365.25*(y+4716))+(long)(30.6001*(m+1))+d+b-1524.5;
  unsigned long ts=(days-2440588)*86400UL + h*3600UL + min*60UL + s;
  return ts;
}

double deg2rad(double deg) { return deg * M_PI / 180.0; }
double rad2deg(double rad) { return rad * 180.0 / M_PI; }

// Distancia en metros entre dos coordenadas GPS
double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
  double R = 6371000.0; // radio de la Tierra [m]
  double dLat = deg2rad(lat2 - lat1);
  double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat/2)*sin(dLat/2) +
             cos(deg2rad(lat1))*cos(deg2rad(lat2))*sin(dLon/2)*sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

// Rumbo desde A hacia B en grados (0=Norte)
double computeBearing(double lat1, double lon1, double lat2, double lon2) {
  double y = sin(deg2rad(lon2 - lon1)) * cos(deg2rad(lat2));
  double x = cos(deg2rad(lat1))*sin(deg2rad(lat2)) -
             sin(deg2rad(lat1))*cos(deg2rad(lat2))*cos(deg2rad(lon2 - lon1));
  double brng = atan2(y, x);
  return fmod((rad2deg(brng) + 360.0), 360.0);
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

void sendJsonNoAckToGS(const JsonDocument& doc) {
  String payload; serializeJson(doc, payload);
  String frame = String("UAV#") + payload + "#END";
  LoRa.beginPacket(); LoRa.print(frame); LoRa.endPacket();
  Serial.println("📤 [UAV→GS] " + frame);
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
  StaticJsonDocument<1024> doc;
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

    else if (strcmp(type, "GET_TRIMS") == 0) {
      sendTrims();
      sendAckToGS(msgId);
      return;
    }

    else if (strcmp(type, "TRIM") == 0) {
      JsonObject d = doc["d"];
      if (!d.isNull()) {
        trims.accel   = d["accelerator"] | trims.accel;  // <- si tu GS usa "accelerator"
        trims.roll_lr = d["roll_lr"]     | trims.roll_lr;
        trims.roll_fb = d["roll_fb"]     | trims.roll_fb;
        trims.rudder  = d["rudder"]      | trims.rudder;
        trims.sw      = d["switch"]      | trims.sw;     // ← clave correcta: "switch"
        saveTrims();
        sendAckToGS(msgId);
      }
      return;
    }

    else if (strcmp(type, "GRIPPER") == 0) {
      Serial.println("🦾 GRIPPER recibido (placeholder)");
      sendAckToGS(msgId);
      return;
    }

    else if (strcmp(type, "ARM") == 0) {
      Serial.println("🚁 ARM recibido");
      if (bmp_ok) { calibrateAltZero(60, 20); }
      zeroGyroOnARM();
      sendAckToGS(msgId);
      return;
    }

    else if (strcmp(type, "DISARM") == 0) {
      Serial.println("🛰️ [LoRa] DISARM recibido → bandera activada");
      loraDisarmCommand = true;
      sendAckToGS(msgId);
      return;
    }
    else if (strcmp(type, "RETURN") == 0) {
      Serial.println("🛰️ [LoRa] RETURN recibido → bandera activada");
      loraReturnCommand = true;
      sendAckToGS(msgId);
      return;
    }

    else if (strcmp(type, "SIM_ON") == 0) {
      simulationMode = true;
      Serial.println("🧪 Modo SIMULACIÓN ACTIVADO");
      sendAckToGS(msgId);
      return;
    }

    else if (strcmp(type, "SIM_OFF") == 0) {
      simulationMode = false;
      Serial.println("🛑 Modo SIMULACIÓN DESACTIVADO");
      sendAckToGS(msgId);
      return;
    }

    else if (strcmp(type, "CALIB_MAG") == 0) {
      Serial.println("🧭 CALIB_MAG recibido desde GS");
      startMagCalibration(12000);    // 12 s
      sendAckToGS(doc["id"] | "");   // si usás ACK
      return;
    }

    else if (strcmp(type, "MISSION_COMPACT") == 0) {
      Serial.println("📦 Recibido MISSION_COMPACT");

      JsonObject d = doc["d"];
      if (d.isNull()) {
        Serial.println("❌ Error: campo 'd' ausente en MISSION_COMPACT");
        return;
      }

      // Limpiar misión anterior
      mission.polygon.clear();

      // Cargar coordenadas del polígono
      JsonArray p = d["p"];
      if (!p.isNull()) {
        Serial.println("📍 Coordenadas del polígono:");
        for (JsonArray::iterator it = p.begin(); it != p.end(); ++it) {
          JsonArray coord = (*it).as<JsonArray>();
          if (coord.size() == 2) {
            Coordinate pt;
            pt.lat = coord[0];
            pt.lon = coord[1];
            mission.polygon.push_back(pt);
            Serial.printf("   Punto: %.6f, %.6f\n", pt.lat, pt.lon);
          } else {
            Serial.println("⚠ Coordenada inválida (no tiene 2 valores)");
          }
        }  // ✅ cierre del for
      }    // ✅ cierre del if (!p.isNull())

      // Cargar HOME
      JsonArray h = d["h"];
      if (!h.isNull() && h.size() == 2) {
        mission.home.lat = h[0];
        mission.home.lon = h[1];
      }

      // Cargar otros parámetros
      mission.altitude = d["a"] | 20.0;
      mission.spacing  = d["s"] | 10.0;

      if (d.containsKey("event_action"))
        mission.event_action = (const char*)d["event_action"];
      else
        mission.event_action = "NONE";

      mission.loaded = true;

      // Debug
      Serial.printf("   HOME:     %.6f, %.6f\n", mission.home.lat, mission.home.lon);
      Serial.printf("   Altitud:  %.1f m\n", mission.altitude);
      Serial.printf("   Spacing:  %.1f m\n", mission.spacing);
      Serial.printf("   Acción:   %s\n", mission.event_action.c_str());
      Serial.printf("   Waypoints: %d\n", mission.polygon.size());

      generateMissionPath(mission);     // ← genera lista de puntos interpolados (cada X metros)
      state = NAVIGATE;          // ← entra al modo de navegación automática
      currentWaypoint = 0;                  // ← reinicia el contador de puntos
      analysisResult = NONE;            // ← limpia resultados anteriores
      missionStartTime = millis();      // ← registra inicio de misión
      Serial.println("🚀 Misión cargada, FSM activada (estado: NAVIGATE)");

      sendAckToGS(doc["id"].as<String>());
      return;
    }  // ✅ cierre del bloque MISSION_COMPACT
  }    // ✅ cierre del bloque fromGS


  // ==========================================================
  // 🔹 2. Mensajes internos del dron (desde RPi / simulador)
  // ==========================================================

  // 🔸 Compatibilidad extendida: acepta "ANALYSIS_RESULT" o mensajes simples ("GO", "FIRE", "PERSON")
  
  else if (strcmp(type, "GO") == 0) {
    analysisResult = GO;
    Serial.println("📩 [RPI] Resultado recibido: GO → continuar misión");
    return;
  }
  else if (strcmp(type, "FIRE") == 0) {
    analysisResult = FIRE;
    Serial.println("🔥 [RPI] Resultado recibido: FIRE → evento detectado");
    unsigned long ts = 0;
    if (gps.date.isValid() && gps.time.isValid()) {
      ts = toUnixTime(gps.date.year(), gps.date.month(), gps.date.day(),
                      gps.time.hour(), gps.time.minute(), gps.time.second());
    }
    sendEventToGS(type, lat_f, lon_f, alt_f, ts);
    return;
  }
  else if (strcmp(type, "PERSON") == 0) {
    analysisResult = PERSON;
    Serial.println("🧍 [RPI] Resultado recibido: PERSON → evento detectado");
    unsigned long ts = 0;
    if (gps.date.isValid() && gps.time.isValid()) {
      ts = toUnixTime(gps.date.year(), gps.date.month(), gps.date.day(),
                      gps.time.hour(), gps.time.minute(), gps.time.second());
    }
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

void handleTelemetry() {
  static unsigned long lastTelemetryTime = 0;
  if (millis() - lastTelemetryTime >= 1000) {

    if (!simulationMode) {
      updateAltitudeBaro();
      if (gps.location.isValid()) {
        double lat, lon, alt_dum;
        filterGPS(gps.location.lat(), gps.location.lng(),
                  gps.altitude.meters(), lat, lon, alt_dum);
        unsigned long ts = 0;
        if (gps.date.isValid() && gps.time.isValid()) {
          ts = toUnixTime(gps.date.year(), gps.date.month(), gps.date.day(),
                          gps.time.hour(), gps.time.minute(), gps.time.second());
        }
        sendTelemetry(lat, lon, alt_baro_f, gps.speed.kmph(), gps.course.deg(), ts);
      }
    } else {
      // 🔹 En modo simulación, enviar posición virtual
      unsigned long ts = millis();
      sendTelemetry(lat_f, lon_f, alt_baro_f, 5.0, 0.0, ts);
    }

    lastTelemetryTime = millis();
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

void sendStableToRPi(const Coordinate &pos) {
  StaticJsonDocument<128> doc;
  doc["t"] = "STABLE";
  doc["lat"] = pos.lat;
  doc["lon"] = pos.lon;
  doc["ts"] = millis();

  String payload;
  serializeJson(doc, payload);
  SerialRPI.println(payload);

  Serial.println("📤 Enviado a RPi: " + payload);
}


void handleLoRa(){
  int size=LoRa.parsePacket();
  if(size){while(LoRa.available())loraRxBuf+=(char)LoRa.read();}
  String js;
  while(extractNextFrame(loraRxBuf,js,GS_HDR)){processIncomingJSON(js,true);}
}

// ============================================================
// 🔧 FUNCIONES IMU MPU9250
// ============================================================
//-----Inicializacion-----
void initMPU9250() {
  Serial.println("🔧 Inicializando MPU9250 (BolderFlight 5.6.0)...");
  Wire.begin();

  if (!mpu.Begin()) {
    Serial.println("❌ Error iniciando MPU9250");
    mpuReady = false;
    return;
  }

  mpu.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_4G);
  mpu.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_500DPS);
  mpu.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_41HZ);
  mpu.ConfigSrd(19);

  loadIMUCalibration();
  mpuReady = true;
  Serial.println("✅ MPU9250 listo");
}

//----Calibración estática-------
void calibrateIMU_Static() {
  if (!mpuReady) return;
  Serial.println("🧪 Calibración estática (ACC + GYRO) — mantener el dron quieto y nivelado...");

  const int N = 500;
  double ax_sum = 0, ay_sum = 0, az_sum = 0;
  double gx_sum = 0, gy_sum = 0, gz_sum = 0;

  for (int i = 0; i < N; i++) {
    if (mpu.Read()) {
      ax_sum += mpu.accel_x_mps2();
      ay_sum += mpu.accel_y_mps2();
      az_sum += mpu.accel_z_mps2();
      gx_sum += mpu.gyro_x_radps();
      gy_sum += mpu.gyro_y_radps();
      gz_sum += mpu.gyro_z_radps();
    }
    delay(5);
  }

  // Promedio y compensación de gravedad
  imuCal.accelBias[0] = ax_sum / N;
  imuCal.accelBias[1] = ay_sum / N;
  imuCal.accelBias[2] = (az_sum / N) - 9.80665;  // quitar gravedad
  imuCal.gyroBias[0] = gx_sum / N;
  imuCal.gyroBias[1] = gy_sum / N;
  imuCal.gyroBias[2] = gz_sum / N;

  saveIMUCalibration();
  loadIMUCalibration();

  Serial.println("✅ Calibración ACC + GYRO completada y guardada");
}

//----Zero Gyro en ARM----
void zeroGyroOnARM() {
  if (!mpuReady) return;
  Serial.println("🟢 Re-zero GYRO en ARM — dron quieto");

  const int N = 200;
  double gx_sum = 0, gy_sum = 0, gz_sum = 0;

  for (int i = 0; i < N; i++) {
    if (mpu.Read()) {
      gx_sum += mpu.gyro_x_radps();
      gy_sum += mpu.gyro_y_radps();
      gz_sum += mpu.gyro_z_radps();
    }
    delay(5);
  }

  imuCal.gyroBias[0] = gx_sum / N;
  imuCal.gyroBias[1] = gy_sum / N;
  imuCal.gyroBias[2] = gz_sum / N;

  saveIMUCalibration();
  loadIMUCalibration();

  Serial.println("✅ Gyros re-centrados en ARM");
}

//----Cálculo yaw compensado por inclinación----

float computeTiltCompensatedYaw(float ax, float ay, float az,
                                float mx, float my, float mz) {
  // Normalizar acelerómetro
  float normA = sqrtf(ax*ax + ay*ay + az*az);
  if (normA == 0) return yaw_f;
  ax /= normA; ay /= normA; az /= normA;

  // Calcular inclinaciones
  float pitch = asinf(-ax);
  float roll  = atan2f(ay, az);

  float sinR = sinf(roll), cosR = cosf(roll);
  float sinP = sinf(pitch), cosP = cosf(pitch);

  // Compensar inclinación en magnetómetro
  float mx_c = mx * cosP + mz * sinP;
  float my_c = mx * sinR * sinP + my * cosR - mz * sinR * cosP;

  float heading = atan2f(-my_c, mx_c) * RAD_TO_DEG;
  if (heading < 0) heading += 360.0f;

  return heading;
}

//----Fusión complementaria del yaw----

void updateYawFusion(float gz, float dt, float yawMag) {
  static float fusedYaw = 0;
  fusedYaw += gz * RAD_TO_DEG * dt;
  if (fusedYaw < 0) fusedYaw += 360.0f;
  if (fusedYaw >= 360.0f) fusedYaw -= 360.0f;
  fusedYaw = (1 - ALPHA_YAW) * fusedYaw + ALPHA_YAW * yawMag;
  yaw_f = fusedYaw;
}

//----Actualización IMU (filtrada)----
void updateIMU() {
  if (!mpuReady) return;
  if (!mpu.Read()) return;  // Actualiza buffers internos

  static unsigned long last = millis();
  float dt = (millis() - last) / 1000.0f;
  last = millis();

  // --- Lecturas brutas --- Valor bias
float ax = mpu.accel_x_mps2() - imuCal.accelBias[0];
float ay = mpu.accel_y_mps2() - imuCal.accelBias[1];
float az = mpu.accel_z_mps2() - imuCal.accelBias[2];
float gx = mpu.gyro_x_radps() - imuCal.gyroBias[0];
float gy = mpu.gyro_y_radps() - imuCal.gyroBias[1];
float gz = mpu.gyro_z_radps() - imuCal.gyroBias[2];
  float mx = mpu.mag_x_ut();
  float my = mpu.mag_y_ut();
  float mz = mpu.mag_z_ut();

  // --- Filtros EMA individuales ---
  ax_f = (1 - ALPHA_FILT) * ax_f + ALPHA_FILT * ax;
  ay_f = (1 - ALPHA_FILT) * ay_f + ALPHA_FILT * ay;
  az_f = (1 - ALPHA_FILT) * az_f + ALPHA_FILT * az;
  gx_f = (1 - ALPHA_FILT) * gx_f + ALPHA_FILT * gx;
  gy_f = (1 - ALPHA_FILT) * gy_f + ALPHA_FILT * gy;
  gz_f = (1 - ALPHA_FILT) * gz_f + ALPHA_FILT * gz;

  // --- Cálculo geométrico de orientación ---
  roll  = atan2f(ay_f, az_f) * RAD_TO_DEG;
  pitch = atan2f(-ax_f, sqrtf(ay_f * ay_f + az_f * az_f)) * RAD_TO_DEG;

  float yawMag = computeTiltCompensatedYaw(ax_f, ay_f, az_f, mx, my, mz);
  updateYawFusion(gz_f, dt, yawMag); //usar yag_f que es la que devuelve del fusionado
}

//----Calibración magnética dinámica----

void startMagCalibration(unsigned long durationMs) {
  if (!mpuReady) return;

  magCal.active = true;
  magCal.samples = 0;
  magCal.startMs = millis();
  magCal.minDurationMs = durationMs;

  // Resetear límites iniciales
  magCal.minv[0] = magCal.minv[1] = magCal.minv[2] =  1e9;
  magCal.maxv[0] = magCal.maxv[1] = magCal.maxv[2] = -1e9;

  Serial.println("🧭 Iniciada calibración MAG — mover en forma de 8 (todas las orientaciones)");

  // Aviso a la Ground Station
  StaticJsonDocument<192> j;
  j["t"] = "MAG_CAL_PROGRESS";
  JsonObject d = j.createNestedObject("d");
  d["phase"] = "START";
  d["duration_ms"] = (int)durationMs;
  j["ts"] = (int)millis();
  sendJsonNoAckToGS(j);
}

void updateMagCalibration() {
  if (!magCal.active || !mpuReady) return;

  // Lectura BFS: cada eje del magnetómetro
  mpu.Read();
  float mx = mpu.mag_x_ut();
  float my = mpu.mag_y_ut();
  float mz = mpu.mag_z_ut();

  // Actualizar extremos
  magCal.minv[0] = fminf(magCal.minv[0], mx);
  magCal.minv[1] = fminf(magCal.minv[1], my);
  magCal.minv[2] = fminf(magCal.minv[2], mz);
  magCal.maxv[0] = fmaxf(magCal.maxv[0], mx);
  magCal.maxv[1] = fmaxf(magCal.maxv[1], my);
  magCal.maxv[2] = fmaxf(magCal.maxv[2], mz);
  magCal.samples++;

  // Reporte de progreso cada 500 ms
  static unsigned long lastProg = 0;
  if (millis() - lastProg > 500) {
    lastProg = millis();

    float rngX = magCal.maxv[0] - magCal.minv[0];
    float rngY = magCal.maxv[1] - magCal.minv[1];
    float rngZ = magCal.maxv[2] - magCal.minv[2];
    float coverage = (rngX + rngY + rngZ) / 3.0f;

    StaticJsonDocument<224> j;
    j["t"] = "MAG_CAL_PROGRESS";
    JsonObject d = j.createNestedObject("d");
    d["phase"] = "RUN";
    d["samples"] = magCal.samples;
    d["rngX"] = rngX;
    d["rngY"] = rngY;
    d["rngZ"] = rngZ;
    d["coverage"] = coverage;
    j["ts"] = (int)millis();
    sendJsonNoAckToGS(j);
  }

  // Evaluar fin de calibración
  if ((millis() - magCal.startMs) > magCal.minDurationMs) {
    float rngX = magCal.maxv[0] - magCal.minv[0];
    float rngY = magCal.maxv[1] - magCal.minv[1];
    float rngZ = magCal.maxv[2] - magCal.minv[2];

    if (rngX > 25 && rngY > 25 && rngZ > 25) {
      // Hard-iron offset
      imuCal.magBias[0] = (magCal.maxv[0] + magCal.minv[0]) * 0.5f;
      imuCal.magBias[1] = (magCal.maxv[1] + magCal.minv[1]) * 0.5f;
      imuCal.magBias[2] = (magCal.maxv[2] + magCal.minv[2]) * 0.5f;

      // Soft-iron scaling (normalización por media)
      float scaleX = (magCal.maxv[0] - magCal.minv[0]) * 0.5f;
      float scaleY = (magCal.maxv[1] - magCal.minv[1]) * 0.5f;
      float scaleZ = (magCal.maxv[2] - magCal.minv[2]) * 0.5f;
      float avg = (scaleX + scaleY + scaleZ) / 3.0f;

      imuCal.magScale[0] = avg / scaleX;
      imuCal.magScale[1] = avg / scaleY;
      imuCal.magScale[2] = avg / scaleZ;

      // Guardar calibración
      saveIMUCalibration();
      loadIMUCalibration();

      // Métrica de simetría
      float sym = fminf(fminf(scaleX, fminf(scaleY, scaleZ)) /
                        fmaxf(scaleX, fmaxf(scaleY, scaleZ)), 1.0f);

      // Reporte de resultado a la GS
      StaticJsonDocument<320> j;
      j["t"] = "MAG_CAL_RESULT";
      JsonObject d = j.createNestedObject("d");
      JsonObject bias = d.createNestedObject("bias");
      bias["x"] = imuCal.magBias[0];
      bias["y"] = imuCal.magBias[1];
      bias["z"] = imuCal.magBias[2];
      JsonObject scale = d.createNestedObject("scale");
      scale["x"] = imuCal.magScale[0];
      scale["y"] = imuCal.magScale[1];
      scale["z"] = imuCal.magScale[2];
      d["symmetry"] = sym;
      d["samples"] = magCal.samples;
      d["result"] = (sym > 0.7f ? "OK" : "FAIR");
      j["ts"] = (int)millis();
      sendJsonNoAckToGS(j);

      Serial.println("✅ MAG calibrado (hard/soft-iron) y guardado");
    } else {
      // Cobertura insuficiente
      StaticJsonDocument<160> j;
      j["t"] = "MAG_CAL_RESULT";
      j["d"]["result"] = "INSUFFICIENT_COVERAGE";
      j["d"]["samples"] = magCal.samples;
      j["ts"] = (int)millis();
      sendJsonNoAckToGS(j);
      Serial.println("⚠️ MAG cal: cobertura insuficiente");
    }

    magCal.active = false;

    // Aviso de fin
    StaticJsonDocument<128> jf;
    jf["t"] = "MAG_CAL_PROGRESS";
    jf["d"]["phase"] = "END";
    jf["ts"] = (int)millis();
    sendJsonNoAckToGS(jf);
  }
}

//----Chequeo de calidad magnética----
void checkMagQualitySuggest() {
  static unsigned long lastChk = 0;
  static int badCount = 0;
  static unsigned long lastBadTime = 0;

  const int badThreshold = 3;
  const unsigned long checkInterval = 3000;   // cada 3 s
  const unsigned long decayInterval = 60000;  // 1 min reinicio contador
  const float MAG_FIELD_REF_uT = 50.0f;       // campo terrestre típico
  const float MAG_RECAL_THRESH = 0.35f;       // 35 % de desviación → alerta

  if (!mpuReady) return;
  if (millis() - lastChk < checkInterval) return;
  lastChk = millis();

  // 🔹 Leer magnetómetro con BolderFlight
  mpu.Read();
  float mx = mpu.mag_x_ut();
  float my = mpu.mag_y_ut();
  float mz = mpu.mag_z_ut();

  // 🔹 Magnitud del campo
  float magStrength = sqrtf(mx * mx + my * my + mz * mz);

  // 🔹 Desviación relativa respecto del campo de referencia
  float deviation = fabsf(magStrength - MAG_FIELD_REF_uT) / MAG_FIELD_REF_uT;

  // 🔹 Reinicio de contador si pasó mucho tiempo
  if (millis() - lastBadTime > decayInterval) badCount = 0;

  // 🔹 Actualizar contador de desviaciones persistentes
  if (deviation > MAG_RECAL_THRESH) {
    badCount++;
    lastBadTime = millis();
  } else {
    badCount = 0;
  }

  // 🔹 Determinar calidad magnética general
  String magQuality;
  if (deviation < 0.15f)      magQuality = "GOOD";
  else if (deviation < 0.35f) magQuality = "WARN";
  else                        magQuality = "BAD";

  // 🔹 Reporte de estado a la Ground Station
  StaticJsonDocument<192> jstatus;
  jstatus["t"] = "MAG_STATUS";
  JsonObject d = jstatus.createNestedObject("d");
  d["quality"] = magQuality;
  d["deviation"] = deviation;
  d["field"] = magStrength;
  jstatus["ts"] = (int)millis();
  sendJsonNoAckToGS(jstatus);

  // 🔹 Enviar alerta si el error es persistente
  if (badCount >= badThreshold && !magCal.active) {
    badCount = 0;

    StaticJsonDocument<192> jalert;
    jalert["t"] = "MAG_RECAL_SUGGEST";
    JsonObject da = jalert.createNestedObject("d");
    da["deviation"] = deviation;
    da["quality"] = magQuality;
    da["hint"] = "Campo magnético inestable. Ejecute calibración dinámica.";
    jalert["ts"] = (int)millis();

    sendJsonNoAckToGS(jalert);
    Serial.printf("⚠️ [MAG] Desviación persistente (%.2f) → sugerir recalibración\n", deviation);
  }
}


// ============================================================
// Funciones navegacion
// ============================================================

void generateMissionPath(Mission& m) {
  pathPoints.clear();
  if (m.polygon.size() < 1) return;

  // Por ahora: HOME -> primer vértice del polígono, con spacing en metros
  Coordinate start = m.home;
  Coordinate end   = m.polygon[0];

  double totalDist = haversineDistance(start.lat, start.lon, end.lat, end.lon);
  int steps = (m.spacing > 0.5) ? (int)(totalDist / m.spacing) : 1;
  if (steps < 1) steps = 1;

  for (int i = 0; i <= steps; i++) {
    double t = (double)i / (double)steps;
    Coordinate pt;
    pt.lat = start.lat + (end.lat - start.lat) * t;
    pt.lon = start.lon + (end.lon - start.lon) * t;
    pathPoints.push_back(pt);
  }

  Serial.printf("🧭 Generados %d puntos entre HOME y destino (spacing=%.1f m, dist=%.1f m)\n",
                (int)pathPoints.size(), m.spacing, totalDist);
}

void navigateTo(const Coordinate& target) {
  double bearing = computeBearing(lat_f, lon_f, target.lat, target.lon);
  double dist = haversineDistance(lat_f, lon_f, target.lat, target.lon);

  Serial.printf("🧭 NAV → bearing=%.1f°, dist=%.1f m (lat=%.6f, lon=%.6f → %.6f, %.6f)\n",
                bearing, dist, lat_f, lon_f, target.lat, target.lon);

  // 🧩 Simulación: mover el dron una fracción hacia el destino
  if (dist > 0.3) { // solo si queda distancia
    double stepFrac = 0.05; // 5 % de avance por ciclo (~20 iteraciones por tramo)
    lat_f += (target.lat - lat_f) * stepFrac;
    lon_f += (target.lon - lon_f) * stepFrac;
  }
}

// ============================================================
// Máquina de estados para vuelo
// ============================================================
void updateStateMachine() {
  switch (state) {

    case IDLE:
      // Espera una misión
      break;

    case TAKEOFF:
      if (alt_baro_f >= mission.altitude) {
        Serial.println("✅ Altitud alcanzada, iniciando navegación");
        state = NAVIGATE;
        stateEntryTime = millis();
      }
      break;

    case NAVIGATE:
      if (currentWaypoint >= pathPoints.size()) {
        Serial.println("🏁 Fin del recorrido → Retorno a HOME");
        state = RETURN_HOME;
        break;
      }

      {
        Coordinate target = pathPoints[currentWaypoint];
        double dist = haversineDistance(lat_f, lon_f, target.lat, target.lon);

        if (dist < 2.0) {
          Serial.printf("📍 Waypoint %d alcanzado\n", currentWaypoint);
          sendStableToRPi(target);
          state = STABILIZE;
          stateEntryTime = millis();
        } else {
          navigateTo(target); // TODO: función PWM + orientación
        }
      }
      break;

    case STABILIZE:
      if (millis() - stateEntryTime > 300) {
        Serial.println("📷 Esperando resultado de análisis...");
        analysisStartTime = millis();
        state = WAIT_ANALYSIS;
      }
      break;

    case WAIT_ANALYSIS:
      // 🔹 Interrupciones LoRa prioritarias
      if (loraReturnCommand) {
        Serial.println("⚠️ RETURN por LoRa → volviendo a HOME");
        loraReturnCommand = false;
        state = RETURN_HOME;
        break;
      }
      if (loraDisarmCommand) {
        Serial.println("🛑 DISARM por LoRa → misión abortada");
        resetMissionState();
        break;
      }

      // 🔹 Evaluar resultado del análisis
      if (analysisResult != NONE) {
        if (analysisResult == GO) {
          Serial.println("▶️ GO recibido → continuar al siguiente punto");
          currentWaypoint++;
          state = NAVIGATE;
        }
        else if (analysisResult == FIRE || analysisResult == PERSON) {
          if (mission.event_action.equalsIgnoreCase("RETURN")) {
            Serial.println("🔥 Evento detectado → misión configurada para RETURN → regresando a HOME");
            state = RETURN_HOME;
          }
          else if (mission.event_action.equalsIgnoreCase("CONTINUE")) {
            Serial.println("🔥 Evento detectado → misión configurada para CONTINUE → continuar con recorrido");
            currentWaypoint++;
            state = NAVIGATE;
          }
          else {
            Serial.println("⚠️ Evento detectado pero sin acción definida → continuar por defecto");
            currentWaypoint++;
            state = NAVIGATE;
          }
        }

        // Limpiar resultado
        analysisResult = NONE;
      }

      // 🔹 Timeout sin respuesta del RPi
      else if (millis() - analysisStartTime > ANALYSIS_TIMEOUT) {
        Serial.println("⌛ Timeout de análisis → continuar automáticamente");
        currentWaypoint++;
        analysisResult = NONE;
        state = NAVIGATE;
      }
      break;


    case RETURN_HOME:
      {
        double distHome = haversineDistance(lat_f, lon_f, mission.home.lat, mission.home.lon);
        if (distHome < 2.0) {
          Serial.println("🏠 HOME alcanzado → aterrizando");
          state = LAND;
        } else {
          navigateTo(mission.home);
        }
      }
      break;

    case LAND:
      if (alt_baro_f < 1.0) {
        Serial.println("🛬 Aterrizaje completo");
        state = COMPLETE;
      }
      break;

    case COMPLETE:
      Serial.println("✅ Misión completada");
      resetMissionState();
      break;
  }
}

void resetMissionState() {
  currentWaypoint = 0;
  pathPoints.clear();
  mission.loaded = false;
  analysisResult = NONE;
  loraReturnCommand = false;
  loraDisarmCommand = false;
  state = IDLE;
  Serial.println("🔄 Estado reiniciado (IDLE)");
}

// ============================================================================
// 🚀 SIMULADOR DE MOVIMIENTO DEL DRON (TEST FSM)
// ============================================================================

// Simula el avance entre waypoints
void simulateDroneMotion() {
  if (!simulationMode || !mission.loaded) return;

  static unsigned long lastStepTime = 0;
  if (millis() - lastStepTime < 600) return; // velocidad de simulación (~1.5Hz)
  lastStepTime = millis();

  switch (state) {

    case IDLE:
      // En simulación, no hace nada hasta recibir misión
      break;

    case TAKEOFF:
      alt_baro_f += 0.4;
      if (alt_baro_f >= mission.altitude) {
        alt_baro_f = mission.altitude;
        Serial.println("🛫 [SIM] Altitud alcanzada → iniciando navegación");
        state = NAVIGATE;
        stateEntryTime = millis();
      } else {
        Serial.printf("⬆️ [SIM] Ascendiendo... alt=%.2f\n", alt_baro_f);
      }
      break;

    case NAVIGATE:
      if (currentWaypoint >= (int)pathPoints.size()) {
        Serial.println("🏁 [SIM] Fin de ruta → RETURN_HOME");
        state = RETURN_HOME;
        break;
      }

      {
        Coordinate target = pathPoints[currentWaypoint];
        double dist = haversineDistance(lat_f, lon_f, target.lat, target.lon);

        // Simular movimiento suave hacia el waypoint
        lat_f += (target.lat - lat_f) * 0.25;
        lon_f += (target.lon - lon_f) * 0.25;
        alt_baro_f = mission.altitude;

        Serial.printf("🧭 [SIM] NAV → WP%d dist=%.2fm\n", currentWaypoint, dist);

        // Si llega, se detiene para análisis
        if (dist < 1.5) {
          Serial.printf("✅ [SIM] Waypoint %d alcanzado → STABILIZE\n", currentWaypoint);
          sendStableToRPi(target);
          state = STABILIZE;
          stateEntryTime = millis();
        }
      }
      break;

    case STABILIZE:
      // En simulación no se mueve; espera resultado de análisis
      if (millis() - stateEntryTime > 300) {
        Serial.println("📷 [SIM] Esperando resultado de análisis...");
        analysisStartTime = millis();
        state = WAIT_ANALYSIS;
      }
      break;

    case WAIT_ANALYSIS:
      // 🔹 Reacción a comandos de LoRa (interrupciones)
      if (loraReturnCommand) {
        Serial.println("⚠️ [SIM] RETURN recibido → volviendo a HOME");
        loraReturnCommand = false;
        state = RETURN_HOME;
        break;
      }
      if (loraDisarmCommand) {
        Serial.println("🛑 [SIM] DISARM recibido → abortando misión");
        resetMissionState();
        break;
      }

      // 🔹 Resultado del análisis
      if (analysisResult == GO) {
        Serial.println("➡️ [SIM] Resultado: GO → siguiente WP");
        currentWaypoint++;
        analysisResult = NONE;
        state = NAVIGATE;
      }
      else if ((analysisResult == FIRE || analysisResult == PERSON)) {
        Serial.printf("🔥 [SIM] Resultado: %s detectado\n",
                      (analysisResult == FIRE) ? "FIRE" : "PERSON");

        if (mission.event_action == "RETURN") {
          Serial.println("↩️ [SIM] event_action=RETURN → RETURN_HOME");
          state = RETURN_HOME;
        } else {
          Serial.println("➡️ [SIM] event_action=CONTINUE → continuar misión");
          currentWaypoint++;
          state = NAVIGATE;
        }

        analysisResult = NONE;
      }

      // 🔹 Timeout
      else if (millis() - analysisStartTime > ANALYSIS_TIMEOUT) {
        Serial.println("⌛ [SIM] Timeout de análisis → CONTINUE");
        currentWaypoint++;
        analysisResult = NONE;
        state = NAVIGATE;
      }
      break;

    case RETURN_HOME:
      lat_f += (mission.home.lat - lat_f) * 0.25;
      lon_f += (mission.home.lon - lon_f) * 0.25;
      alt_baro_f = max(alt_baro_f - 0.05, 0.0);

      {
        double distHome = haversineDistance(lat_f, lon_f, mission.home.lat, mission.home.lon);
        Serial.printf("🏠 [SIM] RETURN_HOME dist=%.2fm\n", distHome);
        if (distHome < 1.5) {
          Serial.println("🛬 [SIM] HOME alcanzado → LAND");
          state = LAND;
        }
      }
      break;

    case LAND:
      alt_baro_f = max(alt_baro_f - 0.15, 0.0);
      Serial.printf("⬇️ [SIM] Descendiendo... alt=%.2f\n", alt_baro_f);
      if (alt_baro_f <= 0.5) {
        Serial.println("🟢 [SIM] Aterrizaje completado");
        state = COMPLETE;
      }
      break;

    case COMPLETE:
      Serial.println("✅ [SIM] Misión finalizada");
      resetMissionState();
      break;
  }
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

  initMPU9250();
  calibrateIMU_Static();
  loadTrims();
  delay(1500);
}

void loop(){
  updateGPS();
  handleTelemetry();
  handleSerialRPI();
  handleLoRa();
  checkPendingAcks();
  updateStateMachine();
  simulateDroneMotion();
  updateIMU();
  updateMagCalibration();
  checkMagQualitySuggest();

}
