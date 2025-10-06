#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>
#include <Adafruit_BMP280.h>
#include <Preferences.h>


// =============================
// Configuración GPS 
// =============================
#define GPS_RX 16     
#define GPS_TX 17     
HardwareSerial SerialGPS(2);
TinyGPSPlus gps;
Preferences gpsPrefs;

// =============================
// Configuración LoRa
// =============================
#define LORA_CS 5
#define LORA_RST 14
#define LORA_IRQ 27
#define LORA_BAND 433E6

// =============================
// Configuración Raspberry
// =============================
#define RPI_RX  4     
#define RPI_TX  2     
HardwareSerial SerialRPI(1);    // UART1 para Raspberry

// =============================
// Configuración BMP280
// =============================
Adafruit_BMP280 bmp;    
bool bmp_ok = false;    
float alt_bmp = 0.0;

// =============================
// Trims
// =============================
Preferences prefs;  

struct TrimValues {
  float accel   = 128.0;
  float roll_lr = 128.0;
  float roll_fb = 128.0;
  float rudder  = 128.0;
  float sw      = 128.0;
} trims;


void loadTrims(){
  prefs.begin("drone",true);
  trims.accel   = prefs.getFloat("accel",128.0);
  trims.roll_lr = prefs.getFloat("roll_lr",128.0);
  trims.roll_fb = prefs.getFloat("roll_fb",128.0);
  trims.rudder  = prefs.getFloat("rudder",128.0);
  trims.sw      = prefs.getFloat("switch",128.0);
  prefs.end();
}

void saveTrims(){
  prefs.begin("drone",false);
  prefs.putFloat("accel",trims.accel);
  prefs.putFloat("roll_lr",trims.roll_lr);
  prefs.putFloat("roll_fb",trims.roll_fb);
  prefs.putFloat("rudder",trims.rudder);
  prefs.putFloat("switch",trims.sw);
  prefs.end();
}

// =============================
// Variables de filtrado GPS
// =============================
double lat_window[4] = {0}, lon_window[4] = {0}, alt_window[4] = {0};
int win_idx = 0;
double lat_f = 0, lon_f = 0, alt_f = 0;
bool have_filter = false;
const double ALPHA = 0.35;

// Control de intervalo de telemetría
unsigned long lastTelemetryTime = 0;    

// =============================
// Filtrado GPS
// =============================
double median4(double a, double b, double c, double d) {
  double arr[4] = {a,b,c,d};
  for(int i=0;i<3;i++) for(int j=i+1;j<4;j++) if(arr[j]<arr[i]) { double t=arr[i]; arr[i]=arr[j]; arr[j]=t; }
  return (arr[1]+arr[2])/2.0;
}

void filterGPS(double lat, double lon, double alt, double &lat_out, double &lon_out, double &alt_out) {
  lat_window[win_idx]=lat; lon_window[win_idx]=lon; alt_window[win_idx]=alt;
  win_idx=(win_idx+1)%4;
  double lat_m = median4(lat_window[0],lat_window[1],lat_window[2],lat_window[3]);
  double lon_m = median4(lon_window[0],lon_window[1],lon_window[2],lon_window[3]);
  double alt_m = median4(alt_window[0],alt_window[1],alt_window[2],alt_window[3]);
  if(!have_filter) { lat_f=lat_m; lon_f=lon_m; alt_f=alt_m; have_filter=true; }
  else { lat_f = ALPHA*lat_m + (1.0-ALPHA)*lat_f; lon_f = ALPHA*lon_m + (1.0-ALPHA)*lon_f; alt_f = ALPHA*alt_m + (1.0-ALPHA)*alt_f; }
  lat_out=lat_f; lon_out=lon_f; alt_out=alt_f;
}

// =============================
// Hotstart GPS
// =============================
// Guardar la última posición y hora
void saveGPSBackup() {
  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
    gpsPrefs.begin("gps", false);
    gpsPrefs.putDouble("lat", gps.location.lat());
    gpsPrefs.putDouble("lon", gps.location.lng());
    gpsPrefs.putULong("date", gps.date.value());   // YYYYMMDD
    gpsPrefs.putULong("time", gps.time.value());   // HHMMSS
    gpsPrefs.end();
  }
}

// Recuperar la posición estimada
void loadGPSBackup() {
  gpsPrefs.begin("gps", true);
  double lat = gpsPrefs.getDouble("lat", 0.0);
  double lon = gpsPrefs.getDouble("lon", 0.0);
  unsigned long date = gpsPrefs.getULong("date", 0);
  unsigned long time = gpsPrefs.getULong("time", 0);
  gpsPrefs.end();

  if (lat != 0.0 && lon != 0.0) {
    Serial.printf("Restaurando posición estimada: lat %.6f, lon %.6f\n", lat, lon);
    // Aquí se podría enviar comandos UBX específicos si se quisiera AID-HOT START
  }
}

// =============================
// Timestamp UNIX
// =============================
unsigned long toUnixTime(int y,int m,int d,int h,int min,int s){
  if(m<=2){y-=1; m+=12;}
  long a=y/100, b=2-a+a/4;
  long days=(long)(365.25*(y+4716))+(long)(30.6001*(m+1))+d+b-1524.5;
  unsigned long ts=(days-2440588)*86400UL + h*3600UL + min*60UL + s;
  return ts;
}

// =============================
// ACK & Reenvío
// =============================
struct PendingMsg {
  String payload;
  unsigned long lastSend;
  int retries;
  bool waitingAck;
  String msgID;                // ✅ Guardamos el id del mensaje
};

#define MAX_PENDING 5
PendingMsg pendingMsgs[MAX_PENDING];
unsigned long ackTimeout = 1000;   // 1 s
int maxRetries = 3;
unsigned long nextMsgCounter = 0;  // ✅ Contador interno para IDs

// ✅ Generar ID como string
String generateMsgID() {
  nextMsgCounter++;
  return String(nextMsgCounter);
}

// ✅ Enviar con ACK y almacenar en la cola
void sendWithAck(const String &msg, const String &id) {
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();
  Serial.println("📤 Enviado con ACK: " + msg);

  for (int i = 0; i < MAX_PENDING; i++) {
    if (!pendingMsgs[i].waitingAck) {
      pendingMsgs[i].payload = msg;
      pendingMsgs[i].lastSend = millis();
      pendingMsgs[i].retries = maxRetries;
      pendingMsgs[i].waitingAck = true;
      pendingMsgs[i].msgID = id;
      return;
    }
  }
  Serial.println("⚠ Cola de pendientes llena");
}

// ✅ Buscar el ACK correspondiente
void handleAck(const String &ackID) {
  for (int i = 0; i < MAX_PENDING; i++) {
    if (pendingMsgs[i].waitingAck && pendingMsgs[i].msgID == ackID) {
      Serial.println("✅ ACK recibido para id: " + ackID);
      pendingMsgs[i].waitingAck = false;
      pendingMsgs[i].payload = "";
      pendingMsgs[i].msgID = "";
      break;
    }
  }
}

// ✅ Reenvío si no se recibe ACK
void checkPendingAcks() {
  unsigned long now = millis();
  for (int i = 0; i < MAX_PENDING; i++) {
    if (pendingMsgs[i].waitingAck) {
      if (now - pendingMsgs[i].lastSend > ackTimeout) {
        if (pendingMsgs[i].retries > 0) {
          LoRa.beginPacket();
          LoRa.print(pendingMsgs[i].payload);
          LoRa.endPacket();
          pendingMsgs[i].lastSend = now;
          pendingMsgs[i].retries--;
          Serial.println("🔁 Reenvío: " + pendingMsgs[i].payload);
        } else {
          Serial.println("❌ No se obtuvo ACK para: " + pendingMsgs[i].payload);
          pendingMsgs[i].waitingAck = false;
        }
      }
    }
  }
}

// ✅ Enviar ACK de respuesta a la GS
void sendAckToGS(const String &originalID) {
  if (originalID == "") {
    Serial.println("⚠ ACK sin ID recibido, no se enviará");
    return;
  }
  StaticJsonDocument<128> doc;
  doc["t"] = "ACK";
  doc["id"] = originalID;     // devolvemos el mismo ID recibido
  doc["ts"] = millis();

  String payload;
  serializeJson(doc, payload);
  String msg = "GS#" + payload + "#END";

  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();

  Serial.println("📤 Enviado ACK: " + msg);
}

// =============================
// Envíos LoRa
// =============================

// Telemetría (no requiere ACK para no saturar)
void sendTelemetry(double lat, double lon, double alt,
                   double speed, double heading, unsigned long ts) {
  StaticJsonDocument<256> doc;
  doc["t"] = "TELEMETRY";
  JsonObject d = doc.createNestedObject("d");
  d["lat"] = lat; d["lon"] = lon; d["alt"] = alt;
  d["speed"] = speed; d["heading"] = heading;
  d["battery"] = 0.0; d["status"] = 0.0;
  doc["ts"] = ts;

  String payload;
  serializeJson(doc, payload);
  String msg = "GS#" + payload + "#END";

  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();
  Serial.println("📤 " + msg);
}

// Enviar trims (requiere ACK)
void sendTrims() {
  StaticJsonDocument<256> doc;
  doc["t"] = "TRIM_DATA";
  JsonObject d = doc.createNestedObject("d");
  d["accel"] = trims.accel; d["roll_lr"] = trims.roll_lr;
  d["roll_fb"] = trims.roll_fb; d["rudder"] = trims.rudder;
  d["switch"] = trims.sw;

  String msgID = generateMsgID();       // ✅ ID único
  doc["id"] = msgID;
  doc["ts"] = millis();

  String payload;
  serializeJson(doc, payload);
  String msg = "GS#" + payload + "#END";

  sendWithAck(msg, msgID);
}

// Enviar eventos desde RPI (requieren ACK)
void sendEventToGS(const String &topic, double lat, double lon, double alt, unsigned long ts) {
  StaticJsonDocument<256> doc;
  doc["t"] = topic;
  JsonObject d = doc.createNestedObject("d");
  d["lat"] = lat; d["lon"] = lon; d["alt"] = alt;

  String msgID = generateMsgID();       // ✅ ID único
  doc["id"] = msgID;
  doc["ts"] = ts;

  String payload;
  serializeJson(doc, payload);
  String msg = "GS#" + payload + "#END";

  sendWithAck(msg, msgID);
}

// =============================
// Procesar JSON entrante
// =============================
void processIncomingJSON(const String &jsonIn, bool fromGS) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, jsonIn);
  if (err) {
    Serial.println("❌ Error JSON entrante");
    return;
  }

  const char* t = doc["t"];
  if (!t) return;

  if (fromGS) {
    // ✅ ACK recibido
    if (strcmp(t, "ACK") == 0) {
      String ackID = doc["id"] | "";
      if (ackID != "") handleAck(ackID);
      return;
    }

    // ✅ Comandos de GS que responden con ACK
    if (strcmp(t, "GET_TRIMS") == 0) {
      sendTrims();
      sendAckToGS(doc["id"].as<String>());
    }
    else if (strcmp(t, "TRIM") == 0) {
      JsonObject d = doc["d"];
      if (!d.isNull()) {
        trims.accel   = d["accel"]   | trims.accel;
        trims.roll_lr = d["roll_lr"] | trims.roll_lr;
        trims.roll_fb = d["roll_fb"] | trims.roll_fb;
        trims.rudder  = d["rudder"]  | trims.rudder;
        trims.sw      = d["switch"]  | trims.sw;
        saveTrims();
        sendAckToGS(doc["id"].as<String>());
      }
    }
    else if (strcmp(t, "ARM") == 0)            { Serial.println("CMD: ARM recibido"); sendAckToGS(doc["id"].as<String>()); }
    else if (strcmp(t, "DISARM") == 0)         { Serial.println("CMD: DISARM recibido"); sendAckToGS(doc["id"].as<String>()); }
    else if (strcmp(t, "RETURN") == 0)         { Serial.println("CMD: RETURN recibido"); sendAckToGS(doc["id"].as<String>()); }
    else if (strcmp(t, "MISSION_COMPACT") == 0){ Serial.println("CMD: MISSION_COMPACT recibido"); sendAckToGS(doc["id"].as<String>()); }
    else if (strcmp(t, "GRIPPER") == 0)        { Serial.println("CMD: GRIPPER recibido"); sendAckToGS(doc["id"].as<String>()); }

    return;
  }

  // Mensajes desde RPI (dron → GS)
  if (strcmp(t, "FIRE") == 0 || strcmp(t, "PERSON") == 0) {
    unsigned long ts = 0;
    if (gps.date.isValid() && gps.time.isValid())
      ts = toUnixTime(gps.date.year(), gps.date.month(), gps.date.day(),
                      gps.time.hour(), gps.time.minute(), gps.time.second());
    sendEventToGS(t, lat_f, lon_f, alt_f, ts);
  } else {
    Serial.println("Evento desconocido desde RPI: " + String(t));
  }
}




// =============================
// Setup
// =============================
void setup(){
  Serial.begin(115200);
  SerialGPS.begin(9600,SERIAL_8N1,GPS_RX,GPS_TX);
  SerialRPI.begin(115200,RPI_RX,RPI_TX);

  loadGPSBackup();  // Restaurar backup al encender
  
  LoRa.setPins(LORA_CS,LORA_RST,LORA_IRQ);
  if(!LoRa.begin(LORA_BAND)){
    Serial.println("{\"error\":\"Fallo inicialización LoRa\"}");
    while(true) delay(1000);
  }
  Serial.println("{\"status\":\"LoRa inicializado correctamente\"}");

  if(bmp.begin(0x76)){
    bmp_ok=true; bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,Adafruit_BMP280::SAMPLING_X2,
                                  Adafruit_BMP280::SAMPLING_X16,Adafruit_BMP280::FILTER_X16,
                                  Adafruit_BMP280::STANDBY_MS_63);
    Serial.println("BMP280 OK");
  } else Serial.println("No se detectó BMP280");

  loadTrims();
}

// =============================
// Loop
// =============================
void loop(){
  updateGPS();
  handleTelemetry(); //Envia telemetria cada 1 segundo
  handleSerialRPI(); //Manejo mensajes desde raspberry
  handleLoRa(); //Maneho mensajes desde gs por lora
  checkPendingAcks(); //check ack sin responder
 
}


// =============================
// FUNCIONES
// =============================

void updateGPS() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
    if (gps.location.isUpdated()) {
      saveGPSBackup(); // Guardar continuamente mientras hay fix
    }
  }
  
}

void handleTelemetry() {
  if (millis() - lastTelemetryTime >= 1000) {
    
    if (gps.location.isValid()) {
      double lat_raw = gps.location.lat();
      double lon_raw = gps.location.lng();
      double alt = bmp_ok ? bmp.readAltitude(1013.25) : gps.altitude.meters();

      double lat, lon, alt_filt;
      filterGPS(lat_raw, lon_raw, alt, lat, lon, alt_filt);

      unsigned long ts = 0;
      if (gps.date.isValid() && gps.time.isValid()) {
        ts = toUnixTime(gps.date.year(), gps.date.month(), gps.date.day(),
                        gps.time.hour(), gps.time.minute(), gps.time.second());
      }

      sendTelemetry(lat, lon, alt_filt, gps.speed.kmph(), gps.course.deg(), ts);
      lastTelemetryTime = millis();
    }
  }
}

void handleSerialRPI() {
  while (SerialRPI.available()) {
    String jsonIn = SerialRPI.readStringUntil('\n');
    processIncomingJSON(jsonIn, false);
  }
}

void handleLoRa() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String jsonIn = "";
    while (LoRa.available()) jsonIn += (char)LoRa.read();
    processIncomingJSON(jsonIn, true);
  }
}
