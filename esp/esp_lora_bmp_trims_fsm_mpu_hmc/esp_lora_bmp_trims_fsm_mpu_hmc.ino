
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
#include <units.h>
#include <mpu9250.h>
#include <eigen.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

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
#define RPI_BUFFER_SIZE 256
#define RPI_TIMEOUT_MS  50
char rpiBuffer[RPI_BUFFER_SIZE];
size_t rpiIndex = 0;
bool receivingJson = false;

// --- IMU MPU9250 ---
bfs::Mpu9250 mpu(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
bool mpuReady = false;
Adafruit_HMC5883_Unified magExt = Adafruit_HMC5883_Unified(12345);
bool magReady = false;

// Estado angular
unsigned long lastIMUUpdate = 0;

// → Ajuste de declinación magnética para Adrogué / Buenos Aires
const float MAG_DECLINATION_DEG = -9.9f;
unsigned long lastAttPrint = 0;
const unsigned long ATT_PRINT_INTERVAL = 500; 

// --- Filtros EMA (suavizado) ---
#define ALPHA_ACC  0.15f
#define ALPHA_GYRO 0.15f
#define ALPHA_MAG  0.20f
#define ALPHA_YAW  0.05f   // fusión: 0.05 = 95% giro + 5% compás

// Variables filtradas globales
float ax_f=0, ay_f=0, az_f=0;
float gx_f=0, gy_f=0, gz_f=0;
float mx_f=0, my_f=0, mz_f=0;

// Salidas globales
float roll=0, pitch=0, yaw_f=0;
float yawGyro = 0;
float yawMag = 0;

// ========================================================
//  🔧 CALIBRACIÓN MAGNÉTICA PRO (ellipsoid fitting)
// ========================================================

#define MAX_MAG_SAMPLES 1200   // hasta ~10 segundos
struct MagSample {
  float x, y, z;
};

MagSample magBuff[MAX_MAG_SAMPLES];
int magCount = 0;
bool magCalActivePRO = false;
unsigned long magCalStartPRO = 0;
unsigned long magCalDurationPRO = 0;


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

struct MagAxis {
  float minVal;
  float maxVal;
};

struct MagCalAxisData {
  bool active;
  unsigned long startMs;
  unsigned long durationMs;
  size_t samples;
  MagAxis x, y, z;
};

MagCalAxisData magCal;

String magQuality = "GOOD";
struct IMUCalibration {
  float accelBias[3];
  float gyroBias[3];
  float magBias[3];
  float magScale[3];
};

// Variable global que contiene los datos actuales de calibración
IMUCalibration imuCal;

struct PIDParams {
  float kp;
  float ki;
  float kd;
};

struct PIDSet {
  PIDParams accel;
  PIDParams roll_lr;
  PIDParams roll_fb;
  PIDParams rudder;
};

PIDSet pidConfig;   // configuración actual de PID

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

void savePID() {
  prefs.begin("pidConfig", false);
  size_t written = prefs.putBytes("config", &pidConfig, sizeof(pidConfig));
  prefs.end();
  Serial.printf("💾 Guardado PID (%u bytes de %u)\n", written, sizeof(pidConfig));
}

void loadPID() {
  prefs.begin("pidConfig", true);
  size_t len = prefs.getBytes("config", &pidConfig, sizeof(pidConfig));
  prefs.end();

  if (len == sizeof(pidConfig)) {
    Serial.println("📥 PID cargado desde NVS correctamente");
  } else {
    Serial.printf("⚠️ PID no encontrado (len=%u), cargando por defecto\n", len);
    pidConfig.accel  = {1.0, 0.0, 0.0};
    pidConfig.roll_lr = {1.0, 0.0, 0.0};
    pidConfig.roll_fb = {1.0, 0.0, 0.0};
    pidConfig.rudder = {1.0, 0.0, 0.0};
    savePID();
  }
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
  //Serial.println("📤 [sendWithAck] "+full);
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
  //Serial.println("📤 [UAV→GS] " + frame);
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
  //Serial.println("📤 [sendTelemetry] "+msg);
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
void sendEventToGS(const String &topic, double lat, double lon, double alt, unsigned long ts, float confidence) {
  StaticJsonDocument<256> doc;
  doc["t"] = topic;

  // Datos de posición del dron
  JsonObject d = doc.createNestedObject("d");
  d["lat"] = lat;
  d["lon"] = lon;
  d["alt"] = alt;
  d["confidence"] = confidence;
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
  Serial.printf("✅ JSON OK, uso de memoria: %u/%u\n", doc.memoryUsage(), doc.capacity());
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

    // =============================================================
    // 🧭 CONFIGURACIÓN PID - GET_PID / PID_SET
    // =============================================================

    else if (strcmp(type, "GET_PID") == 0) {
      StaticJsonDocument<384> docOut;
      docOut["t"] = "PID_DATA";
      JsonObject d = docOut.createNestedObject("d");

      // --- ACCEL ---
      JsonObject accel = d.createNestedObject("ACCEL");
      accel["kp"] = pidConfig.accel.kp;
      accel["ki"] = pidConfig.accel.ki;
      accel["kd"] = pidConfig.accel.kd;

      // --- ROLL_LR ---
      JsonObject roll_lr = d.createNestedObject("ROLL_LR");
      roll_lr["kp"] = pidConfig.roll_lr.kp;
      roll_lr["ki"] = pidConfig.roll_lr.ki;
      roll_lr["kd"] = pidConfig.roll_lr.kd;

      // --- ROLL_FB ---
      JsonObject roll_fb = d.createNestedObject("ROLL_FB");
      roll_fb["kp"] = pidConfig.roll_fb.kp;
      roll_fb["ki"] = pidConfig.roll_fb.ki;
      roll_fb["kd"] = pidConfig.roll_fb.kd;

      // --- RUDDER ---
      JsonObject rudder = d.createNestedObject("RUDDER");
      rudder["kp"] = pidConfig.rudder.kp;
      rudder["ki"] = pidConfig.rudder.ki;
      rudder["kd"] = pidConfig.rudder.kd;

      docOut["ts"] = millis();
      sendJsonNoAckToGS(docOut);
      sendAckToGS(msgId);
      Serial.println("📤 PID enviados a GS");
      return;
    }

    // =============================================================
    // ⚙️ PID_SET - Recibir todos los PID desde la Ground Station
    // =============================================================
    else if (strcmp(type, "PID_SET") == 0) {
      // Imprimir mensaje recibido para diagnóstico
      Serial.printf("📥 RX PID_SET (%d bytes): %s\n", jsonIn.length(), jsonIn.c_str());

      // Documento dedicado para PID (no reusar 'doc' general)
      StaticJsonDocument<1536> docPID;  // tamaño seguro para 5 canales (~1.3 KB máx)

      DeserializationError err = deserializeJson(docPID, jsonIn);
      if (err) {
        Serial.printf("❌ Error parseando JSON PID_SET: %s\n", err.c_str());
        return;
      }

      Serial.printf("✅ JSON OK, uso de memoria: %u/%u bytes\n",
                    docPID.memoryUsage(), docPID.capacity());
      JsonObject d = docPID["d"]["d"];  // ← acceso al nivel interno correcto

      if (d.isNull()) {
        Serial.println("⚠️ JSON PID_SET sin objeto 'd'");
        return;
      }

      // --- función auxiliar segura ---
      auto safe_get = [&](const char* ch, const char* field, float current) -> float {
        if (d.containsKey(ch)) {
          JsonVariant v = d[ch][field];
          if (!v.isNull()) {
            if (v.is<float>()) return v.as<float>();
            if (v.is<int>())   return (float)v.as<int>();
            if (v.is<const char*>()) return atof(v.as<const char*>());
          }
        }
        return current;
      };

      // --- ACCEL ---
      pidConfig.accel.kp = safe_get("ACCEL", "kp", pidConfig.accel.kp);
      pidConfig.accel.ki = safe_get("ACCEL", "ki", pidConfig.accel.ki);
      pidConfig.accel.kd = safe_get("ACCEL", "kd", pidConfig.accel.kd);

      // --- ROLL_LR ---
      pidConfig.roll_lr.kp = safe_get("ROLL_LR", "kp", pidConfig.roll_lr.kp);
      pidConfig.roll_lr.ki = safe_get("ROLL_LR", "ki", pidConfig.roll_lr.ki);
      pidConfig.roll_lr.kd = safe_get("ROLL_LR", "kd", pidConfig.roll_lr.kd);

      // --- ROLL_FB ---
      pidConfig.roll_fb.kp = safe_get("ROLL_FB", "kp", pidConfig.roll_fb.kp);
      pidConfig.roll_fb.ki = safe_get("ROLL_FB", "ki", pidConfig.roll_fb.ki);
      pidConfig.roll_fb.kd = safe_get("ROLL_FB", "kd", pidConfig.roll_fb.kd);

      // --- RUDDER ---
      pidConfig.rudder.kp = safe_get("RUDDER", "kp", pidConfig.rudder.kp);
      pidConfig.rudder.ki = safe_get("RUDDER", "ki", pidConfig.rudder.ki);
      pidConfig.rudder.kd = safe_get("RUDDER", "kd", pidConfig.rudder.kd);

      // Guardado seguro (NVS)
      Serial.printf("🧮 Valores antes de guardar:\n");
      Serial.printf("  ACCEL:  kp=%.3f ki=%.3f kd=%.3f\n", pidConfig.accel.kp, pidConfig.accel.ki, pidConfig.accel.kd);
      Serial.printf("  ROLL_FB: kp=%.3f ki=%.3f kd=%.3f\n", pidConfig.roll_fb.kp, pidConfig.roll_fb.ki, pidConfig.roll_fb.kd);

      savePID();

      sendAckToGS(msgId);
      Serial.println("✅ PID actualizado desde GS y guardado en NVS");

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
      startMagCalibrationPRO(30000);    // 30 s
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

    float confidence = -1;
    if (doc.containsKey("confidence")) confidence = doc["confidence"];
    else if (doc.containsKey("d") && doc["d"].containsKey("confidence"))
      confidence = doc["d"]["confidence"];

    if (gps.date.isValid() && gps.time.isValid()) {
      ts = toUnixTime(gps.date.year(), gps.date.month(), gps.date.day(),
                      gps.time.hour(), gps.time.minute(), gps.time.second());
    }
    sendEventToGS(type, lat_f, lon_f, alt_f, ts, confidence);
    return;
  }

  else if (strcmp(type, "PERSON") == 0) {
    analysisResult = PERSON;
    Serial.println("🧍 [RPI] Resultado recibido: PERSON → evento detectado");
    unsigned long ts = 0;

    float confidence = -1;
    if (doc.containsKey("confidence")) confidence = doc["confidence"];
    else if (doc.containsKey("d") && doc["d"].containsKey("confidence"))
      confidence = doc["d"]["confidence"];

    if (gps.date.isValid() && gps.time.isValid()) {
      ts = toUnixTime(gps.date.year(), gps.date.month(), gps.date.day(),
                      gps.time.hour(), gps.time.minute(), gps.time.second());
    }
    sendEventToGS(type, lat_f, lon_f, alt_f, ts, confidence);
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
  while (SerialRPI.available() > 0) {
    char c = (char)SerialRPI.read();

    // Buscar inicio de JSON
    if (!receivingJson) {
      if (c == '{') {
        receivingJson = true;
        rpiIndex = 0;
        rpiBuffer[rpiIndex++] = c;
      }
      continue;
    }

    // Acumular bytes del JSON
    if (receivingJson) {
      if (rpiIndex < RPI_BUFFER_SIZE - 1) {
        rpiBuffer[rpiIndex++] = c;
      } else {
        Serial.println("[RPI] ⚠️ Overflow de buffer, reinicio.");
        receivingJson = false;
        rpiIndex = 0;
        memset(rpiBuffer, 0, sizeof(rpiBuffer));
        continue;
      }

      // Fin del JSON detectado
      if (c == '}') {
        rpiBuffer[rpiIndex] = '\0';
        receivingJson = false;

        StaticJsonDocument<256> doc;
        DeserializationError err = deserializeJson(doc, rpiBuffer);

        if (!err) {
          // ✅ Serializar a String antes de pasarlo a processIncomingJSON()
          String jsonStr;
          serializeJson(doc, jsonStr);
          processIncomingJSON(jsonStr, false);
        } else {
          Serial.printf("[RPI] ⚠️ Error JSON: %s\n", err.c_str());
        }

        memset(rpiBuffer, 0, sizeof(rpiBuffer));
        rpiIndex = 0;
      }
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
// 🔧 FUNCIONES IMU MPU9250 + HMC5883L
// ============================================================
//-----Inicializacion-----
void initHMC5883L() {
  Serial.println("🔧 Inicializando magnetómetro externo HMC5883L...");

  if (!magExt.begin()) {
    Serial.println("❌ No se detectó HMC5883L");
    magReady = false;
    return;
  }

  Serial.println("✅ HMC5883L listo");
  magReady = true;
}

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
    delay(20);
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

// ============================================================
// 🔥 VERSION DEFINITIVA DE updateIMU() + FUSIONES + TILT COMP.
// ============================================================
// --- compensación de inclinación ---
float tiltCompensatedYaw(float rollDeg, float pitchDeg,
                         float mx, float my, float mz)
{
    float roll  = rollDeg  * DEG_TO_RAD;
    float pitch = pitchDeg * DEG_TO_RAD;

    float sinR = sinf(roll),  cosR = cosf(roll);
    float sinP = sinf(pitch), cosP = cosf(pitch);

    // Rotación clásica
    float mx_c = mx * cosP + mz * sinP;
    float my_c = mx * sinR * sinP + my * cosR - mz * sinR * cosP;

    float heading = atan2f(-my_c, mx_c) * RAD_TO_DEG;
    if (heading < 0) heading += 360.0f;
    return heading;
}

// --- Fusión yaw: compás + giro ---
void fuseYaw(float gz, float dt, float yawMag)
{
    yawGyro = 0;

    // Integración giroscopio
    yawGyro += gz * RAD_TO_DEG * dt;
    if (yawGyro < 0) yawGyro += 360;
    if (yawGyro >= 360) yawGyro -= 360;

    // Complementario
    yawGyro = (1 - ALPHA_YAW) * yawGyro + ALPHA_YAW * yawMag;

    // Corrección declinación
    yaw_f = yawGyro + MAG_DECLINATION_DEG;
    if (yaw_f < 0) yaw_f += 360;
    if (yaw_f >= 360) yaw_f -= 360;
}

void updateIMU() {
    if (!mpuReady) return;
    if (!mpu.Read()) return;

    static unsigned long last = millis();
    float dt = (millis() - last) * 0.001f;
    last = millis();

    // --- ACC + GYRO ---
    float ax = mpu.accel_x_mps2() - imuCal.accelBias[0];
    float ay = mpu.accel_y_mps2() - imuCal.accelBias[1];
    float az = mpu.accel_z_mps2() - imuCal.accelBias[2];

    float gx = mpu.gyro_x_radps() - imuCal.gyroBias[0];
    float gy = mpu.gyro_x_radps() - imuCal.gyroBias[1];
    float gz = mpu.gyro_z_radps() - imuCal.gyroBias[2];

    // --- HMC5883L MAGNETÓMETRO EXTERNO ---
    float mx_raw = 0, my_raw = 0, mz_raw = 0;
    if (magReady) {
        sensors_event_t ev;
        magExt.getEvent(&ev);
        mx_raw = ev.magnetic.x;
        my_raw = ev.magnetic.y;
        mz_raw = ev.magnetic.z;
    }

    // --- Aplicar tu calibración ---
    float mx = (mx_raw - imuCal.magBias[0]) * imuCal.magScale[0];
    float my = (my_raw - imuCal.magBias[1]) * imuCal.magScale[1];
    float mz = (mz_raw - imuCal.magBias[2]) * imuCal.magScale[2];

    // --- Filtros EMA ---
    ax_f = (1 - ALPHA_ACC)  * ax_f + ALPHA_ACC  * ax;
    ay_f = (1 - ALPHA_ACC)  * ay_f + ALPHA_ACC  * ay;
    az_f = (1 - ALPHA_ACC)  * az_f + ALPHA_ACC  * az;

    gx_f = (1 - ALPHA_GYRO) * gx_f + ALPHA_GYRO * gx;
    gy_f = (1 - ALPHA_GYRO) * gy_f + ALPHA_GYRO * gy;
    gz_f = (1 - ALPHA_GYRO) * gz_f + ALPHA_GYRO * gz;

    mx_f = (1 - ALPHA_MAG) * mx_f + ALPHA_MAG * mx;
    my_f = (1 - ALPHA_MAG) * my_f + ALPHA_MAG * my;
    mz_f = (1 - ALPHA_MAG) * mz_f + ALPHA_MAG * mz;

    // --- Roll / Pitch ---
    roll  = atan2f(ay_f, az_f) * RAD_TO_DEG;
    pitch = atan2f(-ax_f, sqrtf(ay_f*ay_f + az_f*az_f)) * RAD_TO_DEG;

    // --- Yaw compensado ---
    yawMag = tiltCompensatedYaw(roll, pitch, mx_f, my_f, mz_f);

    // --- Fusión ---
    fuseYaw(gz_f, dt, yawMag);

    // --- Debug ---
    printAttitude(roll, pitch, yaw_f, yawMag, yawGyro, mx_f, my_f, mz_f);
}

void printAttitude(float roll, float pitch, float yaw_f,float yawMag, float yawGyro, float mx, float my, float mz) {
  unsigned long now = millis();
  if (now - lastAttPrint < ATT_PRINT_INTERVAL) return;
  lastAttPrint = now;
  float magStrength = sqrtf(mx*mx + my*my + mz*mz);

  Serial.printf("\n===== ATTITUDE DEBUG =====\n");
  Serial.printf("Roll:  %.2f°\n", roll);
  Serial.printf("Pitch: %.2f°\n", pitch);
  Serial.printf("Yaw:   %.2f°\n", yaw_f);
  Serial.printf("Yawmag:   %.2f°\n", yawMag);
  Serial.printf("Yawgyro:   %.2f°\n", yawGyro);
  Serial.printf("Mag:   %.2f uT\n", magStrength);
  Serial.printf("Mx:   %.2f ", mx);
  Serial.printf("My:   %.2f ", my);
  Serial.printf("Mz:   %.2f \n", mz);
}


//----Calibración magnética dinámica----

void startMagCalibrationPRO(unsigned long durationMs) {
  if (!mpuReady) return;

  magCalActivePRO = true;
  magCount = 0;
  magCalStartPRO = millis();
  magCalDurationPRO = durationMs;

  Serial.println("🧭 [MAG PRO] Iniciando calibración 3D — rotar en todas las direcciones");

  // Aviso a la GS (mismo formato)
  StaticJsonDocument<192> j;
  j["t"] = "MAG_CAL_PROGRESS";
  JsonObject d = j.createNestedObject("d");
  d["phase"] = "START_3D";
  d["duration_ms"] = (int)durationMs;
  j["ts"] = (int)millis();
  sendJsonNoAckToGS(j);
}


void updateMagCalibrationPRO() {
  if (!magCalActivePRO) return;
  if (!mpu.Read()) return;

  sensors_event_t ev;
  magExt.getEvent(&ev);
  float mx = ev.magnetic.x;
  float my = ev.magnetic.y;
  float mz = ev.magnetic.z;


  if (magCount < MAX_MAG_SAMPLES) {
    magBuff[magCount++] = { mx, my, mz };
  }

  // Reporte de progreso cada 2s
  static unsigned long lastP = 0;
  if (millis() - lastP > 2000) {
    lastP = millis();
    StaticJsonDocument<192> j;
    j["t"] = "MAG_CAL_PROGRESS";
    JsonObject d = j.createNestedObject("d");
    d["phase"] = "RUN_3D";
    d["samples"] = magCount;
    d["elapsed"] = (int)(millis() - magCalStartPRO);
    j["ts"] = (int)millis();
    sendJsonNoAckToGS(j);
  }

  // FIN → procesar
  if (millis() - magCalStartPRO > magCalDurationPRO) {
    magCalActivePRO = false;
    computeMagCalibrationPRO();
  }
}


void computeMagCalibrationPRO() {
  Serial.printf("📊 [MAG PRO] Procesando %d muestras...\n", magCount);

  if (magCount < 200) {
    Serial.println("❌ No hay suficientes muestras para calibración 3D");
    return;
  }

  // 1 — Calcular bias simple (promedio min/max)
  float minX = 1e6, minY = 1e6, minZ = 1e6;
  float maxX = -1e6, maxY = -1e6, maxZ = -1e6;

  for (int i = 0; i < magCount; i++) {
    minX = min(minX, magBuff[i].x);
    minY = min(minY, magBuff[i].y);
    minZ = min(minZ, magBuff[i].z);

    maxX = max(maxX, magBuff[i].x);
    maxY = max(maxY, magBuff[i].y);
    maxZ = max(maxZ, magBuff[i].z);
  }

  float bx = (maxX + minX) * 0.5f;
  float by = (maxY + minY) * 0.5f;
  float bz = (maxZ + minZ) * 0.5f;

  // 2 — Soft iron (diagonal, simple)
  float rangeX = maxX - minX;
  float rangeY = maxY - minY;
  float rangeZ = maxZ - minZ;
  float avgRange = (rangeX + rangeY + rangeZ) / 3.0f;

  float sx = avgRange / rangeX;
  float sy = avgRange / rangeY;
  float sz = avgRange / rangeZ;

  // GUARDAR en tu struct existente (NO SE ROMPE NADA)
  imuCal.magBias[0] = bx;
  imuCal.magBias[1] = by;
  imuCal.magBias[2] = bz;

  imuCal.magScale[0] = sx;
  imuCal.magScale[1] = sy;
  imuCal.magScale[2] = sz;

  saveIMUCalibration();

  Serial.printf("✅ [MAG PRO] Bias(%.3f, %.3f, %.3f)\n", bx, by, bz);
  Serial.printf("   Scale(%.3f, %.3f, %.3f)\n", sx, sy, sz);

  // Enviar a GS igual que antes
  StaticJsonDocument<256> j;
  j["t"] = "MAG_CAL_RESULT";
  JsonObject d = j.createNestedObject("d");

  JsonObject bias = d.createNestedObject("bias");
  bias["x"] = bx;  bias["y"] = by;  bias["z"] = bz;

  JsonObject scale = d.createNestedObject("scale");
  scale["x"] = sx;  scale["y"] = sy;  scale["z"] = sz;

  d["samples"] = magCount;
  d["result"] = "OK_3D";

  j["ts"] = (int)millis();
  sendJsonNoAckToGS(j);

  printMagCalibration();
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

  sensors_event_t ev;
  magExt.getEvent(&ev);
  float mx = ev.magnetic.x;
  float my = ev.magnetic.y;
  float mz = ev.magnetic.z;

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
  /*StaticJsonDocument<192> jstatus;
  jstatus["t"] = "MAG_STATUS";
  JsonObject d = jstatus.createNestedObject("d");
  d["quality"] = magQuality;
  d["deviation"] = deviation;
  d["field"] = magStrength;
  jstatus["ts"] = (int)millis();
  sendJsonNoAckToGS(jstatus); */

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
// ============================================================
// 📡 IMPRIMIR CALIBRACIÓN MAGNÉTICA ACTUAL
// ============================================================
void printMagCalibration() {
  Serial.println("===============================================");
  Serial.println("🔍 Estado actual de calibración del magnetómetro");
  Serial.println("===============================================");

  Serial.printf("Bias (hard-iron):\n");
  Serial.printf("   X: %.3f uT\n", imuCal.magBias[0]);
  Serial.printf("   Y: %.3f uT\n", imuCal.magBias[1]);
  Serial.printf("   Z: %.3f uT\n", imuCal.magBias[2]);

  Serial.println();

  Serial.printf("Scale (soft-iron):\n");
  Serial.printf("   X: %.4f\n", imuCal.magScale[0]);
  Serial.printf("   Y: %.4f\n", imuCal.magScale[1]);
  Serial.printf("   Z: %.4f\n", imuCal.magScale[2]);

  Serial.println();

  Serial.printf("Declinación magnética aplicada: %.2f°\n", MAG_DECLINATION_DEG);

  Serial.println("-----------------------------------------------");

  // Opcional: imprimir fuerza del campo corregido (debug)
  float mx = (mpu.mag_x_ut() - imuCal.magBias[0]) * imuCal.magScale[0];
  float my = (mpu.mag_y_ut() - imuCal.magBias[1]) * imuCal.magScale[1];
  float mz = (mpu.mag_z_ut() - imuCal.magBias[2]) * imuCal.magScale[2];

  float magStrength = sqrtf(mx*mx + my*my + mz*mz);

  Serial.printf("Campo magnético corregido: %.2f uT\n", magStrength);
  Serial.println(" (valor típico entre 45 y 60 uT según región)");
  
  Serial.println("===============================================");
}

void testMagnetometerAK8963() {
  if (!mpuReady) {
    Serial.println("❌ MPU no inicializado");
    return;
  }

  Serial.println("\n🔍 TEST DIAGNÓSTICO DEL MAGNETÓMETRO (AK8963)");
  Serial.println("================================================");

  const int N = 600;   // 600 muestras (~5-6 segundos)
  float mx_min =  1e6, my_min =  1e6, mz_min =  1e6;
  float mx_max = -1e6, my_max = -1e6, mz_max = -1e6;

  Serial.println("👉 Mové el sensor en círculos amplios, roll/pitch/yaw…");

  unsigned long start = millis();
  while (millis() - start < 6000) {   // 6 segundos
    if (mpu.Read()) {
      float mx = mpu.mag_x_ut();
      float my = mpu.mag_y_ut();
      float mz = mpu.mag_z_ut();

      mx_min = fminf(mx_min, mx);
      my_min = fminf(my_min, my);
      mz_min = fminf(mz_min, mz);

      mx_max = fmaxf(mx_max, mx);
      my_max = fmaxf(my_max, my);
      mz_max = fmaxf(mz_max, mz);
    }
    delay(5);
  }

  // Resultados crudos
  float rx = mx_max - mx_min;
  float ry = my_max - my_min;
  float rz = mz_max - mz_min;

  Serial.println("\n📊 Rangos detectados (sin calibración):");
  Serial.printf("   X: %.2f uT\n", rx);
  Serial.printf("   Y: %.2f uT\n", ry);
  Serial.printf("   Z: %.2f uT\n", rz);

  // Campo estimado según movimientos
  float avgRange = (rx + ry + rz) / 3.0f;

  // Magnitud típica en Buenos Aires
  const float EARTH_MIN = 22.0;
  const float EARTH_MAX = 30.0;

  Serial.println("\n📌 Análisis del estado del magnetómetro:");
  bool damagedZ = false;

  // --- Diagnóstico por rangos ---
  if (rx < 8 || ry < 8) {
    Serial.println("⚠️  Problema: Movimiento detectado MUY bajo en X o Y → magnetómetro rígido o interferido.");
  }

  if (rz < 6) {
    Serial.println("❌ Eje Z casi inmóvil → posible daño en AK8963 o interferencia muy fuerte.");
    damagedZ = true;
  }

  // --- Desbalance ---
  float imbalanceXY = fabsf(rx - ry);
  float imbalanceXZ = fabsf(rx - rz);
  float imbalanceYZ = fabsf(ry - rz);

  Serial.printf("   Desbalance X-Y: %.2f uT\n", imbalanceXY);
  Serial.printf("   Desbalance X-Z: %.2f uT\n", imbalanceXZ);
  Serial.printf("   Desbalance Y-Z: %.2f uT\n", imbalanceYZ);

  // --- Interpretación ---
  Serial.println("\n🧠 Diagnóstico final:");

  if (damagedZ) {
    Serial.println("❌ **El eje Z parece DAÑADO** (casi no varía).");
  } else if (avgRange < 10) {
    Serial.println("⚠️ **Rango total bajo** → magnetómetro muy interferido.");
  } else if (imbalanceXZ > 20 || imbalanceYZ > 20) {
    Serial.println("⚠️ **Desbalance fuerte entre ejes** → interferencia fija o hard-iron externo.");
  } else {
    Serial.println("✅ **Magnetómetro OK** — rangos normales.");
  }

  Serial.println("================================================\n");
}


// ============================================================================
// 🚀 SETUP Y LOOP PRINCIPALES
// ============================================================================
void setup(){
  Serial.begin(115200);
  SerialGPS.begin(9600,SERIAL_8N1,GPS_RX,GPS_TX);
  SerialRPI.begin(9600, SERIAL_8N1, RPI_RX, RPI_TX);
  SerialRPI.setTimeout(RPI_TIMEOUT_MS);
  memset(rpiBuffer, 0, sizeof(rpiBuffer));
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
  initHMC5883L();
  calibrateIMU_Static();
  loadTrims();
  loadPID();
  printMagCalibration();
  delay(1500);
}

void loop(){
  //Serial.println("inicio");
  updateGPS();
  //Serial.println("gps");
  handleTelemetry();
  //Serial.println("tm");
  handleSerialRPI();
  //Serial.println("rpi");
  handleLoRa();
  //Serial.println("lora");
  checkPendingAcks();
  //Serial.println("ack");
  updateStateMachine();
  //Serial.println("fsm");
  simulateDroneMotion();
  //Serial.println("fsm-sim");
  updateIMU();
  //Serial.println("imu");
  updateMagCalibrationPRO();
  //Serial.println("magupd");
  checkMagQualitySuggest();
 // Serial.println("magqua");
  if (Serial.available()) {
  char c = Serial.read();
  if (c == 'M') testMagnetometerAK8963();
}

}
