
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <math.h>
#include <vector>
#include <HardwareSerial.h>
#include <MAVLink.h>   
#include <ESP32Servo.h>

using std::vector;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// 🛰️ CONFIGURACIÓN DE HARDWARE
// ============================================================================

// --- LoRa ---
#define LORA_CS  5
#define LORA_RST 14
#define LORA_IRQ 34
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
char  rpiBuffer[RPI_BUFFER_SIZE];
size_t rpiIndex = 0;
bool  receivingJson = false;

// --- Pixhawk MAVLink ---
#define MAV_RX 16   // ESP32 RX ← Pixhawk TX (TELEM2)
#define MAV_TX 17   // ESP32 TX → Pixhawk RX (TELEM2)
HardwareSerial SerialMAV(2);
unsigned long lastGotoMs = 0;
static unsigned long lastHbMs = 0;

//failsafe bateria
bool battery_failsafe_active = false;
const float LOW_VOLTAGE_THRESHOLD = 11.1; // ajustar si querés

//arm
bool awaitingFastArmConfirm = false;
unsigned long armingConfirmationDeadline = 0;

// ============================================================================
// ⚙️ SERVO DE ACCIÓN (detección PERSON)
// ============================================================================
Servo actionServo;

const int SERVO_PIN = 25;      // GPIO donde conectás el servo
const int SERVO_CLOSED = 105;    // grados (cerrado)
const int SERVO_OPEN   = 35;   // grados (abierto)

bool servoActive = false;
unsigned long servoOpenStartMs = 0;
const unsigned long SERVO_OPEN_TIME = 30000;  // 30 segundos

// ============================================================================
// 📌 ESTRUCTURAS Y ESTADO GLOBAL
// ============================================================================

struct Coordinate {
  double lat;
  double lon;
};

struct Mission {
  bool loaded = false;
  vector<Coordinate> polygon;
  Coordinate home;
  double altitude = 0.0;
  double spacing  = 0.0;
  double detect_spacing = 5.0;   // spacing interno entre detecciones
  String event_action;     // "RETURN", "CONTINUE", etc.
} mission;

//---variables parser mavlink
// HEARTBEAT
uint8_t mav_type = 0;
uint8_t mav_autopilot = 0;
uint8_t mav_base_mode = 0;
uint32_t mav_custom_mode = 0;
uint8_t mav_system_status = 0;
bool mav_armed = false;

// GLOBAL POSITION
double mav_lat = 0, mav_lon = 0;
double mav_alt_rel = 0;
double mav_alt_abs = 0;
double mav_ground_speed = 0;
double mav_heading_deg = 0;
double mav_climb_rate = 0;
bool mav_has_fix = false;
unsigned long mav_last_update_ms = 0;

// BATTERY / SYS_STATUS
float mav_batt_voltage = 0.0f;
float mav_batt_current = 0.0f;
int   mav_batt_remaining = -1;

// Máximos ajustables RECEPCION WP CHUNKS
static const int MAX_MISSION_CHUNKS = 64;

String missionChunks[MAX_MISSION_CHUNKS];
bool   missionChunkReceived[MAX_MISSION_CHUNKS];
int    missionExpectedChunks = 0;
int    missionTotalWPs = 0;

bool   missionHasHeader = false;
bool   missionReady = false;

unsigned long missionLastChunkTime = 0;
const unsigned long MISSION_TIMEOUT = 3000;   // 3s → timeout para chunk faltante

// Traduce el custom_mode de ArduCopter a texto legible
String decodeFlightMode(uint32_t m) {
    switch (m) {
        case 0:  return "STABILIZE";
        case 3:  return "AUTO";
        case 4:  return "GUIDED";
        case 5:  return "LOITER";
        case 6:  return "RTL";
        default: return "UNKNOWN";
    }
}

// =======================
// 🛰️ Filtro GPS (tu versión)
// =======================
double lat_window[4] = {0};
double lon_window[4] = {0};
double alt_window[4] = {0};
int    win_idx = 0;

double lat_f = 0.0, lon_f = 0.0, alt_f = 0.0;
bool   have_filter = false;
int    gpsWarmup   = 0;
const double MAX_JUMP_DEG = 0.00030; // ~33 m

// =======================
// ⛓️ ACK LoRa
// =======================
struct PendingMsg {
  String payload;
  unsigned long lastSend;
  int  retries;
  bool waitingAck;
  String msgID;
};
#define MAX_PENDING 5
PendingMsg pendingMsgs[MAX_PENDING];
unsigned long ackTimeout   = 1500;
int           maxRetries   = 4;
unsigned long nextMsgCounter = 0;

// =======================
// 🧭 Máquina de estados
// =======================
enum DroneState {
    IDLE,
    ARMING,
    READY_FOR_MISSION,
    PREPARE_TAKEOFF,
    TAKEOFF,
    NAVIGATE,
    STABILIZE,
    WAIT_ANALYSIS,
    RETURN_HOME,
    LAND,
    COMPLETE
};

DroneState state = IDLE;
unsigned long stateEntryTime    = 0;
unsigned long insideRadiusSince = 0;

// Para control del STATUS
DroneState lastSentState = IDLE;
bool statusDirty         = true;

void resetMissionState();
void sendStatusToGS(DroneState st);
void simulateDroneMotion();

void checkMissionChunkTimeout() {
    if (!missionHasHeader || missionReady) return;

    if (millis() - missionLastChunkTime > MISSION_TIMEOUT) {
        for (int i = 0; i < missionExpectedChunks; i++) {
            if (!missionChunkReceived[i]) {
                Serial.printf("⛔ Falta chunk %d → solicitando reenvío\n", i);
                // construir REQ_WP_CHUNK
                StaticJsonDocument<128> doc;
                doc["t"] = "REQ_WP_CHUNK";
                JsonObject d = doc.createNestedObject("d");
                d["i"] = i;

                String payload;
                serializeJson(doc, payload);
                String frame = String(UAV_HDR) + payload + SFX;

                LoRa.beginPacket();
                LoRa.print(frame);
                LoRa.endPacket();

                // Reiniciar el reloj para volver a chequear luego
                missionLastChunkTime = millis();
                return;  // pedimos 1 por vez
            }
        }
    }
}

void sendMissionError(const char* msg) {
    StaticJsonDocument<128> doc;
    doc["t"] = "MISSION_ERROR";
    JsonObject d = doc.createNestedObject("d");
    d["error"] = msg;

    String p;
    serializeJson(doc, p);
    String frame = String(UAV_HDR) + p + SFX;

    LoRa.beginPacket();
    LoRa.print(frame);
    LoRa.endPacket();
}


// ============================================================================
// 📡 Enviar estado de la FSM a la Ground Station
// ============================================================================
// Convierte el estado de la FSM a texto legible
String fsmStateToString(int s) {
  switch (s) {
    case IDLE:              return "IDLE";
    case ARMING:            return "ARMING";
    case READY_FOR_MISSION: return "READY_FOR_MISSION";
    case PREPARE_TAKEOFF:   return "PREPARE_TAKEOFF";
    case TAKEOFF:           return "TAKEOFF";
    case NAVIGATE:          return "NAVIGATE";
    case STABILIZE:         return "STABILIZE";
    case WAIT_ANALYSIS:     return "WAIT_ANALYSIS";
    case RETURN_HOME:       return "RETURN_HOME";
    case LAND:              return "LAND";
    case COMPLETE:          return "COMPLETE";
    default:                return "UNKNOWN";
  }
}

void sendStatusToGS(DroneState st) {

  if (st == lastSentState && !statusDirty)
    return;

  lastSentState = st;
  statusDirty   = false;

  // ----- Construimos JSON -----
  StaticJsonDocument<128> doc;
  doc["t"]       = "STATUS";
  doc["message"] = fsmStateToString((int)st);
  doc["ts"]      = millis();

  String payload;
  serializeJson(doc, payload);

  // ----- Generamos un msgID único -----
  String msgID = generateMsgID();   // ← ya existe en tu sistema

  // ----- Construimos el frame completo -----
  String frame = String(UAV_HDR) + payload + SFX;   // ej: "UAV#{json}#END"

  // ----- Envio usando ACK -----
  sendWithAck(frame, msgID);

  Serial.println("📤 STATUS enviado (ACK): " + fsmStateToString((int)st));
}


// Análisis RPi
enum AnalysisResult { NONE, GO, FIRE, PERSON };
AnalysisResult analysisResult = NONE;

// Misión / recorrido
int currentWaypoint = 0;
vector<Coordinate> pathPoints;

// Timeout análisis
const unsigned long ANALYSIS_TIMEOUT = 6000;
unsigned long analysisStartTime = 0;

// Comandos por LoRa
bool loraReturnCommand = false;
bool loraDisarmCommand = false;

// ⭐ MODO TEST (vuelo simulado sin mover motores)
bool testMode = false;

// Modo simulación
bool simulationMode = false;

// Heartbeat Pixhawk
unsigned long lastHeartbeatMs = 0;

// ============================================================================
// 🛰️ FILTRO GPS (mediana + warmup + rechazo de saltos)
// ============================================================================
double median4(double a, double b, double c, double d) {
  double arr[4] = {a,b,c,d};
  for (int i = 0; i < 3; i++)
    for (int j = i + 1; j < 4; j++)
      if (arr[j] < arr[i]) {
        double t = arr[i]; arr[i] = arr[j]; arr[j] = t;
      }
  return (arr[1] + arr[2]) / 2.0;
}

void filterGPS(double lat, double lon, double alt,
               double &lat_out, double &lon_out, double &alt_out) {

  if (fabs(lat) < 0.0001 && fabs(lon) < 0.0001)
    return;

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
    Serial.println("[GPS] 🆗 Filtro inicializado");
  }
  else {
    gpsWarmup++;
    if (gpsWarmup < 200) {
      lat_f = 0.7 * lat_m + 0.3 * lat_f;
      lon_f = 0.7 * lon_m + 0.3 * lon_f;
      alt_f = 0.5 * alt + 0.5 * alt_f;

      double dLat = fabs(lat_m - lat_f);
      double dLon = fabs(lon_m - lon_f);
      static int stableCount = 0;
      if (dLat < 0.00002 && dLon < 0.00002) {
        stableCount++;
        if (stableCount >= 8) {
          gpsWarmup = 9999;
          Serial.println("[GPS] ✅ Estabilización completada");
        }
      }
    } else {
      if (fabs(lat_m - lat_f) > MAX_JUMP_DEG || fabs(lon_m - lon_f) > MAX_JUMP_DEG) {
        Serial.println("[GPS] ⚠️ Salto descartado");
        lat_out = lat_f; lon_out = lon_f; alt_out = alt_f;
        return;
      }
      lat_f = 0.25 * lat_m + 0.75 * lat_f;
      lon_f = 0.25 * lon_m + 0.75 * lon_f;
      alt_f = 0.25 * alt    + 0.75 * alt_f;
    }
  }

  lat_out = lat_f;
  lon_out = lon_f;
  alt_out = alt_f;
}

// ============================================================================
// 🌎 Utils geodésicos
// ============================================================================
double deg2rad(double deg) { return deg * M_PI / 180.0; }
double rad2deg(double rad) { return rad * 180.0 / M_PI; }

double haversineDistance(double la1, double lo1, double la2, double lo2) {
  double R = 6371000.0;
  double dLat = deg2rad(la2 - la1);
  double dLon = deg2rad(lo2 - lo1);
  double a = sin(dLat/2)*sin(dLat/2) +
             cos(deg2rad(la1))*cos(deg2rad(la2))*sin(dLon/2)*sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

double computeBearing(double la1, double lo1, double la2, double lo2) {
  double y = sin(deg2rad(lo2 - lo1)) * cos(deg2rad(la2));
  double x = cos(deg2rad(la1))*sin(deg2rad(la2)) -
             sin(deg2rad(la1))*cos(deg2rad(la2))*cos(deg2rad(lo2 - lo1));
  double brng = atan2(y, x);
  return fmod((rad2deg(brng) + 360.0), 360.0);
}

// ============================================================================
// 📡 ACKs LoRa – helpers
// ============================================================================
String generateMsgID() {
  nextMsgCounter++;
  return String(nextMsgCounter);
}

void sendWithAck(const String &jsonPayload, const String &id) {
  String full = jsonPayload;
  if (!jsonPayload.startsWith("UAV#")) full = "UAV#" + jsonPayload;
  if (!full.endsWith("#END")) full += "#END";

  LoRa.beginPacket();
  LoRa.print(full);
  LoRa.endPacket();

  for (int i = 0; i < MAX_PENDING; i++) {
    if (!pendingMsgs[i].waitingAck) {
      pendingMsgs[i].payload    = full;
      pendingMsgs[i].lastSend   = millis();
      pendingMsgs[i].retries    = maxRetries;
      pendingMsgs[i].waitingAck = true;
      pendingMsgs[i].msgID      = id;
      return;
    }
  }
  Serial.println("⚠️ Cola ACK llena");
}

void handleAck(const String &ackID) {
  for (int i = 0; i < MAX_PENDING; i++) {
    if (pendingMsgs[i].waitingAck && pendingMsgs[i].msgID == ackID) {
      String tipo = "Desconocido";
      int posT = pendingMsgs[i].payload.indexOf("\"t\":\"");
      if (posT >= 0) {
        int posEnd = pendingMsgs[i].payload.indexOf("\"", posT + 5);
        if (posEnd > posT) tipo = pendingMsgs[i].payload.substring(posT + 5, posEnd);
      }

      //Serial.print("✅ [ACK] ID="); Serial.print(ackID);
     // Serial.print(" (Tipo=");      Serial.print(tipo);
      //Serial.println(")");

      pendingMsgs[i].waitingAck = false;
      pendingMsgs[i].payload = "";
      pendingMsgs[i].msgID   = "";
      return;
    }
  }
 // Serial.print("⚠️ [ACK desconocido] ID="); Serial.println(ackID);
}

void checkPendingAcks() {
  unsigned long now = millis();
  for (int i = 0; i < MAX_PENDING; i++) {
    if (!pendingMsgs[i].waitingAck) continue;

    if (now - pendingMsgs[i].lastSend > ackTimeout) {
      if (pendingMsgs[i].retries > 0) {
        LoRa.beginPacket();
        LoRa.print(pendingMsgs[i].payload);
        LoRa.endPacket();

        pendingMsgs[i].lastSend = now;
        pendingMsgs[i].retries--;

        String tipo = "Desconocido";
        int posT = pendingMsgs[i].payload.indexOf("\"t\":\"");
        if (posT >= 0) {
          int posEnd = pendingMsgs[i].payload.indexOf("\"", posT + 5);
          if (posEnd > posT) tipo = pendingMsgs[i].payload.substring(posT + 5, posEnd);
        }

       // Serial.print("🔁 [Reintento ACK] ID=");
       // Serial.print(pendingMsgs[i].msgID);
        //Serial.print(" Tipo="); Serial.print(tipo);
       // Serial.print(" quedan "); Serial.print(pendingMsgs[i].retries);
       // Serial.println(" intentos");
      } else {
       // Serial.print("❌ [ACK perdido] ID=");
       // Serial.print(pendingMsgs[i].msgID);
       // Serial.print(" payload="); Serial.println(pendingMsgs[i].payload);

        pendingMsgs[i].waitingAck = false;
        pendingMsgs[i].payload = "";
        pendingMsgs[i].msgID   = "";
      }
    }
  }
}

void sendAckToGS(const String &id) {
    if (id == "") {
        Serial.println("⚠️ ACK sin ID");
        return;
    }

    StaticJsonDocument<128> doc;
    doc["t"] = "ACK";
    doc["id"] = id;        // 🔥 ID AL NIVEL SUPERIOR
    doc["ts"] = millis();

    String p;
    serializeJson(doc, p);

    String msg = String(UAV_HDR) + p + String(SFX);

    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();

    // Debug opcional
    // Serial.println("📤 ACK → " + msg);
}

void sendJsonNoAckToGS(const JsonDocument &doc) {
  String payload;
  serializeJson(doc, payload);
  String frame = String(UAV_HDR) + payload + String(SFX);

  LoRa.beginPacket();
  LoRa.print(frame);
  LoRa.endPacket();
}

// ============================================================================
// 📡 TELEMETRÍA Y EVENTOS HACIA GS
// ============================================================================
void sendTelemetry(double lat,double lon,double alt,double speed,double heading,unsigned long ts) {
    StaticJsonDocument<256> doc;
    doc["t"] = "TELEMETRY";

    JsonObject d = doc.createNestedObject("d");
    d["lat"]     = lat;
    d["lon"]     = lon;
    d["alt"]     = alt;
    d["speed"]   = speed;
    d["heading"] = heading;

    // Datos reales desde MAVLink
    d["battery"] = mav_batt_remaining;      // %
    d["voltage"] = mav_batt_voltage;        // V
    d["armed"]   = mav_armed;               // true/false
    d["mode"]    = decodeFlightMode(mav_custom_mode);

    // 🔥 NUEVO: alerta por voltaje bajo
    const float LOW_VOLTAGE_THRESHOLD = 11.1; 
    bool warn = (mav_batt_voltage > 0) && (mav_batt_voltage < LOW_VOLTAGE_THRESHOLD);
    d["batt_warn"] = warn;

    doc["ts"]    = ts;

    String p;
    serializeJson(doc,p);
    String msg = String(UAV_HDR) + p + String(SFX);

    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();
}



void sendEventToGS(const String &type, double lat, double lon, double alt, 
                   unsigned long wpIndex, float temperature) 
{
  // ----- Construimos JSON -----
  StaticJsonDocument<256> doc;
  doc["t"]       = type;      // Ej: "EVENT_FIRE", "EVENT_PERSON", etc.
  doc["lat"]     = lat;
  doc["lon"]     = lon;
  doc["alt"]     = alt;
  doc["wp"]      = wpIndex;
  doc["temp"]    = temperature;
  doc["ts"]      = millis();

  String payload;
  serializeJson(doc, payload);

  // ----- Generamos msgID único -----
  String msgID = generateMsgID();

  // ----- Armamos frame UAV#...#END -----
  String frame = String(UAV_HDR) + payload + SFX;

  // ----- Envío por ACK -----
  sendWithAck(frame, msgID);

  Serial.println("📤 EVENT enviado (ACK): " + type);
}


// ============================================================================
// 🔎 Extracción de frames LoRa (GS#...#END)
// ============================================================================
bool extractNextFrame(String &buf, String &jsonOut, const char* wantedHdr) {
  int h  = buf.indexOf(wantedHdr);
  int hU = buf.indexOf(UAV_HDR);

  if (hU >= 0 && (hU < h || h < 0)) {
    int endU = buf.indexOf(SFX, hU);
    if (endU < 0) return false;
    buf.remove(0, endU + strlen(SFX));
    return false;
  }

  if (h < 0) {
    if (buf.length() > 2048)
      buf.remove(0, buf.length() - 256);
    return false;
  }

  int lbrace = buf.indexOf('{', h);
  if (lbrace < 0) return false;
  int end = buf.indexOf(SFX, lbrace);
  if (end < 0) return false;

  jsonOut = buf.substring(lbrace, end);
  buf.remove(0, end + strlen(SFX));
  jsonOut.trim();
  return true;
}

// ============================================================================
// 📥 PROCESAMIENTO DE JSON DESDE GS O RPi
// ============================================================================
void processIncomingJSON(const String &jsonIn, bool fromGS) {
  StaticJsonDocument<1024> doc;

  if (deserializeJson(doc, jsonIn)) {
    Serial.println("❌ JSON inválido");
    return;
  }

  const char* type  = doc["t"]  | "";
  const char* msgId = doc["id"] | "";

  Serial.println("📥 [processIncomingJSON] " + jsonIn);

  // ------------------------------------------------------------------
  // 1) MENSAJES DESDE LA GROUND STATION
  // ------------------------------------------------------------------
  if (fromGS) {

    // -------------------------------
    // ACK desde GS
    // -------------------------------
    if (strcmp(type, "ACK") == 0) {
      // Preferimos ID toplevel
      String id = String(msgId);

      // Compatibilidad: si alguien envía {"t":"ACK","d":{"id":"..."}} también lo aceptamos
      if (id == "" && doc.containsKey("d")) {
        id = doc["d"]["id"] | "";
      }

      if (id != "") {
        handleAck(id);
      } else {
        Serial.println("⚠️ ACK recibido sin ID");
      }
      return;
    }

    // ==========================================================
    //  ⭐ SIM_ON → activar testMode + simulación
    // ==========================================================
    if (strcmp(type, "SIM_ON") == 0) {
      simulationMode = true;
      testMode      = true;   // << activar modo test
      mav_armed     = true;   // << finge que está armado
      Serial.println("🧪 MODO TEST + SIMULACIÓN ACTIVADOS");
      sendAckToGS(msgId);
      return;
    }

    // ==========================================================
    //  ⭐ SIM_OFF → desactivar testMode + simulación
    // ==========================================================
    if (strcmp(type, "SIM_OFF") == 0) {
      simulationMode = false;
      testMode      = false;
      mav_armed     = false;   // opcional: volvemos a "no armado"
      Serial.println("🛑 MODO TEST DESACTIVADO");
      sendAckToGS(msgId);
      return;
    }


    //request waypoints para comparar
    if (strcmp(type,"REQ_WP_DEBUG")==0) {
    sendWaypointsDebugToGS();
    sendAckToGS(msgId); 
    }


    // ==========================================================
    //  ARM → en testMode se simula el ARM
    // ==========================================================
    if (strcmp(type, "ARM") == 0) {

      Serial.println("📡 GS → ARM solicitado");
      sendAckToGS(msgId);

      // 1) Siempre GUIDED antes de armar (solo en modo real)
      if (!testMode) {
        setModeGuided();
        delay(150);
      }

      // -------------------------
      // ⭐ MODO TEST → NO ARMAR REAL
      // -------------------------
      if (testMode) {
        Serial.println("🛑 [TEST] ARM ignorado. Simulando armado.");
        mav_armed = true;     // SIMULA ARM REAL
      } else {
        pixhawkArm(true);     // ARM REAL
      }

      // Estado ARMING
      state = ARMING;
      stateEntryTime = millis();
      sendStatusToGS(state);

      // Fast confirm
      armingConfirmationDeadline = millis() + 2000;
      awaitingFastArmConfirm     = true;

      // Confirmación instantánea
      if (mav_armed) {
        Serial.println("✔ ARM confirmado (real o simulado)");
        state = READY_FOR_MISSION;
        sendStatusToGS(state);
      }

      return;
    }

    // ==========================================================
    // DISARM
    // ==========================================================
    if (strcmp(type, "DISARM") == 0) {
      Serial.println("🛑 GS → DISARM");

      if (!testMode) {
        pixhawkDisarmForce();
        delay(50);
        pixhawkDisarmForce();
      }

      loraDisarmCommand = true;
      sendAckToGS(msgId);

      state = IDLE;
      sendStatusToGS(state);
      return;
    }

    // ==========================================================
    // RETURN
    // ==========================================================
    if (strcmp(type, "RETURN") == 0) {
      Serial.println("↩️ RETURN recibido");

      loraReturnCommand = true;
      state = RETURN_HOME;
      sendAckToGS(msgId);

      // ⭐ Enviamos GOTO inmediato a HOME para que el Pix empiece a irse
      if (mission.loaded) {
        sendMavGoto(mission.home.lat, mission.home.lon, mission.altitude);
      }

      return;
    }

    // ==========================================================
    // GRIPPER desde GS: OPEN / CLOSE
    // ==========================================================
    if (strcmp(type, "GRIPPER") == 0) {

        if (!doc.containsKey("d") || !doc["d"].containsKey("state")) {
            Serial.println("⚠️ GRIPPER sin campo 'state'");
            sendAckToGS(msgId);
            return;
        }

        const char* st = doc["d"]["state"];

        if (strcmp(st, "OPEN") == 0) {
            Serial.println("🟢 [GS] GRIPPER OPEN");
            actionServo.write(SERVO_OPEN);
            servoActive = false;  // desactiva auto-cierre automático
        }
        else if (strcmp(st, "CLOSE") == 0) {
            Serial.println("🔴 [GS] GRIPPER CLOSE");
            actionServo.write(SERVO_CLOSED);
            servoActive = false;  // detiene countdown si estaba activo
        }
        else {
            Serial.printf("⚠️ GRIPPER state desconocido: %s\n", st);
        }

        sendAckToGS(msgId);
        return;
    }

    // ==========================================================
    // MISSION_INFO → header de misión desde la GS
    // ==========================================================
    if (strcmp(type, "MISSION_INFO") == 0) {
        Serial.println("📦 RX MISSION_INFO");
        missionLastChunkTime = millis();

        JsonObject d = doc["d"];
        if (d.isNull()) {
            Serial.println("❌ Campo d faltante en MISSION_INFO");
            return;
        }

        // HOME
        JsonArray h = d["home"];
        if (!h.isNull() && h.size() == 2) {
            mission.home.lat = h[0];
            mission.home.lon = h[1];
        }

        mission.altitude        = d["alt"]            | 20.0;
        mission.spacing         = d["spacing"]        | 10.0;
        mission.detect_spacing  = d["detect_spacing"] | 5.0;
        mission.event_action    = d["event_action"]   | "NONE";

        missionExpectedChunks   = d["chunks"]   | 0;
        missionTotalWPs         = d["wp_total"] | 0;

        if (missionExpectedChunks <= 0 || missionExpectedChunks > MAX_MISSION_CHUNKS) {
            Serial.println("❌ MISSION_INFO: chunks inválidos");
            missionExpectedChunks = 0;
            return;
        }

        // Reset buffers
        for (int i = 0; i < MAX_MISSION_CHUNKS; i++) {
            missionChunks[i] = "";
            missionChunkReceived[i] = false;
        }

        missionHasHeader = true;
        missionReady     = false;

        Serial.printf("📦 Header misión: HOME=(%.7f, %.7f), alt=%.1f, chunks=%d, wp_total=%d\n",
                      mission.home.lat, mission.home.lon, mission.altitude,
                      missionExpectedChunks, missionTotalWPs);

        sendAckToGS(msgId);
        return;
    }
    // ==========================================================
    // MISSION_WP_CHUNK → fragmentos de waypoints desde la GS
    // ==========================================================
    if (strcmp(type, "MISSION_WP_CHUNK") == 0) {
        JsonObject d = doc["d"];
        if (d.isNull()) {
            Serial.println("❌ Campo d faltante en MISSION_WP_CHUNK");
            return;
        }

        if (!missionHasHeader) {
            Serial.println("⚠️ Recibí MISSION_WP_CHUNK sin MISSION_INFO previo");
            return;
        }

        int idx   = d["i"] | -1;
        int total = d["n"] | -1;
        const char* chunkStr = d["chunk"] | "";

        if (idx < 0 || idx >= missionExpectedChunks) {
            Serial.printf("⚠️ Chunk index fuera de rango: i=%d\n", idx);
            return;
        }

        if (total != missionExpectedChunks) {
            Serial.printf("⚠️ total chunks mismatch: header=%d, msg=%d\n",
                          missionExpectedChunks, total);
            return;
        }

        missionChunks[idx] = String(chunkStr);
        missionChunkReceived[idx] = true;
        missionLastChunkTime = millis();

        Serial.printf("🧩 Chunk %d/%d recibido (len=%d)\n",
                      idx + 1, missionExpectedChunks,
                      missionChunks[idx].length());

        sendAckToGS(msgId);

        // -------------------------------------------------
        // Chequear si YA ESTÁN TODOS LOS CHUNKS
        // -------------------------------------------------
        bool allReceived = true;
        for (int i = 0; i < missionExpectedChunks; i++) {
            if (!missionChunkReceived[i]) {
                allReceived = false;
                break;
            }
        }

        if (!allReceived) {
            return;  // esperar más
        }

        // -------------------------------------------------
        // Concatenar en ORDEN todos los chunks
        // -------------------------------------------------
        Serial.println("🧷 Todos los chunks recibidos → reconstruyendo lista de WPs");

        std::vector<Coordinate> wps;
        wps.reserve(missionTotalWPs > 0 ? missionTotalWPs : 100);

        for (int c = 0; c < missionExpectedChunks; c++) {
            String &s = missionChunks[c];
            if (s.length() == 0) continue;

            // parsear "lat,lon;lat,lon;..."
            char buffer[512];
            s.toCharArray(buffer, sizeof(buffer));

            char* token = strtok(buffer, ";");
            while (token != nullptr) {
                double lat, lon;
                if (sscanf(token, "%lf,%lf", &lat, &lon) == 2) {
                    Coordinate pt;
                    pt.lat = lat;
                    pt.lon = lon;
                    wps.push_back(pt);
                }
                token = strtok(nullptr, ";");
            }
        }

        Serial.printf("📌 Reconstruidos %d WPs para la misión\n", (int)wps.size());

        // -------------------------------------------------
        // DEBUG: imprimir cada WP reconstruido
        // -------------------------------------------------
        Serial.println("📋 Lista completa de WPs reconstruidos:");
        for (int i = 0; i < wps.size(); i++) {
            Serial.printf("   [%d] %.7f , %.7f\n",
                          i, wps[i].lat, wps[i].lon);
        }

        // opcional: si wps.size() != missionTotalWPs, loguear advertencia
        if (missionTotalWPs > 0 && (int)wps.size() != missionTotalWPs) {
            Serial.printf("⚠️ Advertencia: esperados=%d, reconstruidos=%d\n",
                          missionTotalWPs, (int)wps.size());
        }

        // -------------------------------------------------
        // Asignar a pathPoints (la ruta REAL de vuelo)
        // -------------------------------------------------
        pathPoints.clear();
        for (auto &pt : wps) {
            pathPoints.push_back(pt);
        }

        currentWaypoint = 0;
        mission.loaded  = true;
        missionReady    = true;

        Serial.printf("✅ Misión lista: %d WAYPOINTS en pathPoints\n",
                      (int)pathPoints.size());

        // Avisar a la GS (si querés)
        sendMissionLoadedToGS(pathPoints.size());

        return;
    }


    // ==========================================================
    // START_FLIGHT → iniciar misión manualmente
    // ==========================================================
    if (strcmp(type, "START_FLIGHT") == 0) {
        Serial.println("🚁 START_FLIGHT recibido → PREPARAR DESPEGUE");
        sendAckToGS(msgId);

        // 1) Verificar que la misión esté cargada
        if (!mission.loaded || pathPoints.empty()) {
            Serial.println("❌ No hay misión cargada. Ignorando START_FLIGHT.");
            return;
        }

        // 2) Verificar que esté armado
        if (!mav_armed) {
            Serial.println("⚠️ No armado, ignorando START_FLIGHT");
            return;
        }

        // 3) Verificar home válido
        if (mission.home.lat == 0 || mission.home.lon == 0) {
            Serial.println("❌ HOME inválido, START_FLIGHT cancelado");
            return;
        }

        // 4) Cambiar a modo GUIDED (solo en real)
        if (!testMode) {
            setModeGuided();
            Serial.println("🚁 GUIDED seleccionado");
        }

        // 5) La FSM se encarga del takeoff
        state = PREPARE_TAKEOFF;
        stateEntryTime = millis();
        sendStatusToGS(state);

        Serial.println("🛫 PREPARE_TAKEOFF → TAKEOFF será gestionado por la FSM");

        return;
    }

  } // <-- cierra if (fromGS)

  // ------------------------------------------------------------------
  // 2) MENSAJES DESDE LA RPi (EVENTOS)
  // ------------------------------------------------------------------
  if (strcmp(type, "GO") == 0) {
    analysisResult = GO;
    Serial.println("📩 [RPI] GO");
    return;
  }

  if (strcmp(type, "FIRE") == 0) {
    analysisResult = FIRE;

    float confidence = -1;
    if (doc.containsKey("confidence")) confidence = doc["confidence"];
    else if (doc.containsKey("d") && doc["d"].containsKey("confidence"))
      confidence = doc["d"]["confidence"];

    sendEventToGS("FIRE", mav_lat, mav_lon, mav_alt_rel, millis(), confidence);
    return;
  }

  if (strcmp(type, "PERSON") == 0) {
    analysisResult = PERSON;

    float confidence = -1;
    if (doc.containsKey("confidence")) confidence = doc["confidence"];
    else if (doc.containsKey("d") && doc["d"].containsKey("confidence"))
      confidence = doc["d"]["confidence"];

    sendEventToGS("PERSON", mav_lat, mav_lon, mav_alt_rel, millis(), confidence);
    return;
  }

  Serial.printf("⚠️ Tipo desconocido: %s\n", type);
}

void sendMissionLoadedToGS(int wpCount) {
    StaticJsonDocument<128> doc;
    doc["t"] = "MISSION_LOADED";
    JsonObject d = doc.createNestedObject("d");
    d["total_wp"] = wpCount;

    String p;
    serializeJson(doc, p);
    String msg = String(UAV_HDR) + p + String(SFX);

    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();

    Serial.printf("📨 MISSION_LOADED enviado (total_wp=%d)\n", wpCount);
}

// ============================================================================
// 📡 TELEMETRÍA PERIÓDICA HACIA GS
// ============================================================================
void handleTelemetry() {
    static unsigned long lastTelemetryTime = 0;

    // Enviar telemetría cada 1 segundo
    if (millis() - lastTelemetryTime >= 1000) {

        if (!simulationMode && mav_has_fix) {
            // Datos reales desde Pixhawk
            double lat      = mav_lat;
            double lon      = mav_lon;
            double alt      = mav_alt_rel;
            double speed_km = mav_ground_speed * 3.6;
            double heading  = mav_heading_deg;
            unsigned long ts = millis();

            sendTelemetry(lat, lon, alt, speed_km, heading, ts);
        }
        else if (simulationMode) {
            // Modo simulación → usar valores filtrados
            double lat = lat_f;
            double lon = lon_f;
            double alt = alt_f;
            unsigned long ts = millis();

            sendTelemetry(lat, lon, alt, 5.0, 0.0, ts);
        }

        lastTelemetryTime = millis();
    }
}

// ============================================================================
// 🔄 UART1 – Comunicación con RPi (JSON plano)
// ============================================================================
void handleSerialRPI() {
  while (SerialRPI.available() > 0) {
    char c = (char)SerialRPI.read();

    if (!receivingJson) {
      if (c == '{') {
        receivingJson = true;
        rpiIndex = 0;
        rpiBuffer[rpiIndex++] = c;
      }
      continue;
    }

    if (receivingJson) {
      if (rpiIndex < RPI_BUFFER_SIZE - 1) {
        rpiBuffer[rpiIndex++] = c;
      } else {
        Serial.println("[RPI] ⚠️ Overflow de buffer");
        receivingJson = false;
        rpiIndex = 0;
        memset(rpiBuffer, 0, sizeof(rpiBuffer));
        continue;
      }

      if (c == '}') {
        rpiBuffer[rpiIndex] = '\0';
        receivingJson = false;

        StaticJsonDocument<256> doc;
        DeserializationError err = deserializeJson(doc, rpiBuffer);

        if (!err) {
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

// ============================================================================
// 📤 STABLE → RPi (cuando se llega a un waypoint)
// ============================================================================
void sendStableToRPi(const Coordinate &pos) {
  StaticJsonDocument<128> doc;
  doc["t"]   = "STABLE";
  doc["lat"] = mav_lat;
  doc["lon"] = mav_lon;
  doc["alt"] = mav_alt_rel;
  doc["ts"]  = millis();

  String payload;
  serializeJson(doc, payload);
  SerialRPI.println(payload);

  //Serial.println("📤 [RPi] " + payload);
}

// ============================================================================
// 📡 LoRa handler – RX frames GS#...#END
// ============================================================================
void handleLoRa() {
  int size = LoRa.parsePacket();
  if (size) {
    while (LoRa.available()) {
      loraRxBuf += (char)LoRa.read();
    }
  }

  String js;
  while (extractNextFrame(loraRxBuf, js, GS_HDR)) {
    processIncomingJSON(js, true);
  }
}

// ============================================================================
// 📡 MAVLINK – HEARTBEAT / ARM / MODE / TAKEOFF / GOTO / RX / Failsafe battery
// ============================================================================
void sendHeartbeatToPixhawk() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(
      42,                      // system_id (ESP32)
      200,                     // component_id
      &msg,
      MAV_TYPE_ONBOARD_CONTROLLER,
      MAV_AUTOPILOT_INVALID,
      0,
      0,
      MAV_STATE_ACTIVE);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void pixhawkArm(bool arm) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(
      42, 200,
      &msg,
      1, 0,
      MAV_CMD_COMPONENT_ARM_DISARM,
      0,
      arm ? 1.0f : 0.0f,
      0,0,0,0,0,0);

  SerialMAV.write(buf, mavlink_msg_to_send_buffer(buf, &msg));
}

void pixhawkDisarmForce() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(
        42, 200,       // ESP32 sysid/com pid
        &msg,
        1, 1,          // target Pixhawk
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0.0f,          // param1 = 0 → DISARM
        21196,         // param2 = 21196 → FORCE DISARM
        0,0,0,0,0
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);

    Serial.println("🛑 MAV → FORCE DISARM enviado");
}


void setModeGuided() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_set_mode_pack(
        42, 200,                      // sysid, compid ESP32
        &msg,
        1,                            // target system = Pixhawk
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4                             // GUIDED
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);

    Serial.println("📡 Modo GUIDED solicitado");
}


void pixhawkTakeoff(float alt) {
    // Seguridad: verificar modo y armado
    if (!mav_armed) {
        Serial.println("❌ TAKEOFF cancelado: el dron NO está armado.");
        return;
    }

    if (!testMode) {
        Serial.println("⚠️ TAKEOFF: forzando modo GUIDED...");
        setModeGuided();
        delay(150);  // pequeña espera para que el Pixhawk procese el cambio
    }

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(
        42, 200,                     // sysid, compid del ESP
        &msg,
        1, 0,                        // target Pixhawk
        MAV_CMD_NAV_TAKEOFF,         // comando TAKEOFF
        0,                           // confirmation

        /* PARAMS */
        0, 0, 0, 0,                  // params 1–4 sin uso
        NAN,                         // lat → NAN = usar posición actual
        NAN,                         // lon → NAN = usar posición actual
        alt                          // altitud relativa de despegue
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);

    Serial.printf("🚁 TAKEOFF cmd enviado → %.1f m\n", alt);
}


void sendMavGoto(double lat, double lon, float alt) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint16_t type_mask =
        (1 << 3) | (1 << 4) | (1 << 5) |   // ignorar velocidades
        (1 << 6) | (1 << 7) | (1 << 8) |   // ignorar aceleraciones
        (1 << 9) | (1 << 10);              // ignorar yaw/yaw_rate

    mavlink_msg_set_position_target_global_int_pack(
        42, 200,                     // sysid, compid
        &msg,
        millis(),                    // tiempo
        1, 1,                        // target system, component
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        type_mask,
        (int32_t)(lat * 1e7),
        (int32_t)(lon * 1e7),
        alt,                         // altitud relativa
        0, 0, 0,                     // vx,vy,vz ignorados
        0, 0, 0,                     // ax,ay,az ignorados
        0, 0                         // yaw / yaw rate ignorados
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);

    Serial.printf("🧭 GOTO → %.7f , %.7f  alt=%.1f\n", lat, lon, alt);
}

void pixhawkLand() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(
        42, 200,      // sysid, compid ESP32
        &msg,
        1, 0,         // target: Pixhawk
        MAV_CMD_NAV_LAND,
        0,            // confirmation
        0, 0, 0, 0,   // params 1–4 sin usar
        mission.home.lat,   // opcional
        mission.home.lon,   // opcional
        0                  // altitud objetivo = 0
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);

    Serial.println("⬇️ Enviando comando LAND al Pixhawk...");
}

void pixhawkLandHere() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Construcción del comando LAND vertical
    mavlink_msg_command_long_pack(
        42, 200,          // sysid, compid del ESP32
        &msg,
        1, 0,             // target system/component → Pixhawk
        MAV_CMD_NAV_LAND, // comando LAND
        0,                // confirmation
        0, 0, 0, 0,       // params 1–4 sin usar
        NAN,              // param5 → lat  (NAN = usar posición actual)
        NAN,              // param6 → lon  (NAN = usar posición actual)
        0                 // param7 → alt final (0 = aterrizar)
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);

    Serial.println("⬇️ FAILSAFE: LAND HERE (posición actual) enviado al Pixhawk");
}

void triggerBatteryFailsafe() {
    if (battery_failsafe_active) return;

    battery_failsafe_active = true;
    Serial.println("⚠️ FAILSAFE BATERÍA BAJA ACTIVADO");

    // Forzar estado LAND en la FSM
    state = LAND;

    // Comando MAVLink que realmente inicia el aterrizaje
    pixhawkLandHere();
}

void readMavlink() {
  mavlink_message_t msg;
  mavlink_status_t status;
   int count = 0;


  while (SerialMAV.available()&& count < 25) {
    uint8_t c = SerialMAV.read();
    count++;
    // Parseamos byte a byte
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      // DEBUG (desactivable)
      //Serial.print("MSG ID: ");
      //Serial.println(msg.msgid);

      switch (msg.msgid) {

        // ================================================================
        // 🫀 HEARTBEAT (MSG 0)
        // ================================================================
        case MAVLINK_MSG_ID_HEARTBEAT: {
          mavlink_heartbeat_t hb;
          mavlink_msg_heartbeat_decode(&msg, &hb);

          mav_type           = hb.type;
          mav_autopilot      = hb.autopilot;
          mav_base_mode      = hb.base_mode;
          mav_custom_mode    = hb.custom_mode;
          mav_system_status  = hb.system_status;

          mav_armed = (mav_base_mode & MAV_MODE_FLAG_SAFETY_ARMED);

          // debug
          //Serial.print("ARMED = ");
          //Serial.println(mav_armed);
          mav_last_update_ms = millis();
          break;
        }

        // ================================================================
        // 🛰️ GPS + Altitud relativa + Velocidad (MSG 33)
        // ================================================================
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
          mavlink_global_position_int_t pos;
          mavlink_msg_global_position_int_decode(&msg, &pos);

          double raw_lat = pos.lat * 1e-7;
          double raw_lon = pos.lon * 1e-7;
          double raw_alt = pos.relative_alt / 1000.0; // m
          

          // Filtrado GPS
          filterGPS(raw_lat, raw_lon, raw_alt,
                    lat_f, lon_f, alt_f);

          mav_lat     = lat_f;
          mav_lon     = lon_f;
          mav_alt_rel = alt_f;

          mav_has_fix = true;
          mav_last_update_ms = millis();
          break;
        }
        case MAVLINK_MSG_ID_STATUSTEXT: {
          mavlink_statustext_t st;
          mavlink_msg_statustext_decode(&msg, &st);

          char buf[51];
          memcpy(buf, st.text, 50);
          buf[50] = 0;

          //Serial.print("[STATUSTEXT] ");
          //Serial.println(buf);
          break;
        }

        // ================================================================
        // 🧭 Velocidad, rumbo, altitud baro (MSG 74)
        // ================================================================
        case MAVLINK_MSG_ID_VFR_HUD: {
          mavlink_vfr_hud_t hud;
          mavlink_msg_vfr_hud_decode(&msg, &hud);

          mav_ground_speed = hud.groundspeed;   // m/s
          mav_heading_deg  = hud.heading;        // °
          mav_alt_abs      = hud.alt;            // m
          mav_climb_rate   = hud.climb;          // m/s

          break;
        }

        // ================================================================
        // 🔋 Voltaje / CPU / carga (MSG 1)
        // ================================================================
        case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t sys;
        mavlink_msg_sys_status_decode(&msg, &sys);

        Serial.printf(
        "[SYS_STATUS RAW] volt_raw=%u mV  (%.5f V)  batt_remaining=%d %%\n",
        sys.voltage_battery,
        sys.voltage_battery / 1000.0f,
        sys.battery_remaining
        );

        // Voltaje bruto del mensaje (mV)
        if (sys.voltage_battery == UINT16_MAX || sys.voltage_battery == 0) {
            mav_batt_voltage = -1;   // sin información válida
        } else {
            float vb = sys.voltage_battery / 1000.0f;

            // Filtro EMA para evitar ruido (estable y suave)
            static float vb_filtered = vb;
            vb_filtered = vb_filtered * 0.92f + vb * 0.08f;

            mav_batt_voltage = vb_filtered;
        }

        // Corriente
        mav_batt_current = (sys.current_battery == -1) ? -1 : sys.current_battery / 100.0f;

        // % proporcionado por Pixhawk
        mav_batt_remaining = (sys.battery_remaining < 0) ? -1 : sys.battery_remaining;

        break;
        }


      } // end switch

    } // end parse
  }   // end while
}


// ============================================================================
// ✈ NAVEGACIÓN HACIA UN WAYPOINT (envía GOTO Pixhawk)
// ============================================================================
void navigateTo(const Coordinate& target) {
  double dist    = haversineDistance(mav_lat, mav_lon, target.lat, target.lon);
  double bearing = computeBearing(mav_lat, mav_lon, target.lat, target.lon);

  //Serial.printf("🧭 NAV → bearing=%.1f°, dist=%.1f m → (%.6f, %.6f)\n",
                //bearing, dist, target.lat, target.lon);

      if (millis() - lastGotoMs > 500) {
        sendMavGoto(target.lat, target.lon, mission.altitude);
        lastGotoMs = millis();}

}

// ============================================================================
// 📤 NUEVO: Enviar Waypoint Actual a la GS (para dibujarlo en el mapa)
// ============================================================================
void sendActiveWaypointToGS(int wp, const Coordinate& pt) {
    StaticJsonDocument<128> doc;
    doc["t"]  = "WP_UPDATE";
    doc["wp"] = wp;
    doc["lat"] = pt.lat;
    doc["lon"] = pt.lon;
    doc["ts"]  = millis();

    String payload;
    serializeJson(doc, payload);
    String frame = String(UAV_HDR) + payload + String(SFX);

    LoRa.beginPacket();
    LoRa.print(frame);
    LoRa.endPacket();

    Serial.printf("📤 [GS] WP_UPDATE → wp=%d (%.6f, %.6f)\n", wp, pt.lat, pt.lon);
}


void notifyWaypointReached(int wp) {
  StaticJsonDocument<128> doc;
  doc["t"] = "WAYPOINT_REACHED";
  doc["wp"] = wp;
  doc["ts"] = millis();
  

  String payload;
  serializeJson(doc, payload);
  String frame = String(UAV_HDR) + payload + String(SFX);

  LoRa.beginPacket();
  LoRa.print(frame);
  LoRa.endPacket();

  //Serial.printf("📤 [GS] WAYPOINT_REACHED wp=%d\n", wp);
}

void triggerServoAction() {
  actionServo.write(SERVO_OPEN);
  servoActive = true;
  servoOpenStartMs = millis();
  Serial.println("🟢 SERVO → ABIERTO (inicio)");
}
void handleServoAction() {
  if (!servoActive) return;

  // Si el tiempo pasó → cerrar servo
  if (millis() - servoOpenStartMs >= SERVO_OPEN_TIME) {
    actionServo.write(SERVO_CLOSED);
    servoActive = false;
    Serial.println("🔴 SERVO → CERRADO (fin)");
  }
}

// ============================================================================
// 🚁 STATE MACHINE COMPLETA 
// ============================================================================
void updateStateMachine() {

    static DroneState lastState = IDLE;

    bool inFlight =
        (state == TAKEOFF) ||
        (state == NAVIGATE) ||
        (state == STABILIZE) ||
        (state == WAIT_ANALYSIS) ||
        (state == RETURN_HOME);

    static unsigned long lastFailsafeLog = 0;

    // =========================================================================
    // 🐛 DEBUG: Monitoreo de comandos entrantes
    // =========================================================================
    static unsigned long lastCmdCheck = 0;
    if (millis() - lastCmdCheck > 3000) {
        Serial.println("╔═══════════════════════════════════════╗");
        Serial.printf("║ Estado: %-30s║\n", fsmStateToString(state).c_str());
        Serial.printf("║ Armed: %-31s║\n", mav_armed ? "TRUE" : "FALSE");
        Serial.printf("║ GPS Fix: %-29s║\n", mav_has_fix ? "TRUE" : "FALSE");
        Serial.printf("║ Test Mode: %-27s║\n", testMode ? "TRUE" : "FALSE");
        Serial.printf("║ loraReturnCmd: %-23s║\n", loraReturnCommand ? "PENDING" : "false");
        Serial.printf("║ loraDisarmCmd: %-23s║\n", loraDisarmCommand ? "PENDING" : "false");
        Serial.println("╚═══════════════════════════════════════╝");
        lastCmdCheck = millis();
    }

    // =========================================================================
    // ✅ FIX 1: PRIORIDAD MÁXIMA - Comandos críticos ANTES del switch
    // =========================================================================
    
    // DISARM tiene prioridad absoluta
    if (loraDisarmCommand) {
        Serial.println("🛑 ════════════════════════════════════");
        Serial.println("   DISARM GLOBAL RECIBIDO");
        Serial.println("   Desarmando y reseteando...");
        Serial.println("════════════════════════════════════");
        loraDisarmCommand = false;
        if (!testMode) pixhawkArm(false);
        resetMissionState();
        return;
    }

    // RETURN tiene segunda prioridad (solo si está en vuelo)
    if (loraReturnCommand && inFlight) {
        Serial.println("↩️ ════════════════════════════════════");
        Serial.println("   RETURN HOME RECIBIDO");
        Serial.printf("   Desde estado: %s\n", fsmStateToString(state).c_str());
        Serial.println("════════════════════════════════════");
        loraReturnCommand = false;
        state = RETURN_HOME;
        sendStatusToGS(state);
        // NO hacemos return aquí para que siga ejecutando RETURN_HOME
    }

    // =========================================================================
    // FAILSAFE por BATERÍA BAJA (PRIORIDAD después de DISARM y RETURN, antes de MAVLink Lost)
    // =========================================================================
    if (!testMode && inFlight && !battery_failsafe_active) {

        if (mav_batt_voltage > 0 && mav_batt_voltage < LOW_VOLTAGE_THRESHOLD) {

            Serial.println("🔋❌ FAILSAFE CRÍTICO: BATERÍA BAJA → LAND HERE");

            battery_failsafe_active = true;

            // Cancelar cualquier RETURN pendiente
            loraReturnCommand = false;

            // Forzar estado LAND
            state = LAND;
            sendStatusToGS(state);

            // Comando MAVLink: aterrizar verticalmente donde está
            pixhawkLandHere();

            // No permitir que ningún otro failsafe o estado siga ejecutando
            return;
        }
    }
    // Si estamos en FAILSAFE de batería, se bloquea MAVLink Lost y todo lo demás
    if (battery_failsafe_active) {
        // El dron está en LAND HERE y no debe recibir ninguna interrupción
        return;
    }

    // =========================================================================
    // FAILSAFE por pérdida de MAVLink
    // =========================================================================
    if (!testMode && inFlight && (millis() - mav_last_update_ms > 3000)) {
        if (state != RETURN_HOME) {
            state = RETURN_HOME;
            sendStatusToGS(state);
        }
        if (millis() - lastFailsafeLog > 5000) {
            Serial.println("❌ FAILSAFE: MAVLink perdido → RETURN_HOME");
            lastFailsafeLog = millis();
        }
    }

    // =========================================================================
    // ✅ FIX 2: RESET de timers al cambiar de estado (ANTES del switch)
    // =========================================================================
    if (state != lastState) {
        Serial.printf("🔄 Cambio de estado: %s → %s\n", 
                     fsmStateToString(lastState).c_str(), 
                     fsmStateToString(state).c_str());
        
        // Resetear timers críticos
        stateEntryTime = millis();
        insideRadiusSince = millis();
        
        // Si entramos a WAIT_ANALYSIS, inicializar su timer
        if (state == WAIT_ANALYSIS) {
            analysisStartTime = millis();
            Serial.printf("⏱️ analysisStartTime = %lu\n", analysisStartTime);
        }
        
        lastState = state;
    }

    // =========================================================================
    // MAIN SWITCH
    // =========================================================================
    switch (state) {

    // -------------------------------------------------------------------------
    case IDLE:
        sendStatusToGS(state);
        break;

    // -------------------------------------------------------------------------
    case ARMING: {
        // Fast confirm
        if (awaitingFastArmConfirm && millis() < armingConfirmationDeadline) {
            if (mav_armed) {
                Serial.println("✔ ARM confirmado (fast)");
                awaitingFastArmConfirm = false;
                state = READY_FOR_MISSION;
                sendStatusToGS(state);
                break;
            }
        }

        // Armó correctamente
        if (mav_armed) {
            Serial.println("✔ ARMADO OK");
            state = READY_FOR_MISSION;
            sendStatusToGS(state);
            break;
        }

        // Timeout de armado
        if (millis() - stateEntryTime > 8000) {
            Serial.println("❌ ERROR: Pixhawk NO ARMÓ");
            if (!testMode) pixhawkArm(false);
            state = IDLE;
            sendStatusToGS(state);
            break;
        }

        break;
    }

    // -------------------------------------------------------------------------
    case READY_FOR_MISSION:
        if (!testMode && (!mav_has_fix || mav_alt_rel == 0 || mav_lat == 0)) {
            Serial.println("❌ No GPS FIX, esperando...");
            return;
        }
        break;

    // -------------------------------------------------------------------------
    case PREPARE_TAKEOFF:
        Serial.println("🛫 PREP → TAKEOFF");
        if (!testMode) pixhawkTakeoff(mission.altitude);
        state = TAKEOFF;
        sendStatusToGS(state);
        break;

    // -------------------------------------------------------------------------
    case TAKEOFF:
        // Test mode
        if (testMode) {
            if (millis() - stateEntryTime > 1000) {
                Serial.println("🛫 [TEST] TAKEOFF → NAVIGATE");
                sendActiveWaypointToGS(0, pathPoints[0]);
                state = NAVIGATE;
                sendStatusToGS(state);
            }
            break;
        }

        // Modo real
        if (mav_alt_rel >= mission.altitude * 0.90) {
            Serial.println("🛫 Altitud lograda → NAVIGATE");
            state = NAVIGATE;
            sendStatusToGS(state);
        }
        break;

    // -------------------------------------------------------------------------
    case NAVIGATE: {
        
        // ✅ FIX 3: Comandos ya procesados arriba, pero confirmamos
        if (loraReturnCommand) {
            Serial.println("↩️ RETURN desde NAVIGATE");
            loraReturnCommand = false;
            state = RETURN_HOME;
            sendStatusToGS(state);
            break;
        }

        // Fin de waypoints
        if (currentWaypoint >= (int)pathPoints.size()) {
            Serial.println("🏁 Fin ruta → RETURN_HOME");
            state = RETURN_HOME;
            sendStatusToGS(state);
            break;
        }

        // Ignorar WP0 si el dron está a menos de 10 m de HOME
        if (currentWaypoint == 0) {
            double hd = haversineDistance(
                mav_lat, mav_lon,
                mission.home.lat, mission.home.lon
            );
            if (hd < 12) {
                Serial.println("⏭ Saltando WP0 (HOME demasiado cerca)");
                currentWaypoint++;
                sendActiveWaypointToGS(currentWaypoint, pathPoints[currentWaypoint]);
                break;
            }
        }

        // Cálculo de distancia
        Coordinate target = pathPoints[currentWaypoint];
        double dist = haversineDistance(mav_lat, mav_lon, target.lat, target.lon);
        bool close_enough = (dist < 7.0);
        bool stable_speed = (mav_ground_speed < 2.0);

        // 🐛 DEBUG: Info cada 2 segundos
        static unsigned long lastNavDebug = 0;
        if (millis() - lastNavDebug > 2000) {
           // Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
           // Serial.printf("🧭 NAVIGATE → WP %d/%d\n", currentWaypoint, (int)pathPoints.size() - 1);
           // Serial.printf("   Target: %.6f, %.6f\n", target.lat, target.lon);
            //Serial.printf("   Actual: %.6f, %.6f\n", mav_lat, mav_lon);
            //Serial.printf("   Dist: %.1f m (close=%d)\n", dist, close_enough);
           // Serial.printf("   Speed: %.1f m/s (stable=%d)\n", mav_ground_speed, stable_speed);
            //Serial.printf("   Inside radius: %lu ms\n", millis() - insideRadiusSince);
            //Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            lastNavDebug = millis();
        }
        // Activar análisis solo después de WP0
        bool allowAnalysis = (currentWaypoint > 0);


        // Detección estable
        if (close_enough && stable_speed && allowAnalysis) {
            unsigned long timeInside = millis() - insideRadiusSince;
            
            if (timeInside > 1500) {
                Serial.printf("✔ WP %d ALCANZADO (dist=%.1fm, time=%lums)\n", 
                             currentWaypoint, dist, timeInside);
                notifyWaypointReached(currentWaypoint);
                
                state = STABILIZE;
                analysisResult = NONE;
                sendStableToRPi(target);
                sendStatusToGS(state);
                break;
            } else {
                // 🐛 Muestra progreso cada 500ms
                static unsigned long lastProgressPrint = 0;
                if (millis() - lastProgressPrint > 500) {
                   // Serial.printf("⏳ Dentro del radio: %lu/1500 ms\n", timeInside);
                    lastProgressPrint = millis();
                }
            }
        } else {
            // Se salió del radio
            insideRadiusSince = millis();
        }

        // Enviar GOTO
        if (millis() - lastGotoMs > 500) {
            sendMavGoto(target.lat, target.lon, mission.altitude);
            lastGotoMs = millis();
        }

        // Failsafe desarmado
        if (!testMode && !mav_armed) {
            Serial.println("⚠ Se desarmó en vuelo → IDLE");
            state = IDLE;
            sendStatusToGS(state);
        }

        break;
    }

    // -------------------------------------------------------------------------
    case STABILIZE:
        if (millis() - stateEntryTime > 800) {
            Serial.println("📷 Estabilizado → WAIT_ANALYSIS");
            state = WAIT_ANALYSIS;
            sendStatusToGS(state);
        }
        break;

    // -------------------------------------------------------------------------
    case WAIT_ANALYSIS: {

        bool hasMore = (currentWaypoint + 1 < (int)pathPoints.size());
        unsigned long elapsed = millis() - analysisStartTime;

        // ============================================================
        // 1) PRIORIDADES MÁXIMAS
        // ============================================================

        // DISARM siempre primero
        if (loraDisarmCommand) {
            loraDisarmCommand = false;
            if (!testMode) pixhawkArm(false);
            resetMissionState();
            break;
        }

        // RETURN siempre segundo
        if (loraReturnCommand) {
            loraReturnCommand = false;
            state = RETURN_HOME;
            sendStatusToGS(state);
            break;
        }

        // ============================================================
        // 2) RESULTADOS DESDE LA RPi
        // ============================================================

        if (analysisResult != NONE) {

            if (analysisResult == GO) {
                currentWaypoint++;

                if (!hasMore) {
                    state = RETURN_HOME;
                } else {
                    state = NAVIGATE;
                    sendActiveWaypointToGS(currentWaypoint, pathPoints[currentWaypoint]);
                }
                sendStatusToGS(state);
            }

            else if (analysisResult == FIRE || analysisResult == PERSON) {

                if (analysisResult == PERSON) triggerServoAction();

                if (!hasMore || mission.event_action.equalsIgnoreCase("RETURN")) {
                    state = RETURN_HOME;
                } else {
                    currentWaypoint++;
                    state = NAVIGATE;
                    sendActiveWaypointToGS(currentWaypoint, pathPoints[currentWaypoint]);
                }
                sendStatusToGS(state);
            }

            analysisResult = NONE;
            break;
        }

        // ============================================================
        // 3) EARLY AUTO-CONTINUE (para no frenar el dron innecesariamente)
        // ============================================================

        if (elapsed > 2000 && hasMore) {   // << early continue
            currentWaypoint++;
            state = NAVIGATE;
            sendActiveWaypointToGS(currentWaypoint, pathPoints[currentWaypoint]);
            sendStatusToGS(state);
            break;
        }

        // ============================================================
        // 4) TIMEOUT PRINCIPAL (seguridad)
        // ============================================================

        if (elapsed > ANALYSIS_TIMEOUT) {  // 6000 ms
            currentWaypoint++;

            if (!hasMore) state = RETURN_HOME;
            else {
                state = NAVIGATE;
                sendActiveWaypointToGS(currentWaypoint, pathPoints[currentWaypoint]);
            }

            sendStatusToGS(state);
            break;
        }

        // ============================================================
        // 5) Si no hay nada que hacer → esperar
        // ============================================================

        break;
    }

    // -------------------------------------------------------------------------
    case RETURN_HOME: {
        
        double distHome = haversineDistance(
            mav_lat, mav_lon,
            mission.home.lat, mission.home.lon
        );

        // 🐛 DEBUG: Status cada 2s
        static unsigned long lastReturnDebug = 0;
        if (millis() - lastReturnDebug > 2000) {
            //Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            //Serial.println("🏠 RETURN_HOME");
            //Serial.printf("   HOME: %.6f, %.6f\n", mission.home.lat, mission.home.lon);
            //Serial.printf("   Actual: %.6f, %.6f\n", mav_lat, mav_lon);
            //Serial.printf("   Distancia: %.1f m\n", distHome);
           // Serial.printf("   Speed: %.1f m/s\n", mav_ground_speed);
            //Serial.printf("   Alt: %.1f m\n", mav_alt_rel);
          //  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            lastReturnDebug = millis();
        }

        // GOTO continuo
        if (millis() - lastGotoMs > 500) {
            sendMavGoto(mission.home.lat, mission.home.lon, mission.altitude);
            lastGotoMs = millis();
        }

        bool close_home = (distHome < 3.5);
        bool slow_home  = (mav_ground_speed < 2.0);

        if (close_home && slow_home) {
            Serial.println("🏠 HOME alcanzado → LAND");
            Serial.printf("   Final dist: %.1fm, speed: %.1fm/s\n", distHome, mav_ground_speed);
            if (!testMode) pixhawkLand();
            state = LAND;
            sendStatusToGS(state);
        }

        break;
    }

    // -------------------------------------------------------------------------
    case LAND:
        if (mav_alt_rel < 0.3 && mav_ground_speed < 0.2) {
            Serial.println("🟢 Aterrizaje completo");
            state = COMPLETE;
            sendStatusToGS(state);
        }
        break;

    // -------------------------------------------------------------------------
    case COMPLETE:
        //Serial.println("🎉 COMPLETE → reset");
        sendStatusToGS(state);
        resetMissionState();
        break;
    }
}

void resetMissionState() {
  currentWaypoint     = 0;
  pathPoints.clear();
  mission.loaded      = false;
  analysisResult      = NONE;
  loraReturnCommand   = false;
  loraDisarmCommand   = false;
  state               = IDLE;
  battery_failsafe_active = false;

  statusDirty = true;          // forzamos reenvío
  sendStatusToGS(state);       // avisamos a la GS que volvimos a IDLE

  Serial.println("🔄 Estado reiniciado (IDLE)");
}


void requestMavlinkStreams() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len;

    // ============================================================
    // 🛰 STREAM DE POSICIÓN (GLOBAL_POSITION_INT – MSG 33)
    // ============================================================
    mavlink_msg_request_data_stream_pack(
        42, 199,           // sysid, compid (ESP32)
        &msg,
        1, 1,              // target system/component (Pixhawk)
        MAV_DATA_STREAM_POSITION,
        5,                 // 5 Hz
        1                  // start
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);

    // ============================================================
    // 🛰 STREAM EXTRA2 (VFR_HUD – MSG 74)
    // ============================================================
    mavlink_msg_request_data_stream_pack(
        42, 199,
        &msg,
        1, 1,
        MAV_DATA_STREAM_EXTRA2,
        10,                // 10 Hz
        1
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);

    // ============================================================
    // 🔋 *** STREAM EXTENDED_STATUS (SYS_STATUS – MSG 1) ***
    // ============================================================
    // ESTE stream contiene:
    //   - voltage_battery (mV)
    //   - current_battery (cA)
    //   - battery_remaining (%)
    //   - load, drop rate, etc
    // SIN esto → ¡NO HAY VOLTAJE NI %!
    mavlink_msg_request_data_stream_pack(
        42, 199,
        &msg,
        1, 1,
        MAV_DATA_STREAM_EXTENDED_STATUS,   // << EL QUE TE FALTABA
        2,                                 // 2 Hz suficiente
        1
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);

    // Opcional: mensaje de debug
    Serial.println("📡 Solicitud de streams MAVLink enviada (POS, EXTRA2, EXTENDED_STATUS)");
}


void sendWaypointsDebugToGS() {
    if (pathPoints.empty()) {
        Serial.println("⚠️ No hay waypoints para enviar a GS");
        return;
    }

    // 1) Serializar todos los waypoints en un string plano
    String flat = "";
    for (size_t i = 0; i < pathPoints.size(); i++) {
        flat += String(pathPoints[i].lat, 7);
        flat += ",";
        flat += String(pathPoints[i].lon, 7);
        flat += ";";   // separador de puntos
    }

    const int CHUNK_SIZE = 120; // tamaño seguro de fragmento para LoRa
    int totalLen = flat.length();
    int totalChunks = (totalLen + CHUNK_SIZE - 1) / CHUNK_SIZE;

    Serial.printf("🧪 Enviando %d waypoints en %d fragmentos\n",
                  (int)pathPoints.size(), totalChunks);

    for (int idx = 0; idx < totalChunks; idx++) {
        int start = idx * CHUNK_SIZE;
        int end   = min(start + CHUNK_SIZE, totalLen);
        String part = flat.substring(start, end);

        // JSON con framing UAV#/…/#END lo hace sendWithAck
        StaticJsonDocument<256> doc;
        doc["t"] = "WP_CHUNK";
        JsonObject d = doc.createNestedObject("d");
        d["i"] = idx;          // índice de fragmento
        d["n"] = totalChunks;  // total de fragmentos
        d["chunk"] = part;     // datos parciales

        String payload;
        serializeJson(doc, payload);

        String msgId = "WPCH" + String(idx);
        sendWithAck(payload, msgId);

        delay(40);  // pequeño respiro para no saturar LoRa
    }

    Serial.println("🧪 Envío de waypoints a GS completado");
}


// ============================================================================
// 🚀 SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n===============================");
  Serial.println("   DRON THERMAL SEARCH v2.0");
  Serial.println("===============================");

  // UART MAVLink
  SerialMAV.begin(115200, SERIAL_8N1, MAV_RX, MAV_TX);
  Serial.println("✅ UART2 Pixhawk (MAVLink) inicializada");

  // UART Raspberry
  SerialRPI.begin(9600, SERIAL_8N1, RPI_RX, RPI_TX);
  SerialRPI.setTimeout(RPI_TIMEOUT_MS);
  memset(rpiBuffer, 0, sizeof(rpiBuffer));
  Serial.println("✅ UART1 RPi lista");

  // LoRa
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("❌ ERROR: LoRa no inició");
    while (1) delay(1000);
  }
  Serial.println("✅ LoRa listo");

  have_filter = false;
  gpsWarmup   = 0;

  // Servo de acción
  actionServo.attach(SERVO_PIN);
  actionServo.write(SERVO_CLOSED);
  Serial.println("✅ Servo listo (cerrado)");
  delay(6000);
  requestMavlinkStreams();
  delay(1500);
  Serial.println("🚀 Sistema iniciado");
}
void simulateDroneMotion() {
    if (!simulationMode || !mission.loaded) return;

    static unsigned long lastStepTime = 0;
    if (millis() - lastStepTime < 600)
        return;
    lastStepTime = millis();

    switch (state) {

        case IDLE:
            sendStatusToGS(state);
            break;

        case TAKEOFF:
            alt_f += 0.4;
            if (alt_f >= mission.altitude) {
                alt_f = mission.altitude;
                //Serial.println("🛫 [SIM] TAKEOFF → NAVIGATE");
                state = NAVIGATE;
                stateEntryTime = millis();
                sendStatusToGS(state);
            }
            break;

        case NAVIGATE: {
            if (currentWaypoint >= (int)pathPoints.size()) {
                //Serial.println("🏁 [SIM] → RETURN_HOME");
                state = RETURN_HOME;
                sendStatusToGS(state);
                break;
            }

            Coordinate target = pathPoints[currentWaypoint];
            double dist = haversineDistance(lat_f, lon_f, target.lat, target.lon);

            lat_f += (target.lat - lat_f) * 0.25;
            lon_f += (target.lon - lon_f) * 0.25;
            alt_f  = mission.altitude;

            if (dist < 1.5) {
                //Serial.printf("✔ [SIM] WP%d → STABILIZE\n", currentWaypoint);
                notifyWaypointReached(currentWaypoint);
                sendStableToRPi(target);
                state = STABILIZE;
                stateEntryTime = millis();
                sendStatusToGS(state);
            }
        } break;

        case STABILIZE:
            if (millis() - stateEntryTime > 700) {
                state = WAIT_ANALYSIS;
                sendStatusToGS(state);
            }
            break;

        case WAIT_ANALYSIS: {

    // PRIORIDAD 1 — RETURN
    if (loraReturnCommand) {
        //Serial.println("↩️ RETURN durante análisis");
        loraReturnCommand = false;
        state = RETURN_HOME;
        sendStatusToGS(state);
        break;
    }

    // PRIORIDAD 2 — DISARM
    if (loraDisarmCommand) {
        //Serial.println("🛑 DISARM durante análisis");
        pixhawkArm(false);
        resetMissionState();
        sendStatusToGS(state);
        break;
    }

    bool hasMore = (currentWaypoint + 1 < (int)pathPoints.size());

    // -------------------------------
    // RESULTADO RPi
    // -------------------------------
    if (analysisResult != NONE) {

        if (analysisResult == GO) {
            currentWaypoint++;
            state = hasMore ? NAVIGATE : RETURN_HOME;
            stateEntryTime = millis();
            if (hasMore) sendActiveWaypointToGS(currentWaypoint, pathPoints[currentWaypoint]);
            sendStatusToGS(state);
        }

        else if (analysisResult == FIRE || analysisResult == PERSON) {

            if (analysisResult == PERSON) triggerServoAction();

            if (!hasMore || mission.event_action.equalsIgnoreCase("RETURN")) {
                state = RETURN_HOME;
            } else {
                currentWaypoint++;
                state = NAVIGATE;
                stateEntryTime = millis();
                sendActiveWaypointToGS(currentWaypoint, pathPoints[currentWaypoint]);
            }
            sendStatusToGS(state);
        }

        analysisResult = NONE;
        break;
    }


    // -------------------------------
    // 🚀 FIX: SALIDA AUTOMÁTICA SIN RPi
    // -------------------------------
    if (millis() - analysisStartTime > 2000) {   // <<---- antes del timeout grande
        Serial.println("⏳ WAIT_ANALYSIS auto-continue → NAVIGATE");
        currentWaypoint++;
        if (!hasMore) {
            state = RETURN_HOME;
        } else {
            state = NAVIGATE;
            stateEntryTime = millis();
            sendActiveWaypointToGS(currentWaypoint, pathPoints[currentWaypoint]);
        }
        sendStatusToGS(state);
        break;
    }


    // -------------------------------
    // TIMEOUT MAYOR (por seguridad)
    // -------------------------------
    if (millis() - analysisStartTime > ANALYSIS_TIMEOUT) {
        Serial.println("⌛ TIMEOUT análisis → NAVIGATE");
        currentWaypoint++;
        if (!hasMore) {
            state = RETURN_HOME;
        } else {
            state = NAVIGATE;
            stateEntryTime = millis();
            sendActiveWaypointToGS(currentWaypoint, pathPoints[currentWaypoint]);
        }
        sendStatusToGS(state);
    }

    break;
}


        case RETURN_HOME:
            lat_f += (mission.home.lat - lat_f) * 0.25;
            lon_f += (mission.home.lon - lon_f) * 0.25;
            alt_f  = max(alt_f - 0.05, 0.0);

            if (haversineDistance(lat_f, lon_f, mission.home.lat, mission.home.lon) < 1.5) {
                state = LAND;
                sendStatusToGS(state);
            }
            break;

        case LAND:
            alt_f = max(alt_f - 0.15, 0.0);
            if (alt_f <= 0.5) {
                state = COMPLETE;
                sendStatusToGS(state);
            }
            break;

        case COMPLETE:
            resetMissionState();
            sendStatusToGS(state);
            break;
    }
}

// ============================================================================
// 🔁 LOOP
// ============================================================================
void loop() {

    unsigned long now = millis();

    handleLoRa(); 

    checkPendingAcks();

    readMavlink();

    handleSerialRPI();

    updateStateMachine();

    if (now - lastHbMs > 1000) {
        sendHeartbeatToPixhawk();
        lastHbMs = now;
    }

    handleTelemetry();

    checkMissionChunkTimeout();

    simulateDroneMotion();

    handleServoAction();
}

