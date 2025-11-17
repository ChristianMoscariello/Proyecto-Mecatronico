// ============================================================================
// 📡 DRON BUSCADOR TÉRMICO – ESP32 + LoRa + Pixhawk (MAVLink Oleg Kalachev)
// Versión: integración Pixhawk, takeoff nativo, waypoints con paradas (opción A)
// ============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <math.h>
#include <vector>
#include <HardwareSerial.h>
#include <MAVLink.h>   // Librería de Oleg Kalachev
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

// ============================================================================
// ⚙️ SERVO DE ACCIÓN (detección PERSON)
// ============================================================================
Servo actionServo;

const int SERVO_PIN = 25;      // GPIO donde conectás el servo
const int SERVO_CLOSED = 0;    // grados (cerrado)
const int SERVO_OPEN   = 90;   // grados (abierto)

bool servoActive = false;
unsigned long servoOpenStartMs = 0;
const unsigned long SERVO_OPEN_TIME = 10000;  // 10 segundos

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
  String event_action;     // "RETURN", "CONTINUE", etc.
} mission;

// =======================
// 🌐 Telemetría MAVLink
// =======================
bool   mav_has_fix       = false;
double mav_lat           = 0.0;
double mav_lon           = 0.0;
double mav_alt_rel       = 0.0;    // altitud relativa (m)
double mav_ground_speed  = 0.0;    // m/s
double mav_heading_deg   = 0.0;    // deg
unsigned long mav_last_update_ms = 0;

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
  IDLE,          // Esperando misión
  TAKEOFF,       // Despegando hasta altitud
  NAVIGATE,      // Navegando a waypoint
  STABILIZE,     // Estabilizar cámaras
  WAIT_ANALYSIS, // Esperando GO/FIRE/PERSON desde RPi
  RETURN_HOME,   // Volver a HOME
  LAND,          // Aterrizar
  COMPLETE       // Fin de misión
};

DroneState state = IDLE;
unsigned long stateEntryTime   = 0;
unsigned long insideRadiusSince = 0;

// ============================================================================
// 📡 Enviar estado de la FSM a la Ground Station
// ============================================================================
String stateToString(DroneState s) {
  switch (s) {
    case IDLE:          return "IDLE";
    case TAKEOFF:       return "TAKEOFF";
    case NAVIGATE:      return "NAVIGATE";
    case STABILIZE:     return "STABILIZE";
    case WAIT_ANALYSIS: return "WAIT_ANALYSIS";
    case RETURN_HOME:   return "RETURN_HOME";
    case LAND:          return "LAND";
    case COMPLETE:      return "COMPLETE";
    default:            return "UNKNOWN";
  }
}

void sendStatusToGS(DroneState s) {
  StaticJsonDocument<128> doc;
  doc["t"] = "STATUS";
  doc["state"] = stateToString(s);
  doc["ts"] = millis();

  String payload;
  serializeJson(doc, payload);

  String frame = String(UAV_HDR) + payload + String(SFX);

  LoRa.beginPacket();
  LoRa.print(frame);
  LoRa.endPacket();

  Serial.printf("📤 [STATUS] %s\n", stateToString(s).c_str());
}


// Análisis RPi
enum AnalysisResult { NONE, GO, FIRE, PERSON };
AnalysisResult analysisResult = NONE;

// Misión / recorrido
int currentWaypoint = 0;
vector<Coordinate> pathPoints;

// Timeout análisis
const unsigned long ANALYSIS_TIMEOUT = 90000;
unsigned long analysisStartTime = 0;

// Comandos por LoRa
bool loraReturnCommand = false;
bool loraDisarmCommand = false;

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
// 📡 MAVLINK – HEARTBEAT / ARM / MODE / TAKEOFF / GOTO / RX
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

void setModeGuided() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_set_mode_pack(
      42, 200,
      &msg,
      1,
      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
      4);   // 4 → GUIDED

  SerialMAV.write(buf, mavlink_msg_to_send_buffer(buf, &msg));
}

void pixhawkTakeoff(float alt) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(
      42, 200,
      &msg,
      1, 0,
      MAV_CMD_NAV_TAKEOFF,
      0,
      0,0,0,0,
      mission.home.lat,
      mission.home.lon,
      alt);

  SerialMAV.write(buf, mavlink_msg_to_send_buffer(buf, &msg));
  Serial.printf("🚁 TAKEOFF → %.1f m\n", alt);
}

void sendGotoPixhawk(const Coordinate &target) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  uint16_t type_mask =
      (1<<3)|(1<<4)|(1<<5)|
      (1<<6)|(1<<7)|(1<<8)|
      (1<<9)|(1<<10);

  mavlink_msg_set_position_target_global_int_pack(
      42, 200,
      &msg,
      millis(),
      1, 1,
      MAV_FRAME_GLOBAL_RELATIVE_ALT,
      type_mask,
      (int32_t)(target.lat * 1e7),
      (int32_t)(target.lon * 1e7),
      mission.altitude,
      0,0,0,
      0,0,0,
      0,0);

  SerialMAV.write(buf, mavlink_msg_to_send_buffer(buf, &msg));
}

void readMavlink() {
  mavlink_message_t msg;
  mavlink_status_t  status;

  while (SerialMAV.available()) {
    uint8_t c = SerialMAV.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      switch (msg.msgid) {

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
          mavlink_global_position_int_t pos;
          mavlink_msg_global_position_int_decode(&msg, &pos);

          double raw_lat = pos.lat / 1e7;
          double raw_lon = pos.lon / 1e7;
          double raw_alt = pos.relative_alt / 1000.0;

          filterGPS(raw_lat, raw_lon, raw_alt, lat_f, lon_f, alt_f);

          mav_lat      = lat_f;
          mav_lon      = lon_f;
          mav_alt_rel  = alt_f;
          mav_has_fix  = true;
          mav_last_update_ms = millis();
          break;
        }

        case MAVLINK_MSG_ID_VFR_HUD: {
          mavlink_vfr_hud_t hud;
          mavlink_msg_vfr_hud_decode(&msg, &hud);
          mav_ground_speed = hud.groundspeed;
          mav_heading_deg  = hud.heading;
          break;
        }
      }
    }
  }
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

      Serial.print("✅ [ACK] ID="); Serial.print(ackID);
      Serial.print(" (Tipo=");      Serial.print(tipo);
      Serial.println(")");

      pendingMsgs[i].waitingAck = false;
      pendingMsgs[i].payload = "";
      pendingMsgs[i].msgID   = "";
      return;
    }
  }
  Serial.print("⚠️ [ACK desconocido] ID="); Serial.println(ackID);
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

        Serial.print("🔁 [Reintento ACK] ID=");
        Serial.print(pendingMsgs[i].msgID);
        Serial.print(" Tipo="); Serial.print(tipo);
        Serial.print(" quedan "); Serial.print(pendingMsgs[i].retries);
        Serial.println(" intentos");
      } else {
        Serial.print("❌ [ACK perdido] ID=");
        Serial.print(pendingMsgs[i].msgID);
        Serial.print(" payload="); Serial.println(pendingMsgs[i].payload);

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
  JsonObject d = doc.createNestedObject("d");
  d["id"] = id;
  doc["ts"] = millis();

  String p;
  serializeJson(doc, p);
  String msg = String(UAV_HDR) + p + String(SFX);

  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();

  Serial.println("📤 [sendAckToGS] " + msg);
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
  d["battery"] = 0;
  d["status"]  = 0;
  doc["ts"]    = ts;

  String p;
  serializeJson(doc,p);
  String msg = String(UAV_HDR) + p + String(SFX);

  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();
}

void sendEventToGS(const String &topic, double lat, double lon, double alt,
                   unsigned long ts, float confidence) {
  StaticJsonDocument<256> doc;
  doc["t"] = topic;

  JsonObject d = doc.createNestedObject("d");
  d["lat"]        = lat;
  d["lon"]        = lon;
  d["alt"]        = alt;
  d["confidence"] = confidence;

  String msgID = generateMsgID();
  doc["id"] = msgID;
  doc["ts"] = ts;

  String payload;
  serializeJson(doc, payload);
  sendWithAck(payload, msgID);

  Serial.print("📤 [EVENT] "); Serial.print(topic);
  Serial.print(" ID=");        Serial.println(msgID);
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

    // ACK desde GS
    if (strcmp(type, "ACK") == 0) {
      String id = "";
      if (doc.containsKey("d")) id = doc["d"]["id"] | "";
      if (id != "") handleAck(id);
      return;
    }

    // ARM → GUIDED + ARM
    if (strcmp(type, "ARM") == 0) {
      Serial.println("🟢 ARM recibido");
      setModeGuided();
      delay(300);
      pixhawkArm(true);

      state = IDLE;
      sendAckToGS(msgId);
      return;
    }

    // DISARM (bandera, se atiende en la FSM)
    if (strcmp(type, "DISARM") == 0) {
      Serial.println("🛑 DISARM recibido");
      loraDisarmCommand = true;
      sendAckToGS(msgId);
      return;
    }

    // RETURN (bandera)
    if (strcmp(type, "RETURN") == 0) {
      Serial.println("↩️ RETURN recibido");
      loraReturnCommand = true;
      sendAckToGS(msgId);
      return;
    }

    // SIM_ON / SIM_OFF
    if (strcmp(type, "SIM_ON") == 0) {
      simulationMode = true;
      Serial.println("🧪 SIMULACIÓN ACTIVADA");
      sendAckToGS(msgId);
      return;
    }

    if (strcmp(type, "SIM_OFF") == 0) {
      simulationMode = false;
      Serial.println("🛑 SIMULACIÓN DESACTIVADA");
      sendAckToGS(msgId);
      return;
    }

    // --------------------------------------------------------------
    // MISSION_COMPACT → ruta + takeoff nativo → FSM: TAKEOFF
    // --------------------------------------------------------------
    if (strcmp(type, "MISSION_COMPACT") == 0) {
      Serial.println("📦 RX MISSION_COMPACT");

      JsonObject d = doc["d"];
      if (d.isNull()) {
        Serial.println("❌ Campo d faltante");
        return;
      }

      mission.polygon.clear();

      JsonArray p = d["p"];
      if (!p.isNull()) {
        for (JsonArray::iterator it = p.begin(); it != p.end(); ++it) {
          JsonArray coord = (*it).as<JsonArray>();
          if (coord.size() == 2) {
            Coordinate pt;
            pt.lat = coord[0];
            pt.lon = coord[1];
            mission.polygon.push_back(pt);
          }
        }
      }

      JsonArray h = d["h"];
      if (!h.isNull() && h.size() == 2) {
        mission.home.lat = h[0];
        mission.home.lon = h[1];
      }

      mission.altitude     = d["a"] | 20.0;
      mission.spacing      = d["s"] | 10.0;
      mission.event_action = d["event_action"] | "NONE";

      mission.loaded = true;

      generateMissionPath(mission);
      sendAckToGS(msgId);

      Serial.println("🚁 TAKEOFF NATIVO → GUIDED");
      setModeGuided();
      delay(250);
      pixhawkTakeoff(mission.altitude);

      state = TAKEOFF;
      stateEntryTime = millis();
      return;
    }
  }

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

// ============================================================================
// 📡 TELEMETRÍA PERIÓDICA HACIA GS
// ============================================================================
void handleTelemetry() {
  static unsigned long lastTelemetryTime = 0;

  if (millis() - lastTelemetryTime >= 1000) {

    if (!simulationMode && mav_has_fix) {
      double lat      = mav_lat;
      double lon      = mav_lon;
      double alt      = mav_alt_rel;
      double speed_km = mav_ground_speed * 3.6;
      double heading  = mav_heading_deg;
      unsigned long ts = millis();

      sendTelemetry(lat, lon, alt, speed_km, heading, ts);
    }
    else if (simulationMode) {
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

  Serial.println("📤 [RPi] " + payload);
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
// 🧭 GENERACIÓN DE WAYPOINTS (HOME → primer vértice)
// ============================================================================
void generateMissionPath(Mission& m) {
  pathPoints.clear();
  if (m.polygon.size() < 1) return;

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
  notifyWaypointCountToGS();
  Serial.printf("🧭 %d waypoints generados (dist=%.1fm spacing=%.1f)\n",
                (int)pathPoints.size(), totalDist, m.spacing);
}

void notifyWaypointCountToGS() {
  StaticJsonDocument<128> doc;
  doc["t"] = "WAYPOINTS_INFO";
  doc["total"] = pathPoints.size();
  doc["ts"] = millis();

  String payload;
  serializeJson(doc, payload);
  String frame = String(UAV_HDR) + payload + String(SFX);

  LoRa.beginPacket();
  LoRa.print(frame);
  LoRa.endPacket();

  Serial.printf("📤 [GS] WAYPOINTS_INFO total=%d\n", pathPoints.size());
}


// ============================================================================
// ✈ NAVEGACIÓN HACIA UN WAYPOINT (envía GOTO Pixhawk)
// ============================================================================
void navigateTo(const Coordinate& target) {
  double dist    = haversineDistance(mav_lat, mav_lon, target.lat, target.lon);
  double bearing = computeBearing(mav_lat, mav_lon, target.lat, target.lon);

  Serial.printf("🧭 NAV → bearing=%.1f°, dist=%.1f m → (%.6f, %.6f)\n",
                bearing, dist, target.lat, target.lon);

  sendGotoPixhawk(target);
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

  Serial.printf("📤 [GS] WAYPOINT_REACHED wp=%d\n", wp);
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

  // Prioridad: DISARM / RETURN por LoRa
  if (loraDisarmCommand) {
    Serial.println("🛑 DISARM por LoRa → abortando misión");
    pixhawkArm(false);
    resetMissionState();
    return;
  }

  if (loraReturnCommand && state != RETURN_HOME && state != LAND) {
    Serial.println("↩️ RETURN por LoRa → HOME");
    state = RETURN_HOME;
    loraReturnCommand = false;
  }

  switch (state) {

    case IDLE:
      // Espera ARM + MISSION_COMPACT
      sendStatusToGS(state);
      break;

    case TAKEOFF: {
      if (mav_alt_rel >= mission.altitude * 0.90) {
        Serial.println("🛫 Altitud de seguridad → NAVIGATE");
        state = NAVIGATE;
        stateEntryTime = millis();
        sendStatusToGS(state);
      }
      break;
    }

    case NAVIGATE: {

      if (currentWaypoint >= (int)pathPoints.size()) {
        Serial.println("🏁 Fin de ruta → RETURN_HOME");
        state = RETURN_HOME;
        sendStatusToGS(state);
        break;
      }

      Coordinate target = pathPoints[currentWaypoint];
      double dist = haversineDistance(mav_lat, mav_lon, target.lat, target.lon);

      bool close_enough = dist < 3.0;
      bool slow_enough  = mav_ground_speed < 0.5;

      if (close_enough && slow_enough) {
        if (millis() - insideRadiusSince > 1500) {
          Serial.printf("✔ WAYPOINT %d alcanzado\n", currentWaypoint);
          notifyWaypointReached(currentWaypoint);
          state = STABILIZE;
          stateEntryTime = millis();
          analysisResult = NONE;
          sendStableToRPi(target);
          return;
        }
      } else {
        insideRadiusSince = millis();
        navigateTo(target);
      }
      break;
    }

    case STABILIZE:
      if (millis() - stateEntryTime > 350) {
        Serial.println("📷 Estabilizado → WAIT_ANALYSIS");
        analysisStartTime = millis();
        state = WAIT_ANALYSIS;
        sendStatusToGS(state);
      }
      break;

    case WAIT_ANALYSIS: {

      if (loraReturnCommand) {
        Serial.println("↩️ RETURN durante análisis → HOME");
        state = RETURN_HOME;
        loraReturnCommand = false;
        break;
      }

      if (loraDisarmCommand) {
        Serial.println("🛑 DISARM durante análisis → abortando");
        pixhawkArm(false);
        resetMissionState();
        break;
      }

      if (analysisResult != NONE) {

        if (analysisResult == GO) {
          Serial.println("➡️ GO → siguiente WP");
          currentWaypoint++;
          state = NAVIGATE;
        }
        else if (analysisResult == FIRE) {
          Serial.println("🔥 Evento FIRE detectado por RPi");

          if (mission.event_action.equalsIgnoreCase("RETURN")) {
            Serial.println("↩️ event_action=RETURN → HOME");
            state = RETURN_HOME;
          } else {
            Serial.println("➡️ event_action=CONTINUE → siguiente WP");
            currentWaypoint++;
            state = NAVIGATE;
          }
        }

        else if (analysisResult == PERSON) {
          Serial.println("🧍 PERSON detectado → activando SERVO 10s");
          triggerServoAction(); 
          if (mission.event_action.equalsIgnoreCase("RETURN")) {
            Serial.println("↩️ event_action=RETURN → HOME");
            state = RETURN_HOME;
          } else {
            Serial.println("➡️ event_action=CONTINUE → siguiente WP");
            currentWaypoint++;
            state = NAVIGATE; 
          }
        }

        analysisResult = NONE;
        break;
      }

      if (millis() - analysisStartTime > ANALYSIS_TIMEOUT) {
        Serial.println("⌛ Timeout análisis → continuar");
        currentWaypoint++;
        analysisResult = NONE;
        state = NAVIGATE;
      }
      sendStatusToGS(state);
      break;
    }

    case RETURN_HOME: {
      double distHome = haversineDistance(mav_lat, mav_lon,
                                          mission.home.lat, mission.home.lon);

      navigateTo(mission.home);

      if (distHome < 2.0 && mav_ground_speed < 0.5) {
        Serial.println("🏠 HOME alcanzado → LAND");
        state = LAND;
      }
      sendStatusToGS(state);
      break;
    }

    case LAND:
      if (mav_alt_rel < 0.8) {
        Serial.println("🟢 Aterrizaje completo → COMPLETE");
        state = COMPLETE;
      }
      sendStatusToGS(state);
      break;

    case COMPLETE:
      Serial.println("🎉 Misión finalizada");
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
  Serial.println("🔄 Estado reiniciado (IDLE)");
}

// ============================================================================
// 🧪 SIMULADOR INTERNO (solo con SIM_ON, sin Pixhawk)
// ============================================================================
void simulateDroneMotion() {
  if (!simulationMode || !mission.loaded) return;

  static unsigned long lastStepTime = 0;
  if (millis() - lastStepTime < 600) return;
  lastStepTime = millis();

  switch (state) {

    case IDLE:
      sendStatusToGS(state);
      break;

    case TAKEOFF:
      alt_f += 0.4;
      if (alt_f >= mission.altitude) {
        alt_f = mission.altitude;
        Serial.println("🛫 [SIM] Altitud alcanzada → NAVIGATE");
        state = NAVIGATE;
        stateEntryTime = millis();
        sendStatusToGS(state);
      }
      break;

    case NAVIGATE:
      if (currentWaypoint >= (int)pathPoints.size()) {
        Serial.println("🏁 [SIM] Último WP → RETURN_HOME");
        state = RETURN_HOME;
        sendStatusToGS(state);
        break;
      }

      {
        Coordinate target = pathPoints[currentWaypoint];
        double dist = haversineDistance(lat_f, lon_f, target.lat, target.lon);

        lat_f += (target.lat - lat_f) * 0.25;
        lon_f += (target.lon - lon_f) * 0.25;
        alt_f  = mission.altitude;

        Serial.printf("🧭 [SIM] NAV → WP%d dist=%.2fm\n", currentWaypoint, dist);

        if (dist < 1.5) {
          Serial.printf("✔ [SIM] WP%d alcanzado → STABILIZE\n", currentWaypoint);
          notifyWaypointReached(currentWaypoint);
          sendStableToRPi(target);
          state = STABILIZE;
          stateEntryTime = millis();
          sendStatusToGS(state);
        }
      }
      break;

    case STABILIZE:
      if (millis() - stateEntryTime > 350) {
        Serial.println("📷 [SIM] WAIT_ANALYSIS");
        analysisStartTime = millis();
        state = WAIT_ANALYSIS;
        sendStatusToGS(state);
      }
      break;

    case WAIT_ANALYSIS:

      // --- Comandos externos ---
      if (loraReturnCommand) {
        Serial.println("↩️ [SIM] RETURN");
        loraReturnCommand = false;
        state = RETURN_HOME;
        sendStatusToGS(state);
        break;
      }

      if (loraDisarmCommand) {
        Serial.println("🛑 [SIM] DISARM");
        resetMissionState();
        break;
      }

      // --- Análisis recibido desde RPi ---
      if (analysisResult == GO) {
        Serial.println("➡️ [SIM] GO → siguiente WP");
        currentWaypoint++;
        analysisResult = NONE;
        state = NAVIGATE;
        sendStatusToGS(state);
        break;
      }

      else if (analysisResult == FIRE) {
        Serial.println("🔥 [SIM] FIRE detectado");

        if (mission.event_action.equalsIgnoreCase("RETURN")) {
          Serial.println("↩️ event_action=RETURN → HOME");
          state = RETURN_HOME;
        } else {
          Serial.println("➡️ event_action=CONTINUE → siguiente WP");
          currentWaypoint++;
          state = NAVIGATE;
        }

        analysisResult = NONE;
        sendStatusToGS(state);
        break;
      }

      else if (analysisResult == PERSON) {
        Serial.println("🧍 [SIM] PERSON detectado → SERVO 10s");
        triggerServoAction();

        if (mission.event_action.equalsIgnoreCase("RETURN")) {
          Serial.println("↩️ event_action=RETURN → HOME");
          state = RETURN_HOME;
        } else {
          Serial.println("➡️ event_action=CONTINUE → siguiente WP");
          currentWaypoint++;
          state = NAVIGATE;
        }

        analysisResult = NONE;
        sendStatusToGS(state);
        break;
      }

      // --- Timeout de análisis ---
      if (millis() - analysisStartTime > ANALYSIS_TIMEOUT) {
        Serial.println("⌛ [SIM] Timeout → siguiente WP");
        currentWaypoint++;
        analysisResult = NONE;
        state = NAVIGATE;
        sendStatusToGS(state);
      }

      break;

    case RETURN_HOME:
      lat_f += (mission.home.lat - lat_f) * 0.25;
      lon_f += (mission.home.lon - lon_f) * 0.25;
      alt_f  = max(alt_f - 0.05, 0.0);

      {
        double distHome = haversineDistance(lat_f, lon_f, mission.home.lat, mission.home.lon);
        Serial.printf("🏠 [SIM] RETURN_HOME dist=%.2fm\n", distHome);
        if (distHome < 1.5) {
          Serial.println("🛬 [SIM] HOME → LAND");
          state = LAND;
          sendStatusToGS(state);
        }
      }
      break;

    case LAND:
      alt_f = max(alt_f - 0.15, 0.0);
      Serial.printf("⬇️ [SIM] Landing alt=%.2f\n", alt_f);
      if (alt_f <= 0.5) {
        Serial.println("🟢 [SIM] Aterrizaje completo");
        state = COMPLETE;
        sendStatusToGS(state);
      }
      break;

    case COMPLETE:
      Serial.println("🎉 [SIM] Fin misión");
      resetMissionState();
      sendStatusToGS(state);
      break;
  }
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
  SerialMAV.begin(57600, SERIAL_8N1, MAV_RX, MAV_TX);
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

  delay(1500);
  Serial.println("🚀 Sistema iniciado");
}

// ============================================================================
// 🔁 LOOP
// ============================================================================
void loop() {

  // 1) Leer MAVLink desde Pixhawk
  readMavlink();

  // 2) Heartbeat a Pixhawk
  if (millis() - lastHeartbeatMs >= 1000) {
    sendHeartbeatToPixhawk();
    lastHeartbeatMs = millis();
  }

  // 3) Telemetría hacia GS
  handleTelemetry();

  // 4) RPi → eventos JSON
  handleSerialRPI();

  // 5) GS → dron (LoRa)
  handleLoRa();

  // 6) ACKs pendientes
  checkPendingAcks();

  // 7) FSM vuelo
  updateStateMachine();

  // 8) Simulador (si SIM_ON)
  simulateDroneMotion();

  handleServoAction();

}
