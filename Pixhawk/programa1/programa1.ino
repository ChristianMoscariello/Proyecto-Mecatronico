
// ============================================================================
// 📡 DRON BUSCADOR TÉRMICO – TELEMETRÍA LoRa + GPS + BMP280
// Organización modular y comentarios por bloques funcionales
// ============================================================================

#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <math.h>
extern "C" {
  #include <mavlink.h>
}
// ============================================================================
// 🛰️ CONFIGURACIÓN DE HARDWARE
// ============================================================================

// --- LoRa ---
#define LORA_CS 5
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
char rpiBuffer[RPI_BUFFER_SIZE];
size_t rpiIndex = 0;
bool receivingJson = false;

// --- Pixhawk MAVLink (antes era el GPS externo) ---
#define MAV_RX 16        // conectar a TX de TELEM2
#define MAV_TX 17        // conectar a RX de TELEM2
HardwareSerial SerialMAV(2);
// YA NO usamos TinyGPS++ ni gpsPrefs: todo viene de Pixhawk


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

// =======================
// 🌐 Telemetría MAVLink
// =======================
bool mav_has_fix = false;
double mav_lat = 0.0, mav_lon = 0.0;
double mav_alt_rel = 0.0;           // altitud relativa en m
double mav_ground_speed = 0.0;      // m/s
double mav_heading_deg = 0.0;       // grados
unsigned long mav_last_update_ms = 0;

// Reuso tus variables filtradas:
extern double lat_f, lon_f, alt_f;  // ya están declaradas
extern double alt_baro_f;           // la vamos a alimentar desde MAVLink

// Heartbeat hacia Pixhawk
unsigned long lastHeartbeatMs = 0;
unsigned long insideRadiusSince = 0;

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

// ============================================================
// 🔧 FUNCIONES Mavlink
// ============================================================
// =======================
// 📥 Lectura MAVLink
// =======================
void sendHeartbeatToPixhawk() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(
      42,                    // system_id del ESP32
      199,                   // component_id
      &msg,
      MAV_TYPE_ONBOARD_CONTROLLER,
      MAV_AUTOPILOT_INVALID,
      0,                     // base_mode
      0,                     // custom_mode
      MAV_STATE_ACTIVE
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void readMavlink() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (SerialMAV.available()) {
    uint8_t c = SerialMAV.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch (msg.msgid) {

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
          mavlink_global_position_int_t pos;
          mavlink_msg_global_position_int_decode(&msg, &pos);

          double raw_lat = pos.lat / 1e7;
          double raw_lon = pos.lon / 1e7;
          double raw_alt = pos.relative_alt / 1000.0;  // m, relativo al home

          // Reuso tu filtro de GPS:
          filterGPS(raw_lat, raw_lon, raw_alt, lat_f, lon_f, alt_f);

          alt_baro_f   = alt_f;   // ahora la "altitud baro" viene del Pixhawk
          mav_lat      = lat_f;
          mav_lon      = lon_f;
          mav_alt_rel  = alt_baro_f;
          mav_has_fix  = true;
          mav_last_update_ms = millis();
          break;
        }

        case MAVLINK_MSG_ID_VFR_HUD: {
          mavlink_vfr_hud_t hud;
          mavlink_msg_vfr_hud_decode(&msg, &hud);
          mav_ground_speed = hud.groundspeed;   // m/s
          mav_heading_deg  = hud.heading;       // grados
          break;
        }

        default:
          break;
      }
    }
  }
}

// ==================================================
//  🚁 SET_MODE → GUIDED
// ==================================================
void setModeGuided() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_set_mode_pack(
      42,
      199,
      &msg,
      1,
      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
      4); // GUIDED

  SerialMAV.write(buf, mavlink_msg_to_send_buffer(buf, &msg));
}

// ==================================================
//  🚁 ARM / DISARM
// ==================================================
void pixhawkArm(bool arm) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(
      42,
      199,
      &msg,
      1, 0,
      MAV_CMD_COMPONENT_ARM_DISARM,
      0,
      arm ? 1.0 : 0.0,
      0,0,0,0,0,0);

  SerialMAV.write(buf, mavlink_msg_to_send_buffer(buf, &msg));
}

// ==================================================
//  🚁 GOTO (SET_POSITION_TARGET_GLOBAL_INT)
// ==================================================
void sendGotoPixhawk(const Coordinate &target) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  uint16_t type_mask =
      (1 << 3) | (1 << 4) | (1 << 5) |
      (1 << 6) | (1 << 7) | (1 << 8) |
      (1 << 9) | (1 << 10);

  mavlink_msg_set_position_target_global_int_pack(
      42,
      199,
      &msg,
      millis(),
      1, 1,
      MAV_FRAME_GLOBAL_RELATIVE_ALT,
      type_mask,
      (int32_t)(target.lat * 1e7),
      (int32_t)(target.lon * 1e7),
      mission.altitude,
      0, 0, 0,
      0, 0, 0,
      0, 0);

  SerialMAV.write(buf, mavlink_msg_to_send_buffer(buf, &msg));
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
      setModeGuided();
      delay(300);
      pixhawkArm(true);
      state = WAITING_MISSION;
      Serial.println("Pixhawk ARM + GUIDED enviado");
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

    sendEventToGS(type, mav_lat, mav_lon, mav_alt_rel, ts, confidence);
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
    sendEventToGS(type, mav_lat, mav_lon, mav_alt_rel, ts, confidence);
    return;
  }

  Serial.printf("⚠️ [RPI] Evento desconocido: %s\n", type);
}
// ============================================================================
// 🔄 MANEJO DE SENSORES Y COMUNICACIONES
// ============================================================================

void handleTelemetry() {
  static unsigned long lastTelemetryTime = 0;
  if (millis() - lastTelemetryTime >= 1000) {

    if (!simulationMode && mav_has_fix) {
      double lat     = mav_lat;
      double lon     = mav_lon;
      double alt     = mav_alt_rel;                  // relativa al home
      double speed_k = mav_ground_speed * 3.6;       // km/h
      double heading = mav_heading_deg;
      unsigned long ts = millis();                   // o podés mandar boot_ms

      sendTelemetry(lat, lon, alt, speed_k, heading, ts);
    }
    else if (simulationMode) {
      // Si querés seguir probando sin Pixhawk, podés mantener esto
      double lat = lat_f;
      double lon = lon_f;
      unsigned long ts = millis();
      sendTelemetry(lat, lon, alt_baro_f, 5.0, 0.0, ts);
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
  doc["lat"] = mav_lat;
  doc["lon"] = mav_lon;
  doc["alt"] = mav_alt_rel;
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
// Funciones navegacion
// ============================================================
void navigateTo(const Coordinate& target) {
  double dist    = haversineDistance(lat_f, lon_f, target.lat, target.lon);
  double bearing = computeBearing(lat_f, lon_f, target.lat, target.lon);

  Serial.printf("🧭 NAV (Pixhawk) → bearing=%.1f°, dist=%.1f m (lat=%.6f, lon=%.6f → %.6f, %.6f)\n",
                bearing, dist, lat_f, lon_f, target.lat, target.lon);

  // En lugar de simular movimiento, mandamos el goto al Pixhawk
  sendGotoPixhawk(target);
}

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

        Coordinate target = pathPoints[currentWaypoint];
        double dist = haversineDistance(lat_f, lon_f, target.lat, target.lon);

        bool close_enough = dist < 3.0;
        bool slow_enough  = mav_ground_speed < 0.5;

        if (close_enough && slow_enough) {

            if (millis() - insideRadiusSince > 1500) {
                Serial.println("WAYPOINT REACHED ✔");
                state = STABILIZE;
                sendRPINotification("WAYPOINT_REACHED");
                return;
            }
        } else {
            insideRadiusSince = millis();
            sendGotoPixhawk(target);  // Reenvía orden si hace falta
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
  SerialMAV.begin(57600, SERIAL_8N1, MAV_RX, MAV_TX);
  Serial.println("✅ UART2 (Pixhawk MAVLink) inicializado 57600 bps");
  SerialRPI.begin(9600, SERIAL_8N1, RPI_RX, RPI_TX);
  SerialRPI.setTimeout(RPI_TIMEOUT_MS);
  memset(rpiBuffer, 0, sizeof(rpiBuffer));
  Serial.println("✅ UART0 (USB) OK");
  Serial.println("✅ UART1 (RPI) inicializado 9600 bps");
  LoRa.setPins(LORA_CS,LORA_RST,LORA_IRQ);
  if(!LoRa.begin(LORA_BAND)){Serial.println("❌ LoRa fallo");while(1)delay(1000);}
  Serial.println("✅ LoRa listo");

  delay(1500);
}

void loop(){
  readMavlink();
  // 2) Enviar heartbeat al Pixhawk cada 1 s
  if (millis() - lastHeartbeatMs >= 1000) {
    sendHeartbeatToPixhawk();
    lastHeartbeatMs = millis();
  }

  handleTelemetry();
  handleSerialRPI();
  handleLoRa();
  checkPendingAcks();
  updateStateMachine();
  simulateDroneMotion();
}


