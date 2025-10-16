#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>
#include <Adafruit_BMP280.h>
#include <Preferences.h>

// =============================
// Configuración GPS 
// =============================
#define GPS_RX 16     // ESP32 RX2 <- TX GPS
#define GPS_TX 17     // ESP32 TX2 -> RX GPS (usualmente no se usa)
HardwareSerial SerialGPS(2);
TinyGPSPlus gps;

// =============================
// Configuración LoRa
// =============================
#define LORA_CS 5
#define LORA_RST 14
#define LORA_IRQ 27
#define LORA_BAND 433E6

// =============================
// Configuración Rspbrry
// =============================
#define RPI_RX  4     // UART1 RX (desde Raspberry)
#define RPI_TX  2     // UART1 TX (hacia Raspberry)
HardwareSerial SerialRPI(1);    // UART1 para Raspberry

// =============================
// Configuración BMP280
// =============================
Adafruit_BMP280 bmp;    // objeto BMP280
bool bmp_ok = false;    // estado del sensor
float alt_bmp = 0.0;

// =============================
// Trims (guardados en Preferences)
// =============================
Preferences prefs;  // objeto de Preferences

float trim_roll     = 0.0;
float trim_pitch    = 0.0;
float trim_yaw      = 0.0;
float trim_throttle = 0.0;
float trim_switch   = 0.0; 

// =============================
// Variables de filtrado
// =============================
double lat_window[4] = {0, 0, 0, 0};
double lon_window[4] = {0, 0, 0, 0};
double alt_window[4] = {0, 0, 0, 0};
int win_idx = 0;

double lat_f = 0, lon_f = 0, alt_f = 0;
bool have_filter = false;
const double ALPHA = 0.35;

// =============================
// Funciones de filtrado
// =============================
double median4(double a, double b, double c, double d) {
  double arr[4] = {a, b, c, d};
  for (int i = 0; i < 3; i++) {
    for (int j = i + 1; j < 4; j++) {
      if (arr[j] < arr[i]) {
        double tmp = arr[i];
        arr[i] = arr[j];
        arr[j] = tmp;
      }
    }
  }
  return (arr[1] + arr[2]) / 2.0;
}

void filterGPS(double lat, double lon, double alt,
               double &lat_out, double &lon_out, double &alt_out) {
  lat_window[win_idx] = lat;
  lon_window[win_idx] = lon;
  alt_window[win_idx] = alt;
  win_idx = (win_idx + 1) % 4;

  double lat_m = median4(lat_window[0], lat_window[1], lat_window[2], lat_window[3]);
  double lon_m = median4(lon_window[0], lon_window[1], lon_window[2], lon_window[3]);
  double alt_m = median4(alt_window[0], alt_window[1], alt_window[2], alt_window[3]);

  if (!have_filter) {
    lat_f = lat_m;
    lon_f = lon_m;
    alt_f = alt_m;
    have_filter = true;
  } else {
    lat_f = ALPHA * lat_m + (1.0 - ALPHA) * lat_f;
    lon_f = ALPHA * lon_m + (1.0 - ALPHA) * lon_f;
    alt_f = ALPHA * alt_m + (1.0 - ALPHA) * alt_f;
  }

  lat_out = lat_f;
  lon_out = lon_f;
  alt_out = alt_f;
}

// =============================
// Conversión de fecha/hora GPS a timestamp UNIX
// =============================
unsigned long toUnixTime(int year, int month, int day, int hour, int minute, int second) {
  // Ajuste de meses/años para algoritmo tipo Zeller
  if (month <= 2) {
    year -= 1;
    month += 12;
  }
  long a = year / 100;
  long b = 2 - a + a / 4;

  long days = (long)(365.25 * (year + 4716))
            + (long)(30.6001 * (month + 1))
            + day + b - 1524.5;

  // Pasamos días a segundos y sumamos la hora
  unsigned long ts = (days - 2440588) * 86400UL; // diferencia con epoch 1970
  ts += hour * 3600UL + minute * 60UL + second;
  return ts;
}

// =============================
// Funciones para Trims
// =============================
void loadTrims() {
  prefs.begin("drone", true);
  trim_roll     = prefs.getFloat("trim_roll", 0.0);
  trim_pitch    = prefs.getFloat("trim_pitch", 0.0);
  trim_yaw      = prefs.getFloat("trim_yaw", 0.0);
  trim_throttle = prefs.getFloat("trim_throttle", 0.0);
  trim_switch   = prefs.getFloat("trim_switch", 0.0);
  prefs.end();
}

void saveTrims() {
  prefs.begin("drone", false);
  prefs.putFloat("trim_roll",     trim_roll);
  prefs.putFloat("trim_pitch",    trim_pitch);
  prefs.putFloat("trim_yaw",      trim_yaw);
  prefs.putFloat("trim_throttle", trim_throttle);
  prefs.putFloat("trim_switch",   trim_switch);
  prefs.end();
}

// =============================
// Enviar trims a GS
// =============================
void sendTrimsToGS() {
  StaticJsonDocument<256> doc;
  doc["t"] = "TRIM_VALUES";

  JsonObject d = doc.createNestedObject("d");
  d["roll"]     = trim_roll;
  d["pitch"]    = trim_pitch;
  d["yaw"]      = trim_yaw;
  d["throttle"] = trim_throttle;
  d["switch"]   = trim_switch;

  String payload;
  serializeJson(doc, payload);
  String msg = "GS#" + payload + "#END";

  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();

  Serial.println("Trims enviados: " + msg);
}

// =============================
// Actualizar trims desde GS
// =============================
void updateTrimFromGS(String axis, float value) {
  if (axis == "roll")        trim_roll = value;
  else if (axis == "pitch")  trim_pitch = value;
  else if (axis == "yaw")    trim_yaw = value;
  else if (axis == "throttle") trim_throttle = value;
  else if (axis == "switch") trim_switch = value;

  saveTrims(); // guardamos cada vez que se actualiza
}

// =============================
// Función para enviar Telemetría
// =============================
void sendTelemetry(double lat, double lon, double alt, 
                   double speed, double heading, unsigned long ts) {
  StaticJsonDocument<256> doc;

  doc["t"] = "TELEMETRY";

  JsonObject d = doc.createNestedObject("d");
  d["lat"]       = lat;
  d["lon"]       = lon;
  d["alt"]       = alt;
  d["speed"]     = speed;
  d["heading"]   = heading;
  d["battery"]   = 0.0;
  d["status"]    = 0.0;
  d["current_wp"]= 0.0;
  d["total_wp"]  = 0.0;

  doc["ts"] = ts;

  String payload;
  serializeJson(doc, payload);

  String msg = "GS#" + payload + "#END";

  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();

  Serial.println(msg);
}

// =============================
// Funcion para enviar evento FIRE / PERSON
// =============================
void sendEventToGS(const String &topic, double lat, double lon, double alt, unsigned long ts) {
  StaticJsonDocument<256> doc;
  doc["t"] = topic;            // "FIRE" o "PERSON"
  JsonObject d = doc.createNestedObject("d");
  d["lat"] = lat; d["lon"] = lon; d["alt"] = alt;
  doc["ts"] = ts;

  String payload;
  serializeJson(doc, payload);
  String msg = "GS#" + payload + "#END";
  LoRa.beginPacket(); LoRa.print(msg); LoRa.endPacket();
  Serial.println("Evento enviado: " + msg);
}


// =============================
// Setup
// =============================
void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Serial.println("Inicializando LoRa...");
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("Fallo al iniciar LoRa!");
    while (1);
  }
  Serial.println("LoRa OK.");

// Inicializar BMP280
  Serial.println("Inicializando BMP280...");
  if (bmp.begin(0x76)) {             // Dirección I2C habitual: 0x76 o 0x77
    bmp_ok = true;
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,   // temperatura
                    Adafruit_BMP280::SAMPLING_X16,  // presión
                    Adafruit_BMP280::FILTER_X16,    // filtro interno
                    Adafruit_BMP280::STANDBY_MS_63);
    Serial.println("BMP280 OK.");
  } else {
    Serial.println("No se detectó BMP280!");
  }

  loadTrims();  // cargamos trims desde la memoria
  Serial.printf("Trims cargados: Roll %.2f | Pitch %.2f | Yaw %.2f | Throttle %.2f | Switch %.2f\n", trim_roll, trim_pitch, trim_yaw, trim_throttle, trim_switch);
}

// =============================
// Loop principal
// =============================
void loop() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());

    if (gps.location.isUpdated()) {
      double lat_raw = gps.location.lat();
      double lon_raw = gps.location.lng();
      double alt_gps = gps.altitude.meters();
      double speed_raw = gps.speed.kmph();
      double heading_raw = gps.course.deg();
      // Usar BMP280 si disponible, sino GPS
      if (bmp_ok) {
        alt_bmp = bmp.readAltitude(1013.25); // presión estándar a nivel del mar
      }
      double alt_raw = bmp_ok ? alt_bmp : alt_gps;


      double lat, lon, alt;
      filterGPS(lat_raw, lon_raw, alt_raw, lat, lon, alt);

      unsigned long ts = 0;
      if (gps.date.isValid() && gps.time.isValid()) {
        ts = toUnixTime(gps.date.year(), gps.date.month(), gps.date.day(),
                        gps.time.hour(), gps.time.minute(), gps.time.second());
      }

      sendTelemetry(lat, lon, alt, speed_raw, heading_raw, ts);
    }
  }
  // Procesar mensajes desde Raspberry 
  if (SerialRPI.available()) {
    String jsonIn = SerialRPI.readStringUntil('\n');   // asume mensajes terminados en \n
    StaticJsonDocument<128> docIn;
    DeserializationError err = deserializeJson(docIn, jsonIn);

    if (!err) {
      String topic = docIn["t"] | "";
      if (topic == "FIRE" || topic == "PERSON") {
        // Tomamos última posición filtrada y timestamp
        unsigned long ts = 0;
        if (gps.date.isValid() && gps.time.isValid()) {
          ts = toUnixTime(gps.date.year(), gps.date.month(), gps.date.day(),
                          gps.time.hour(), gps.time.minute(), gps.time.second());
        }
        sendEventToGS(topic, lat_f, lon_f, alt_f, ts);
      } else {
        Serial.println("Evento desconocido: " + topic);
      }
    } else {
      Serial.println("Error al parsear JSON desde Raspberry");
    }
  }
}
