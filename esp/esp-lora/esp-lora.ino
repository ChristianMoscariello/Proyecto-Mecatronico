#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>

// =============================
// Configuración GPS y LoRa
// =============================
#define GPS_RX 16     // ESP32 RX2 <- TX GPS
#define GPS_TX 17     // ESP32 TX2 -> RX GPS (usualmente no se usa)
#define LORA_CS 5
#define LORA_RST 14
#define LORA_IRQ 27
#define LORA_BAND 433E6

HardwareSerial SerialGPS(2);
TinyGPSPlus gps;

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
      double alt_raw = gps.altitude.meters();
      double speed_raw = gps.speed.kmph();
      double heading_raw = gps.course.deg();

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
}
