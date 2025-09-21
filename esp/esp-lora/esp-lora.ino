/*
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>

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

// Ventana para filtro de mediana
double lat_window[3] = {0, 0, 0};
double lon_window[3] = {0, 0, 0};
double alt_window[3] = {0, 0, 0};
int win_idx = 0;

// EMA (suavizado exponencial)
double lat_f = 0, lon_f = 0, alt_f = 0;
bool have_filter = false;
const double ALPHA = 0.35;   // 0.1 = muy suave, 0.5 = rápido

// =============================
// Funciones de filtrado
// =============================
double median3(double a, double b, double c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

void filterGPS(double lat, double lon, double alt,
               double &lat_out, double &lon_out, double &alt_out) {
  // Guardar en ventana circular
  lat_window[win_idx] = lat;
  lon_window[win_idx] = lon;
  alt_window[win_idx] = alt;
  win_idx = (win_idx + 1) % 3;

  // Calcular medianas
  double lat_m = median3(lat_window[0], lat_window[1], lat_window[2]);
  double lon_m = median3(lon_window[0], lon_window[1], lon_window[2]);
  double alt_m = median3(alt_window[0], alt_window[1], alt_window[2]);

  // EMA
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
  LoRa.setSpreadingFactor(10);       // robusto
  LoRa.setSignalBandwidth(125E3);    // 125 kHz
  LoRa.setCodingRate4(5);            // 4/5
  LoRa.enableCrc();                  // CRC activado
  LoRa.setTxPower(17);               // potencia (máx 20)
  LoRa.setSyncWord(0x12);            // igual que receptor

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

      // Aplicar filtro
      double lat, lon, alt;
      filterGPS(lat_raw, lon_raw, alt_raw, lat, lon, alt);

      // Formato JSON (Ground Station espera GS#...)
      String msg = "GS#{";
      msg += "\"lat\":" + String(lat, 6);      // 6 decimales (precisión ~10 cm)
      msg += ",\"lon\":" + String(lon, 6);
      msg += ",\"alt\":" + String(alt, 1);
      msg += ",\"speed\":" + String(speed_raw, 2);
      msg += ",\"heading\":" + String(heading_raw, 1);
      msg += ",\"battery\":0.0";               // Placeholder batería
      msg += "}";

      // Enviar por LoRa
      LoRa.beginPacket();
      LoRa.print(msg);
      LoRa.endPacket();

      // Enviar también por Serial (debug)
      Serial.println(msg);
    }
  }
}
*/
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>

// =================== Pines LoRa ===================
#define LORA_SS   5
#define LORA_RST  14
#define LORA_DIO0 2
#define LORA_BAND 433E6

// =================== Pines GPS ===================
#define GPS_RX 16   // GPS TX → ESP32 RX2
#define GPS_TX 17   // GPS RX → ESP32 TX2

TinyGPSPlus gps;

// =================== Variables ===================
unsigned long lastSend = 0;
int seq = 0;

// =============================
// Variables de filtrado
// =============================

// Ventana para filtro de mediana
double lat_window[3] = {0, 0, 0};
double lon_window[3] = {0, 0, 0};
double alt_window[3] = {0, 0, 0};
int win_idx = 0;

// EMA (suavizado exponencial)
double lat_f = 0, lon_f = 0, alt_f = 0;
bool have_filter = false;
const double ALPHA = 0.35;   // 0.1 = muy suave, 0.5 = rápido

// =============================
// Funciones de filtrado
// =============================
double median3(double a, double b, double c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

void filterGPS(double lat, double lon, double alt,
               double &lat_out, double &lon_out, double &alt_out) {
  // Guardar en ventana circular
  lat_window[win_idx] = lat;
  lon_window[win_idx] = lon;
  alt_window[win_idx] = alt;
  win_idx = (win_idx + 1) % 3;

  // Calcular medianas
  double lat_m = median3(lat_window[0], lat_window[1], lat_window[2]);
  double lon_m = median3(lon_window[0], lon_window[1], lon_window[2]);
  double alt_m = median3(alt_window[0], alt_window[1], alt_window[2]);

  // EMA
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



// =================== Setup ===================
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // --- LoRa ---
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("LoRa init failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(10);       // robusto
  LoRa.setSignalBandwidth(125E3);    // 125 kHz
  LoRa.setCodingRate4(5);            // 4/5
  LoRa.enableCrc();                  // CRC activado
  LoRa.setTxPower(17);               // potencia (máx 20)
  LoRa.setSyncWord(0x12);            // igual que receptor

  Serial.println("LoRa OK, esperando GPS...");
}

// =================== Loop ===================
void loop() {
  // Procesar datos entrantes del GPS
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }

  unsigned long now = millis();
  if (now - lastSend > 1000) {   // enviar cada 1s
    lastSend = now;
    sendTelemetry();
  }
}

// =================== Función de envío ===================
void sendTelemetry() {
  if (gps.location.isValid() && gps.altitude.isValid() && gps.speed.isValid()) {


    double lat_raw = gps.location.lat();
    double lon_raw = gps.location.lng();
    double alt_raw = gps.altitude.meters();
    double speed_raw = gps.speed.kmph();
    double heading_raw = gps.course.deg();

      // Aplicar filtro
    double lat, lon, alt;
    filterGPS(lat_raw, lon_raw, alt_raw, lat, lon, alt);

    /*float lat = gps.location.lat();
    float lon = gps.location.lng();
    float alt = gps.altitude.meters();
    float speed = gps.speed.kmph();
    float heading = gps.course.deg();*/

    // Construir mensaje JSON con envoltura GS# ... #END
    String msg = "GS#{\"seq\":" + String(seq++) +
                 ",\"lat\":" + String(lat, 6) +
                 ",\"lon\":" + String(lon, 6) +
                 ",\"alt\":" + String(alt, 1) +
                 ",\"speed\":" + String(speed_raw, 2) +
                 ",\"heading\":" + String(heading_raw, 1) +
                 ",\"battery\":0.0}#END";

    // Enviar por LoRa
    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();

    // Debug por Serial
    Serial.println(msg);
  } else {
    Serial.println("Esperando fix GPS...");
  }
}
