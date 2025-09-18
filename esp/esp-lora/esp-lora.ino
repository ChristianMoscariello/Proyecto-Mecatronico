/*
  ESP32 LoRa Telemetry Sender (433 MHz)
  - Lee GPS u-blox NEO-7M (TinyGPS++)
  - Empaqueta telemetría en JSON
  - Envía por LoRa (SX1278) la cadena: GS#<json>#END
  - Imprime por Serial para debug
*/

#include <TinyGPS++.h>
#include <LoRa.h>
#include <Arduino.h>
#include <Wire.h>

// --------- CONFIG: ajustar según tu cableado ----------
#define LORA_SS   5    // NSS/CS
#define LORA_RST  14   // RESET
#define LORA_DIO0 26   // DIO0 (IRQ)
#define LORA_BAND 433E6  // 433 MHz (ajustar según módulo y región)

// UART para GPS (Serial2)
#define GPS_RX 16   // GPS TX -> ESP32 RX2
#define GPS_TX 17   // GPS RX -> ESP32 TX2 (opcional)


// Si querés leer batería (opcional), conecta divisor a pin ADC
#define BATTERY_PIN 35   // por ejemplo GPIO35 (ADC1_CH7). Ajustar si usás otro pin.

// TinyGPS++ objeto
TinyGPSPlus gps;

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL_MS = 1000; // intervalo de envío (ms)

// Helpers: JSON construcción simple
String buildTelemetryJson(double lat, double lon, double alt, double speed_mps, double course_deg, float batt_pct) {
  // speed: convertir m/s -> km/h
  double speed_kmh = speed_mps * 3.6;
  // Construir JSON manualmente (evitar librería pesada)
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,\"speed\":%.2f,\"heading\":%.1f,\"battery\":%.1f}",
    lat, lon, alt, speed_kmh, course_deg, batt_pct);
  return String(buf);
}

float readBatteryPercent() {
  // Lectura ADC simulada / ejemplo: si tienes divisor, mapear al 0-100%
  // Ajustá Vref y divisor según tu circuito.
  // En ESP32 el ADC devuelve 0..4095 (12-bit); si usás otra resolución ajustá.
  int raw = analogRead(BATTERY_PIN); // requires pin to be ADC1 (32..39)
  float voltage = (raw / 4095.0f) * 3.3f; // si hay divisor, multiplicar por factor adecuado
  // Si usás divisor (por ejemplo 2:1), voltage *= 2;
  // A continuación convertimos a % asumiendo batería 3.3..4.2V (ajustar según pack)
  // Este es un ejemplo aproximado:
  float v_batt = voltage * 2.0f; // <<-- AJUSTAR según divisor real (si no tienes divisor, quitar *2)
  float pct = (v_batt - 3.3f) / (4.2f - 3.3f) * 100.0f;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 LoRa Telemetry - starting...");

  // Iniciar UART del GPS
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS UART started on RX=" + String(GPS_RX) + " TX=" + String(GPS_TX));

  // Iniciar LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  Serial.print("Inicializando LoRa en ");
  Serial.print(LORA_BAND/1e6);
  Serial.println(" MHz ...");
  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("ERROR: LoRa init failed. Revisa pines/frecuencia.");
    while (true) delay(1000);
  }
  Serial.println("LoRa inicializado correctamente.");

  // Config ADC pin (si usás lectura de batería)
  analogReadResolution(12); // 12-bit (0-4095)
  pinMode(BATTERY_PIN, INPUT);
}

void loop() {
  // 1) Leer todo lo que venga del GPS
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    gps.encode(c);
  }

  // 2) Si hay una ubicación actualizada, crear telemetría
  if (gps.location.isUpdated()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    double alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    double speed = gps.speed.isValid() ? gps.speed.mps() : 0.0;
    double course = gps.course.isValid() ? gps.course.deg() : 0.0;
    float batt = readBatteryPercent(); // opcional, ajustar circuito

    String json = buildTelemetryJson(lat, lon, alt, speed, course, batt);

    // Envolver con HEADER/FOOTER para que tu GS lo parseé: GS#<json>#END
    String packet = String("GS#") + json + String("#END");

    // Enviar por LoRa
    LoRa.beginPacket();
    LoRa.print(packet);
    LoRa.endPacket();

    // También imprimir por Serial para debug
    Serial.print("TX ");
    Serial.println(packet);

    lastSend = millis();
  }

  // 3) Optional: enviar telemetría periódica aunque no haya 'isUpdated'
  // En caso que quieras enviar cada segundo la última posición, incluso si no cambió:
  // else if (millis() - lastSend > SEND_INTERVAL_MS && gps.location.isValid()) { ... }

  // pequeño delay para evitar uso excesivo CPU
  delay(50);
}
