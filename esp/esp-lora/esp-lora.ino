#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>

// ==============================
// Configuración GPS (Serial2)
// ==============================
#define GPS_RX 16
#define GPS_TX 17
TinyGPSPlus gps;

// ==============================
// Configuración LoRa
// ==============================
#define LORA_SS    18
#define LORA_RST   14
#define LORA_DIO0  26
#define LORA_BAND  433E6

// ==============================
// Variables
// ==============================
unsigned long lastSend = 0;
const unsigned long sendInterval = 1000; // ms
double lastLat = 0.0, lastLon = 0.0, lastAlt = 0.0;
double lastSpeed = 0.0, lastHeading = 0.0;
float batteryLevel = 100.0;

#define MAX_BUFFER 10
String telemetryBuffer[MAX_BUFFER];
int bufferStart = 0, bufferEnd = 0;

// ==============================
// Setup
// ==============================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Configurar GPS
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Inicializar LoRa
  SPI.begin(5, 19, 26, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("Error inicializando LoRa");
    while (1);
  }
  Serial.println("✅ LoRa inicializado");
}

// ==============================
// Loop principal
// ==============================
void loop() {
  readGPS();
  handleTelemetry();
  simulateBattery();
}

// ==============================
// Funciones GPS
// ==============================
void readGPS() {
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }

  if (gps.location.isValid()) {
    lastLat = gps.location.lat();
    lastLon = gps.location.lng();
    lastAlt = gps.altitude.meters();
    lastSpeed = gps.speed.kmph();
    lastHeading = gps.course.deg();
  }
}

// ==============================
// Funciones Telemetría
// ==============================
void handleTelemetry() {
  if (millis() - lastSend >= sendInterval) {
    lastSend = millis();

    if (!gps.location.isValid()) {
      Serial.println("⚠️ GPS no válido, esperando señal...");
      return;
    }

    String payload = String("{\"lat\":") + String(lastLat, 6) +
                     ",\"lon\":" + String(lastLon, 6) +
                     ",\"alt\":" + String(lastAlt, 1) +
                     ",\"speed\":" + String(lastSpeed, 2) +
                     ",\"heading\":" + String(lastHeading, 1) +
                     ",\"battery\":" + String(batteryLevel, 1) +
                     "}";

    String message = "GS#" + payload + "#END";

    // Guardar en buffer
    bufferTelemetry(message);

    // Enviar buffer
    sendBuffer();
  }
}

void bufferTelemetry(String message) {
  telemetryBuffer[bufferEnd] = message;
  bufferEnd = (bufferEnd + 1) % MAX_BUFFER;
  if (bufferEnd == bufferStart) { // buffer lleno
    bufferStart = (bufferStart + 1) % MAX_BUFFER;
  }
}

void sendBuffer() {
  while (bufferStart != bufferEnd) {
    String msg = telemetryBuffer[bufferStart];
    bufferStart = (bufferStart + 1) % MAX_BUFFER;

    // Serial debug
    Serial.println(msg);

    // LoRa
    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();
    LoRa.idle();
  }
}

// ==============================
// Simulación de batería
// ==============================
void simulateBattery() {
  batteryLevel -= 0.01;
  if (batteryLevel < 0) batteryLevel = 100.0;
}
