#include <SPI.h>
#include <LoRa.h>

// ==========================
// Pines LoRa - ESP32
// ==========================
#define LORA_SS   5
#define LORA_RST  14
#define LORA_DIO0 2
#define LORA_BAND 433E6

// ==========================
// Variables
// ==========================
String incomingUSB = "";  // Buffer para mensajes desde la GS (USB)

// ==========================
// Setup
// ==========================
void setup() {
  Serial.begin(115200);          // Comunicación con la Ground Station
  while (!Serial);

  // Comentado: logs de estado
  // Serial.println("{\"status\":\"Iniciando puente LoRa-USB\"}");

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_BAND)) {
    // Serial.println("{\"error\":\"Fallo inicialización LoRa\"}");
    while (true) { delay(1000); }
  }

  // Serial.println("{\"status\":\"LoRa inicializado correctamente\"}");
  // Serial.println("{\"status\":\"Puente listo\"}");
}

// ==========================
// Loop principal
// ==========================
void loop() {

  // ==========================
  // 1. Recepción LoRa → USB
  // ==========================
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String payload = "";
    while (LoRa.available()) {
      payload += (char)LoRa.read();
    }

    // Envía frame completo a la Ground Station
    Serial.println(payload);

    // Comentado: RSSI/SNR opcional
    /*
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();
    Serial.print("{\"debug\":\"RSSI:");
    Serial.print(rssi);
    Serial.print(" SNR:");
    Serial.print(snr);
    Serial.println("\"}");
    */
  }

  // ==========================
  // 2. Recepción USB → LoRa
  // ==========================
  while (Serial.available()) {
    char c = Serial.read();
    incomingUSB += c;

    // Detectar fin de frame por "#END"
    if (incomingUSB.endsWith("#END")) {
      LoRa.beginPacket();
      LoRa.print(incomingUSB);
      LoRa.endPacket();

      // Serial.println("{\"info\":\"Frame enviado por LoRa\"}");
      incomingUSB = "";
    }

    // Evitar crecimiento infinito del buffer
    if (incomingUSB.length() > 2048)
      incomingUSB = "";
  }
}
