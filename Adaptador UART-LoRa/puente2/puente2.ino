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
String incomingUSB = "";     // Buffer para datos desde la GS (USB)
unsigned long ledOffTime = 0;

// ==========================
// Setup
// ==========================
void setup() {
  Serial.begin(115200);      // Comunicación con la Ground Station
  while (!Serial);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_BAND)) {
    // Si falla LoRa, queda en bucle silencioso
    while (true) { delay(1000); }
  }
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

    // Transmitir al puerto USB SIN saltos de línea extra
    Serial.print(payload);
    Serial.flush();
  }

  // ==========================
  // 2. Recepción USB → LoRa
  // ==========================
  while (Serial.available()) {
    char c = Serial.read();
    incomingUSB += c;

    // Procesar todos los frames completos detectando "#END"
    int idx;
    while ((idx = incomingUSB.indexOf("#END")) != -1) {
      String frame = incomingUSB.substring(0, idx + 4); // incluye "#END"

      LoRa.beginPacket();
      LoRa.print(frame);
      LoRa.endPacket();

      incomingUSB.remove(0, idx + 4); // eliminar el frame procesado
    }

    // Evitar crecimiento infinito del buffer
    if (incomingUSB.length() > 2048)
      incomingUSB = "";
  }

}
