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
String incomingUSB = "";  // Buffer para mensajes desde la GS por USB

void setup() {
  Serial.begin(115200);         // USB a PC / GS
  while (!Serial);

  Serial.println("{\"status\":\"Iniciando Receptor/Transmisor LoRa\"}");

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("{\"error\":\"Fallo inicialización LoRa\"}");
    while (true) { delay(1000); }
  }

  Serial.println("{\"status\":\"LoRa inicializado correctamente\"}");
  Serial.println("{\"status\":\"Listo para recibir/enviar\"}");
}

void loop() {
  // ==========================
  // 1. Recepción LoRa -> GS
  // ==========================
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String payload = "";
    while (LoRa.available()) {
      payload += (char)LoRa.read();
    }

    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    // Envía al USB (hacia la GS)
    Serial.println(payload);

    // Debug RSSI/SNR
    Serial.print("{\"debug\":\"RSSI:");
    Serial.print(rssi);
    Serial.print(" SNR:");
    Serial.print(snr);
    Serial.println("\"}");
  }

  // ==========================
  // 2. Recepción GS -> LoRa
  // ==========================
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (incomingUSB.length() > 0) {
        // Mensaje completo listo para enviar al dron
        Serial.print("{\"info\":\"Enviando a LoRa: ");
        Serial.print(incomingUSB);
        Serial.println("\"}");

        LoRa.beginPacket();
        LoRa.print(incomingUSB);
        LoRa.endPacket();

        incomingUSB = "";  // Limpia buffer
      }
    } else {
      incomingUSB += c;
    }
  }
}
