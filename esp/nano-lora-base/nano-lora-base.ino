#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>

// Pines LoRa (ajusta si usás otros)
#define LORA_SS   5
#define LORA_RST  14
#define LORA_DIO0 2
#define LORA_BAND 433E6   // frecuencia SX1278

unsigned long lastPacketMillis = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH);

  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("❌ Error iniciando LoRa");
    while (1);
  }

  Serial.println("📡 Receptor LoRa ESP32 listo en 433 MHz");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }

    int startIdx = incoming.indexOf("GS#");
    int endIdx   = incoming.indexOf("#END");

    if (startIdx != -1 && endIdx != -1 && endIdx > startIdx) {
      String jsonStr = incoming.substring(startIdx + 3, endIdx);

      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, jsonStr);

      if (!error) {
        // Reenvío directo a la PC (para Python)
        Serial.print("GS#");
        serializeJson(doc, Serial);
        Serial.println("#END");

        // Debug adicional en el monitor serie
        Serial.print("RSSI: "); Serial.print(LoRa.packetRssi());
        Serial.print(" dBm | SNR: "); Serial.println(LoRa.packetSnr(), 2);

        lastPacketMillis = millis();
      } else {
        Serial.println("❌ Error decodificando JSON");
      }
    } else {
      Serial.print("RAW inválido: "); Serial.println(incoming);
    }
  }

  // Watchdog LoRa (si pasan 30s sin paquetes, resetea módulo)
  if (millis() - lastPacketMillis > 30000) {
    Serial.println("⚠️ Watchdog: reiniciando LoRa...");
    digitalWrite(LORA_RST, LOW);
    delay(100);
    digitalWrite(LORA_RST, HIGH);
    delay(100);
    LoRa.begin(LORA_BAND);
    lastPacketMillis = millis();
  }
}
