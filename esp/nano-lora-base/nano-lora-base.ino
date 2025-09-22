#include <SPI.h>
#include <LoRa.h>

// Pines LoRa - ESP32
#define LORA_SS   5    // NSS / CS
#define LORA_RST  14   // Reset
#define LORA_DIO0 2    // DIO0
#define LORA_BAND 433E6  // Frecuencia en Hz

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("{\"status\":\"Iniciando Receptor LoRa\"}");

  // Configuración de pines SPI y LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("{\"error\":\"Fallo inicialización LoRa\"}");
    while (true) {
      delay(1000);  // No avanza hasta que se corrija
    }
  }

  Serial.println("{\"status\":\"LoRa inicializado correctamente\"}");
}

void loop() {
  int packetSize = LoRa.parsePacket();  

  if (packetSize) {
    String payload = "";

    while (LoRa.available()) {
      payload += (char)LoRa.read();
    }

    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    // Envia JSON por Serial USB
    
    Serial.println(payload);
    
  }

  delay(1);  
}
