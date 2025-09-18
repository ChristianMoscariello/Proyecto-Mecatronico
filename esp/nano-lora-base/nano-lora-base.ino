
/*
  Arduino Nano LoRa Receiver (433 MHz)
  - Usa módulo SX1278 conectado por SPI
  - Recibe paquetes desde ESP32 (o cualquier otro transmisor LoRa)
  - Reenvía al PC por Serial con envoltura: GS#<payload>#END
  - Debug por Serial a 9600 baudios
*/

#include <SPI.h>
#include <LoRa.h>

// --------- CONFIGURAR PINES SEGÚN TU CONEXIÓN ----------
#define LORA_SS   10   // NSS/CS al pin D10
#define LORA_RST   9   // RST al pin D9
#define LORA_DIO0  2   // DIO0 al pin D2 (interrupción)
#define LORA_BAND 433E6  // 433 MHz

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Nano LoRa Receiver - iniciando...");

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("Error inicializando LoRa. Revisa cableado/frecuencia.");
    while (true);
  }

  Serial.println("LoRa inicializado correctamente.");
}

void loop() {
  // Revisar si hay paquetes entrantes
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }

    // Reenviar al PC con envoltura GS# ... #END
    Serial.print("GS#");
    Serial.print(incoming);
    Serial.println("#END");

    // Debug: mostrar RSSI y SNR en el monitor serie
    Serial.print("DEBUG -> RSSI: ");
    Serial.print(LoRa.packetRssi());
    Serial.print(" dBm | SNR: ");
    Serial.println(LoRa.packetSnr());
  }
}
