#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>

// ==============================
// CONFIGURACIÓN DEL LORA
// ==============================
#define LORA_SS    10
#define LORA_RST    9   // Pin de reset del módulo LoRa
#define LORA_DIO0   2
#define LORA_BAND  433E6  // Ajustar según tu región

// ==============================
// WATCHDOG SOFTWARE
// ==============================
unsigned long lastPacketMillis = 0;
const unsigned long WATCHDOG_TIMEOUT = 9000; // 5s sin paquetes -> reset LoRa

// ==============================
// FUNCIONES
// ==============================
void resetLoRaModule() {
  Serial.println("⚠️ Reiniciando módulo LoRa...");
  digitalWrite(LORA_RST, LOW);
  delay(100);
  digitalWrite(LORA_RST, HIGH);
  delay(100);
  LoRa.begin(LORA_BAND);
}

void printTelemetry(const String& msg) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, msg);

  if (error) {
    Serial.println("JSON inválido");
    return;
  }

  int seq     = doc["seq"];
  float lat   = doc["lat"];
  float lon   = doc["lon"];
  float alt   = doc["alt"];
  float speed = doc["speed"];
  float heading = doc["heading"];
  float battery = doc["battery"];

  Serial.print("SEQ: "); Serial.println(seq);
  Serial.print("Lat: "); Serial.println(lat, 6);
  Serial.print("Lon: "); Serial.println(lon, 6);
  Serial.print("Alt: "); Serial.println(alt);
  Serial.print("Speed: "); Serial.println(speed);
  Serial.print("Heading: "); Serial.println(heading);
  Serial.print("Battery: "); Serial.println(battery);

  // RSSI y SNR
  Serial.print("RSSI: "); Serial.print(LoRa.packetRssi()); Serial.print(" dBm | ");
  Serial.print("SNR: "); Serial.println(LoRa.packetSnr(), 2);
  Serial.println("-----");
}

// ==============================
// SETUP
// ==============================
void setup() {
  Serial.begin(9600);
  LoRa.setSpreadingFactor(10);       // robusto
  LoRa.setSignalBandwidth(125E3);    // 125 kHz
  LoRa.setCodingRate4(5);            // 4/5
  LoRa.enableCrc();                  // CRC activado
  LoRa.setTxPower(17);               // potencia (máx 20)
  LoRa.setSyncWord(0x12);            // igual que receptor
  while(!Serial);

  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH); // Mantener LoRa activo

  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("Error iniciando LoRa!");
    while (1);
  }

  Serial.println("📡 LoRa listo para recibir datos");
  lastPacketMillis = millis();
}

// ==============================
// LOOP
// ==============================
void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }

    // Quitar headers si los hay (GS# ... #END)
    int startIdx = incoming.indexOf("GS#");
    int endIdx   = incoming.indexOf("#END");

    if (startIdx != -1 && endIdx != -1 && endIdx > startIdx) {
      String jsonStr = incoming.substring(startIdx + 3, endIdx);
      printTelemetry(jsonStr);
    } else {
      Serial.print("RAW: "); Serial.println(incoming);
    }

    lastPacketMillis = millis(); // Reset watchdog
  }

  // Watchdog software: reinicia LoRa si se traba
  if (millis() - lastPacketMillis > WATCHDOG_TIMEOUT) {
    resetLoRaModule();
    lastPacketMillis = millis();
  }
}
