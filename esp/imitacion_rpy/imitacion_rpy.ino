#include <Arduino.h>
#include <ArduinoJson.h>

// =============================
// CONFIGURACIÓN HARDWARE
// =============================
#define FIRE_BTN   12     // Pulsador FIRE → masa
#define PERSON_BTN 13     // Pulsador PERSON → masa
#define UART_TX    17     // TX hacia el dron (RPI_TX en el dron)
#define UART_RX    16     // RX desde el dron
#define LED_PIN     2     // LED interno del ESP32 DevKitV1
HardwareSerial SerialToDrone(2);

// =============================
// VARIABLES GLOBALES
// =============================
unsigned long lastDebounceFire = 0;
unsigned long lastDebouncePerson = 0;
bool lastFireState = HIGH;
bool lastPersonState = HIGH;
const unsigned long debounceDelay = 50;

unsigned long nextMsgCounter = 0;
String rxBuffer = "";

// =============================
// FUNCIONES AUXILIARES
// =============================

// Parpadeo LED (3 veces)
void blinkLedStable() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

// Generar un ID incremental simple
String generateMsgID() {
  nextMsgCounter++;
  return String(nextMsgCounter);
}

// Envía un evento con solo "t" e "id"
void sendEvent(const char* type) {
  StaticJsonDocument<128> doc;
  doc["t"] = type;
  String payload;
  serializeJson(doc, payload);

  SerialToDrone.print(payload);
  //SerialToDrone.print('\n'); 
  Serial.println("📤 Enviado → Dron (UART2): " + payload);
}

// Procesar JSON recibido desde el dron
void processIncomingJSON(const String& jsonIn) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, jsonIn);
  if (err) {
    Serial.print("⚠️ JSON inválido recibido: ");
    Serial.println(err.c_str());
    return;
  }

  const char* type = doc["t"] | "";
  JsonObject d = doc["d"];
  unsigned long ts = doc["ts"] | 0;

  if (strcmp(type, "STABLE") == 0) {
    double lat = d["lat"] | 0.0;
    double lon = d["lon"] | 0.0;
    double alt = d["alt"] | 0.0;

    Serial.println("📷 [STABLE] recibido del dron:");
    Serial.printf("   ↳ Lat: %.6f | Lon: %.6f | Alt: %.2f m | ts: %lu\n", lat, lon, alt, ts);
    blinkLedStable();
 
  } 


  else {
    Serial.print("ℹ️ Tipo desconocido: ");
    Serial.println(type);
  }
}

// Leer mensajes entrantes desde el dron
void handleSerialInput() {
  while (SerialToDrone.available()) {
    char c = SerialToDrone.read();
    if (c == '\n') {
      if (rxBuffer.length() > 0) {
        processIncomingJSON(rxBuffer);
        rxBuffer = "";
      }
    } else {
      rxBuffer += c;
    }
  }
}


// =============================
// SETUP
// =============================
void setup() {
  Serial.begin(9600);
  SerialToDrone.begin(9600, SERIAL_8N1, UART_RX, UART_TX);

  pinMode(FIRE_BTN, INPUT_PULLUP);
  pinMode(PERSON_BTN, INPUT_PULLUP);

  Serial.println("🚀 Simulador de RPi listo. Pulsa:");
  Serial.println("   🔥 GPIO12 → FIRE");
  Serial.println("   🧍 GPIO13 → PERSON");
}

// =============================
// LOOP PRINCIPAL
// =============================
void loop() {
  bool fireState = digitalRead(FIRE_BTN);
  bool personState = digitalRead(PERSON_BTN);

  handleSerialInput(); 

  // FIRE pulsado
  if (fireState == LOW && lastFireState == HIGH && (millis() - lastDebounceFire) > debounceDelay) {
    lastDebounceFire = millis();
    Serial.println("🔥 FIRE activado");
    sendEvent("FIRE");
    delay(1000);
  }

  // PERSON pulsado
  if (personState == LOW && lastPersonState == HIGH && (millis() - lastDebouncePerson) > debounceDelay) {
    lastDebouncePerson = millis();
    Serial.println("🧍 PERSON activado");
    sendEvent("PERSON");
    delay(1000);
  }

  lastFireState = fireState;
  lastPersonState = personState;
}
