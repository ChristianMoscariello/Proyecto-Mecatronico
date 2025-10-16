#include <Arduino.h>
#include <ArduinoJson.h>

// =============================
// CONFIGURACIÓN HARDWARE
// =============================
#define FIRE_BTN   12     // Pulsador FIRE → masa
#define PERSON_BTN 13     // Pulsador PERSON → masa
#define UART_TX    17     // TX hacia el dron (RPI_TX en el dron)
#define UART_RX    16     // RX desde el dron (no usado)
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

// =============================
// FUNCIONES AUXILIARES
// =============================

// Generar un ID incremental simple
String generateMsgID() {
  nextMsgCounter++;
  return String(nextMsgCounter);
}

// Envía un evento con solo "t" e "id"
void sendEvent(const char* type) {
  StaticJsonDocument<128> doc;
  doc["t"] = type;
  //doc["id"] = generateMsgID();
  //doc["ts"] = millis();

  String payload;
  serializeJson(doc, payload);

  SerialToDrone.print(payload);
  //SerialToDrone.print('\n'); 
  Serial.println("📤 Enviado → Dron (UART2): " + payload);
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
