#include <Arduino.h>
#include <ArduinoJson.h>

// =============================
// CONFIGURACIÓN HARDWARE
// =============================
#define FIRE_BTN     12     // 🔥 Pulsador FIRE → masa
#define PERSON_BTN   13     // 🧍 Pulsador PERSON → masa
#define GO_BTN       14     // ▶️ Pulsador GO → masa
#define UART_TX      17     // TX hacia el dron (RPI_TX en el dron)
#define UART_RX      16     // RX desde el dron
#define LED_PIN       2     // LED interno del ESP32 DevKitV1

HardwareSerial SerialToDrone(2);

// =============================
// VARIABLES GLOBALES
// =============================
unsigned long lastDebounceFire = 0;
unsigned long lastDebouncePerson = 0;
unsigned long lastDebounceGo = 0;

bool lastFireState = HIGH;
bool lastPersonState = HIGH;
bool lastGoState = HIGH;

const unsigned long debounceDelay = 50;
unsigned long nextMsgCounter = 0;
String rxBuffer = "";

// =============================
// FUNCIONES AUXILIARES
// =============================

// 🔸 Función genérica de parpadeo LED
void blinkLedPattern(int blinks, int onTime = 150, int offTime = 150) {
  for (int i = 0; i < blinks; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(onTime);
    digitalWrite(LED_PIN, LOW);
    delay(offTime);
  }
}

// 🔸 Generar un ID incremental simple
String generateMsgID() {
  nextMsgCounter++;
  return String(nextMsgCounter);
}

// 🔸 Generar valor aleatorio de confianza (0.72–0.98)
float generateRandomConfidence() {
  // random(720, 980) devuelve enteros en ese rango → /1000 para escala [0–1]
  int raw = random(720, 981);
  return raw / 1000.0f;
}

// 🔸 Enviar mensaje de análisis (fire, person o go)
void sendAnalysisResult(const char* result) {
  StaticJsonDocument<128> doc;
  doc["t"] = result;        // "FIRE", "PERSON" o "GO"
  doc["ts"] = millis();

  // Solo FIRE y PERSON incluyen confianza
  if (strcmp(result, "FIRE") == 0 || strcmp(result, "PERSON") == 0) {
    doc["confidence"] = generateRandomConfidence();
  }

  String payload;
  serializeJson(doc, payload);

  SerialToDrone.println(payload);
  Serial.println("📤 Enviado → Dron (UART2): " + payload);
}

// 🔸 Procesar JSON recibido desde el dron
void processIncomingJSON(const String& jsonIn) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, jsonIn);
  if (err) {
    Serial.print("⚠️ JSON inválido recibido: ");
    Serial.println(err.c_str());
    return;
  }

  const char* type = doc["t"] | "";
  double lat = doc["lat"] | 0.0;
  double lon = doc["lon"] | 0.0;
  unsigned long ts = doc["ts"] | 0;

  if (strcmp(type, "STABLE") == 0) {
    Serial.println("📷 [STABLE] recibido del dron:");
    Serial.printf("   ↳ Lat: %.6f | Lon: %.6f | ts: %lu\n", lat, lon, ts);
    blinkLedPattern(3, 200, 200);
  } else {
    Serial.print("ℹ️ Tipo desconocido: ");
    Serial.println(type);
  }
}

// 🔸 Leer mensajes entrantes desde el dron
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
  pinMode(GO_BTN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  randomSeed(analogRead(0));  // inicializa generador de aleatorios

  Serial.println("🚀 Simulador de RPi listo. Pulsa:");
  Serial.println("   🔥 GPIO12 → FIRE");
  Serial.println("   🧍 GPIO13 → PERSON");
  Serial.println("   ▶️ GPIO14 → GO");
}

// =============================
// LOOP PRINCIPAL
// =============================
void loop() {
  bool fireState = digitalRead(FIRE_BTN);
  bool personState = digitalRead(PERSON_BTN);
  bool goState = digitalRead(GO_BTN);

  handleSerialInput();

  // FIRE pulsado
  if (fireState == LOW && lastFireState == HIGH && (millis() - lastDebounceFire) > debounceDelay) {
    lastDebounceFire = millis();
    Serial.println("🔥 FIRE activado");
    sendAnalysisResult("FIRE");
    blinkLedPattern(3, 100, 100);
  }

  // PERSON pulsado
  if (personState == LOW && lastPersonState == HIGH && (millis() - lastDebouncePerson) > debounceDelay) {
    lastDebouncePerson = millis();
    Serial.println("🧍 PERSON activado");
    sendAnalysisResult("PERSON");
    blinkLedPattern(2, 250, 250);
  }

  // GO pulsado
  if (goState == LOW && lastGoState == HIGH && (millis() - lastDebounceGo) > debounceDelay) {
    lastDebounceGo = millis();
    Serial.println("▶️ GO activado");
    sendAnalysisResult("GO");
    blinkLedPattern(1, 400, 150);
  }

  lastFireState = fireState;
  lastPersonState = personState;
  lastGoState = goState;
}
