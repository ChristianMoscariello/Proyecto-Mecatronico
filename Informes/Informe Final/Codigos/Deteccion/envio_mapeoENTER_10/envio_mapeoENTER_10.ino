/*
 * FIRMWARE M5STICK - MODO "PULL" (SOLICITUD) - VERSIÓN MEJORADA
 * 
 * Cambios respecto a la versión anterior:
 * - Solo envía JSON, sin mensajes de debug en medio
 * - Agrega delimitador de inicio/fin (#START / #END)
 * - Limpia el buffer serial antes de enviar
 * - Pantalla actualizada después de envío
 * 
 * LIBRERÍAS REQUERIDAS:
 * - M5StickC.h
 * - Wire.h
 * - Adafruit_MLX90640.h
 * - ArduinoJson.h
 */

#include <M5StickC.h>
#include <Wire.h>
#include "Adafruit_MLX90640.h"
#include <ArduinoJson.h>

// --- Variables Globales ---
Adafruit_MLX90640 mlx;
float frame[32 * 24]; // 768 píxeles

// ================== FUNCIÓN PARA ACTUALIZAR LA PANTALLA ==================
void mostrarMensaje(String mensaje, uint16_t colorFondo) {
  M5.Lcd.fillScreen(colorFondo);
  M5.Lcd.setCursor(10, 30);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_WHITE);
  M5.Lcd.print(mensaje);
}
// ========================================================================

void setup() {
  M5.begin();
  
  // Velocidad debe coincidir con VELOCIDAD_M5 en Python
  Serial.begin(115200); 

  M5.Lcd.setRotation(3);
  Wire.begin(0, 26); // Pines I2C del M5StickC

  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("ERROR: Sensor MLX90640 no encontrado");
    mostrarMensaje("ERROR SENSOR", TFT_RED);
    while (1);
  }
  
  // Configuración del sensor
  mlx.setMode(MLX90640_INTERLEAVED);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX90640_2_HZ);

  mostrarMensaje("Listo.\nEsperando\ncomando...", TFT_BLUE);
  delay(500);
}

// ================= FUNCIÓN DE CAPTURA Y ENVÍO =================
void getFrameAndSendJson() {
  // Mostrar que se está capturando
  mostrarMensaje("Capturando...", TFT_BLACK);
  
  // Leer el frame del sensor
  if (mlx.getFrame(frame) != 0) {
    mostrarMensaje("ERROR LECTURA", TFT_RED);
    delay(1000);
    mostrarMensaje("Listo.\nEsperando\ncomando...", TFT_BLUE);
    return;
  }

  // Crear el documento JSON
  StaticJsonDocument<8192> doc;
  JsonArray arr = doc.createNestedArray("temperaturas");
  for (int i = 0; i < 768; i++) {
    arr.add(round(frame[i] * 100) / 100.0);
  }

  // Serializar a String
  String jsonStr;
  serializeJson(doc, jsonStr);

  // IMPORTANTE: Limpiar buffer antes de enviar
  while (Serial.available() > 0) {
    Serial.read();
  }
  delay(50);

  // Enviar delimitador de inicio
  Serial.println("#START");
  
  // Enviar el JSON
  Serial.println(jsonStr);
  
  // Enviar delimitador de fin
  Serial.println("#END");

  // Mostrar que se envió correctamente
  mostrarMensaje("Enviado!\nEsperando...", TFT_DARKGREEN);
  delay(1000);
  mostrarMensaje("Listo.\nEsperando\ncomando...", TFT_BLUE);
}
// ================================================================

void loop() {
  // Comprobar si la Pi ha enviado datos
  if (Serial.available() > 0) {
    
    // Leer el comando completo hasta el salto de línea
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // Si el comando es el correcto
    if (cmd.equals("GET_FRAME")) {
      getFrameAndSendJson();
    } 
    // Ignorar otros comandos silenciosamente
  }
  
  delay(10); // Pequeño delay para no saturar el CPU
}