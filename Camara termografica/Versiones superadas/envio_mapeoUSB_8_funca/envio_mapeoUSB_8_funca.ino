#include <M5StickC.h>
#include <Wire.h>
#include "Adafruit_MLX90640.h"
#include <ArduinoJson.h>

// ================== VARIABLE CONFIGURABLE ==================
const unsigned long INTERVALO_CAPTURA_MS = 8000; // 8 segundos
// ===========================================================

Adafruit_MLX90640 mlx;
float frame[32 * 24]; // 768 píxeles

// Variables para control de tiempo y pausa
unsigned long ultimo_envio = 0;
bool envio_activo = true; // Controla si el envío está activo o pausado

// ================== FUNCIÓN PARA ACTUALIZAR LA PANTALLA ==================
void mostrarMensaje(String mensaje, uint16_t colorFondo) { // <-- AÑADIDO
  M5.Lcd.fillScreen(colorFondo);
  M5.Lcd.setCursor(10, 40);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_WHITE);
  M5.Lcd.print(mensaje);
}
// ========================================================================

void setup() {
  M5.begin();
  Serial.begin(115200);

  M5.Lcd.setRotation(3); // <-- AÑADIDO: Orienta la pantalla correctamente.

  Wire.begin(0, 26);

  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("Error al iniciar el sensor MLX90640.");
    mostrarMensaje("ERROR SENSOR", TFT_RED); // <-- AÑADIDO: Muestra error en pantalla.
    while (1);
  }
  Serial.println("Sensor inicializado. Presiona el botón A para pausar/reanudar.");
  mostrarMensaje("Iniciando...", TFT_BLUE); // <-- AÑADIDO: Mensaje inicial.

  mlx.setMode(MLX90640_INTERLEAVED);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX90640_2_HZ);
}

void loop() {
  M5.update(); // Actualiza el estado de los botones

  // Si se presiona el botón A, se cambia el estado de envío
  if (M5.BtnA.wasPressed()) {
    envio_activo = !envio_activo;
    if (envio_activo) {
      Serial.println(">>> Envío REANUDADO <<<");
      mostrarMensaje("Reanudado", TFT_DARKCYAN); // <-- AÑADIDO
      ultimo_envio = 0;
    } else {
      Serial.println(">>> Envío PAUSADO <<<");
      mostrarMensaje("Pausado", TFT_ORANGE); // <-- AÑADIDO
    }
  }

  // Se comprueba si el envío está activo Y si ha pasado el tiempo suficiente
  if (envio_activo && (millis() - ultimo_envio >= INTERVALO_CAPTURA_MS)) {
    ultimo_envio = millis();

    mostrarMensaje("Capturando...", TFT_BLACK); // <-- AÑADIDO: Muestra que está trabajando.

    if (mlx.getFrame(frame) != 0) {
      Serial.println("Error al leer el frame del sensor.");
      mostrarMensaje("ERROR LECTURA", TFT_RED); // <-- AÑADIDO
      return;
    }

    // Cambiamos el mensaje para indicar que los datos se están enviando
    mostrarMensaje("Enviando...", TFT_PURPLE); // <-- AÑADIDO

    StaticJsonDocument<8192> doc;
    JsonArray arr = doc.createNestedArray("temperaturas");
    for (int i = 0; i < 768; i++) {
      arr.add(round(frame[i] * 100) / 100.0);
    }

    String jsonStr;
    serializeJson(doc, jsonStr);

    // Enviar la cadena JSON por el puerto serie (USB)
    Serial.println(jsonStr);

    // Tras enviar, mostramos un mensaje de confirmación en verde
    mostrarMensaje("Enviado OK!", TFT_DARKGREEN); // <-- AÑADIDO
  }
}