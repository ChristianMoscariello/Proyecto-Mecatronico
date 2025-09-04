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

void setup() {
  M5.begin();
  Serial.begin(115200);

  Wire.begin(0, 26);

  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("Error al iniciar el sensor MLX90640.");
    while (1);
  }
  Serial.println("Sensor inicializado. Presiona el botón A para pausar/reanudar.");

  mlx.setMode(MLX90640_INTERLEAVED);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX90640_2_HZ);
}

void loop() {
  M5.update(); // Actualiza el estado de los botones

  // Si se presiona el botón A, se cambia el estado de envío
  if (M5.BtnA.wasPressed()) {
    envio_activo = !envio_activo; // Invierte el valor (true -> false, false -> true)
    if (envio_activo) {
      Serial.println(">>> Envío REANUDADO <<<");
      // Forzamos un envío inmediato en el siguiente ciclo
      ultimo_envio = 0;
    } else {
      Serial.println(">>> Envío PAUSADO <<<");
    }
  }

  // Se comprueba si el envío está activo Y si ha pasado el tiempo suficiente
  if (envio_activo && (millis() - ultimo_envio >= INTERVALO_CAPTURA_MS)) {
    ultimo_envio = millis();

    if (mlx.getFrame(frame) != 0) {
      Serial.println("Error al leer el frame del sensor.");
      return;
    }

    StaticJsonDocument<8192> doc;
    JsonArray arr = doc.createNestedArray("temperaturas");
    for (int i = 0; i < 768; i++) {
      arr.add(round(frame[i] * 100) / 100.0);
    }

    String jsonStr;
    serializeJson(doc, jsonStr);

    // Enviar la cadena JSON por el puerto serie (USB)
    Serial.println(jsonStr);
  }
}