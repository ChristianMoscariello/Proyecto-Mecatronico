#include <M5StickC.h>
#include <Wire.h>
#include "Adafruit_MLX90640.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ===== CONFIGURACIÓN =====
// Reemplaza con el nombre y contraseña de tu red WiFi
const char* ssid = "INTERNETLOCAL_9000";
const char* password = "37019105";

// Reemplaza con la IP de la PC que corre el script de Python
// IMPORTANTE: Usa la IP que te da el servidor de Flask (192.168.10.107)
const char* serverURL = "http://192.168.10.107:6001/recibir";

Adafruit_MLX90640 mlx;
float frame[32*24]; // Array para almacenar los 768 valores de temperatura

const int BLOCK_SIZE = 128;

void setup() {
  M5.begin();
  Serial.begin(115200);
  
  // Conexión a la red WiFi
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n¡Conectado a la red WiFi!");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());

  // ====================== CORRECCIÓN AÑADIDA ======================
  // Inicializar el bus I2C en los pines correctos para el HAT del M5StickC Plus (SDA=0, SCL=26)
  Wire.begin(0, 26);
  // ================================================================

  // Inicializar el sensor de cámara térmica MLX90640
  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("Error al iniciar el sensor MLX90640. Verifica las conexiones.");
    while (1);
  }
  Serial.println("Sensor MLX90640 inicializado correctamente.");

  mlx.setMode(MLX90640_INTERLEAVED);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX90640_2_HZ); 
}

void loop() {
  // Intentar leer un frame completo de temperaturas (768 valores)
  if (mlx.getFrame(frame) != 0) {
    Serial.println("Error al leer el frame del sensor.");
    delay(500);
    return;
  }

  // Enviar el frame completo en varios bloques
  int totalPixels = 32 * 24;
  int totalBlocks = (totalPixels + BLOCK_SIZE - 1) / BLOCK_SIZE;

  for (int b = 0; b < totalBlocks; b++) {
    int startIdx = b * BLOCK_SIZE;
    int endIdx = min(startIdx + BLOCK_SIZE, totalPixels);
    
    StaticJsonDocument<2048> doc;
    JsonArray arr = doc.createNestedArray("temperaturas");
    for (int i = startIdx; i < endIdx; i++) {
      arr.add(frame[i]);
    }
    doc["bloque"] = b;
    doc["total_bloques"] = totalBlocks;

    String jsonStr;
    serializeJson(doc, jsonStr);

    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(serverURL);
      http.addHeader("Content-Type", "application/json");
      
      int httpResponseCode = http.POST(jsonStr);
      
      if (httpResponseCode != 200) {
        Serial.printf("Error en la petición POST, código: %d\n", httpResponseCode);
      }
      
      http.end();
    } else {
      Serial.println("¡Se ha perdido la conexión WiFi!");
    }
    delay(50);
  }

  delay(1000);
}