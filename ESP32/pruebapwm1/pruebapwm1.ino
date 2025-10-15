#include <Arduino.h>
#include <ESP32Servo.h>
#include "driver/rmt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

#define NUM_CHANNELS 5

// Entradas del receptor (PWM por canal)
static const int inputPins[NUM_CHANNELS]  = {32, 33, 34, 35, 21}; // 34-36: input-only
// Salidas que replican la señal
static const int outputPins[NUM_CHANNELS] = {25, 26, 27, 14, 12};

Servo outSrv[NUM_CHANNELS];
volatile uint16_t pulseUs[NUM_CHANNELS] = {1500,1500,500,1500,1500};

RingbufHandle_t rb[NUM_CHANNELS] = {nullptr};

// ---- RMT RX init para un canal ----
static void rmt_init_rx(int idx, gpio_num_t pin, rmt_channel_t ch) {
  rmt_config_t cfg = {};
  cfg.rmt_mode = RMT_MODE_RX;
  cfg.channel = ch;
  cfg.gpio_num = pin;
  cfg.clk_div = 80;                 // 80 MHz / 80 = 1 tick = 1 us
  cfg.mem_block_num = 1;
  cfg.rx_config.filter_en = true;   // filtra glitches
  cfg.rx_config.filter_ticks_thresh = 50;  // <50 us se descarta
  cfg.rx_config.idle_threshold = 3000;     // gap >3 ms = fin de pulso/frame
  ESP_ERROR_CHECK(rmt_config(&cfg));
  ESP_ERROR_CHECK(rmt_driver_install(ch, 2048, 0));
  ESP_ERROR_CHECK(rmt_get_ringbuf_handle(ch, &rb[idx]));
  ESP_ERROR_CHECK(rmt_rx_start(ch, true)); // empezar a recibir
}

// ---- Tarea que lee el ring buffer (no bloquea loop) ----
void rmt_reader_task(void *arg) {
  (void)arg;
  while (true) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      if (!rb[i]) continue;
      size_t len = 0;
      rmt_item32_t *items = (rmt_item32_t*) xRingbufferReceive(rb[i], &len, 0); // no bloquea
      if (items) {
        const int n = len / sizeof(rmt_item32_t);
        for (int j = 0; j < n; j++) {
          // Cada item tiene (level0,duration0) + (level1,duration1)
          // El pulso de servo suele ser HIGH ~1–2 ms, luego LOW.
          uint16_t width = 0;
          if (items[j].level0 == 1) width = items[j].duration0; else width = items[j].duration1; // ticks de 1 us
          if (width >= 900 && width <= 2100) {
            pulseUs[i] = width; // guarda el último valor válido
          }
        }
        vRingbufferReturnItem(rb[i], (void*)items);
      }
    }
    vTaskDelay(1); // cede CPU (~1 ms)
  }
}

void setup() {
  Serial.begin(115200);

  // Salidas (replica PWM tipo servo)
  for (int i = 0; i < NUM_CHANNELS; i++) {
    // attach(pin, minUs, maxUs) asegura rango típico RC
    outSrv[i].attach(outputPins[i], 1000, 2000);
  }

  // Entradas (RMT RX)
  for (int i = 0; i < NUM_CHANNELS; i++) {
    rmt_channel_t ch = (rmt_channel_t)(RMT_CHANNEL_0 + i);
    rmt_init_rx(i, (gpio_num_t)inputPins[i], ch);
  }

  // Tarea lectora en Core 0, prioridad alta
  xTaskCreatePinnedToCore(rmt_reader_task, "rmt_reader", 4096, nullptr, 20, nullptr, 0);
}

void loop() {
  // Replica en salidas (no bloquea)
  for (int i = 0; i < NUM_CHANNELS; i++) {
    outSrv[i].writeMicroseconds(pulseUs[i]);
  }
  // Tu lógica adicional puede ir aquí
  delay(2); // opcional para bajar carga
}
