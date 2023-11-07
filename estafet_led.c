#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdio.h>

/* Define LED GPIO */
#define LED1 GPIO_NUM_5
#define LED2 GPIO_NUM_18
#define LED3 GPIO_NUM_19
#define LED4 GPIO_NUM_21

/* Define Push Button GPIO*/
#define PB1 GPIO_NUM_23
#define PB2 GPIO_NUM_33

#define ESP_INTR_FLAG_DEFAULT 0

#define GPIO_OUTPUT_PIN_SEL                                                    \
  (1ULL << LED1 | 1ULL << LED2 | 1ULL << LED3 | 1ULL << LED4)

#define GPIO_INPUT_PIN_SEL (1ULL << PB1 | 1ULL << PB2)

static uint8_t ledSelector = 0;
static QueueHandle_t gpio_evt_queue;
static QueueHandle_t gpio_evt2_queue;

static void IRAM_ATTR isr_handler(void *arg) {
  int pinNumber = (int)arg;
  xQueueSendFromISR(gpio_evt_queue, &pinNumber, NULL);
}

static void IRAM_ATTR isr2_handler(void *arg) {
  int pinNumber = (int)arg;
  xQueueSendFromISR(gpio_evt2_queue, &pinNumber, NULL);
}

void LED_PB2_TASK(void *params) {
  int pinNumber;
  static uint8_t input = 1;
  int prev_input;

  while (1) {
    if (xQueueReceive(gpio_evt2_queue, &pinNumber, portMAX_DELAY)) {
      vTaskDelay(100 / portTICK_PERIOD_MS); // wait for 100ms
      prev_input = gpio_get_level(PB2);

      if ((ledSelector != 0) && (input == prev_input)) {
        ledSelector--;
      }
    }
  }
}

void LED_PB1_TASK(void *params) {
  int pinNumber;
  static uint8_t input = 1;
  int prev_input;

  while (1) {
    if (xQueueReceive(gpio_evt_queue, &pinNumber, portMAX_DELAY)) {
      vTaskDelay(100 / portTICK_PERIOD_MS); // wait for 100ms
      prev_input = gpio_get_level(PB1);

      if ((ledSelector != 5) && (input == prev_input)) {
        ledSelector++;
      }
    }
  }
}

void app_main(void) {
  // Output configuration
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&io_conf);

  /* Input interrupt 1 configuration */
  io_conf.pin_bit_mask = (1ULL << PB1);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.intr_type = GPIO_INTR_POSEDGE;
  io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
  gpio_config(&io_conf);

  /* Input interrupt 2 configuration */
  io_conf.pin_bit_mask = (1ULL << PB2);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.intr_type = GPIO_INTR_POSEDGE;
  io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
  gpio_config(&io_conf);

  gpio_evt_queue = xQueueCreate(10, sizeof(int));
  gpio_evt2_queue = xQueueCreate(10, sizeof(int));

  xTaskCreate(LED_PB1_TASK, "PB1_TASK", 2048, NULL, 1, NULL);
  xTaskCreate(LED_PB2_TASK, "PB2_TASK", 2048, NULL, 1, NULL);
  // Sets up interrupt service
  ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
  ESP_ERROR_CHECK(gpio_isr_handler_add(PB1, isr_handler, (void *)PB1));
  ESP_ERROR_CHECK(gpio_isr_handler_add(PB2, isr2_handler, (void *)PB2));

  while (1) {
    gpio_set_level(LED1, (ledSelector % 5 == 1));
    gpio_set_level(LED2, (ledSelector % 5 == 2));
    gpio_set_level(LED3, (ledSelector % 5 == 3));
    gpio_set_level(LED4, (ledSelector % 5 == 4));
    vTaskDelay(10);
  }
}
