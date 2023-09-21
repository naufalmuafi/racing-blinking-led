#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/semphr.h"

#define LED1_PIN 18
#define LED2_PIN 19
#define LED3_PIN 22
#define LED4_PIN 23

SemaphoreHandle_t xSemaphore;

void init_leds()
{
  gpio_set_direction(LED1_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED2_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED3_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED4_PIN, GPIO_MODE_OUTPUT);
}

void blink_led(void *param)
{
  int led_pin = (int)param;
  TickType_t delay = 1000; // Default delay: 1 second

  while (1)
  {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE)
    {
      // Toggle LED state
      gpio_set_level(led_pin, !gpio_get_level(led_pin));

      // Delay for the specified period
      vTaskDelay(delay / portTICK_PERIOD_MS);
    }
  }
}

void uart_task(void *param)
{
  uint8_t data;

  while (1)
  {
    if (uart_read_bytes(UART_NUM_0, &data, 1, 20 / portTICK_PERIOD_MS) > 0)
    {
      switch (data)
      {
      case '1':
        xSemaphoreGive(xSemaphore); // Enable LED blinking
        break;
      case '2':
        xSemaphoreTake(xSemaphore, portMAX_DELAY); // Disable LED blinking
        break;
      case '3':
        // Change LED blink period to 500ms
        vTaskSuspendAll();
        vTaskDelay(10 / portTICK_PERIOD_MS);
        xTaskResumeAll();
        break;
      case '4':
        // Change LED blink period to 1000ms
        vTaskSuspendAll();
        vTaskDelay(20 / portTICK_PERIOD_MS);
        xTaskResumeAll();
        break;
      default:
        break;
      }
    }
  }
}

void app_main()
{
  init_leds();

  // Create tasks for blinking LEDs
  xTaskCreate(blink_led, "LED1 Blink Task", 1024, (void *)LED1_PIN, 1, NULL);
  xTaskCreate(blink_led, "LED2 Blink Task", 1024, (void *)LED2_PIN, 1, NULL);
  xTaskCreate(blink_led, "LED3 Blink Task", 1024, (void *)LED3_PIN, 1, NULL);
  xTaskCreate(blink_led, "LED4 Blink Task", 1024, (void *)LED4_PIN, 1, NULL);

  // Create a task for UART communication
  xTaskCreate(uart_task, "UART Task", 1024 * 2, NULL, 2, NULL);

  // Initialize UART0
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  uart_param_config(UART_NUM_0, &uart_config);
  uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

  // Create a semaphore
  xSemaphore = xSemaphoreCreateBinary();
  if (xSemaphore == NULL)
  {
    printf("Semaphore creation failed\n");
  }

  // Start the tasks
  vTaskStartScheduler();
}
