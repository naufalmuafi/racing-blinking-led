#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/semphr.h"

#define LED1_PIN 18
#define LED2_PIN 19
#define LED3_PIN 22
#define LED4_PIN 23

#define UART_DELAY_MS 100

SemaphoreHandle_t led1_mutex, led2_mutex, led3_mutex, led4_mutex;

uint16_t led1_period = 300,
         led2_period = 400,
         led3_period = 500,
         led4_period = 600;

float led1_duty = 0.5,
      led2_duty = 0.5,
      led3_duty = 0.5,
      led4_duty = 0.5;

void init_setup()
{
  // Initialize GPIO Pin
  esp_rom_gpio_pad_select_gpio(LED1_PIN);
  gpio_set_direction(LED1_PIN, GPIO_MODE_OUTPUT);
  esp_rom_gpio_pad_select_gpio(LED2_PIN);
  gpio_set_direction(LED2_PIN, GPIO_MODE_OUTPUT);
  esp_rom_gpio_pad_select_gpio(LED3_PIN);
  gpio_set_direction(LED3_PIN, GPIO_MODE_OUTPUT);
  esp_rom_gpio_pad_select_gpio(LED4_PIN);
  gpio_set_direction(LED4_PIN, GPIO_MODE_OUTPUT);

  // Initialize UART0
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  uart_param_config(UART_NUM_0, &uart_config);  
  uart_driver_install(UART_NUM_0, 1028 * 2, 1028 * 2, 10, NULL, 0);

  // Create Mutex
  led1_mutex = xSemaphoreCreateMutex();
  led2_mutex = xSemaphoreCreateMutex();
  led3_mutex = xSemaphoreCreateMutex();
  led4_mutex = xSemaphoreCreateMutex();  
}

void uart_task(void *pvParameters)
{
  uint8_t buffer[128], led_selected;
  uint32_t led_period;
  float led_duty;

  printf("===== LED CONFIGURATION =====");
  printf("[PIN 1-4],[PERIOD_IN_MS],[DUTY]");
  printf("Ex: 1,250,0.5");

  while (1)
  {
    int len = uart_read_bytes(UART_NUM_0, buffer, sizeof(buffer), pdMS_TO_TICKS(UART_DELAY_MS));
    if (len != 0)
    {      
      buffer[len] = '\0';
      
      char *token;
      token = strtok((char *)buffer, "/");

      // Receive the selected LED
      if (token != NULL)
      {
        led_selected = (uint8_t)atoi(token);
      }
      else
      {
        continue;
      }

      // Receive the LED period
      token = strtok(NULL, "/");
      if (token != NULL)
      {
        led_period = (uint32_t)atoi(token);
      }
      else
      {
        continue;
      }

      // Receive the LED duty
      token = strtok(NULL, "/");
      if (token != NULL)
      {
        led_duty = atof(token);
      }
      else
      {
        continue;
      }

      switch (led_selected)
      {
        case 1:
          xSemaphoreTake(led1_mutex, portMAX_DELAY);
          led1_period = led_period;
          led1_duty = led_duty;
          xSemaphoreGive(led1_mutex);

          printf("LED 1 Adjusted.");

          break;

        case 2:
          xSemaphoreTake(led2_mutex, portMAX_DELAY);
          led2_period = led_period;
          led2_duty = led_duty;
          xSemaphoreGive(led2_mutex);

          printf("LED 2 Adjusted.");

          break;
          
        case 3:
          xSemaphoreTake(led3_mutex, portMAX_DELAY);
          led3_period = led_period;
          led3_duty = led_duty;
          xSemaphoreGive(led3_mutex);

          printf("LED 3 Adjusted.");

          break;
          
        case 4:
          xSemaphoreTake(led4_mutex, portMAX_DELAY);
          led4_period = led_period;
          led4_duty = led_duty;
          xSemaphoreGive(led4_mutex);

          printf("LED 4 Adjusted.");

          break;
          
        default:
          break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(UART_DELAY_MS));
  }
}

// blink the LED
void blink_led(uint8_t pin, uint32_t period, float duty)
{
  gpio_set_level(pin, 0);
  vTaskDelay(pdMS_TO_TICKS((uint32_t)(((float)period) * duty)));
  gpio_set_level(pin, 1);
  vTaskDelay(pdMS_TO_TICKS((uint32_t)(((float)period) * (1.0 - duty))));
}

// LED 1 Task
void led1_task(void* pvParameter)
{
  uint32_t period;
  float duty;

  while(1)
  {
    xSemaphoreTake(led1_mutex, portMAX_DELAY);
    period = led1_period;
    duty = led1_duty;
    xSemaphoreGive(led1_mutex);
    
    blink_led(LED1_PIN, period, duty);
  }
}

// LED 2 Task
void led2_task(void* pvParameter)
{
  uint32_t period;
  float duty;

  while(1)
  {
    xSemaphoreTake(led2_mutex, portMAX_DELAY);
    period = led2_period;
    duty = led2_duty;
    xSemaphoreGive(led2_mutex);
    
    blink_led(LED2_PIN, period, duty);
  }
}

// LED 3 Task
void led3_task(void* pvParameter)
{
  uint32_t period;
  float duty;

  while(1)
  {
    xSemaphoreTake(led3_mutex, portMAX_DELAY);
    period = led3_period;
    duty = led3_duty;
    xSemaphoreGive(led3_mutex);
    
    blink_led(LED3_PIN, period, duty);
  }
}

// LED 4 Task
void led4_task(void* pvParameter)
{
  uint32_t period;
  float duty;

  while(1)
  {
    xSemaphoreTake(led4_mutex, portMAX_DELAY);
    period = led4_period;
    duty = led4_duty;
    xSemaphoreGive(led4_mutex);
    
    blink_led(LED4_PIN, period, duty);
  }
}

void app_main()
{
  init_setup();

  // Create a task for UART communication
  xTaskCreate(uart_task, "UART Task", 1024 * 2, NULL, 2, NULL);  

  // Create tasks for blinking LEDs
  xTaskCreate(led1_task, "LED1 Blink Task", 1024, NULL, 1, NULL);
  xTaskCreate(led2_task, "LED2 Blink Task", 1024, NULL, 1, NULL);
  xTaskCreate(led3_task, "LED3 Blink Task", 1024, NULL, 1, NULL);
  xTaskCreate(led4_task, "LED4 Blink Task", 1024, NULL, 1, NULL);  
}
