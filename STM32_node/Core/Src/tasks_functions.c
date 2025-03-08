/**
 * tasks_function.c
 * 
 * include tasks' definitions and relevants
 */

#include "tasks_functions.h"

#include "ssd1306.h"

#include "stm32f1xx_hal_uart.h"

#include <string.h>
#include <stdio.h>

/* Tasks handler------------*/
TaskHandle_t ScreenUpdateHandler;
TaskHandle_t UARTUpdateHandler;
TaskHandle_t DeviceUpdateHandler;

/* Semaphore---------------*/
SemaphoreHandle_t DeviceUpdateSemaphore;
SemaphoreHandle_t binSem;

/* ADC1 CH16 (built-in temperature sensor), ADC CH0, ADC CH1*/
volatile uint32_t adcValues[ADC_CH_NUM];

/* Temperature*/
float temp;

/* Devices' states*/
volatile uint8_t led1State = LED_OFF,
                 led2State = LED_OFF,
                 led3State = LED_OFF;

/* Buffers used for ssd1306 display*/
char line1Buffer[50];
char line2Buffer[50];
char line3Buffer[50];

/* Buffers used for UART transmission*/
char uartBuffer[50];

/* UART1 Handler*/
extern UART_HandleTypeDef huart1;

/* Functions definition */


/* RTOS Setup*/
uint8_t setupRTOS(void)
{
     /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  DeviceUpdateSemaphore = xSemaphoreCreateCounting(2, 0);
  binSem = xSemaphoreCreateBinary();
  
  if(binSem == NULL)
    return 0;
  //led2Semaphore = xSemaphoreCreateBinary();

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  if(xTaskCreate(ScreenUpdate, "Screen", 256, NULL, 2, &ScreenUpdateHandler) != pdPASS)
    return 0;
  if(xTaskCreate(UARTUpdate,   "UART",   256, NULL, 3, &UARTUpdateHandler) != pdPASS)
    return 0;
  if(xTaskCreate(DeviceUpdate, "Device", 256, NULL, 6, &DeviceUpdateHandler) != pdPASS)
    return 0;

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
  return 1;
}

/* EXTI Callback ----------------------------------*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if(GPIO_Pin == LED1_BTN)
    led1State = (led1State == LED_ON) ? LED_OFF : LED_ON;
  else if(GPIO_Pin == LED2_BTN)
    led2State = (led2State == LED_ON) ? LED_OFF : LED_ON;

  if(xSemaphoreGiveFromISR(DeviceUpdateSemaphore, &xHigherPriorityTaskWoken) == pdTRUE)
  {
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

/* Task function definition ----------------------*/

/**
 * UART Update Task
 * Update every 1 second
 */
void UARTUpdate(void* parameter)
{
  while (1)
  {
    sprintf(uartBuffer, "%.2f/%ld/%ld/%d/%d/%d\n", temp, adcValues[1], adcValues[2], led1State, led2State, led3State);
    if (huart1.gState == HAL_UART_STATE_READY)
    {
      HAL_UART_Transmit_IT(&huart1, (uint8_t*)uartBuffer, strlen(uartBuffer));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/**
 * Screen Update Task
 * Priority = 2
 * Update every 0.5 seconds
 */
void ScreenUpdate(void* parameter)
{
  while (1)
  {
    float vSensor = (float)adcValues[0] * 3.3 / 4096;
    temp = ((1.43 - vSensor) / 0.0043) + 25;
    sprintf(line1Buffer, "Temp: %.1f  LED1: %s", temp, (led1State == LED_OFF) ? "off" : "on");
    sprintf(line2Buffer, "CH0 : %4ld  LED2: %s", adcValues[1], (led2State == LED_OFF) ? "off" : "on");
    sprintf(line3Buffer, "CH1 : %4ld  LED3: %s", adcValues[2], (led3State == LED_OFF) ? "off" : "on");
    while (hi2c1.State == HAL_I2C_STATE_BUSY);
    
    ssd1306_Fill(Black);
    ssd1306_SetCursor(1, 10);
    ssd1306_WriteString(line1Buffer, Font_6x8, White);

    ssd1306_SetCursor(1, 30);
    ssd1306_WriteString(line2Buffer, Font_6x8, White);

    ssd1306_SetCursor(1, 50);
    ssd1306_WriteString(line3Buffer, Font_6x8, White);
    ssd1306_UpdateScreen();

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/**
 * Device Update Task
 * Waiting for EXTI event to update device state
 */
void DeviceUpdate(void* parameter)
{
  //xSemaphoreTake(binSem, portMAX_DELAY);
  while (1)
  {
    xSemaphoreTake(DeviceUpdateSemaphore, portMAX_DELAY);
    HAL_GPIO_WritePin(LED_PORT, LED1, led1State);
    HAL_GPIO_WritePin(LED_PORT, LED2, led2State);
  }
}