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
TaskHandle_t SenSorsUpdateHandler;
TaskHandle_t DeviceUpdateHandler;
TaskHandle_t SendDataHandler;

/* Semaphore---------------*/
SemaphoreHandle_t DeviceUpdateSemaphore;
SemaphoreHandle_t binSem;

/* Queue-------------------*/
QueueHandle_t TxMsgQueue;

/* Mutex ------------------*/
SemaphoreHandle_t uartSemaphore;

/* ADC1 CH16 (built-in temperature sensor), ADC CH0, ADC CH1*/
volatile uint32_t adcValues[ADC_CH_NUM];

/* Temperature*/
float temp;

/* Devices' states*/
volatile uint8_t led1State = LED_OFF,
                 led2State = LED_OFF,
                 led3State = LED_OFF;

/* Buffers used for ssd1306 display*/
char line1Buffer[25];
char line2Buffer[25];
char line3Buffer[25];

/* Buffers used for UART transmission*/
char TxBuffer[20];

/* UART1 Handler*/
extern UART_HandleTypeDef huart1;

/* Functions definition */


/* RTOS Setup*/
uint8_t setupRTOS(void)
{
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  uartSemaphore = xSemaphoreCreateBinary();
  if(uartSemaphore == NULL)
    return 0;
  xSemaphoreGive(uartSemaphore);
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
  TxMsgQueue = xQueueCreate(5, sizeof(uint8_t) * 20);
  if(TxMsgQueue == NULL)
    return 0;
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  if(xTaskCreate(ScreenUpdate, "Screen",    256, NULL, 1, &ScreenUpdateHandler) != pdPASS)
    return 0;
  if(xTaskCreate(SensorsUpdate,"Sensor",    256, NULL, 2, &SenSorsUpdateHandler) != pdPASS)
    return 0;
  if(xTaskCreate(SendData,     "Send data", 256, NULL, 4, &SendDataHandler) != pdPASS)
    return 0;
  if(xTaskCreate(DeviceUpdate, "Device",    256, NULL, 6, &DeviceUpdateHandler) != pdPASS)
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
  else if(GPIO_Pin == LED3_BTN)
    led3State = (led3State == LED_ON) ? LED_OFF : LED_ON;

  if(xSemaphoreGiveFromISR(DeviceUpdateSemaphore, &xHigherPriorityTaskWoken) == pdTRUE)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* UART Rx Completion callback*/
/**
 * Give semaphore to ensure every transmission is properly completed
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if(xSemaphoreGiveFromISR(uartSemaphore, &xHigherPriorityTaskWoken) == pdTRUE)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* Task function definition ----------------------*/

/**
 * Sensor Update Task
 * Continously update sensors' values every 1 second
 */
void SensorsUpdate(void* parameter)
{
  while (1)
  {
    char tempBuffer[20];
    sprintf(tempBuffer, "%.2f/%ld/%ld\n", temp, adcValues[1], adcValues[2]);
    xQueueSend(TxMsgQueue, tempBuffer, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/**
 * Screen Update Task
 * Priority = 2
 * Update every 0.1 seconds
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
    // add update request to queue
    char tempBuffer[20];
    sprintf(tempBuffer, "L%d%d%d\n", led1State, led2State, led3State);
    xQueueSend(TxMsgQueue, tempBuffer, portMAX_DELAY);
  }
}

/**
 * UART Update Task
 * Send the msg in queue
 */
void SendData(void* parameter)
{
  char tempBuffer1[20];
  while(1)
  {
    xQueueReceive(TxMsgQueue, tempBuffer1, portMAX_DELAY);
    xSemaphoreTake(uartSemaphore, portMAX_DELAY);
    HAL_UART_Transmit_IT(&huart1, (uint8_t*)tempBuffer1, strlen(tempBuffer1));
  }
}
