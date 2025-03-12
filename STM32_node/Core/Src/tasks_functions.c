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

/* Some constants ----------*/
#define ADC_MAX_VALUE 4096      // 12-bit resolution
#define VREF 3300               // in millivolts
#define TEMP_SENSOR_V25 1430    // in millivolts
#define TEMP_SENSOR_SLOPE 43    // in millivolts per degree Celsius

/* Tasks handler------------*/
TaskHandle_t ScreenUpdateHandler;
TaskHandle_t SenSorsUpdateHandler;
TaskHandle_t DeviceUpdateHandler;
TaskHandle_t SendDataHandler;
TaskHandle_t ReceiveDataHandler;

/* Semaphore---------------*/
SemaphoreHandle_t DeviceUpdateSemaphore;
SemaphoreHandle_t UartSemaphore;

/* Mutex ------------------*/
SemaphoreHandle_t LedMutex;

/* Queue-------------------*/
QueueHandle_t TxMsgQueue;
QueueHandle_t RxMsgQueue;

/* ADC1 CH16 (built-in temperature sensor), ADC CH0, ADC CH1*/
volatile uint32_t adcValues[ADC_CH_NUM];

/* Devices' states*/
volatile uint8_t led1State = LED_OFF,
                 led2State = LED_OFF,
                 led3State = LED_OFF;
              
/* temperature * 1000 */
volatile uint32_t temp;

/* Buffers used for ssd1306 display*/
char line1Buffer[26];
char line2Buffer[26];
char line3Buffer[26];

/* Buffers used for UART transmission*/
char TxBuffer[20];
char RxBuffer[3];

/* UART1 Handler*/
extern UART_HandleTypeDef huart1;

/* Functions definition */


/* RTOS Setup*/
uint8_t setupRTOS(void)
{
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  LedMutex = xSemaphoreCreateMutex();
  if(LedMutex == NULL)
    return 0;
  
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  UartSemaphore = xSemaphoreCreateBinary();
  if(UartSemaphore == NULL)
    return 0;
  xSemaphoreGive(UartSemaphore);

  DeviceUpdateSemaphore = xSemaphoreCreateCounting(2, 0);
  if(DeviceUpdateSemaphore == NULL)
    return 0;
  //led2Semaphore = xSemaphoreCreateBinary();

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  TxMsgQueue = xQueueCreate(4, sizeof(uint8_t) * 20);
  if(TxMsgQueue == NULL)
    return 0;

  RxMsgQueue = xQueueCreate(4, sizeof(uint8_t) * 4);
  if(RxMsgQueue == NULL)
    return 0;
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  if(xTaskCreate(ScreenUpdate, "Screen",    128, NULL, 1, &ScreenUpdateHandler) != pdPASS)
    return 0;
  if(xTaskCreate(SensorsUpdate,"Sensor",    128, NULL, 2, &SenSorsUpdateHandler) != pdPASS)
    return 0;
  if(xTaskCreate(SendData,     "Send",      256, NULL, 4, &SendDataHandler) != pdPASS)
    return 0;
  if(xTaskCreate(ReceiveData, "Receieve",  256, NULL, 5, &ReceiveDataHandler) != pdPASS)
    return 0;
  if(xTaskCreate(DeviceUpdate, "Device",    128, NULL, 6, &DeviceUpdateHandler) != pdPASS)
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

  // Update leds' states
  if(GPIO_Pin == LED1_BTN)
    led1State = (led1State == LED_ON) ? LED_OFF : LED_ON;
  else if(GPIO_Pin == LED2_BTN)
    led2State = (led2State == LED_ON) ? LED_OFF : LED_ON;
  else if(GPIO_Pin == LED3_BTN)
    led3State = (led3State == LED_ON) ? LED_OFF : LED_ON;

  if(xSemaphoreGiveFromISR(DeviceUpdateSemaphore, &xHigherPriorityTaskWoken) == pdTRUE)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* UART Tx Completion callback*/
/**
 * Give semaphore to ensure every transmission is properly completed
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Allow other tasks to use uart Tx by giving UartSemaphore
  if(xSemaphoreGiveFromISR(UartSemaphore, &xHigherPriorityTaskWoken) == pdTRUE)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* UART Rx Completion callback*/
/**
 * Receive string from UART Rx
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Send a copy of RxBuffer to RxMsgQueue avoid data lost if RxBuffer
  uint8_t tempBuffer[3];
  memcpy(tempBuffer, RxBuffer, sizeof(tempBuffer));

  if(xQueueSendFromISR(RxMsgQueue, tempBuffer, &xHigherPriorityTaskWoken) == pdTRUE)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/* Task function definition ----------------------*/

/**
 * Device Update Task
 * Priority = 6
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
    HAL_GPIO_WritePin(LED_PORT, LED3, led3State);

    char tempBuffer[6];
    snprintf(tempBuffer, sizeof(tempBuffer), "L%1u%1u%1u\n", led1State, led2State, led3State);

    xQueueSend(TxMsgQueue, tempBuffer, portMAX_DELAY);
  }
}

/**
 * UART Receive Task
 * Priority = 5
 * Receive command from UART Rx
 */
void ReceiveData(void* parameter)
{
  char tempBuffer[3]; 
  while(1)
  {
    xQueueReceive(RxMsgQueue, tempBuffer, portMAX_DELAY);

    uint8_t temp1 = 0, temp2 = 1, temp3 = 2;
    uint8_t scanned = sscanf(tempBuffer, "%1c%1c%1c", &temp1, &temp2, &temp3);

    xSemaphoreTake(LedMutex, portMAX_DELAY);
    led1State = tempBuffer[0] - '0';
    led2State = tempBuffer[1] - '0';  
    led3State = tempBuffer[2] - '0';
    xSemaphoreGive(LedMutex);

    xSemaphoreGive(DeviceUpdateSemaphore);
  }
}


 /**
 * UART Update Task
 * Priority = 4
 * Send the msg in queue
 */
void SendData(void* parameter)
{
  char tempBuffer[20];
  while(1)
  {
    xQueueReceive(TxMsgQueue, tempBuffer, portMAX_DELAY);
    xSemaphoreTake(UartSemaphore, portMAX_DELAY);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)tempBuffer, strlen(tempBuffer));
  }
}

/**
 * Sensor Update Task
 * Priority = 2
 * Continously update sensors' values every 1 second
 */
void SensorsUpdate(void* parameter)
{
  const TickType_t delayInTicks = pdMS_TO_TICKS(1000);
  while (1)
  {
    int32_t vSensor = (adcValues[0] * 3300) / 4096;
    temp = ((1430 - vSensor) * 1000) / 43 + 25000;
    int32_t tempIntegerPart = temp / 1000;
    int32_t tempFractionalPart = (temp % 1000) / 100;

    char tempBuffer[20];
    snprintf(tempBuffer, sizeof(tempBuffer), "%2ld.%1ld/%ld/%ld\n", tempIntegerPart, tempFractionalPart, adcValues[1], adcValues[2]);

    xQueueSend(TxMsgQueue, tempBuffer, portMAX_DELAY);
    vTaskDelay(delayInTicks);
  }
}
 
/**
 * Screen Update Task
 * Priority = 1
 * Update every 0.1 seconds
 */
void ScreenUpdate(void* parameter)
{
  const TickType_t delayInTicks = pdMS_TO_TICKS(100);
  while (1)
  {
    int32_t tempIntegerPart = temp / 1000;
    int32_t tempFractionalPart = (temp % 1000) / 100; 

    sprintf(line1Buffer, "Temp: %2ld.%1ld  LED1: %s", tempIntegerPart, tempFractionalPart, (led1State == LED_OFF) ? "off" : "on");
    sprintf(line2Buffer, "CH0 : %4ld  LED2: %s", adcValues[1], (led2State == LED_OFF) ? "off" : "on");
    sprintf(line3Buffer, "CH1 : %4ld  LED3: %s", adcValues[2], (led3State == LED_OFF) ? "off" : "on");
    
    if (hi2c1.State == HAL_I2C_STATE_READY)
    {
      ssd1306_Fill(Black);
      ssd1306_SetCursor(1, 10);
      ssd1306_WriteString(line1Buffer, Font_6x8, White);

      ssd1306_SetCursor(1, 30);
      ssd1306_WriteString(line2Buffer, Font_6x8, White);

      ssd1306_SetCursor(1, 50);
      ssd1306_WriteString(line3Buffer, Font_6x8, White);
      ssd1306_UpdateScreen();
    }
    vTaskDelay(delayInTicks);
  }
}
