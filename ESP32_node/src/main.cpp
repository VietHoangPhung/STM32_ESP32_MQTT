#include <Arduino.h>
#include <BluetoothSerial.h>
#include <FreeRTOSConfig.h>

#define LED_OFF     1
#define LED_ON      0

#define LED         2
#define BUTTON1     5
#define BUTTON2     18
#define BUTTON3     19

#define DEBOUNCE_DELAY 200 

HardwareSerial toSTM32Serial(2);

/* Tasks handlers */
TaskHandle_t ScreenUpdateHandler;
TaskHandle_t DeviceUpdateHandler;
TaskHandle_t SendDataHandler;
TaskHandle_t ReceiveDataHandler;
TaskHandle_t ButtonPollingHandler;

/* Rx message queue */
QueueHandle_t RxMsgQueue;
QueueHandle_t TxMsgQueue;

/* Semaphore */
SemaphoreHandle_t DeviceUpdateSemaphore;

/* Mutex for shared variables */
SemaphoreHandle_t xMutex;

/* Function prototypes */
uint8_t setupRTOS(void);
void ScreenUpdate(void* parameter);
void DeviceUpdate(void* parameter);
void SendData(void* parameter);
void ReceiveData(void* parameter);
void ButtonPolling(void* parameter);

/* Shared variables */
volatile uint8_t led1State = LED_OFF,
                 led2State = LED_OFF,
                 led3State = LED_OFF;

volatile uint32_t sensorsValues[3] = {0};

/* UART RX Buffer */
char rxData[32] = {0};
char rxBuffer[32];
uint8_t rxIndex;

/* Last sent LED state */
char lastSentLedState[4] = "000";

/* UART RX Interrupt Handler */
void IRAM_ATTR onSTM32Rx(void)
{
  while (toSTM32Serial.available())
  {
    uint8_t temp;
    toSTM32Serial.readBytes(&temp, 1);

    if (rxIndex < sizeof(rxBuffer) - 1)
    {
      rxBuffer[rxIndex++] = temp;
    }

    if (temp == '\n' || temp == '\0')
    {
      rxBuffer[rxIndex - 1] = '\0'; 
      strcpy(rxData, rxBuffer);

      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xQueueSendFromISR(RxMsgQueue, rxData, &xHigherPriorityTaskWoken);
      memset(rxBuffer, 0, sizeof(rxBuffer));
      rxIndex = 0;
      
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}

void setup() 
{
  Serial.begin(115200);

  // GPIO config
  pinMode(LED, OUTPUT);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);

  toSTM32Serial.begin(115200, SERIAL_8N1, 16, 17);
  toSTM32Serial.setRxFIFOFull(1);
  toSTM32Serial.onReceive(onSTM32Rx);

  if (!setupRTOS())
    Serial.println("Setting up RTOS failed");
  else
    Serial.println("RTOS OK");
}

void loop() 
{
}

/* RTOS Initialization */
uint8_t setupRTOS(void)
{
  /* Semaphores */
  DeviceUpdateSemaphore = xSemaphoreCreateCounting(2, 0);
  if (DeviceUpdateSemaphore == NULL)
    return 0;

  /* Message Queue */
  RxMsgQueue = xQueueCreate(4, sizeof(char) * 32);
  if (RxMsgQueue == NULL)
    return 0;

  TxMsgQueue = xQueueCreate(4, sizeof(char) * 5);
  if (TxMsgQueue == NULL)
    return 0;

  /* Mutex */
  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL)
    return 0;

  /* Create tasks */
  if (xTaskCreatePinnedToCore(ScreenUpdate, "Screen", 2048, NULL, 1, &ScreenUpdateHandler, 0) != pdPASS)
    return 0;

  if(xTaskCreatePinnedToCore(DeviceUpdate,  "Device", 1024, NULL, 6, &DeviceUpdateHandler, 0) != pdPASS)
    return 0;

  if (xTaskCreatePinnedToCore(ReceiveData, "Receive", 4096, NULL, 5, &ReceiveDataHandler, 1) != pdPASS)
    return 0;

  if (xTaskCreatePinnedToCore(SendData,     "Send",   4096, NULL, 4, &SendDataHandler, 1) != pdPASS)
    return 0;

  if (xTaskCreatePinnedToCore(ButtonPolling, "ButtonPoll", 4096, NULL, 2, &ButtonPollingHandler, 0) != pdPASS)
    return 0;

  return 1;
}

void ButtonPolling(void* parameter)
{
  TickType_t lastPress1 = 0, lastPress2 = 0, lastPress3 = 0;
  bool pressed = false;
  while(1)
  {
    TickType_t now = xTaskGetTickCount();
    
    if (digitalRead(BUTTON1) == LOW && (now - lastPress1 > pdMS_TO_TICKS(DEBOUNCE_DELAY)))
    {
      xSemaphoreTake(xMutex, portMAX_DELAY);
      led1State = !led1State;
      xSemaphoreGive(xMutex);
      lastPress1 = now;
      pressed = true;
    }

    if (digitalRead(BUTTON2) == LOW && (now - lastPress2 > pdMS_TO_TICKS(DEBOUNCE_DELAY)))
    {
      xSemaphoreTake(xMutex, portMAX_DELAY);
      led2State = !led2State;
      xSemaphoreGive(xMutex);
      lastPress2 = now;
      pressed = true;
    }

    if (digitalRead(BUTTON3) == LOW && (now - lastPress3 > pdMS_TO_TICKS(DEBOUNCE_DELAY)))
    {
      xSemaphoreTake(xMutex, portMAX_DELAY);
      led3State = !led3State;
      //digitalWrite(LED, led3State);
      xSemaphoreGive(xMutex);
      lastPress3 = now;
      pressed = true;
    }

    if(pressed)
      xSemaphoreGive(DeviceUpdateSemaphore);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/* Receive Data Task */
void ReceiveData(void* parameter)
{
  char tempBuffer[32];

  while (1)
  {
    if (xQueueReceive(RxMsgQueue, tempBuffer, portMAX_DELAY) == pdPASS)
    {
      // Update LED states
      if (tempBuffer[0] == 'L')
      {
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
        {
          led1State = tempBuffer[1] - '0';
          led2State = tempBuffer[2] - '0';
          led3State = tempBuffer[3] - '0';

          // Ensure valid states (0 or 1)
          led1State = (led1State == 0 || led1State == 1) ? led1State : 0;
          led2State = (led2State == 0 || led2State == 1) ? led2State : 0;
          led3State = (led3State == 0 || led3State == 1) ? led3State : 0;
          digitalWrite(LED, led3State);

          xSemaphoreGive(xMutex);
        }
      }
      // Update sensor values
      else if (tempBuffer[0] == 'S')
      {
        float sensor1;
        uint32_t sensor2, sensor3;

        if (sscanf(tempBuffer + 1, "%f/%u/%u", &sensor1, &sensor2, &sensor3) == 3)
        {
          if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
          {
            sensorsValues[0] = (uint32_t)(sensor1 * 100);
            sensorsValues[1] = sensor2;
            sensorsValues[2] = sensor3;

            xSemaphoreGive(xMutex);
          }
        }
      }
    }
  }
}

/* Screen Update Task */
void ScreenUpdate(void* parameter)
{
  while (1)
  {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
      Serial.printf("Temp: %2d.%1d || CH1 : %ld || CH2 : %ld\nLED1 : %s || LED2 : %s || LED3 : %s\n\n\n", 
                    sensorsValues[0] / 100, sensorsValues[0] % 100, 
                    sensorsValues[1],
                    sensorsValues[2],
                    (led1State == LED_OFF) ? "off" : "on",
                    (led2State == LED_OFF) ? "off" : "on",
                    (led3State == LED_OFF) ? "off" : "on");

      xSemaphoreGive(xMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void DeviceUpdate(void* parameter)
{
  while(1)
  {
    xSemaphoreTake(DeviceUpdateSemaphore, portMAX_DELAY);
    xSemaphoreTake(xMutex, portMAX_DELAY);

    digitalWrite(LED, led3State);
    char tempBuffer[4];

    snprintf(tempBuffer, sizeof(tempBuffer), "%d%d%d", led1State, led2State, led3State);

    // Check if the new state is different from the last sent state
    if (strcmp(tempBuffer, lastSentLedState) != 0) {
      strcpy(lastSentLedState, tempBuffer); // Update the last sent state
      xQueueSend(TxMsgQueue, tempBuffer, portMAX_DELAY);
    }

    xSemaphoreGive(xMutex);
  }
}

void SendData(void* parameter)
{
  char tempBuffer[4];
  while(1)
  {
    if (xQueueReceive(TxMsgQueue, tempBuffer, portMAX_DELAY) == pdPASS)
    {
      toSTM32Serial.write(tempBuffer, 3);
    }
  }
}