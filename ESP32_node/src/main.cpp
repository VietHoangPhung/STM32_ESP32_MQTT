#include <Arduino.h>
#include <FreeRTOSConfig.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
// #include <BluetoothSerial.h>

#define LED_OFF     1
#define LED_ON      0

#define LED         2
#define BUTTON1     5
#define BUTTON2     18
#define BUTTON3     19

#define DEBOUNCE_DELAY      200 
#define MQTT_UPDATE_DELAY   2000      // send data to mqtt server every 2 seconds

HardwareSerial toSTM32Serial(2);

WiFiClientSecure espClient;
PubSubClient client(espClient);

// BluetoothSerial SerialBT;

/* Tasks handlers */
TaskHandle_t ScreenUpdateHandler;
TaskHandle_t DeviceUpdateHandler;
TaskHandle_t SendDataHandler;
TaskHandle_t ReceiveDataHandler;
TaskHandle_t ButtonPollingHandler;
//TaskHandle_t BluetoothHandler;
TaskHandle_t MqttUpdateHandler;


/* Rx message queue */
QueueHandle_t RxMsgQueue;
QueueHandle_t TxMsgQueue;

/* Semaphore */
SemaphoreHandle_t DeviceUpdateSemaphore;

/* Mutex */
SemaphoreHandle_t xMutex;
SemaphoreHandle_t serialMutex;

/* Function prototypes */
uint8_t setupRTOS(void);
void callback(char* topic, byte* payload, unsigned int length);

/* Tasks ---------------*/
void ScreenUpdate(void* parameter);
void DeviceUpdate(void* parameter);
void SendData(void* parameter);
void ReceiveData(void* parameter);
void ButtonPolling(void* parameter);
//void Bluetooth(void* parameter);
void MqttUpdate(void* parameter);

// MQTT broker details
const char* mqttServer = "cd0ab787182f4d5c88b09e519a46143d.s1.eu.hivemq.cloud";  // Public broker for testing
const int mqttPort = 8883;

const char* mqttUsername = "esp32";
const char* mqttPassword = "Esp32123";

const char* mqttUpdateTopic = "esp32/update";

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
char lastSentLedState[6] = "L000\n";

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

  // Connect to Wi-Fi

  if (!setupRTOS())
    Serial.print("Setting up RTOS failed\n");
  else
    Serial.print("RTOS OK");
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

  TxMsgQueue = xQueueCreate(4, sizeof(char) * 6);
  if (TxMsgQueue == NULL)
    return 0;

  /* Mutex */
  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL)
    return 0;

  serialMutex = xSemaphoreCreateMutex();
  if (serialMutex == NULL)
    return 0;

  /* Create tasks */
  if (xTaskCreatePinnedToCore(ScreenUpdate, "Screen", 2048, NULL, 1, &ScreenUpdateHandler, 0) != pdPASS)
    return 0;

  if(xTaskCreatePinnedToCore(DeviceUpdate,  "Device", 2048, NULL, 6, &DeviceUpdateHandler, 0) != pdPASS)
    return 0;

  if (xTaskCreatePinnedToCore(ReceiveData, "Receive", 4096, NULL, 5, &ReceiveDataHandler, 1) != pdPASS)
    return 0;

  if (xTaskCreatePinnedToCore(SendData,     "Send",   2048, NULL, 4, &SendDataHandler, 1) != pdPASS)
    return 0;

  if (xTaskCreatePinnedToCore(ButtonPolling, "ButtonPoll", 1024, NULL, 2, &ButtonPollingHandler, 1) != pdPASS)
    return 0;

        // if (xTaskCreatePinnedToCore(Bluetooth, "Bluetooth", 10240, NULL, 3, &BluetoothHandler, 1) != pdPASS)
        //   return 0;

  if (xTaskCreatePinnedToCore(MqttUpdate, "Mqtt", 8192, NULL, 3, &MqttUpdateHandler, 0) != pdPASS)
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
      led1State ^= 1;
      xSemaphoreGive(xMutex);
      lastPress1 = now;
      pressed = true;
    }

    if (digitalRead(BUTTON2) == LOW && (now - lastPress2 > pdMS_TO_TICKS(DEBOUNCE_DELAY)))
    {
      xSemaphoreTake(xMutex, portMAX_DELAY);
      led2State ^= 1;
      xSemaphoreGive(xMutex);
      lastPress2 = now;
      pressed = true;
    }

    if (digitalRead(BUTTON3) == LOW && (now - lastPress3 > pdMS_TO_TICKS(DEBOUNCE_DELAY)))
    {
      xSemaphoreTake(xMutex, portMAX_DELAY);
      led3State ^= 1;
      //digitalWrite(LED, led3State);
      xSemaphoreGive(xMutex);
      lastPress3 = now;
      pressed = true;
    }

    if(pressed)
    {
      pressed = false;
      xSemaphoreGive(DeviceUpdateSemaphore);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/* Receive Data Task */
void ReceiveData(void* parameter)
{
  char tempBuffer[32];

  while (1)
  {
    xQueueReceive(RxMsgQueue, tempBuffer, portMAX_DELAY);
    
      // Update LED states
    if (tempBuffer[0] == 'L')
    {
      xSemaphoreTake(xMutex, portMAX_DELAY);
      led1State = tempBuffer[1] - '0';
      led2State = tempBuffer[2] - '0';
      led3State = tempBuffer[3] - '0';

      // Ensure valid states (0 or 1)
      led1State = (led1State == 0 || led1State == 1) ? led1State : 0;
      led2State = (led2State == 0 || led2State == 1) ? led2State : 0;
      led3State = (led3State == 0 || led3State == 1) ? led3State : 0;
      //digitalWrite(LED, led3State);

      xSemaphoreGive(xMutex);
      xSemaphoreGive(DeviceUpdateSemaphore);
      
    }
    // Update sensor values
    else if (tempBuffer[0] == 'S')
    {
      float sensor1;
      uint32_t sensor2, sensor3;

      if (sscanf(tempBuffer + 1, "%f/%u/%u", &sensor1, &sensor2, &sensor3) == 3)
      {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        
        sensorsValues[0] = (uint32_t)(sensor1 * 100);
        sensorsValues[1] = sensor2;
        sensorsValues[2] = sensor3;

        xSemaphoreGive(xMutex);
      }
    }
    // Wake-up msg, request current devices states
    else if (tempBuffer[0] == 'W')
    {
      char newTx[6];
      xSemaphoreTake(xMutex, portMAX_DELAY);
      snprintf(newTx, sizeof(newTx), "L%d%d%d\n", led1State, led2State, led3State);
      xSemaphoreGive(xMutex);
      xQueueSend(TxMsgQueue, newTx, portMAX_DELAY);
    }
  }
}

            // void Bluetooth(void* parameter)
            // {
            //   SerialBT.begin("ESP32", 1);
            //   TickType_t lastPrint = 0, now = 0;
            //   char tempBuffer[32];
            //   while(1)
            //   {
            //     if(SerialBT.hasClient())
            //     {
            //       if(SerialBT.available())
            //       {
            //         int len = SerialBT.readBytesUntil('\n', tempBuffer, sizeof(tempBuffer) - 1);
            //         tempBuffer[len] = '\0'; 

            //         xQueueSend(RxMsgQueue, tempBuffer, portMAX_DELAY);
            //       }

            //       now = xTaskGetTickCount();
            //       if((now - lastPrint) > pdMS_TO_TICKS(1000))
            //       {
            //         xSemaphoreTake(serialMutex, portMAX_DELAY);
            //         SerialBT.printf("L%d%d%dS%2d.%1d/%ld/%ld\n",
            //                           led1State,
            //                           led2State,
            //                           led3State,
            //                           sensorsValues[0] / 100, sensorsValues[0] % 100, 
            //                           sensorsValues[1],
            //                           sensorsValues[2]);
            //         xSemaphoreGive(serialMutex);
            //       }
            //     }
            //     vTaskDelay(pdMS_TO_TICKS(100));
            //   }
            // }

/* Screen Update Task */
void ScreenUpdate(void* parameter)
{
  while (1)
  {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.printf("Temp: %2d.%1d || CH1 : %ld || CH2 : %ld\nLED1 : %d || LED2 : %d || LED3 : %d\n\n\n", 
                  sensorsValues[0] / 100, sensorsValues[0] % 100, 
                  sensorsValues[1],
                  sensorsValues[2],
                  led1State,
                  led2State,
                  led3State);

    xSemaphoreGive(serialMutex);

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
    char tempBuffer[6];

    snprintf(tempBuffer, sizeof(tempBuffer), "L%d%d%d\n", led1State, led2State, led3State);

    if (strcmp(tempBuffer, lastSentLedState) != 0) 
    {
      xQueueSend(TxMsgQueue, tempBuffer, portMAX_DELAY);
    }

    xSemaphoreGive(xMutex);
  }
}

void SendData(void* parameter)
{
  // Send wake-up notigy to request current devices states
  toSTM32Serial.write("W\n\n\n\n", 5);
  char tempBuffer[6];
  while(1)
  {
    xQueueReceive(TxMsgQueue, tempBuffer, portMAX_DELAY);
    strcpy(lastSentLedState, tempBuffer);
    toSTM32Serial.write(tempBuffer, 5);
  }
}

void MqttUpdate(void* parameter)
{
  uint64_t lastUpdate = 0;
  char mqttTxMsg[80];
  WiFi.begin("viethoang-2.4GHz", "12345679");
  while(1)
  {
    if(WiFi.status() == WL_CONNECTED)     // If wifi connected
    {
      if(!client.connected())             // If client not connected
      {
        espClient.setInsecure();
        client.setServer(mqttServer, mqttPort);
        client.setCallback(callback);
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client", mqttUsername, mqttPassword)) 
        {
          Serial.println("connected");
          client.subscribe("esp32/test");
        } 
        else 
        {
          Serial.print("failed, rc=");
          Serial.print(client.state());
          Serial.println(" try again in 5 seconds");
          vTaskDelay(pdMS_TO_TICKS(5000));  // Delay before retrying
        }
      }
      else
      {
        client.loop();
        uint64_t now = millis();
        if((now - lastUpdate) > MQTT_UPDATE_DELAY)
        {
          snprintf(mqttTxMsg, sizeof(mqttTxMsg), "Temp: %2d.%1d || CH1 : %ld || CH2 : %ld ||| LED1 : %s || LED2 : %s || LED3 : %s", 
          sensorsValues[0] / 100, sensorsValues[0] % 100, 
          sensorsValues[1],
          sensorsValues[2],
          (led1State == LED_OFF) ? "off" : "on",
          (led2State == LED_OFF) ? "off" : "on",
          (led3State == LED_OFF) ? "off" : "on");

          client.publish(mqttUpdateTopic, mqttTxMsg, true);
          lastUpdate = now;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
      }
    }
    else
    {
      WiFi.begin("viethoang-2.4GHz", "12345679");
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  if (strcmp(topic, "esp32/test") == 0)
  {
    char tempBuffer[length + 1];
    memcpy(tempBuffer, payload, length);
    tempBuffer[length] = '\0';

    xQueueSend(RxMsgQueue, tempBuffer, portMAX_DELAY);
  }
  
}