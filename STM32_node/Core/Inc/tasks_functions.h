#ifndef TASKS_FUNCTIONS_H
#define TASKS_FUNCTIONS_H


#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/** Some define */
#define ADC_CH_NUM  3
#define LED_ON      0
#define LED_OFF     1

#define LED_PORT    GPIOC
#define LED1        GPIO_PIN_13
#define LED2        GPIO_PIN_14
#define LED1_BTN    GPIO_PIN_4
#define LED2_BTN    GPIO_PIN_5
#define LED3_BTN    GPIO_PIN_6

/** Tasks handler------------*/
extern TaskHandle_t ScreenUpdateHandler;
extern TaskHandle_t UARTUpdateHandler;
extern TaskHandle_t DeviceUpdateHandler;

/**  Semaphore---------------*/
extern SemaphoreHandle_t DeviceUpdateSemaphore;
extern SemaphoreHandle_t binSem;



/** Functions declaration ------*/
/**
 * Set up RTOS
 */
uint8_t setupRTOS(void);

/**
 * EXTI Callback
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/**
 * Screen Update Task Function
 */
void ScreenUpdate(void* parameter);

/**
 * UART Update Task Function
 */
void UARTUpdate(void* parameter);

/**
 * Device Update Task Function
 */
void DeviceUpdate(void* parameter);

#endif // TASKS_FUNCTIONS_H