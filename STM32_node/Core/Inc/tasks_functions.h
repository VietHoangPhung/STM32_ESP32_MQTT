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
#define LED3        GPIO_PIN_15

#define LED1_BTN    GPIO_PIN_4
#define LED2_BTN    GPIO_PIN_5
#define LED3_BTN    GPIO_PIN_6

/* UART Rx buffer */
extern char RxBuffer[3];


/** Functions declaration ------*/
/**
 * Set up RTOS
 */
uint8_t setupRTOS(void);

/**
 * Screen Update Task Function
 */
void ScreenUpdate(void* parameter);

/**
 * Sensor Update Task Function
 */
void SensorsUpdate(void* parameter);

/**
 * Device Update Task Function
 */
void DeviceUpdate(void* parameter);

/**
 * UART Tx Task Function
 */
void SendData(void* parameter);

/** 
 * UART Rx Task Function
 */
void ReceiveData(void* parameter);

#endif // TASKS_FUNCTIONS_H