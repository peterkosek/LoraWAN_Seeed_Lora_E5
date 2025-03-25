#pragma once
/*
 * sensors_kosek.h
 *
 *  Created on: Mar 19, 2025
 *      Author: pkose
 */

#ifndef APPLICATION_USER_CORE_SENSORS_KOSEK_H_
#define APPLICATION_USER_CORE_SENSORS_KOSEK_H_

#include <stdint.h>
#include "app_lorawan.h"
#include "float.h"
#include "stm32wlxx_hal.h"
#include "stm32wlxx_hal_i2c.h"
#include "stm32wlxx_ll_usart.h"

/*
 * port definition (appPort) as below in the lora_app.c file.
 * se_identity.h : DEVEUI, APPKEY and JOINEUI
 * change REGION to US915 and BAND if needed:
 * in sys.conf.h :  Enable debugger if needed <line 61>
 *
 * 	5:  Heltec cubeCell AB,2 one soil sensor
 * 	6:  Seeed Lora-E5, two soil one air sensor
 * 	10: Seeed S1000 weather station
 *
 *
 */

#define USART2_Tx_nRx_Port 	GPIOB
#define USART2_Tx_nRx_Pin 	GPIO_PIN_4
#define PWR_33_Port			GPIOA
#define PWR_33_Pin			GPIO_PIN_9
#define PWR_5V_Port			GPIOB
#define PWR_5V_Pin			GPIO_PIN_10
#define LED_Port			GPIOB
#define LED_Pin				GPIO_PIN_5
#define USART2_Tx_Port		GPIOA
#define USART2_Tx_Pin		GPIO_PIN_2
#define USART2_Rx_Port		GPIOA
#define USART2_Rx_Pin		GPIO_PIN_3
#define SDA2_Port			GPIOA
#define SDA2_Pin			GPIO_PIN_15
#define SCL2_Port			GPIOB
#define SCL2_Pin			GPIO_PIN_15
#define USART_Rx_Pin 		GPIO_PIN_6
#define USART_Rx_Port 		GPIOB
#define USART_Tx_Pin 		GPIO_PIN_7
#define USART_Tx_Port 		GPIOB


extern uint8_t sTempC[4]; // shallow and then deep sensors, MSB then LSB then MSB LSB
extern uint8_t sMoist[4]; //
extern uint8_t aTempC[2]; // for the SHT sensor data MSB LSB (16 bit data uint16_t when two bytes combined)
extern uint8_t aMoist[2];
extern float realAirTemp;
extern float realAirMoist;
extern uint8_t aRxBuffer[100];

extern uint8_t batteryLevel;
extern uint8_t startFast;

extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

void GPIO_Init(void);
void I2C2_Init(void);
void USARTs_Init(void);
void PwrCtl(uint8_t set_reset);
void RS485Get(void);
void RS485Sub(uint8_t depth);
void AirTnHGet(void);

#endif /* APPLICATION_USER_CORE_SENSORS_KOSEK_H_ */


