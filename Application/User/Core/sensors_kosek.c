/*
 * sensors_kosek.c
 *
 *  Created on: Mar 19, 2025
 *      Author: pkose
 */


#include "sensors_kosek.h"
#include "stm32wlxx_hal.h"
#include "stm32wlxx_hal_i2c.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

uint8_t sTempC[4]; // shallow and then deep sensors, MSB then LSB then MSB LSB
uint8_t sMoist[4]; //
uint8_t aTempC[2]; // for the SHT sensor data MSB LSB (16 bit data uint16_t when tow bytes combined)
uint8_t aMoist[2];
float realAirTemp;
float realAirMoist;
//  buffer for RS485 transmission
uint8_t aTxBuffer0[8] = {0x01, 0x03, 0x00, 0x01, 0x00, 0x02, 0x95, 0xcb}; // first soil moisture sensor message
uint8_t aTxBuffer1[8] = {0x02, 0x03, 0x00, 0x01, 0x00, 0x02, 0x95, 0xf8};  //  second soil moisture message
/* Buffer used for rs485 reception */
uint8_t aRxBuffer[100];

uint8_t TryCount = 0;
uint8_t testnValid_1;
uint8_t testnValid_2;
uint8_t i2cTrace = 0;
uint8_t SHTaddr = 0x44;
uint8_t SHT31Tx[2] = {0x2c, 0x06};  //  bytes to send to request data from SHT31
uint8_t SHT31Rx[6]; 				//  for the 6 bytes of data, bytes 3 and 6 are crc codes, format is MSB LSB CRC MSN LSB CRC, temp first
uint8_t SHT31HeatOn[] = {0x03, 0x6d};  //  turn  on the internal sensor heater for a few degrees of warmth
uint8_t SHT31Reset[] = {0x00, 0x06};

uint8_t batteryLevel;
uint8_t startFast = 10;		//  cycles to run fast at startup, then every three hrs by default


I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart1;


void RS485Get(void)
{

		HAL_Delay(1500);

		//HAL_UART_Transmit(&huart2, aTxBuffer, ubNbDataToTransmit, 25);  //  first transmissions is not the best data, send twice
		//first three char from the RS485 should be 0x01, 0x03, 0x04.  Try up to 10 times to get those... otherwise send 0s.


        TryCount = 0;
        testnValid_1 = 0;
        testnValid_2 = 0;
		do {

			RS485Sub(0);  // shallow probe
			HAL_Delay(200);
			TryCount++;
			if (((aRxBuffer[0] == 0x01) && (aRxBuffer[1] == 0x03) && (aRxBuffer[2] = 0x04)))
				{testnValid_1++;}
			if (((aRxBuffer[1] == 0x01) && (aRxBuffer[2] == 0x03) && (aRxBuffer[3] = 0x04)))
				{
				testnValid_2++;
				}  //  left shift as first element is invalid
			} while (((testnValid_1 | testnValid_2) < 2) && (TryCount < 10));
		if (testnValid_1)
		{
				sTempC[0] = (uint8_t)(aRxBuffer[3]);
				sTempC[1] = (uint8_t)(aRxBuffer[4]);
			    sMoist[0] = (uint8_t)(aRxBuffer[5]);
			    sMoist[1] = (uint8_t)(aRxBuffer[6]);
		}
		else if (testnValid_2)
		{
						sTempC[0] = (uint8_t)(aRxBuffer[4]);
						sTempC[1] = (uint8_t)(aRxBuffer[5]);
					    sMoist[0] = (uint8_t)(aRxBuffer[6]);
					    sMoist[1] = (uint8_t)(aRxBuffer[7]);
				}

			    for (int i=0;i<10;i++){aRxBuffer[i] = 0;}

		HAL_Delay(250);
		TryCount = 0;
		testnValid_1 = 0;
		testnValid_2 = 0;
		do {

			RS485Sub(1);  // deep probe
			HAL_Delay(200);
			TryCount++;
			if (((aRxBuffer[0] == 0x02) && (aRxBuffer[1] == 0x03) && (aRxBuffer[2] = 0x04)))
				{testnValid_1++;}
			if (((aRxBuffer[1] == 0x02) && (aRxBuffer[2] == 0x03) && (aRxBuffer[3] = 0x04)))
				{testnValid_2++;}  // frame shift is common, do not know why
			} while (((testnValid_1 | testnValid_2) < 2) && (TryCount < 10));
		if (testnValid_1)
		{
				sTempC[2] = (uint8_t)(aRxBuffer[3]);
				sTempC[3] = (uint8_t)(aRxBuffer[4]);
				sMoist[2] = (uint8_t)(aRxBuffer[5]);
				sMoist[3] = (uint8_t)(aRxBuffer[6]);
		} else if (testnValid_2)
				{
						sTempC[2] = (uint8_t)(aRxBuffer[4]);
						sTempC[3] = (uint8_t)(aRxBuffer[5]);
						sMoist[2] = (uint8_t)(aRxBuffer[6]);
						sMoist[3] = (uint8_t)(aRxBuffer[7]);
				}

				for (int i=0;i<9;i++){aRxBuffer[i] = 0;}
}

void RS485Sub(uint8_t depth)
{
	HAL_GPIO_WritePin(USART2_Tx_nRx_Port, USART2_Tx_nRx_Pin, GPIO_PIN_SET);  // set high to Tx data
	HAL_Delay(50);
		if (depth == 0)
		{
			 HAL_UART_Transmit(&huart2, aTxBuffer0, sizeof(aTxBuffer0), 150);
		}
		else
		{
			HAL_UART_Transmit(&huart2, aTxBuffer1, sizeof(aTxBuffer1), 150);
		}

	HAL_GPIO_WritePin(USART2_Tx_nRx_Port, USART2_Tx_nRx_Pin, GPIO_PIN_RESET);  // set low to receive data
	//HAL_Delay(1);
	HAL_UART_Receive(&huart2, aRxBuffer, 9, 200);			//  flash light if data is Rx
				/*{for (int i=0;i<4;i++)
					{
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
					HAL_Delay(400);
					}
				}*/
	//HAL_GPIO_WritePin(USART2_Tx_nRx_Port, USART2_Tx_nRx_Pin, GPIO_PIN_RESET);  // set low to receive data
}

void AirTnHGet(void)
{
	if (HAL_I2C_IsDeviceReady(&hi2c2, 0x44, 10, 50) == HAL_OK)
		{
		i2cTrace ++;  //(I2C_HandleTypeDef, SHT31 default device address is 0x44, trys, timeout)
		}

	HAL_I2C_Master_Transmit(&hi2c2, (0x44 << 1), SHT31Tx, 2, 150);
	HAL_I2C_Master_Receive(&hi2c2, ((0x44 << 1)| 1), SHT31Rx, 6, 250);

	aTempC[0] = SHT31Rx[0];
	aTempC[1] = SHT31Rx[1];
	aMoist[0] = SHT31Rx[3];
	aMoist[1] = SHT31Rx[4];

	realAirTemp = (float) (175 * (( aTempC[0] <<8 | aTempC[1])/0xFFFe));
	realAirTemp -= 45;

	realAirMoist = (float) (100 * ((aMoist[0] << 8 | aMoist[1])/0xFFFe));
}

void PwrCtl(uint8_t set_reset)
{
	switch (set_reset) {

	case 0:
		HAL_GPIO_WritePin(PWR_33_Port, PWR_33_Pin, GPIO_PIN_RESET);  //  turn off 3.3 V peripherals
		HAL_GPIO_WritePin(PWR_5V_Port , PWR_5V_Pin, GPIO_PIN_RESET);  // turn off 5 V peripherals
	break;
	case 1:
		HAL_GPIO_WritePin(PWR_33_Port, PWR_33_Pin, GPIO_PIN_SET);  //   High for power to RS485 chip and 3.3 V peripherals
		HAL_GPIO_WritePin(PWR_5V_Port , PWR_5V_Pin, GPIO_PIN_SET);  //  High for power to 5 V peripherals (the sensor prefers 5v)
	break;

	}
}

void GPIO_Init(void)
{
	 /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();

	  /* The I2C C
	   * SDA2 PA15
	   * SCL2 PB15
	   */

  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = SDA2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
   HAL_GPIO_Init(SDA2_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = SCL2_Pin;
   HAL_GPIO_Init(SCL2_Port, &GPIO_InitStruct);

  //__HAL_RCC_I2C2_CLK_ENABLE();


  /* The UART2
  	   * USART2_Rx A3
  	   * USART2_Tx A2
  	   */
  GPIO_InitStruct.Pin = USART2_Rx_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
     HAL_GPIO_Init(USART2_Rx_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = USART2_Tx_Pin;
     HAL_GPIO_Init(USART2_Tx_Port, &GPIO_InitStruct);

     /* The UART1, for serial monitor
     	   * USART_Rx B6
     	   * USART_Tx B7
     	   */
     GPIO_InitStruct.Pin = USART_Rx_Pin;
     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
     GPIO_InitStruct.Pull = GPIO_PULLUP;
     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
     GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(USART_Rx_Port, &GPIO_InitStruct);
     GPIO_InitStruct.Pin = USART_Tx_Pin;
        HAL_GPIO_Init(USART_Tx_Port, &GPIO_InitStruct);
  /*  the power controls and the RS485_DE line as well as the LED
   *
   */
     memset(&GPIO_InitStruct,0,sizeof(GPIO_InitStruct));
     /*Configure GPIO pin Output Level */
       HAL_GPIO_WritePin(GPIOB, USART2_Tx_nRx_Pin|LED_Pin|PWR_5V_Pin, GPIO_PIN_RESET);

       /*Configure GPIO pin Output Level */
       HAL_GPIO_WritePin(PWR_33_Port, PWR_33_Pin, GPIO_PIN_RESET);

       /*Configure GPIO pins : USART2_Tx_nRx_Pin LED_Pin PWR_5V_Pin */
       GPIO_InitStruct.Pin = USART2_Tx_nRx_Pin|LED_Pin|PWR_5V_Pin;
       GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
       GPIO_InitStruct.Pull = GPIO_PULLDOWN;
       GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
       HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

       /*Configure GPIO pin : PWR_33_Pin */
       GPIO_InitStruct.Pin = PWR_33_Pin;
       GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
       GPIO_InitStruct.Pull = GPIO_PULLDOWN;
       GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
       HAL_GPIO_Init(PWR_33_Port, &GPIO_InitStruct);
}

void USARTs_Init(void)
{

	//  clocks first

	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct1 = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct2 = {0};

	PeriphClkInitStruct1.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	    PeriphClkInitStruct1.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
	    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct1) != HAL_OK)
	    {
	      Error_Handler();
	    }

	    /* USART1 clock enable */
	    __HAL_RCC_USART1_CLK_ENABLE();

	 PeriphClkInitStruct2.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	     PeriphClkInitStruct2.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
	     if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct2) != HAL_OK)
	     {
	       Error_Handler();
	     }

	        /* USART2 clock enable */
	     __HAL_RCC_USART2_CLK_ENABLE();

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_UARTEx_EnableFifoMode(&huart1) != HAL_OK)
    {
      Error_Handler();
    }

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits   = UART_STOPBITS_1;
  huart2.Init.Parity     = UART_PARITY_NONE;
  huart2.Init.Mode       = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_UARTEx_EnableFifoMode(&huart2) != HAL_OK)
    {
      Error_Handler();
    }
}
void I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
	PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_SYSCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
	    Error_Handler();
	}
	__HAL_RCC_I2C2_CLK_ENABLE();

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x0000207A;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    printf("I2C failed to init with error \n\r");
  }

}
