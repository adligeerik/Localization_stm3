/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stdint.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define STM3_CTS_Pin GPIO_PIN_3
#define STM3_CTS_GPIO_Port GPIOC
#define STM3_CTS_EXTI_IRQn EXTI3_IRQn
#define STM3_UART_TX_Pin GPIO_PIN_0
#define STM3_UART_TX_GPIO_Port GPIOA
#define STM3_UART_RX_Pin GPIO_PIN_1
#define STM3_UART_RX_GPIO_Port GPIOA
#define STM3_RESET_Pin GPIO_PIN_2
#define STM3_RESET_GPIO_Port GPIOA
#define STM3_RTS_Pin GPIO_PIN_3
#define STM3_RTS_GPIO_Port GPIOA
#define FPGA_TX_Pin GPIO_PIN_10
#define FPGA_TX_GPIO_Port GPIOB
#define FPGA_RX_Pin GPIO_PIN_11
#define FPGA_RX_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_12
#define LED2_GPIO_Port GPIOB
#define UART_EMUL_TX_Pin GPIO_PIN_13
#define UART_EMUL_TX_GPIO_Port GPIOB
#define BU_3_Pin GPIO_PIN_9
#define BU_3_GPIO_Port GPIOD
#define BU_2_Pin GPIO_PIN_10
#define BU_2_GPIO_Port GPIOD
#define BU_1_Pin GPIO_PIN_11
#define BU_1_GPIO_Port GPIOD
#define SD_SW_B_Pin GPIO_PIN_6
#define SD_SW_B_GPIO_Port GPIOC
#define SD_SW_A_Pin GPIO_PIN_7
#define SD_SW_A_GPIO_Port GPIOC
#define COM_PS_TX_Pin GPIO_PIN_9
#define COM_PS_TX_GPIO_Port GPIOA
#define COM_GPS_RX_Pin GPIO_PIN_10
#define COM_GPS_RX_GPIO_Port GPIOA
#define COM_GPS_REST_Pin GPIO_PIN_11
#define COM_GPS_REST_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define UART_BUFFER 256

volatile uint8_t UART1_FULL;
volatile uint8_t UART3_FULL;
volatile uint8_t UART4_FULL;

volatile uint8_t UART1_DONE;
volatile uint8_t UART3_DONE;
volatile uint8_t UART4_DONE;

uint8_t DMA_TX_UART1_BUFFER[UART_BUFFER];
uint8_t DMA_TX_UART3_BUFFER[UART_BUFFER];
uint8_t DMA_TX_UART4_BUFFER[26];
uint8_t test[14];

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
