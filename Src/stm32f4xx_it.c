/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 * Erik von Keyserlingk 16/11-2017
 * WOLF localization program for REXUS 24
 *
 * This file handle the interrupts
 *
 * When STM3 is ready to receive a message the interrupt: EXTI3_IRQHandler is
 * called. When a UART buffer is full; HAL_UART_RxCpltCallbac is called to
 * set the corresponding flag to full. When a message has been transmitted
 * HAL_UART_TxCpltCallbac is called.
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

extern uint8_t DMA_TX_UART1_BUFFER[UART_BUFFER];
extern uint8_t DMA_TX_UART3_BUFFER[UART_BUFFER];
extern uint8_t DMA_TX_UART4_BUFFER[26];
extern uint8_t test[14];

volatile extern uint8_t UART1_FULL;
volatile extern uint8_t UART3_FULL;
volatile extern uint8_t UART4_FULL;

volatile extern uint8_t UART1_DONE;
volatile extern uint8_t UART3_DONE;
volatile extern uint8_t UART4_DONE;

int nu;


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */

	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1) {
	}
	/* USER CODE BEGIN HardFault_IRQn 1 */

	/* USER CODE END HardFault_IRQn 1 */
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1) {
	}
	/* USER CODE BEGIN MemoryManagement_IRQn 1 */

	/* USER CODE END MemoryManagement_IRQn 1 */
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void) {
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1) {
	}
	/* USER CODE BEGIN BusFault_IRQn 1 */

	/* USER CODE END BusFault_IRQn 1 */
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1) {
	}
	/* USER CODE BEGIN UsageFault_IRQn 1 */

	/* USER CODE END UsageFault_IRQn 1 */
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) {
	/* USER CODE BEGIN SVCall_IRQn 0 */

	/* USER CODE END SVCall_IRQn 0 */
	/* USER CODE BEGIN SVCall_IRQn 1 */

	/* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void) {
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) {
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles EXTI line3 interrupt.
 */



void EXTI3_IRQHandler(void) {
	/* USER CODE BEGIN EXTI3_IRQn 0 */
	//HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
	//HAL_UART_Transmit_DMA(&huart4, DMA_TX_UART4_BUFFER, 26);
//	for(int i = 0; i < 1000000; i++){
//		HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
//	}


	if (nu == 1){
		HAL_UART_Transmit_DMA(&huart4,DMA_TX_UART4_BUFFER, 26);
	}
	/* USER CODE END EXTI3_IRQn 0 */
	nu = 1;
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
	/* USER CODE BEGIN EXTI3_IRQn 1 */

	/* USER CODE END EXTI3_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream1 global interrupt.
 */
void DMA1_Stream1_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

	/* USER CODE END DMA1_Stream1_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart3_rx);
	/* USER CODE BEGIN DMA1_Stream1_IRQn 1 */
	//UART3_FULL = 1;
	//HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
	/* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream2 global interrupt.
 */
void DMA1_Stream2_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

	/* USER CODE END DMA1_Stream2_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_uart4_rx);
	/* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

	/* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream3 global interrupt.
 */
void DMA1_Stream3_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

	/* USER CODE END DMA1_Stream3_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart3_tx);
	/* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

	/* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream4 global interrupt.
 */
void DMA1_Stream4_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

	/* USER CODE END DMA1_Stream4_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_uart4_tx);
	/* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

	/* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
 * @brief This function handles TIM4 global interrupt.
 */
void TIM4_IRQHandler(void) {
	/* USER CODE BEGIN TIM4_IRQn 0 */
	int CTS = HAL_GPIO_ReadPin(GPIOC, STM3_CTS_Pin);
	if (CTS == 1) {
		HAL_GPIO_WritePin(GPIOA, STM3_RTS_Pin, GPIO_PIN_SET);
		HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
	}
	/* USER CODE END TIM4_IRQn 0 */
	HAL_TIM_IRQHandler(&htim4);
	/* USER CODE BEGIN TIM4_IRQn 1 */

	/* USER CODE END TIM4_IRQn 1 */
}

/**
 * @brief This function handles USART3 global interrupt.
 */
void USART3_IRQHandler(void) {
	/* USER CODE BEGIN USART3_IRQn 0 */

	/* USER CODE END USART3_IRQn 0 */
	HAL_UART_IRQHandler(&huart3);
	/* USER CODE BEGIN USART3_IRQn 1 */

	/* USER CODE END USART3_IRQn 1 */
}

/**
 * @brief This function handles UART4 global interrupt.
 */
void UART4_IRQHandler(void) {
	/* USER CODE BEGIN UART4_IRQn 0 */

	/* USER CODE END UART4_IRQn 0 */
	HAL_UART_IRQHandler(&huart4);
	/* USER CODE BEGIN UART4_IRQn 1 */

	/* USER CODE END UART4_IRQn 1 */
}

/**
 * @brief This function handles DMA2 stream2 global interrupt.
 */
void DMA2_Stream2_IRQHandler(void) {
	/* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

	/* USER CODE END DMA2_Stream2_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart1_rx);
	/* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

	/* USER CODE END DMA2_Stream2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart == &huart1) {
		//HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
		HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
		UART1_DONE = 1;
	}
	if (huart == &huart3) {
		UART3_DONE = 1;
		//HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
	}
	// When message have been sent to STM3 RTS is pulled high
	if (huart == &huart4) {
		UART4_DONE = 1;
		HAL_GPIO_WritePin(GPIOA, STM3_RTS_Pin, GPIO_PIN_SET);
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		UART1_FULL = 1;
		//HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
	}
	if (huart == &huart3) {
		UART3_FULL = 1;
		//HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
	}
	if (huart == &huart4) {
		UART4_FULL = 1;
	}
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
