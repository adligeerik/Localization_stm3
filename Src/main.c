/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 * Erik von Keyserlingk 16/11-2017
 * WOLF localization program for REXUS 24
 * The satellite modem, STM3, is on UART4
 * VHF UART is implemented in software, because UART1 and UART5 is used for FSMC
 * FPGA is on UART3
 * Commercial GPS is on UART1
 *
 * How the code works:
 * Using DMA, UART1 (when GPS has power) receives NMEA formated message from GPS.
 * When DMA_TX_UART1_BUFFER is full a flag is set and the buffer is parsed. The
 * correct formated message is stored in; msg_send. msg_send is only updated when
 * a new valid GPS coordinate has been received. For sending a message to STM3
 * the pin PC3 (CTS) is set low and if STM3 answer with pulling PA3 (RTS) low.
 * An interrupt is called and the coordinates (with preamble and CRC) is send to
 * STM3 on UART4.
 *
 * For transmitting data to FPGA use:
 * HAL_UART_Transmit_DMA(&huart3, DATA, LENGTH_OF_DATA, 5)
 *
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "dwt_stm32_delay.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

NOR_HandleTypeDef hnor1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define UART_BUFFER 256
#define PAYLOAD_LENGTH 21
#define STM3_DATA 26 // PAYLOAD_LENGTH +5
#define TIME_LENGTH 6


// UART buffer, data to send and data recived
uint8_t DMA_RX_UART1_BUFFER[UART_BUFFER];
uint8_t DMA_RX_UART3_BUFFER[UART_BUFFER];
uint8_t DMA_RX_UART4_BUFFER[UART_BUFFER];

uint8_t DMA_TX_UART1_BUFFER[UART_BUFFER];
uint8_t DMA_TX_UART3_BUFFER[UART_BUFFER];
uint8_t DMA_TX_UART4_BUFFER[26]  = {0xAA, 0x0E, 0x06 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x03 , 0x18 , 0x30 , 0x00 , 0xCB , 0x12};
/*
 * {0xAA, 0x0E, 0x06 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x03 , 0x18 , 0x30 , 0x00 , 0xCB , 0x12}; //Channel A
 * {0xAA, 0x0E, 0x06 , 0x00 , 0x00 , 0x00 , 0x00 , 0x01 , 0x03 , 0x01 , 0x03 , 0x00 , 0xCB , 0x12}; //Channel B
 * {0xAA, 0x0E, 0x06 , 0x00 , 0x00 , 0x00 , 0x00 , 0x02 , 0x03 , 0x01 , 0x03 , 0x00 , 0x07 , 0x0F}; //Channel B
 * {0xAA, 0x0E, 0x06 , 0x00 , 0x00 , 0x00 , 0x00 , 0x03 , 0x03 , 0x01 , 0x03 , 0x00 , 0x43 , 0x04}; //Channel B
 */
uint8_t test[14]= {0xAA , 0x0E , 0x00 , 0x02 , 0x02 , 0x03 , 0x04 , 0x05 , 0x06 , 0x07 , 0x08 , 0x09 , 0xBE , 0xE8};


// Flag for when to read from buffer
volatile uint8_t UART1_FULL = 0;
volatile uint8_t UART3_FULL = 0;
volatile uint8_t UART4_FULL = 0;

// Flag for when transmition is finished
volatile uint8_t UART1_DONE = 1;
volatile uint8_t UART3_DONE = 1;
volatile uint8_t UART4_DONE = 1;

TIM_HandleTypeDef TIM_Handle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FSMC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint16_t CrcCalculate(uint8_t* array, uint8_t length);
uint8_t * parse_nmea(uint8_t* uart_buffer);
void send_to_stm3(uint8_t* send_stm3, uint8_t data_length);
void send_to_VHF(uint8_t* Data, uint8_t data_length);
void time_log(uint8_t *time);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

	int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_FSMC_Init();
	MX_SDIO_SD_Init();
	MX_UART4_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_FATFS_Init();
	MX_TIM4_Init();

	/* USER CODE BEGIN 2 */

	// Enable interrupt for CTS from STM3
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_NVIC_SetPriority(TIM4_IRQn, 2, 2);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);

	// Initi Uart reciver
	HAL_UART_Receive_DMA(&huart1, DMA_RX_UART1_BUFFER, UART_BUFFER);
	HAL_UART_Receive_DMA(&huart3, DMA_RX_UART3_BUFFER, UART_BUFFER);
	HAL_UART_Receive_DMA(&huart4, DMA_RX_UART4_BUFFER, UART_BUFFER);

	if (DWT_Delay_Init()) {
		Error_Handler(); /* Call Error Handler */
	}

	HAL_GPIO_TogglePin(GPIOA, COM_GPS_REST_Pin);
	HAL_GPIO_WritePin(GPIOA, STM3_RTS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, STM3_RESET_Pin, GPIO_PIN_SET);
	HAL_GPIO_TogglePin(GPIOA, STM3_RESET_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOA, STM3_RESET_Pin);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	//uint8_t Data[12] = {0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x20, 0x57, 0x6f, 0x72, 0x6c, 0x64, 0x21} ;
	//uint8_t setup_msg[10] = {0xAA, 0x0E, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x09, 0x18, 0x00};
	uint8_t msg_send[PAYLOAD_LENGTH];
	uint8_t time[TIME_LENGTH];

	while (1) {
		HAL_Delay(10);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (UART1_FULL == 1) { //Com GPS
			//HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
			//HAL_UART_Transmit_DMA(&huart3, &time, TIME_LENGTH);
			//HAL_UART_Transmit_DMA(&huart3, DMA_RX_UART1_BUFFER, PAYLOAD_LENGTH);

			// Parses the received message from commercial GPS
			uint8_t *coordinates = parse_nmea(DMA_RX_UART1_BUFFER);

			//HAL_UART_Transmit_DMA(&huart3, coordinates, PAYLOAD_LENGTH+6);

			// Check for a new valid coordinate message
			if (coordinates[PAYLOAD_LENGTH - 1] != '\0') {
				for (int i = 0; i < PAYLOAD_LENGTH; i++) {
					msg_send[i] = coordinates[i];
				}
				for (int i = 0; i < TIME_LENGTH; i++){
					time [i] = coordinates[21+i];
				}
			}

			// Updates the message to be send to STM3
			send_to_stm3(msg_send, PAYLOAD_LENGTH);
			memset(&coordinates[0], '\0', PAYLOAD_LENGTH);
			UART1_FULL = 0;
		}
		if (UART3_FULL == 1) { //FPGA
			//HAL_UART_Transmit(&huart3,DMA_RX_UART3_BUFFER , UART_BUFFER,5);
			//HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
			UART3_FULL = 0;
		}
		if (UART4_FULL == 1) {		  //STM3
			//HAL_UART_Transmit(&huart3,DMA_RX_UART4_BUFFER , UART_BUFFER,5);
			//HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
			UART4_FULL = 0;
		}


		// Set pin low for request to send to STM3
		HAL_GPIO_WritePin(GPIOA, STM3_RTS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_TogglePin(GPIOB, LED1_Pin);

		// send coordinates message to VHF transmitter
		send_to_VHF(msg_send, PAYLOAD_LENGTH);
		time_log(&time);


	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SDIO init function */
static void MX_SDIO_SD_Init(void) {

	hsd.Instance = SDIO;
	hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
	hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
	hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
	hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
	hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd.Init.ClockDiv = 0;

}

/* TIM4 init function */
static void MX_TIM4_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 256;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 26250;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* UART4 init function */
static void MX_UART4_Init(void) {

	huart4.Instance = UART4;
	huart4.Init.BaudRate = 9600;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART1 init function */
static void MX_USART1_UART_Init(void) {

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART3 init function */
static void MX_USART3_UART_Init(void) {

	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE()
	;
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	/* DMA1_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
	/* DMA1_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	/* DMA1_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOE_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, STM3_RESET_Pin | STM3_RTS_Pin | COM_GPS_REST_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED2_Pin | LED1_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(UART_EMUL_TX_GPIO_Port, UART_EMUL_TX_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, BU_3_Pin | BU_2_Pin | BU_1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SD_SW_B_GPIO_Port, SD_SW_B_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : STM3_CTS_Pin */
	GPIO_InitStruct.Pin = STM3_CTS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(STM3_CTS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : STM3_RESET_Pin STM3_RTS_Pin COM_GPS_REST_Pin */
	GPIO_InitStruct.Pin = STM3_RESET_Pin | STM3_RTS_Pin | COM_GPS_REST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LED2_Pin UART_EMUL_TX_Pin LED1_Pin */
	GPIO_InitStruct.Pin = LED2_Pin | UART_EMUL_TX_Pin | LED1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : BU_3_Pin BU_2_Pin BU_1_Pin */
	GPIO_InitStruct.Pin = BU_3_Pin | BU_2_Pin | BU_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : SD_SW_B_Pin */
	GPIO_InitStruct.Pin = SD_SW_B_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SD_SW_B_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SD_SW_A_Pin */
	GPIO_InitStruct.Pin = SD_SW_A_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SD_SW_A_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void) {
	FSMC_NORSRAM_TimingTypeDef Timing;

	/** Perform the NOR1 memory initialization sequence
	 */
	hnor1.Instance = FSMC_NORSRAM_DEVICE;
	hnor1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
	/* hnor1.Init */
	hnor1.Init.NSBank = FSMC_NORSRAM_BANK1;
	hnor1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_ENABLE;
	hnor1.Init.MemoryType = FSMC_MEMORY_TYPE_NOR;
	hnor1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_8;
	hnor1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_ENABLE;
	hnor1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
	hnor1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
	hnor1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
	hnor1.Init.WriteOperation = FSMC_WRITE_OPERATION_DISABLE;
	hnor1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
	hnor1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
	hnor1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
	hnor1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
	hnor1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
	/* Timing */
	Timing.AddressSetupTime = 15;
	Timing.AddressHoldTime = 15;
	Timing.DataSetupTime = 255;
	Timing.BusTurnAroundDuration = 15;
	Timing.CLKDivision = 16;
	Timing.DataLatency = 17;
	Timing.AccessMode = FSMC_ACCESS_MODE_A;
	/* ExtTiming */

	if (HAL_NOR_Init(&hnor1, &Timing, NULL) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USER CODE BEGIN 4 */

/*
 * Sends the time for when STM3 did send its message on UART3 (FPGA)
 */
void time_log(uint8_t* time) {

	for (int i = 0; i < UART_BUFFER; i++) {

		// Response for a valid message received by STM3: AA 05 00 D9 C4
		if (DMA_RX_UART4_BUFFER[i] == (int) 0x00 && DMA_RX_UART4_BUFFER[i + 1] == (int) 0xD9
						&& DMA_RX_UART4_BUFFER[i + 2] == (int) 0xC4) {
			HAL_UART_Transmit_DMA(&huart3, time, TIME_LENGTH);

		}
	}
	memset(&DMA_RX_UART4_BUFFER, 0x00, UART_BUFFER );
}


/*
 * send_to_VHF is a soft UART and send on 2400 baud, change us to change the baud rate
 * 416us 2400 baud
 * 208us 4800 baud
 * 104us 9600 baud
 */
void send_to_VHF(uint8_t* Data, uint8_t data_length) {

	HAL_GPIO_TogglePin(GPIOB, LED2_Pin);

	// Delay for uart at 2400 baud
	int us = 416;
	for (uint8_t i = 0; i < data_length; i++) {
		HAL_GPIO_WritePin(GPIOB, UART_EMUL_TX_Pin, GPIO_PIN_RESET);
		DWT_Delay_us(us);
		uint8_t byte = Data[i];
		for (uint8_t u = 0; u < 8; u++) {
			if (((byte >> u) & 0x1) == 0x1) {
				HAL_GPIO_WritePin(GPIOB, UART_EMUL_TX_Pin, GPIO_PIN_SET);
			}
			if (((byte >> u) & 0x01) == 0x00) {
				HAL_GPIO_WritePin(GPIOB, UART_EMUL_TX_Pin, GPIO_PIN_RESET);
			}

			DWT_Delay_us(us);
		}
		HAL_GPIO_WritePin(GPIOB, UART_EMUL_TX_Pin, GPIO_PIN_SET);
		DWT_Delay_us(us);
	}
	HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
}

/*
 * send_to_stm3 takes adds required preamble bytes and CRC to UART4_buffer
 */
void send_to_stm3(uint8_t* send_stm3, uint8_t data_length) {
	static uint8_t message[STM3_DATA];
	//*message = (uint8_t*)malloc(sizeof(uint8_t)*(data_length+5));
	//uint8_t length = data_length+3;
	message[0] = 0xAA;
	message[1] = 0x1A; // 0x1A = 26
	message[2] = 0x00;
	for (int i = 0; i < data_length; i++) {
		message[i + 3] = send_stm3[i]; //send_stm3[i];
	}

	uint8_t CRC_array[2];

	uint16_t crc = CrcCalculate(message, 24);
	message[24] = crc & 0xff;
	message[25] = (crc >> 8);

	for (int i = 0; i < STM3_DATA; i++) {
		DMA_TX_UART4_BUFFER[i] = message[i];
	}
}

/*
 * Parses the received message from the commercial GPS into valid coordinates
 */
uint8_t * parse_nmea(uint8_t* uart_buffer) {
	static uint8_t send_stm3[27]; // 21 for only coordinates 27 with time, 6 last is the time

	int send_index = 0;
	int time_index = 0;

	for (int i = 0; i < UART_BUFFER; i++) {
		if (uart_buffer[i] == (int) 'G' && uart_buffer[i + 1] == (int) 'G'
				&& uart_buffer[i + 2] == (int) 'A') {
			//HAL_UART_Transmit(&huart3, &uart_command[i+1], 1, 5);
			int numb_comma = 0;
			//HAL_UART_Transmit(&huart3, (int)'¤', 1, 5);
			for (i = i + 2; i < UART_BUFFER; i++) {
				if (uart_buffer[i] == (int) ',') {
					numb_comma++;
					i++;
				}
				if (uart_buffer[i] == (int) '.') {
					i++;
				}
				if(numb_comma == 1){
					send_stm3[time_index+21] = uart_buffer[i];
					time_index++;
				}
				if (numb_comma >= 2 && uart_buffer[i] != (int) ',') { //
				//HAL_UART_Transmit(&huart3, &uart_buffer[i], 1, 5);
					send_stm3[send_index] = uart_buffer[i];
					send_index++;
				}
				if (numb_comma == 6) {
					i = UART_BUFFER;
				}
			}
			i = UART_BUFFER;
		}
	}

	send_index = 0;
	return send_stm3;
}

// Computes 16 bit CRC value for an array of bytes using the polynomial
// selected for the STM3. The first argument is a pointer to the array of data
// that the function should compute the CRC for. The second argument is the
// number of bytes in that array. The function returns the CRC, ready for
// insertion into a command to the STM3 or for comparison against a response
// from the STM3.
//
// MISRA C types are used to clearly communicate the size of the variables.
//
uint16_t CrcCalculate(uint8_t* array, uint8_t length) {
	uint16_t crc = 0xFFFF;
	uint8_t x;
	// Step through each byte in the array.
	while (length--) {
		// XOR the CRC with the next byte in the array.
		crc = crc ^ (uint16_t) *array++;
		// Step through each bit in the byte.
		for (x = 0; x < 8; x++) {
			if (crc & 0x0001)
				crc = (crc >> 1) ^ 0x8408;
			else
				crc >>= 1;
		}
	}
	// Invert and return the calculated CRC.
	return ~crc;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
