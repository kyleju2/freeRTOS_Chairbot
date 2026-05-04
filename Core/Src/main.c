/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "fir_coeffs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	int32_t ch0;
	int32_t ch1;
	int32_t ch2;
	int32_t ch3;
	int32_t raw0;
	int32_t raw1;
	int32_t raw2;
	int32_t raw3;
} adc_result_t;

typedef struct {
	float   buf[FIR_LEN];
	int16_t idx;
} fir_filter_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef  hspi1;
TIM_HandleTypeDef  htim2;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
	.name       = "defaultTask",
	.stack_size = 128 * 4,
	.priority   = (osPriority_t) osPriorityNormal,
};

/* USER CODE BEGIN PV */

osThreadId_t ADC_transfer_task_handle;
const osThreadAttr_t ADC_transfer_task_attributes = {
	.name       = "ADC_transfer_task",
	.stack_size = 1024 * 4,
	.priority   = (osPriority_t) osPriorityHigh,
};

osThreadId_t control_task_handle;
const osThreadAttr_t control_task_attributes = {
	.name       = "control_task",
	.stack_size = 1024 * 4,
	.priority   = (osPriority_t) osPriorityAboveNormal,
};

osSemaphoreId_t drdy_sem_handle;
const osSemaphoreAttr_t drdy_sem_attr = { .name = "drdy_sem" };

osMessageQueueId_t adc_queue_handle;

/* 18-byte SPI communication frames */
static uint8_t adc_tx[18] = {0};
static uint8_t adc_rx[18] = {0};

volatile uint16_t status = 0;

/* ADC offset calibration */
static int64_t sum0 = 0;
static int64_t sum1 = 0;
static int64_t sum2 = 0;
static int64_t sum3 = 0;

static int32_t adc_ch0_offset = 0;
static int32_t adc_ch1_offset = 0;
static int32_t adc_ch2_offset = 0;
static int32_t adc_ch3_offset = 0;

static uint8_t offset_done = 0;
static int32_t adc_count   = 0;

/* SPS measurement */
static uint32_t sps_start_tick = 0;
static uint32_t sps_count      = 0;
static uint32_t measured_sps   = 0;
static uint8_t  sps_measured   = 0;

/* CoP and control variables */
static int32_t cop_count      = 0;
static float   cop_sum_x      = 0.0f;
static float   cop_sum_y      = 0.0f;
static uint8_t cop_initialized = 0;

float initial_cop_x = 0.0f;
float initial_cop_y = 0.0f;

/* Sensor positions (meters) — FR, RR, FL, RL */
float pos_x0 =  2.0f;   /* FR */
float pos_y0 = -2.0f;

float pos_x1 =  2.0f;   /* RR */
float pos_y1 =  2.0f;

float pos_x2 = -2.0f;   /* FL */
float pos_y2 = -2.0f;

float pos_x3 = -2.0f;   /* RL */
float pos_y3 =  2.0f;

/* Control gains */
float Kv = 2.0f;
float Kw = 1.5f;
const float dt = 0.01f;

/* FIR filter instances */
static fir_filter_t fir0 = {0};
static fir_filter_t fir1 = {0};
static fir_filter_t fir2 = {0};
static fir_filter_t fir3 = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void  init_ADC(void);
static float fir_update(fir_filter_t *f, float new_val);
static void  ADC_transfer_task(void *argument);
static void  control_task(void *argument);
int _write(int file, char *ptr, int len);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	HAL_Init();

	/* USER CODE BEGIN Init */
	/* USER CODE END Init */

	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* USER CODE END SysInit */

	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_USART3_UART_Init();
	MX_TIM2_Init();

	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_Delay(5);
	init_ADC();
	HAL_Delay(1);
	/* USER CODE END 2 */

	osKernelInitialize();

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	drdy_sem_handle = osSemaphoreNew(1, 0, &drdy_sem_attr);
	__HAL_GPIO_EXTI_CLEAR_IT(DRDY_Pin);
	HAL_NVIC_ClearPendingIRQ(DRDY_EXTI_IRQn);
	HAL_NVIC_EnableIRQ(DRDY_EXTI_IRQn);
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_QUEUES */
	adc_queue_handle = osMessageQueueNew(10, sizeof(adc_result_t), NULL);
	/* USER CODE END RTOS_QUEUES */

	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
	configASSERT(defaultTaskHandle != NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	ADC_transfer_task_handle = osThreadNew(ADC_transfer_task, NULL, &ADC_transfer_task_attributes);
	configASSERT(ADC_transfer_task_handle != NULL);

	control_task_handle = osThreadNew(control_task, NULL, &control_task_attributes);
	configASSERT(control_task_handle != NULL);
	/* USER CODE END RTOS_THREADS */

	osKernelStart();

	/* We should never get here */
	while (1) {}
}

/* USER CODE BEGIN 4 */

/**
 * @brief Initialise the ADS131M04 ADC over SPI.
 */
static void init_ADC(void)
{
	memset(adc_tx, 0, 18);
	memset(adc_rx, 0, 18);

	/* Reset pulse */
	HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_SET);
	HAL_Delay(10);

	/* Flush startup frame */
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, adc_tx, adc_rx, 18, 100);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	/* CLOCK register: all 4 channels enabled, OSR = 2048 (~2 kSPS) */
	uint16_t CLOCK = (0xF << 8) | (4 << 2);

	/* GAIN register: all channels at gain 32 (101b) */
	uint16_t GAIN1 =
		(5 << 12) |   /* CH3 */
		(5 <<  8) |   /* CH2 */
		(5 <<  4) |   /* CH1 */
		(5 <<  0);    /* CH0 */

	/* Write CLOCK and GAIN1 registers in one burst */
	uint16_t cmd = 0x6000 | (0x03 << 7) | 1;   /* WREG | addr=3 | count=1 */

	memset(adc_tx, 0, 18);
	adc_tx[0] = (cmd   >> 8) & 0xFF;
	adc_tx[1] =  cmd         & 0xFF;
	adc_tx[2] = 0x00;

	adc_tx[3] = (CLOCK >> 8) & 0xFF;
	adc_tx[4] =  CLOCK       & 0xFF;
	adc_tx[5] = 0x00;

	adc_tx[6] = (GAIN1 >> 8) & 0xFF;
	adc_tx[7] =  GAIN1       & 0xFF;
	adc_tx[8] = 0x00;

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, adc_tx, adc_rx, 18, 100);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	/* Flush response */
	memset(adc_tx, 0, 18);
	memset(adc_rx, 0, 18);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, adc_tx, adc_rx, 18, 100);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief FIR filter update — pushes one sample and returns filtered output.
 */
static float fir_update(fir_filter_t *f, float new_val)
{
	f->buf[f->idx] = new_val;
	f->idx = (f->idx + 1) % FIR_LEN;

	float   acc = 0.0f;
	int16_t tmp;
	for (int16_t i = 0; i < FIR_LEN; i++) {
		tmp = f->idx - 1 - i;
		if (tmp < 0) tmp += FIR_LEN;
		acc += fir_coeffs[i] * f->buf[tmp];
	}
	return acc;
}


static void ADC_transfer_task(void *argument)
{
	for (;;) {
		osSemaphoreAcquire(drdy_sem_handle, osWaitForever);

		/* SPS measurement — runs once */
		if (!sps_measured) {
			sps_count++;
			if (sps_count == 1)
				sps_start_tick = osKernelGetTickCount();
			if (sps_count == 1001) {
				uint32_t elapsed = osKernelGetTickCount() - sps_start_tick;
				measured_sps = 1000000UL / elapsed;
				sps_measured = 1;
				printf("Measured SPS: %lu\r\n", measured_sps);
			}
		}

		/* SPI transfer */
		memset(adc_tx, 0, 18);
		memset(adc_rx, 0, 18);

		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1, adc_tx, adc_rx, 18, 100);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

		/* Parse status word */
		uint16_t status16 = ((uint16_t)adc_rx[0] << 8) | adc_rx[1];
		status = status16;

		/* Reconstruct 24-bit signed channel values */
		uint32_t u0 = ((uint32_t)adc_rx[3]  << 16) | ((uint32_t)adc_rx[4]  << 8) | adc_rx[5];
		uint32_t u1 = ((uint32_t)adc_rx[6]  << 16) | ((uint32_t)adc_rx[7]  << 8) | adc_rx[8];
		uint32_t u2 = ((uint32_t)adc_rx[9]  << 16) | ((uint32_t)adc_rx[10] << 8) | adc_rx[11];
		uint32_t u3 = ((uint32_t)adc_rx[12] << 16) | ((uint32_t)adc_rx[13] << 8) | adc_rx[14];

		/* Sign extend from 24-bit */
		if (u0 & 0x00800000u) u0 |= 0xFF000000u;
		if (u1 & 0x00800000u) u1 |= 0xFF000000u;
		if (u2 & 0x00800000u) u2 |= 0xFF000000u;
		if (u3 & 0x00800000u) u3 |= 0xFF000000u;

		adc_result_t result;
		result.ch0 = (int32_t)u0;
		result.ch1 = (int32_t)u1;
		result.ch2 = (int32_t)u2;
		result.ch3 = (int32_t)u3;

		adc_count++;

		if (!offset_done) {
			/* Accumulate samples for offset calibration */
			if (adc_count > 500) {
				sum0 += result.ch0;
				sum1 += result.ch1;
				sum2 += result.ch2;
				sum3 += result.ch3;

				if (adc_count == 1500) {
					adc_ch0_offset = (int32_t)(sum0 / 1000);
					adc_ch1_offset = (int32_t)(sum1 / 1000);
					adc_ch2_offset = (int32_t)(sum2 / 1000);
					adc_ch3_offset = (int32_t)(sum3 / 1000);
					offset_done = 1;
				}
			}

		} else {

			result.ch0 -= adc_ch0_offset;
			result.ch1 -= adc_ch1_offset;
			result.ch2 -= adc_ch2_offset;
			result.ch3 -= adc_ch3_offset;

			result.raw0 = result.ch0;
			result.raw1 = result.ch1;
			result.raw2 = result.ch2;
			result.raw3 = result.ch3;

			/* Apply FIR filter */
			result.ch0 = (int32_t)fir_update(&fir0, (float)result.ch0);
			result.ch1 = (int32_t)fir_update(&fir1, (float)result.ch1);
			result.ch2 = (int32_t)fir_update(&fir2, (float)result.ch2);
			result.ch3 = (int32_t)fir_update(&fir3, (float)result.ch3);

			osMessageQueuePut(adc_queue_handle, &result, 0, 0);
			osThreadYield();
		}
	}
}


static void control_task(void *argument)
{
	float prev_theta = 0.0f;

	for (;;) {
		osDelay(10);

		/* Wait for ADC calibration */
		if (!offset_done) continue;

		/* Drain queue, keep latest */
		adc_result_t result = {0};
		adc_result_t latest = {0};

		while (osMessageQueueGet(adc_queue_handle, &result, NULL, 0) == osOK)
			latest = result;

		float f0 = (float)latest.ch0;
		float f1 = (float)latest.ch1;
		float f2 = (float)latest.ch2;
		float f3 = (float)latest.ch3;

		float f_sum = f0 + f1 + f2 + f3;

		if (f_sum < 10000.0f) {
			printf("V 0.0000 0.0000 0.0000\r\n");
			continue;
		}

		/* Center of Pressure */
		float x = (f0*pos_x0 + f1*pos_x1 + f2*pos_x2 + f3*pos_x3) / f_sum;
		float y = (f0*pos_y0 + f1*pos_y1 + f2*pos_y2 + f3*pos_y3) / f_sum;

		if (!cop_initialized) {
			cop_sum_x += x;
			cop_sum_y += y;
			cop_count++;
			if (cop_count >= 100) {
				initial_cop_x = cop_sum_x / cop_count;
				initial_cop_y = cop_sum_y / cop_count;
				cop_initialized = 1;
			}
			continue;
		}

		/* Control law */
		float dx    = x - initial_cop_x;
		float dy    = y - initial_cop_y;
		float r     = sqrtf(dx*dx + dy*dy);
		float theta = atan2f(dy, dx);

		/* Angle difference wrapped to [-pi, pi] */
		float dtheta = atan2f(sinf(theta - prev_theta), cosf(theta - prev_theta));
		float omega  = Kw * dtheta / dt;

		float v  = Kv * r;
		float vx = v * cosf(theta);
		float vy = v * sinf(theta);

		prev_theta = theta;

		/* Output for ROS/Gazebo */
		printf("V %.4f %.4f %.4f\r\n", vx, vy, omega);

		/* Output raw vs filtered channel data for validation */
//		printf("C %ld %ld %ld %ld %ld %ld %ld %ld\r\n",
//			latest.raw0, latest.raw1, latest.raw2, latest.raw3,
//			latest.ch0,  latest.ch1,  latest.ch2,  latest.ch3);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == DRDY_Pin) {
		if (drdy_sem_handle != NULL)
			osSemaphoreRelease(drdy_sem_handle);
	}
}

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}

/* USER CODE END 4 */

void StartDefaultTask(void *argument)
{
	for (;;)
		osDelay(1);
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState            = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM            = 4;
	RCC_OscInitStruct.PLL.PLLN            = 60;
	RCC_OscInitStruct.PLL.PLLP            = 2;
	RCC_OscInitStruct.PLL.PLLQ            = 5;
	RCC_OscInitStruct.PLL.PLLR            = 2;
	RCC_OscInitStruct.PLL.PLLRGE          = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL       = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN        = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

	RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                                 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
	                                 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void)
{
	hspi1.Instance               = SPI1;
	hspi1.Init.Mode              = SPI_MODE_MASTER;
	hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;
	hspi1.Init.NSS               = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial     = 0x0;
	hspi1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
	hspi1.Init.NSSPolarity       = SPI_NSS_POLARITY_LOW;
	hspi1.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
	hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi1.Init.MasterSSIdleness          = SPI_MASTER_SS_IDLENESS_00CYCLE;
	hspi1.Init.MasterInterDataIdleness   = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	hspi1.Init.MasterReceiverAutoSusp    = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	hspi1.Init.MasterKeepIOState         = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	hspi1.Init.IOSwap                    = SPI_IO_SWAP_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef      sConfigOC     = {0};

	htim2.Instance               = TIM2;
	htim2.Init.Prescaler         = 0;
	htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim2.Init.Period            = 29;   /* 240MHz / 30 = 8 MHz MCLK */
	htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();

	sConfigOC.OCMode     = TIM_OCMODE_PWM1;
	sConfigOC.Pulse      = 14;   /* 50% duty cycle of period=29 */
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

	HAL_TIM_MspPostInit(&htim2);
}

static void MX_USART3_UART_Init(void)
{
	huart3.Instance            = USART3;
	huart3.Init.BaudRate       = 115200;
	huart3.Init.WordLength     = UART_WORDLENGTH_8B;
	huart3.Init.StopBits       = UART_STOPBITS_1;
	huart3.Init.Parity         = UART_PARITY_NONE;
	huart3.Init.Mode           = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling   = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
	if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
	if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
	if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_GPIO_Port,   CS_Pin,   GPIO_PIN_SET);

	/* DRDY interrupt pin */
	GPIO_InitStruct.Pin  = DRDY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DRDY_GPIO_Port, &GPIO_InitStruct);

	/* SYNC pin */
	GPIO_InitStruct.Pin   = SYNC_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(SYNC_GPIO_Port, &GPIO_InitStruct);

	/* CS pin */
	GPIO_InitStruct.Pin   = CS_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(DRDY_EXTI_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DRDY_EXTI_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	HAL_NVIC_DisableIRQ(DRDY_EXTI_IRQn);   /* re-enabled after semaphore is created */
	/* USER CODE END MX_GPIO_Init_2 */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6)
		HAL_IncTick();
}

void Error_Handler(void)
{
	__disable_irq();
	while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
	/* printf("Wrong parameters value: file %s on line %d\r\n", file, line); */
}
#endif /* USE_FULL_ASSERT */
