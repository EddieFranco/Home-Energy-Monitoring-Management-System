/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <math.h>

#include "lcd_i2c.h"   // I2C 16x2/20x4 LCD driver (LCD_Init, LCD_PrintAt, etc.)
#include "onewire.h"    // ONEWIRE_Init(), ONEWIRE_Reset(), byte/bit I/O, microsecond delays
#include "ds18b20.h"    // DS18B20_StartConversion_SkipROM(), DS18B20_ReadScratchpad()
#include "pid.h"
#include "bts7960_pwm.h"
#include "telemetry.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
	FAULT_NONE = 0,
	FAULT_OVERTEMP,
	FAULT_OVERCURRENT,
	FAULT_SENSOR_FAIL
} FaultCode_t;


typedef struct {
	float volts, amps, watts;
} PowerSample_t;


static volatile FaultCode_t g_fault = FAULT_NONE;

static PID_t g_pid;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for SafetyTask */
osThreadId_t SafetyTaskHandle;
const osThreadAttr_t SafetyTask_attributes = {
		.name = "SafetyTask",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PIDTask */
osThreadId_t PIDTaskHandle;
const osThreadAttr_t PIDTask_attributes = {
		.name = "PIDTask",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TempTask */
osThreadId_t TempTaskHandle;
const osThreadAttr_t TempTask_attributes = {
		.name = "TempTask",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PowerTask */
osThreadId_t PowerTaskHandle;
const osThreadAttr_t PowerTask_attributes = {
		.name = "PowerTask",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UITask */
osThreadId_t UITaskHandle;
const osThreadAttr_t UITask_attributes = {
		.name = "UITask",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TempQ */
osMessageQueueId_t TempQHandle;
const osMessageQueueAttr_t TempQ_attributes = {
		.name = "TempQ"
};
/* Definitions for SetPointQ */
osMessageQueueId_t SetPointQHandle;
const osMessageQueueAttr_t SetPointQ_attributes = {
		.name = "SetPointQ"
};
/* Definitions for PowerQ */
osMessageQueueId_t PowerQHandle;
const osMessageQueueAttr_t PowerQ_attributes = {
		.name = "PowerQ"
};
/* Definitions for FaultQ */
osMessageQueueId_t FaultQHandle;
const osMessageQueueAttr_t FaultQ_attributes = {
		.name = "FaultQ"
};
/* Definitions for LCD_Mutex */
osMutexId_t LCD_MutexHandle;
const osMutexAttr_t LCD_Mutex_attributes = {
		.name = "LCD_Mutex"
};
/* Definitions for UART_Mutex */
osMutexId_t UART_MutexHandle;
const osMutexAttr_t UART_Mutex_attributes = {
		.name = "UART_Mutex"
};
/* Definitions for OneWire_Mutex */
osMutexId_t OneWire_MutexHandle;
const osMutexAttr_t OneWire_Mutex_attributes = {
		.name = "OneWire_Mutex"
};
/* Definitions for PIDParam_Mutex */
osMutexId_t PIDParam_MutexHandle;
const osMutexAttr_t PIDParam_Mutex_attributes = {
		.name = "PIDParam_Mutex"
};
/* Definitions for PID_TickSem */
osSemaphoreId_t PID_TickSemHandle;
const osSemaphoreAttr_t PID_TickSem_attributes = {
		.name = "PID_TickSem"
};
/* Definitions for Power_DoneSem */
osSemaphoreId_t Power_DoneSemHandle;
const osSemaphoreAttr_t Power_DoneSem_attributes = {
		.name = "Power_DoneSem"
};
/* Definitions for BtnEventSem */
osSemaphoreId_t BtnEventSemHandle;
const osSemaphoreAttr_t BtnEventSem_attributes = {
		.name = "BtnEventSem"
};
/* Definitions for FaultTripSem */
osSemaphoreId_t FaultTripSemHandle;
const osSemaphoreAttr_t FaultTripSem_attributes = {
		.name = "FaultTripSem"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
void StartSafetyTask(void *argument);
void StartPIDTask(void *argument);
void StartTempTask(void *argument);
void StartPowerTask(void *argument);
void StartUITask(void *argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

static inline void LCD_ShowFaultBanner(const char *line1, const char *line2);
static inline void LCD_ShowLinesNoClear(const char *l1, const char *l2);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
	uint8_t c = (uint8_t)ch;
	HAL_UART_Transmit(&huart2, &c, 1, HAL_MAX_DELAY);
	return ch;
}


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

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
	MX_I2C1_Init();
	MX_USART2_UART_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	ONEWIRE_Init();   // sets PD7 to open-drain + pull-up and starts DWT microsecond timer
	// after MX_I2C1_Init(); (make sure I2C1 is initialized in CubeMX)
	// ...
	LCD_Init();        // init controller & clear    <-- one-time init
	LCD_SetBacklight(1);
	LCD_Clear();
	LCD_PrintAt(0,0, "System Starting...");

	BTS7960_PWM_Init(); // start TIM3 CH3/CH4 at 0% (PB0/PB1)


	//  Sample time: Release PID_TickSem every 200 ms (example). If you use a different period,
	//  set dt_s accordingly (e.g., 0.5f for 500 ms).
	// --- PID controller init (do this ONCE here) ---
	const float KP = 2.0f;
	const float KI = 0.25f;   // 1/s
	const float KD = 0.10f;    // s
	const float DT_S    = 0.2f;   // must match your PID tick period (see semaphore)
	const float OUT_MIN = 0.0f;   // 0% duty
	const float OUT_MAX = 100.0f; // 100% duty
	const float D_ALPHA = 0.2f;   // derivative smoothing

	PID_Init(&g_pid, KP, KI, KD, OUT_MIN, OUT_MAX, DT_S, D_ALPHA);
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();
	/* Create the mutex(es) */
	/* creation of LCD_Mutex */
	LCD_MutexHandle = osMutexNew(&LCD_Mutex_attributes);

	/* creation of UART_Mutex */
	UART_MutexHandle = osMutexNew(&UART_Mutex_attributes);

	/* creation of OneWire_Mutex */
	OneWire_MutexHandle = osMutexNew(&OneWire_Mutex_attributes);

	/* creation of PIDParam_Mutex */
	PIDParam_MutexHandle = osMutexNew(&PIDParam_Mutex_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of PID_TickSem */
	PID_TickSemHandle = osSemaphoreNew(1, 1, &PID_TickSem_attributes);

	/* creation of Power_DoneSem */
	Power_DoneSemHandle = osSemaphoreNew(1, 1, &Power_DoneSem_attributes);

	/* creation of BtnEventSem */
	BtnEventSemHandle = osSemaphoreNew(1, 1, &BtnEventSem_attributes);

	/* creation of FaultTripSem */
	FaultTripSemHandle = osSemaphoreNew(1, 1, &FaultTripSem_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	Telemetry_Init();   // creates Telemetry_Mutex + starts TelemetryTask (UART @ huart2)
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of TempQ */
	TempQHandle = osMessageQueueNew (16, sizeof(float), &TempQ_attributes);

	/* creation of SetPointQ */
	SetPointQHandle = osMessageQueueNew (16, sizeof(float), &SetPointQ_attributes);

	/* creation of PowerQ */
	PowerQHandle = osMessageQueueNew (16, sizeof(PowerSample_t), &PowerQ_attributes);

	/* creation of FaultQ */
	FaultQHandle = osMessageQueueNew (16, sizeof(FaultCode_t), &FaultQ_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of SafetyTask */
	SafetyTaskHandle = osThreadNew(StartSafetyTask, NULL, &SafetyTask_attributes);

	/* creation of PIDTask */
	PIDTaskHandle = osThreadNew(StartPIDTask, NULL, &PIDTask_attributes);

	/* creation of TempTask */
	TempTaskHandle = osThreadNew(StartTempTask, NULL, &TempTask_attributes);

	/* creation of PowerTask */
	PowerTaskHandle = osThreadNew(StartPowerTask, NULL, &PowerTask_attributes);

	/* creation of UITask */
	UITaskHandle = osThreadNew(StartUITask, NULL, &UITask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 50;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 8399;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA0 PA1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PD12 PD13 PD14 PD15 */
	GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Immediate, short FAULT banner for emergencies (SafetyTask may call this)
static inline void LCD_ShowFaultBanner(const char *line1, const char *line2)
{
	if (osMutexAcquire(LCD_MutexHandle, 10) == osOK) {
		LCD_Clear();                    // (driver already waits the required time)
		LCD_PrintAt(0,0, line1);        // top line
		LCD_PrintAt(0,1, line2);        // second line (adjust if 20x4)
		osMutexRelease(LCD_MutexHandle);
	}
}

// For regular UI refreshes (no clear → minimal flicker)
// Pads lines to 16 chars so old characters don’t linger.
static inline void LCD_ShowLinesNoClear(const char *l1, const char *l2)
{
	char line1[24], line2[24];
	snprintf(line1, sizeof(line1), "%-16.16s", l1);
	snprintf(line2, sizeof(line2), "%-16.16s", l2);

	if (osMutexAcquire(LCD_MutexHandle, 10) == osOK) {
		LCD_PrintAt(0,0, line1);
		LCD_PrintAt(0,1, line2);
		osMutexRelease(LCD_MutexHandle);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSafetyTask */
/**
 * @brief  Function implementing the SafetyTask thread.
 * @param  argument: Not used
 * @retval None
 */


/* ===================== Task stubs ===================== */
/* USER CODE END Header_StartSafetyTask */
void StartSafetyTask(void *argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	(void)argument;

	FaultCode_t fault;

//	for (;;) {
//
//		// React if EITHER a queued code is available OR a trip sem arrives.
//		if (osMessageQueueGet(FaultQHandle, &fault, NULL, 0) == osOK ||osSemaphoreAcquire(FaultTripSemHandle, 0) == osOK) {
//
//			// If we didn't get a code from the queue yet, try again quickly;
//			// if still none, fall back to a sensible default
//			if (osMessageQueueGet(FaultQHandle, &fault, NULL, 0) != osOK) {
//				fault = FAULT_NONE;  // fallback reason
//			}
//
//			// 1) Trip immediately: force heater OFF
//			BTS7960_SetDuty_CH3(0);
//
//			// 2) Latch the code for UI
//			g_fault = fault;
//
//			// 3) Wake UI to redraw the FAULT screen now
//			osSemaphoreRelease(BtnEventSemHandle);
//		}
//
//		osDelay(10);// runs every 10 ms
//	}

	for (;;) {
	    // Wait until FaultTripSem is given by another task / Keep me asleep until something happens (fault in queue or trip semaphore)
	    osSemaphoreAcquire(FaultTripSemHandle, osWaitForever);

	    // Try to read a code from the queue
	    if (osMessageQueueGet(FaultQHandle, &fault, NULL, 0) != osOK) {
	      fault = FAULT_NONE;   // default reason if queue was empty
	    }

	    // 1) Turn heater OFF
	    BTS7960_SetDuty_CH3(0);

	    // 2) Remember the fault for UI
	    g_fault = fault;

	    // 3) Wake UI so it redraws with the FAULT screen
	    osSemaphoreRelease(BtnEventSemHandle);
	  }



	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartPIDTask */
/**
 * @brief Function implementing the PIDTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPIDTask */
void StartPIDTask(void *argument)
{
	/* USER CODE BEGIN StartPIDTask */
	/* Infinite loop */
	(void)argument;

	//float temp;
	//float setpt  ;

	// Keep the last known values so we never use uninitialized data
	  float last_temp = 25.0f;   // start assumption
	  float last_sp   = 50.0f;   // seed this in UI or main() too
	  const float EPS = 2.2f;    // small deadband (°C) to avoid chatter near setpoint

	for (;;) {
		//	    // 1) Wait for the sample tick (post PID_TickSem every DT_S seconds)
		//	    //osSemaphoreAcquire(PID_TickSemHandle, osWaitForever);
		//
		//	    // 2) Grab latest measurement + setpoint (non-blocking reads are fine)
		//	    (void)osMessageQueueGet(TempQHandle,     &temp_c, NULL, 0);
		//	    (void)osMessageQueueGet(SetPointQHandle, &setpt,  NULL, 0);
		//
		//	    // 3) Sensor failure sentinel from TempTask => heater OFF
		//	    if (temp_c < -999.0f) {
		//	      BTS7960_SetDuty_CH3(0.0f);
		//	      continue;
		//	    }
		//
		//	    // 4) Mutexed tunings read (keeps your original pattern and allows future live-tuning)
		//	    osMutexAcquire(PIDParam_MutexHandle, osWaitForever);
		//	    float kp = g_pid.kp, ki = g_pid.ki, kd = g_pid.kd;  // currently the same values set in main()
		//	    osMutexRelease(PIDParam_MutexHandle);
		//
		//	    // If you later change KP/KI/KD elsewhere under the same mutex, this applies them safely:
		//	    PID_SetTunings(&g_pid, kp, ki, kd);
		//
		//	    // 5) Compute control (percent duty due to output limits in PID_Init)
		//	    float duty = PID_Compute(&g_pid, setpt, temp_c);
		//
		//	    // 6) Extra explicit clamp (redundant)
		//	    if (duty < 0.0f)   duty = 0.0f;
		//	    if (duty > 100.0f) duty = 100.0f;
		//
		//	    // 7) Drive BTS7960 (single-direction heater)
		//	    BTS7960_SetDuty_CH3(duty);



		// run PID every 200 ms
		// 1) Try to get newest TEMP and SETPOINT (non-blocking). If there’s nothing, keep last_*.
		    float t_in, sp_in;
		    if (osMessageQueueGet(TempQHandle, &t_in, NULL, 0) == osOK) {
		      last_temp = t_in;
		    }
		    if (osMessageQueueGet(SetPointQHandle, &sp_in, NULL, 0) == osOK) {
		      last_sp = sp_in;
		    }

		    // 2) Sensor-failure sentinel: if TEMP < -999, force heater OFF and skip PID
		    if (last_temp < -999.0f) {
		      BTS7960_SetDuty_CH3(0.0f);
		      BTS7960_SetDuty_CH4(0.0f);
		      osDelay(200);                 // MUST match PID_Init dt_s (0.2 s)
		      continue;
		    }

		    // 3) Optional guard: if we are already at/above setpoint (within EPS), don’t heat.
		    //    This makes behavior intuitive for beginners. PID would also trend to ~0% anyway.
		    float duty = 0.0f;
		    if (last_temp < (last_sp - EPS)) {
		      // (Optional) still allow live-tuning; otherwise you can just remove the mutex reads.
		      osMutexAcquire(PIDParam_MutexHandle, osWaitForever);
		      float kp = g_pid.kp, ki = g_pid.ki, kd = g_pid.kd;
		      osMutexRelease(PIDParam_MutexHandle);
		      PID_SetTunings(&g_pid, kp, ki, kd);

		      duty = PID_Compute(&g_pid, last_sp, last_temp);
		      if (duty < 0.0f)
		    	  duty = 0.0f;
		      if (duty > 100.0f)
		    	  duty = 100.0f;
		    } else {
		      duty = 0.0f;  // above (or very near) setpoint → no heating
		    }

		    // 4) Drive the H-bridge for heating: CH3 = duty, CH4 = 0 (avoid braking)
		    BTS7960_SetDuty_CH3(duty);
		    BTS7960_SetDuty_CH4(0.0f);

		    // 5) Fixed-rate loop: keep this equal to dt_s you used in PID_Init
		    osDelay(200);   // 200 ms → dt_s = 0.2f
		  }
	/* USER CODE END StartPIDTask */
}

/* USER CODE BEGIN Header_StartTempTask */
/**
  * Temperature producer task:
 * - Sole owner of the DS18B20 sensor read.
 * - Publishes ONLY the freshest, clean value to TempQ (float).
 * - Debounces brief sensor glitches; raises a fault only after consecutive fails.
 * - UI and PID just consume from TempQ; no filtering needed downstream.
 */
/* USER CODE END Header_StartTempTask */
void StartTempTask(void *argument)
{
	/* USER CODE BEGIN StartTempTask */
	/* Infinite loop */
	(void)argument;

	  DS18B20_Init(25.0f);  // seed at boot with a reasonable ambient

	  for (;;) {
	    float temp_c = 0.0f;
	    bool  trip_now = false;

	    // Own the 1-Wire bus while reading
	    if (osMutexAcquire(OneWire_MutexHandle, osWaitForever) == osOK) {
	      (void)DS18B20_ReadTempC_Watchdog(&temp_c, &trip_now);
	      osMutexRelease(OneWire_MutexHandle);
	    }

	    if (trip_now) {
	      // The driver decided we've gone >15s without a good read
	      FaultCode_t f = FAULT_SENSOR_FAIL;
	      (void)osMessageQueuePut(FaultQHandle, &f, 0, 0);
	      osSemaphoreRelease(FaultTripSemHandle);
	    }

	    Telemetry_UpdateTemp(temp_c);
	    // Publish ONLY the newest value (float queue!)
	    while (osMessageQueueGetCount(TempQHandle) > 0) {
	      float junk;
	      (void)osMessageQueueGet(TempQHandle, &junk, NULL, 0);
	    }
	    (void)osMessageQueuePut(TempQHandle, &temp_c, 0, 0);

	    // If you want PID to step each time:
	    // osSemaphoreRelease(PID_TickSemHandle);

	    osDelay(50); // conversion time (~750–800 ms) dominates
	  }
	/* USER CODE END StartTempTask */
}

/* USER CODE BEGIN Header_StartPowerTask */
/**
 * @brief Function implementing the PowerTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPowerTask */
void StartPowerTask(void *argument)
{
	/* USER CODE BEGIN StartPowerTask */
	/* Infinite loop */
	(void)argument;
	for (;;) {
		osSemaphoreAcquire(Power_DoneSemHandle, osWaitForever);

		PowerSample_t p = {0};
		// p = power_sensor_read_frame();  // compute volts/amps/watts/ return an arrys

		(void)osMessageQueuePut(PowerQHandle, &p, 0, 0);

		if (/* p.watts > limit */ 0) {// we have to have a limit of power
			FaultCode_t f = FAULT_OVERCURRENT;
			(void)osMessageQueuePut(FaultQHandle, &f, 0, 0);
			osSemaphoreRelease(FaultTripSemHandle);
		}
	}
	/* USER CODE END StartPowerTask */
}

/* USER CODE BEGIN Header_StartUITask */
/**
 * @brief Function implementing the UITask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUITask */
void StartUITask(void *argument)
{
	/* USER CODE BEGIN StartUITask */
	/* Infinite loop */
	(void)argument;

	float last_temp  = 0.0f;
	float set_point  = 30.0f;   // default setpoint
	float last_watts = 0.0f;
	const char *mode;
	const float threshold = 2.0f;  // 2% threshold to avoid flicker

	// One-time publish so the rest of the system knows our starting SP
	(void)osMessageQueuePut(SetPointQHandle, &set_point, 0, 0);
	Telemetry_UpdateSetpoint(set_point);

	for (;;) {
		//		// WAKE ON IN BUTTONS OPTION
		//		if (osSemaphoreAcquire(BtnEventSemHandle, 50) == osOK) {
		//			// debounce; update setpoint if changed:
		//			(void)osMessageQueuePut(SetPointQHandle, &set_point, 0, 0);
		//			Telemetry_UpdateSetpoint(set_point);
		//
		//		}

		// Fetch latest values
		(void)osMessageQueueGet(TempQHandle,  &last_temp, NULL, 0);

		PowerSample_t power;
		if (osMessageQueueGet(PowerQHandle, &power, NULL, 0) == osOK)
			last_watts = power.watts;

		// If fault latched => show FAULT banner (UI is the only printer)
		if (g_fault != FAULT_NONE) {
			switch (g_fault) {
			case FAULT_OVERTEMP:
				LCD_ShowFaultBanner("FAULT: OVERTEMP", "Heater  OFF     ");
				break;
			case FAULT_OVERCURRENT:
				LCD_ShowFaultBanner("FAULT: OVERCURR", "Heater  OFF     ");
				break;
			case FAULT_SENSOR_FAIL:
				LCD_ShowFaultBanner("FAULT: SENSOR  ", "Check wiring    ");
				break;
			default:
				LCD_ShowFaultBanner("!!!   FAULT   !!!", "Heater  OFF     ");
				break;
			}
			osDelay(200);
			continue; // keep showing the banner until g_fault is cleared
		}

		// Get current PWM duty (percentage)
		float duty = BTS7960_GetDuty_CH3();   // or CH3 channel drives the heater
		if (duty > threshold) {
		    mode = "HEAT";
		} else {
		    mode = "IDLE";
		}

		char l1[24], l2[24];
		// fills them with formatted tex
		snprintf(l1, sizeof(l1), "P:%5.1fW T:%4.1f", last_watts, last_temp);
		snprintf(l2, sizeof(l2), "SP:%4.1f  %s",     set_point,   mode);

		// Smooth update (no LCD_Clear => minimal flicker)
		LCD_ShowLinesNoClear(l1, l2);

		osDelay(200); // ~5 Hz refresh
	}
	/* USER CODE END StartUITask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6)
	{
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
