/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "ssd1306.h"
//#include "ssd13062.h"
#include "ssd1306_tests.h"
//#include "ssd1306.c"
//#include "ssd1306_tests.c"
#include "stm32f4xx_it.h"
#include "semphr.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "myConstants.h"
#include <string>
#include "math.h"
#include "Atan_Overdrive.h"
#include "cubic_overdrive.h"
#include "EQ_High_Shelving.h"
#include "EQ_Low_Shelving.h"
#include "Exp_Distortion.h"
#include "First_Order_High_Pass.h"
#include "MOD_Tremolo.h"
#include "my_distortion.h"
#include "Tremolo.h"
#include "MOD_Flanger_1.h"
#include <stdio.h>
#include <string.h>
#include "ee24.h"
#include "Delay.h"
#include "Delay2.h"
#include "CF.h"
#include "reverb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
using namespace std;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim4_up;

UART_HandleTypeDef huart2;

/* Definitions for blinker */
osThreadId_t blinkerHandle;
const osThreadAttr_t blinker_attributes = {
  .name = "blinker",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for oled */
osThreadId_t oledHandle;
const osThreadAttr_t oled_attributes = {
  .name = "oled",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for metronomeSync */
osThreadId_t metronomeSyncHandle;
const osThreadAttr_t metronomeSync_attributes = {
  .name = "metronomeSync",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for click */
osThreadId_t clickHandle;
const osThreadAttr_t click_attributes = {
  .name = "click",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

EE24_HandleTypeDef ee24;

#define pi 3.1415926
#define Fs 50000
//int Fs = 50000;

SemaphoreHandle_t screenChangeSem;
SemaphoreHandle_t blinkSem;
SemaphoreHandle_t clickSem;
SemaphoreHandle_t soundSem;
//#define PAGE_NUM  64*4
//#define PAGE_SIZE 16*2
#define PAGE_NUM  256
#define PAGE_SIZE 32

//#define EEPROM_ADDR 0xA0
#define EEPROM_ADDR 0x50

#define EEPROM_I2C &hi2c2
int rotationDirection=1;

int currentBPM=120;
bool metronomeLED=false;
bool metronomeClick=false;
bool metronomeIsOn = false;
int totalNum=10;
string buttonPressed="right";

float dacFloatValue=0.2;
uint32_t dacIntValue=0;

#define bufferSize 500
uint32_t adcBuffer[bufferSize];
uint32_t dacBuffer[bufferSize];

#define maxMem 8000

float xMemory [maxMem];
float yMemory [maxMem];
CF CF1,CF2,CF3,CF4;
delay2 AP1,AP2,AP3;
float yCF1[1900];
float yCF2[1600];
float yCF3[2100];
float yCF4[2300];

float yAP1[300];
float yAP2[100];
float yAP3[50];

functionFind currentChainFunctions[MAX_PEDALS_PER_CHAIN];
allGuitarChains allPedalsData;

High_1st myFirstOrder;





int currentMenuOption=0;
int currentPedalOption=0;
int currentscreen=menuNumber;
int currentChainOption=0;
int metronomeMenuOption=0;
int individualPedaln=0;
string currentScreen="menu";
uint32_t PEDALADDRESS=12;
int currentPedalChosen = 0;
int individualPedalMenu=0;
int individualPedalMenuOption=0;
int pedalMenu=0;
int pedalMenuOption=0;
int parameterMenu = 0;
int changePedalScreen=0;

int timeSignature=4;
int timesSigned=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM7_Init(void);
static void MX_I2C3_Init(void);
void StartBlinker(void *argument);
void StartOled(void *argument);
void startMetronomeSync(void *argument);
void startClick(void *argument);

/* USER CODE BEGIN PFP */

int currentPedalChain = 0;

void EEPROM_Read (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
void EEPROM_Write (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int32_t dacSinValues2[bufferSize];
int32_t dacSinValues[bufferSize];

void getSinVal(int frequency, double amplitude){
	for(int i = 0; i <bufferSize;i++){
		dacSinValues2[i]=(sin(i*2*pi/(Fs/frequency))*(4096*amplitude/3.3));
	}
}

void getSinVal2(int frequency, double amplitude){

	for(int i = 0; i <bufferSize;i++){
			dacSinValues[i]=(sin(i*2*pi/(Fs/frequency))*(4096*amplitude/3.3));
		}

}






void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
  if(GPIO_Pin==interruptButton_Pin){

      buttonPressed="right";
	  HAL_GPIO_TogglePin(interruptLED_GPIO_Port, interruptLED_Pin);
      xSemaphoreGiveFromISR(screenChangeSem, NULL); // Give semaphore from interrupt


  }else if(GPIO_Pin==interruptButton2_Pin){

	  	buttonPressed="left";
	  	HAL_GPIO_TogglePin(interruptLED_GPIO_Port, interruptLED_Pin);
	    xSemaphoreGiveFromISR(screenChangeSem, NULL); // Give semaphore from interrupt

  }else if(GPIO_Pin==rotation1_1_Pin){
	  int res = HAL_GPIO_ReadPin(rotation1_2_GPIO_Port,rotation1_2_Pin);
	  buttonPressed="rotation";
	  if(res==0){
		  rotationDirection=1;

	  }else{
		  rotationDirection=0;
	  }
	  xSemaphoreGiveFromISR(screenChangeSem, NULL);


  }else if(GPIO_Pin==interruptButton3_Pin){
	  buttonPressed="plus";
	  HAL_GPIO_TogglePin(interruptLED_GPIO_Port, interruptLED_Pin);
      xSemaphoreGiveFromISR(screenChangeSem, NULL); // Give semaphore from interrupt

  }

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
  HAL_Delay(20);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_DAC_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_TIM7_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
//  MOD_Tremolo_init(&tremPedal, 0.9f ,0.9f, (1000000.0f)/11.0f);
//  First_Order_High_Set_Parameters(&hp1Pedal,20000.0f,(1000000.0f)/11.0f);
//  Exp_Distortion_Init_Default(&expDistPedal,60);
  //HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  getSinVal(2000,0.1);
  getSinVal2(1000,0.1);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start(&htim8);

  for(int i=0; i< maxMem; i++){
	xMemory[i] = 0.0f;
	yMemory[i] = 0.0f;
  }

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
  ssd1306_Init();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  First_Order_High_Init(&myFirstOrder,10,Fs);



  	HAL_Delay(1000);




  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  screenChangeSem = xSemaphoreCreateBinary();
  blinkSem = xSemaphoreCreateBinary();
  clickSem = xSemaphoreCreateBinary();
  soundSem = xSemaphoreCreateBinary();
  xSemaphoreGive(soundSem);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of blinker */
  blinkerHandle = osThreadNew(StartBlinker, NULL, &blinker_attributes);

  /* creation of oled */
  oledHandle = osThreadNew(StartOled, NULL, &oled_attributes);
  /* creation of metronomeSync */
  metronomeSyncHandle = osThreadNew(startMetronomeSync, NULL, &metronomeSync_attributes);

  /* creation of click */
  clickHandle = osThreadNew(startClick, NULL, &click_attributes);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, dacBuffer, bufferSize, DAC_ALIGN_12B_R);
  HAL_ADC_Start_DMA(&hadc1, adcBuffer, bufferSize);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T8_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 900-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 12500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 90-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 18-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 180-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 20-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|interruptLED_Pin|interruptLED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(flashLED_GPIO_Port, flashLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin interruptLED_Pin interruptLED2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|interruptLED_Pin|interruptLED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : interruptButton_Pin interruptButton2_Pin */
  GPIO_InitStruct.Pin = interruptButton_Pin|interruptButton2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : rotation1_1_Pin */
  GPIO_InitStruct.Pin = rotation1_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(rotation1_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : interruptButton3_Pin */
  GPIO_InitStruct.Pin = interruptButton3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(interruptButton3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : myButtonPin2_Pin */
  GPIO_InitStruct.Pin = myButtonPin2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(myButtonPin2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : flashLED_Pin */
  GPIO_InitStruct.Pin = flashLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(flashLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : rotation1_2_Pin */
  GPIO_InitStruct.Pin = rotation1_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(rotation1_2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//

uint32_t adcValue;

float processVal(float toProcess, functionFind* currentPedal){
    switch (currentPedal->pedalType) {
		case noChang:
			return toProcess;
		case OverA:
			return Atan_Overdrive_update((Atan_OD*)currentPedal->structPointer, toProcess);
		case OverC:
			return cubic_overdrive_update((cubic_overdrive*)currentPedal->structPointer, toProcess);
		case eqHigh:
			return EQ_High_Shelving_Update((HS*)currentPedal->structPointer, toProcess);
		case eqLow:
			return EQ_Low_Shelving_Update((LS*)currentPedal->structPointer, toProcess);
		case distE:
			return Exp_Distortion_Update((Exp_Dist*)currentPedal->structPointer, toProcess);
		case HP1:
			return First_Order_High_Update((High_1st*)currentPedal->structPointer, toProcess);
		case tremM:
			return MOD_Tremolo_Update((MOD_Tremolo*)currentPedal->structPointer, toProcess);

		case Delay:
			//return Tremolo_update((Tremolo*)currentPedal->structPointer, toProcess);
			return delay_Update((delay*)currentPedal->structPointer,toProcess);
		case Reverb:
//			return my_distortion_update((my_distortion_struct*)currentPedal->structPointer, toProcess);
			return reverb_Update((reverb*)currentPedal->structPointer,toProcess);



    }
    return 0.0f;

}

//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//	if (xSemaphoreTakeFromISR(soundSem, &xHigherPriorityTaskWoken) == pdTRUE){
//
//		float valueInFloat;
//				for(int i = 0; i < bufferSize/2;i++){
//					//valueInFloat = (float)((int32_t)(adcBuffer[i] - (1 << 11)) / (pow(2.0f, 11)));
//					//valueInFloat=(static_cast<float>(adcBuffer[i]) / 2047.5f) - 1.0f;
//					valueInFloat=((float)adcBuffer[i])/2048;
//					valueInFloat = First_Order_High_Update(&myFirstOrder,valueInFloat);
//					for(int j = 0; j < MAX_PEDALS_PER_CHAIN;j++){
//						valueInFloat=processVal(valueInFloat,&currentChainFunctions[j]);
//
//					}
//					dacBuffer[i]=(uint32_t)((valueInFloat+1)*(2048));
//		}
//		if(metronomeIsOn){
//			for(int i = 0 ; i < bufferSize/2;i++){
//				   //int currInt = adcBuffer[i];
//					dacBuffer[i]+=dacSinValues2[i];
//			}
//		}
//		xSemaphoreGiveFromISR(soundSem, &xHigherPriorityTaskWoken);
//	}
//
//	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//
//
//}
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//	if (xSemaphoreTakeFromISR(soundSem, &xHigherPriorityTaskWoken) == pdTRUE){
//
//		float valueInFloat;
//		for(int i = bufferSize/2; i < bufferSize;i++){
//			//valueInFloat = (float)((int32_t)(adcBuffer[i] - (1 << 11)) / (pow(2.0f, 11)));
//			//valueInFloat=(static_cast<float>(adcBuffer[i]) / 2047.5f) - 1.0f;
//			valueInFloat=((float)adcBuffer[i])/2048;
//			valueInFloat = First_Order_High_Update(&myFirstOrder,valueInFloat);
//			for(int j = 0; j < MAX_PEDALS_PER_CHAIN;j++){
//				valueInFloat=processVal(valueInFloat,&currentChainFunctions[j]);
//
//			}
//			dacBuffer[i]=(uint32_t)((valueInFloat+1)*(2048));
//		}
//		if(metronomeIsOn){
//			for(int i = bufferSize/2 ; i < bufferSize;i++){
//				   //int currInt = adcBuffer[i];
//					dacBuffer[i]+=dacSinValues2[i];
//			}
//		}
//		xSemaphoreGiveFromISR(soundSem, &xHigherPriorityTaskWoken);
//	}
//
//    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//
//}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (xSemaphoreTakeFromISR(soundSem, &xHigherPriorityTaskWoken) == pdTRUE){

		float valueInFloat;
		for(int i = 0; i < bufferSize/2;i++){
			//valueInFloat = (float)((int32_t)(adcBuffer[i] - (1 << 11)) / (pow(2.0f, 11)));
			//valueInFloat=(static_cast<float>(adcBuffer[i]) / 2047.5f) - 1.0f;
			valueInFloat=((float)adcBuffer[i])/2048;
			valueInFloat = First_Order_High_Update(&myFirstOrder,valueInFloat);
			for(int j = 0; j < MAX_PEDALS_PER_CHAIN;j++){
				valueInFloat=processVal(valueInFloat,&currentChainFunctions[j]);

			}
			dacBuffer[i]=(uint32_t)((valueInFloat+1)*(2048));
		}
		if(metronomeIsOn){
			if(timesSigned==0)
			for(int i = 0 ; i < bufferSize/2;i++){
				   //int currInt = adcBuffer[i];
					dacBuffer[i]+=dacSinValues2[i];
			}
			else{
				for(int i = 0 ; i < bufferSize/2;i++){
					   //int currInt = adcBuffer[i];
						dacBuffer[i]+=dacSinValues[i];
				}
			}

		}
		xSemaphoreGiveFromISR(soundSem, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);


}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (xSemaphoreTakeFromISR(soundSem, &xHigherPriorityTaskWoken) == pdTRUE){

		float valueInFloat;
		for(int i = bufferSize/2; i < bufferSize;i++){
			valueInFloat=((float)adcBuffer[i])/2048;
			valueInFloat = First_Order_High_Update(&myFirstOrder,valueInFloat);
			for(int j = 0; j < MAX_PEDALS_PER_CHAIN;j++){
				valueInFloat=processVal(valueInFloat,&currentChainFunctions[j]);
			}
			dacBuffer[i]=(uint32_t)((valueInFloat+1)*(2048));
		}
		if(metronomeIsOn){
					if(timesSigned==0)
					for(int i = bufferSize/2 ; i < bufferSize;i++){
						   //int currInt = adcBuffer[i];
							dacBuffer[i]+=dacSinValues2[i];
					}
					else{
						for(int i = bufferSize/2 ; i < bufferSize;i++){
							   //int currInt = adcBuffer[i];
								dacBuffer[i]+=dacSinValues[i];
						}
					}
		}
		xSemaphoreGiveFromISR(soundSem, &xHigherPriorityTaskWoken);
	}
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}












//SemaphoreHandle_t screenChangeSem;

void drawMenuScreen(){
	//char snum[5];
  //  itoa(totalNum, snum, 10);
	int num = sizeof(MENUSTRINGS) / sizeof(MENUSTRINGS[0]);
	  ssd1306_Fill(Black);
	  ssd1306_DrawBitmap(126,0,epd_bitmap__scrollBar,1,64,White);
	  ssd1306_DrawBitmap(0,22,epd_bitmap__border,106,21,White);
	  ssd1306_DrawBitmap(3,2,MENUICONS[(currentMenuOption+(num-1))%num],16,16,White);
	  ssd1306_SetCursor(3+16+4, 2+4);
	  ssd1306_WriteString(MENUSTRINGS[(currentMenuOption+(num-1))%num], Font_7x10, White);
	  ssd1306_DrawBitmap(3,24,MENUICONS[(currentMenuOption+num)%num],16,16,White);
	  ssd1306_SetCursor(3+16+4, 24+4);
	  ssd1306_WriteString(MENUSTRINGS[(currentMenuOption+num)%num], Font_7x10, White);
	  ssd1306_DrawBitmap(3,46,MENUICONS[(currentMenuOption+(num+1))%num],16,16,White);
	  ssd1306_SetCursor(3+16+4, 46+4);
	  ssd1306_WriteString(MENUSTRINGS[(currentMenuOption+(num+1))%num], Font_7x10, White);
	  ssd1306_DrawBitmap(125,(64-7)*currentMenuOption/(num-1),epd_bitmap__handle,3,7,White);
	  ssd1306_UpdateScreen();
	  osDelay(1);
}
void drawPedalMenu(){
	char str[4];

	ssd1306_Fill(Black);
	ssd1306_DrawBitmap(126,0,epd_bitmap__scrollBar,1,64,White);
	ssd1306_DrawBitmap(0,22,epd_bitmap__border,106,21,White);

	ssd1306_SetCursor(3+16+4, 2+4);
	ssd1306_WriteString(allPedalsData.allChains[(pedalMenu+MAX_CHAINS-1)%MAX_CHAINS].name, Font_7x10, White);
	ssd1306_SetCursor(3+4, 2+4);
	sprintf(str, "%d", (pedalMenu+MAX_CHAINS-1)%MAX_CHAINS+1);
	ssd1306_WriteString(str, Font_7x10, White);

	ssd1306_SetCursor(3+16+4, 24+4);
	ssd1306_WriteString(allPedalsData.allChains[(pedalMenu)%MAX_CHAINS].name, Font_7x10, White);
	ssd1306_SetCursor(3+4, 24+4);
	sprintf(str, "%d", (pedalMenu)%MAX_CHAINS+1);
	ssd1306_WriteString(str, Font_7x10, White);

	ssd1306_SetCursor(3+16+4, 46+4);
	ssd1306_WriteString(allPedalsData.allChains[(pedalMenu+1)%MAX_CHAINS].name, Font_7x10, White);
	ssd1306_SetCursor(3+4, 46+4);
	sprintf(str, "%d", (pedalMenu+1)%MAX_CHAINS+1);
	ssd1306_WriteString(str, Font_7x10, White);

	ssd1306_DrawBitmap(125,(64-7)*pedalMenu/4,epd_bitmap__handle,3,7,White);
	ssd1306_UpdateScreen();
	osDelay(1);
}
void drawPedalOptions(){
	int num = sizeof(PEDALSMENUOPTIONSSTRINGS)/sizeof(PEDALSMENUOPTIONSSTRINGS[0]);
	ssd1306_Fill(Black);
	ssd1306_DrawBitmap(126,0,epd_bitmap__scrollBar,1,64,White);
	ssd1306_DrawBitmap(0,22,epd_bitmap__border,106,21,White);

	ssd1306_SetCursor(3+16+4, 2+4);
	ssd1306_WriteString(PEDALSMENUOPTIONSSTRINGS[(pedalMenuOption+num-1)%num], Font_7x10, White);

	ssd1306_SetCursor(3+16+4, 24+4);
	ssd1306_WriteString(PEDALSMENUOPTIONSSTRINGS[(pedalMenuOption)%num], Font_7x10, White);
	ssd1306_SetCursor(3+16+4, 46+4);
	ssd1306_WriteString(PEDALSMENUOPTIONSSTRINGS[(pedalMenuOption+1)%num], Font_7x10, White);
	ssd1306_DrawBitmap(125,(64-7)*pedalMenu/4,epd_bitmap__handle,3,7,White);
	ssd1306_UpdateScreen();
	osDelay(1);
}
void savePedals(){
	EE24_Write(&ee24, 0, (uint8_t*)&allPedalsData, sizeof(allPedalsData), 1000);
}
string indentifyPedal(uint8_t num){
	switch(num){
		case 0:
			return "nothing";
		case 1:
			return "atanOverdrive";
		case 2:
			return "cubicOverdrive";
		case 3:
			return "EQ high";
		case 4:
			return "EQ low";
		case 5:
			return "exp distortion";
		case 6:
			return "high pass filt";
		case 7:
			return "MOD tremolo";

		case Delay:
			return "delay";
		case Reverb:
			return "reverb";
	}
	return "error";





}
string nameArray;
char currentCharacter;
void drawEditNameMenu(){
	ssd1306_Fill(Black);
	ssd1306_SetCursor(3+4, 46+4);
	ssd1306_WriteString((char*)"delete - done - next", Font_6x8, White);
	ssd1306_SetCursor(3+4, 24+4);
	ssd1306_WriteString((char*)"11 chars max", Font_7x10, White);
	ssd1306_SetCursor(3+4, 2+4);
	ssd1306_WriteString(const_cast<char*>(nameArray.c_str()), Font_7x10, White);
	ssd1306_UpdateScreen();
	osDelay(1);
}
void drawIndividualPedalMenu(){
	//int num = sizeof(PEDALSMENUOPTIONSSTRINGS)/sizeof(PEDALSMENUOPTIONSSTRINGS[0]);
	ssd1306_Fill(Black);
	char str[4];
	ssd1306_DrawBitmap(126,0,epd_bitmap__scrollBar,1,64,White);
	ssd1306_DrawBitmap(0,22,epd_bitmap__border,106,21,White);
    string pedalName0 = indentifyPedal(allPedalsData.allChains[pedalMenu].allPedals[(individualPedalMenu-1+MAX_PEDALS_PER_CHAIN)%MAX_PEDALS_PER_CHAIN].parameters[0]);
    string pedalName1 = indentifyPedal(allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[0]);
    string pedalName2 = indentifyPedal(allPedalsData.allChains[pedalMenu].allPedals[(individualPedalMenu+1)%MAX_PEDALS_PER_CHAIN].parameters[0]);

    ssd1306_SetCursor(3+4, 2+4);
	sprintf(str, "%d", (individualPedalMenu-1+MAX_PEDALS_PER_CHAIN)%MAX_PEDALS_PER_CHAIN+1);
    ssd1306_WriteString(str, Font_7x10, White);
    ssd1306_SetCursor(3+16+4, 2+4);
	//ssd1306_WriteString(const_cast<char*>(pedalName.c_str()), Font_7x10, White);
	ssd1306_WriteString(const_cast<char*>(pedalName0.c_str()), Font_7x10, White);

	ssd1306_SetCursor(3+4, 24+4);
	sprintf(str, "%d", individualPedalMenu+1);
	ssd1306_WriteString(str, Font_7x10, White);
	ssd1306_SetCursor(3+16+4, 24+4);
	ssd1306_WriteString(const_cast<char*>(pedalName1.c_str()), Font_7x10, White);

	ssd1306_SetCursor(3+4, 46+4);
	sprintf(str, "%d",  (individualPedalMenu+1)%MAX_PEDALS_PER_CHAIN+1);
	ssd1306_WriteString(str, Font_7x10, White);
	ssd1306_SetCursor(3+16+4, 46+4);
	ssd1306_WriteString(const_cast<char*>(pedalName2.c_str()), Font_7x10, White);

	ssd1306_DrawBitmap(125,(64-7)*pedalMenu/4,epd_bitmap__handle,3,7,White);
	ssd1306_UpdateScreen();
	osDelay(1);
}
void drawIndividualPedalMenuOptions(){
	int num = sizeof(INDIVIDUALPEDALOPTIONSTRINGS)/sizeof(INDIVIDUALPEDALOPTIONSTRINGS[0]);
		ssd1306_Fill(Black);
		ssd1306_DrawBitmap(126,0,epd_bitmap__scrollBar,1,64,White);
		ssd1306_DrawBitmap(0,22,epd_bitmap__border,106,21,White);

		ssd1306_SetCursor(3+16+4, 2+4);
		ssd1306_WriteString(INDIVIDUALPEDALOPTIONSTRINGS[(individualPedalMenuOption+num-1)%num], Font_7x10, White);

		ssd1306_SetCursor(3+16+4, 24+4);
		ssd1306_WriteString(INDIVIDUALPEDALOPTIONSTRINGS[(individualPedalMenuOption)%num], Font_7x10, White);
		ssd1306_SetCursor(3+16+4, 46+4);
		ssd1306_WriteString(INDIVIDUALPEDALOPTIONSTRINGS[(individualPedalMenuOption+1)%num], Font_7x10, White);
		ssd1306_DrawBitmap(125,(64-7)*individualPedalMenuOption/(num-1),epd_bitmap__handle,3,7,White);
		ssd1306_UpdateScreen();
		osDelay(1);
}

char** numToArray(int num, int* size){
	switch(num){
			case 0:
	            *size = 0;
				return (char**)NOTHING_STRINGS;
			case 1:
				*size = 1;
				return (char**)ATAN_STRINGS;
			case 2:
				*size = 2;
				return (char**)CUBIC_STRINGS;
			case 3:
				*size = 2;
				return (char**)EQ_HIGH_STRINGS;
			case 4:
				*size = 2;
				return (char**)EQ_LOW_STRINGS;
			case 5:
				*size = 2;
				return (char**)EXP_DIST_STRINGS;
			case 6:
				*size = 1;
				return (char**)FIRST_ORDER_STRINGS;
			case 7:
				*size = 2;
				return (char**)MOD_TREM_STRINGS;
			case 8:
				*size = 3;
				return (char**)DELAY_STRINGS;
			case 9:
				*size = 3;
				return (char**)REVERB_STRINGS;
		}
	return 0;
}
void drawParameterEditMenu(){
	//int num = sizeof(PEDALSMENUOPTIONSSTRINGS)/sizeof(PEDALSMENUOPTIONSSTRINGS[0]);
	ssd1306_Fill(Black);
	ssd1306_DrawBitmap(126,0,epd_bitmap__scrollBar,1,64,White);




	int size;
	char** stringPointer = numToArray(allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[0],&size);
	if(size>0){
		ssd1306_DrawBitmap(0,22*parameterMenu,epd_bitmap__border,106,21,White);
	}
    if(size>0){
    	ssd1306_SetCursor(3+4, 2+4);
    	ssd1306_WriteString(stringPointer[0], Font_7x10, White);
    	ssd1306_SetCursor(3+90+4, 2+4);
    	char str[4];
		sprintf(str, "%d", allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[1]);
		ssd1306_WriteString(str, Font_7x10, White);
    }
    if(size>1){
    	ssd1306_SetCursor(3+4, 24+4);
    	ssd1306_WriteString(stringPointer[1], Font_7x10, White);
    	ssd1306_SetCursor(3+90+4, 24+4);
    	char str[4];
    	sprintf(str, "%d", allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[2]);
		ssd1306_WriteString(str, Font_7x10, White);
    }
	if(size>2){
		ssd1306_SetCursor(3+4, 46+4);
		ssd1306_WriteString(stringPointer[2], Font_7x10, White);
		ssd1306_SetCursor(3+90+4, 46+4);
		char str[4];
		sprintf(str, "%d", allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[3]);
		ssd1306_WriteString(str, Font_7x10, White);
	}

	ssd1306_DrawBitmap(125,(64-7)*pedalMenu/4,epd_bitmap__handle,3,7,White);
	ssd1306_UpdateScreen();
	osDelay(1);
}
int choosePedalMenu=0;
void drawchoosePedalMenu(){
	int num = sizeof(INDIVIDUALPEDALSSTRING)/sizeof(INDIVIDUALPEDALSSTRING[0]);
	ssd1306_Fill(Black);
	ssd1306_DrawBitmap(126,0,epd_bitmap__scrollBar,1,64,White);
	ssd1306_DrawBitmap(0,22,epd_bitmap__border,106,21,White);

	ssd1306_SetCursor(3+16+4, 2+4);
	ssd1306_WriteString(INDIVIDUALPEDALSSTRING[(choosePedalMenu+num-1)%num], Font_7x10, White);

	ssd1306_SetCursor(3+16+4, 24+4);
	ssd1306_WriteString(INDIVIDUALPEDALSSTRING[(choosePedalMenu)%num], Font_7x10, White);
	ssd1306_SetCursor(3+16+4, 46+4);
	ssd1306_WriteString(INDIVIDUALPEDALSSTRING[(choosePedalMenu+1)%num], Font_7x10, White);
	ssd1306_DrawBitmap(125,(64-7)*pedalMenu/4,epd_bitmap__handle,3,7,White);
	ssd1306_UpdateScreen();
	osDelay(1);
}
void drawMetronomeMenu(){
//	while(true){
//	  	HAL_I2C_Mem_Read(&hi2c2, EEPROM_ADDR, 0 + 16 * 0, 1, (uint8_t*)(&read[0]), 16,1000);
//	  	osDelay(100);
//	  }
	  ssd1306_Fill(Black);
	  ssd1306_DrawBitmap(126,0,epd_bitmap__scrollBar,1,64,White);
	  ssd1306_DrawBitmap(0,22,epd_bitmap__border,106,21,White);


	  ssd1306_SetCursor(3+16+4, 2+4);
	  ssd1306_WriteString(METRONOMEMENUSTRINGS[(metronomeMenuOption+4)%5], Font_7x10, White);
	  //ssd1306_DrawBitmap(3,24,PEDALNAMES[(currentPedalOption+1)%5],16,16,White);
	  ssd1306_SetCursor(3+16+4+50, 2+4);
	  if(metronomeMenuOption==1){
		  char str[6];
		  		      sprintf(str, "%d", currentBPM);
		  	  		  ssd1306_WriteString(str, Font_7x10, White);
	  }else if(metronomeMenuOption==2){
		  if(metronomeLED==true){
		  	  	  ssd1306_WriteString((char*)"ON", Font_7x10, White);
		        }else	{
		      	  ssd1306_WriteString((char*)"OFF", Font_7x10, White);
		   }
	  }else if(metronomeMenuOption==3){
		  if(metronomeClick==true){
				  ssd1306_WriteString((char*)"ON", Font_7x10, White);
				}else	{
				  ssd1306_WriteString((char*)"OFF", Font_7x10, White);
		   }

	  }else if(metronomeMenuOption==0){
		  char str[6];
			  sprintf(str, "%d", timeSignature);
			  ssd1306_WriteString(str, Font_7x10, White);
	  }


	  ssd1306_SetCursor(3+16+4, 24+4);
	  ssd1306_WriteString(METRONOMEMENUSTRINGS[(metronomeMenuOption+5)%5], Font_7x10, White);
	  ssd1306_SetCursor(3+16+4+50, 24+4);
	  if(metronomeMenuOption==0){
		      char str[6];
		      itoa(currentBPM, str, 10);
	  		  ssd1306_WriteString(str, Font_7x10, White);
	  	  }else if(metronomeMenuOption==1){
	  		  if(metronomeLED==true){
	  		  	  	  ssd1306_WriteString((char*)"ON", Font_7x10, White);
	  		        }else	{
	  		      	  ssd1306_WriteString((char*)"OFF", Font_7x10, White);
	  		   }
	  	  }else if(metronomeMenuOption==2){
	  		  if(metronomeClick==true){
	  				  ssd1306_WriteString((char*)"ON", Font_7x10, White);
	  				}else	{
	  				  ssd1306_WriteString((char*)"OFF", Font_7x10, White);
	  		   }
	  	  }else if(metronomeMenuOption==4){
	  		char str[6];
	  		sprintf(str, "%d", timeSignature);
	  		ssd1306_WriteString(str, Font_7x10, White);
	  	  }

	  //ssd1306_DrawBitmap(3,46,MENUICONS[(currentPedalOption+2)%5],16,16,White);
	  ssd1306_SetCursor(3+16+4, 46+4);
	  ssd1306_WriteString(METRONOMEMENUSTRINGS[(metronomeMenuOption+6)%5], Font_7x10, White);
	  ssd1306_SetCursor(3+16+4+50, 46+4);

	  if(metronomeMenuOption==3){
		  char str[6];
		  		      sprintf(str, "%d", timeSignature);
		  	  		  ssd1306_WriteString(str, Font_7x10, White);
	  	  }else if(metronomeMenuOption==0){
	  		  if(metronomeLED==true){
	  		  	  	  ssd1306_WriteString((char*)"ON", Font_7x10, White);
	  		        }else	{
	  		      	  ssd1306_WriteString((char*)"OFF", Font_7x10, White);
	  		   }
	  	  }else if(metronomeMenuOption==4){
	  		char str[6];
		    sprintf(str, "%d", currentBPM);
		    ssd1306_WriteString(str, Font_7x10, White);
	  	  }
	  	  else if(metronomeMenuOption==1){
	  		  if(metronomeClick==true){
	  				  ssd1306_WriteString((char*)"ON", Font_7x10, White);
	  				}else	{
	  				  ssd1306_WriteString((char*)"OFF", Font_7x10, White);
	  		   }
	  	  }else if(timeSignature==3){
	  		char str[6];
		  sprintf(str, "%d", timeSignature);
		  ssd1306_WriteString(str, Font_7x10, White);
	  	  }
	  ssd1306_DrawBitmap(125,(64-7)*metronomeMenuOption/4,epd_bitmap__handle,3,7,White);
	  ssd1306_UpdateScreen();
	  osDelay(1);
}

void drawChainMenu(){

}

void drawPedalMenuOptions(){
	  ssd1306_Fill(Black);
	  ssd1306_DrawBitmap(126,0,epd_bitmap__scrollBar,1,64,White);
	  ssd1306_DrawBitmap(0,22,epd_bitmap__border,106,21,White);

	  //ssd1306_DrawBitmap(3,2,PEDALNAMES[currentPedalOption%5],16,16,White);
	  ssd1306_SetCursor(3+16+4, 2+4);
	  ssd1306_WriteString(PEDALSMENUOPTIONSSTRINGS[(pedalMenuOption+2)%3], Font_7x10, White);
	  //ssd1306_DrawBitmap(3,24,PEDALNAMES[(currentPedalOption+1)%5],16,16,White);
	  ssd1306_SetCursor(3+16+4, 24+4);
	  ssd1306_WriteString(PEDALSMENUOPTIONSSTRINGS[(pedalMenuOption+3)%3], Font_7x10, White);
	  //ssd1306_DrawBitmap(3,46,MENUICONS[(currentPedalOption+2)%5],16,16,White);
	  ssd1306_SetCursor(3+16+4, 46+4);
	  ssd1306_WriteString(PEDALSMENUOPTIONSSTRINGS[(pedalMenuOption+4)%3], Font_7x10, White);
	  ssd1306_DrawBitmap(125,(64-7)*pedalMenuOption/2,epd_bitmap__handle,3,7,White);
	  ssd1306_UpdateScreen();
	  osDelay(1);
}

void deletePedal(int myP){

}

void freeOldFunctions(){
	for(int i = 0; i < (int)sizeof(currentChainFunctions)/(int)sizeof(currentChainFunctions[0]);i++){
		free(currentChainFunctions[i].structPointer);
	}
}

void makeNewPedalFunctions(){
    singlePedalChain currentPedalData = allPedalsData.allChains[pedalMenu];
    for(int i = 0; i < MAX_PEDALS_PER_CHAIN;i++){
    	uint8_t* params=currentPedalData.allPedals[i].parameters;
    	currentChainFunctions[i].pedalType=params[0];
    	switch(params[0]){
    		case noChang:
    			currentChainFunctions[i].structPointer=nullptr;
    	    	break;
    	    case OverA:
    	    	currentChainFunctions[i].structPointer = malloc(sizeof(Atan_OD));
    	    	Atan_Overdrive_init((Atan_OD*)currentChainFunctions[i].structPointer,params[1]);
    	    	break;
    	    case OverC:
    	    	currentChainFunctions[i].structPointer = malloc(sizeof(cubic_overdrive));
    	    	cubic_overdrive_init((cubic_overdrive*)currentChainFunctions[i].structPointer,params[1],params[2]);
    	        break;
    	    case eqHigh:
    	    	currentChainFunctions[i].structPointer = malloc(sizeof(HS));
    	    	EQ_High_Shelving_Init((HS*)currentChainFunctions[i].structPointer,Fs,params[1],params[2]);
    	        break;
    	    case eqLow:
    	    	currentChainFunctions[i].structPointer = malloc(sizeof(LS));
    	    	EQ_Low_Shelving_Init((LS*)currentChainFunctions[i].structPointer,Fs,params[1],params[2]);
    			break;
    	    case distE:
    	    	currentChainFunctions[i].structPointer = malloc(sizeof(Exp_Dist));
    	    	Exp_Distortion_Init_Default((Exp_Dist*)currentChainFunctions[i].structPointer,params[1]);
    	    	break;
    	    case HP1:
    	    	currentChainFunctions[i].structPointer = malloc(sizeof(High_1st));
    	    	First_Order_High_Init((High_1st*)currentChainFunctions[i].structPointer,params[1],Fs);
    			break;
    	    case tremM:
    	    	currentChainFunctions[i].structPointer = malloc(sizeof(MOD_Tremolo));
    	    	MOD_Tremolo_init((MOD_Tremolo*)currentChainFunctions[i].structPointer,Fs,params[2],params[1]);
    			break;

    	    case Delay:
    	    	currentChainFunctions[i].structPointer = malloc(sizeof(delay));
    	    	delay_init((delay*)currentChainFunctions[i].structPointer,Fs,params[1],params[2],params[3],xMemory,yMemory,maxMem);
    			break;
    	    case Reverb:
//    	    	currentChainFunctions[i].structPointer = malloc(sizeof(my_distortion_struct));
//    	    	my_disortion_init((my_distortion_struct*)currentChainFunctions[i].structPointer);
//    			break;
				currentChainFunctions[i].structPointer = malloc(sizeof(reverb));
				reverb_init((reverb*)currentChainFunctions[i].structPointer,Fs,params[1],params[2],xMemory,
						maxMem, &CF1, &CF2, &CF3, &CF4, &AP1, &AP2, &AP3, yCF1, yCF2, yCF3, yCF4, yAP1, yAP2, yAP3);
				//MOD_Flanger_1_init((Flanger_1*)currentChainFunctions[i].structPointer,Fs,params[1],params[2],0.7,0.7,0.7,x,y,400);
				break;
    	}
    }
}
void makeNewPedals(){
	if (xSemaphoreTake(soundSem, portMAX_DELAY)){
		freeOldFunctions();
		makeNewPedalFunctions();
	}
	xSemaphoreGive(soundSem);

}
void menuChange(){
	if(currentScreen=="individualPedalsMenu"){  //this is each individual (distortion, delay...)
        if(buttonPressed=="right"){
            individualPedalMenu++;
            if(individualPedalMenu>=MAX_PEDALS_PER_CHAIN)individualPedalMenu=individualPedalMenu-MAX_PEDALS_PER_CHAIN;
            drawIndividualPedalMenu();
        }else if(buttonPressed == "left"){
        	individualPedalMenu--;
        	if(individualPedalMenu<0)individualPedalMenu=MAX_PEDALS_PER_CHAIN-1;
            drawIndividualPedalMenu();
        }else if(buttonPressed == "plus"){
        	individualPedalMenuOption=0;
        	currentScreen="individualPedalsMenuOption";
        	drawIndividualPedalMenuOptions();
        }

	}else if(currentScreen=="individualPedalsMenuOption"){
        if(buttonPressed=="right"){
        	individualPedalMenuOption++;
        	individualPedalMenuOption=individualPedalMenuOption%(sizeof(INDIVIDUALPEDALOPTIONSTRINGS)/sizeof(INDIVIDUALPEDALOPTIONSTRINGS[0]));
        	drawIndividualPedalMenuOptions();
        }else if(buttonPressed == "left"){
        	individualPedalMenuOption--;
        	if(individualPedalMenuOption<0)individualPedalMenuOption=(sizeof(INDIVIDUALPEDALOPTIONSTRINGS)/sizeof(INDIVIDUALPEDALOPTIONSTRINGS[0]))-1;
        	drawIndividualPedalMenuOptions();
        }else if(buttonPressed == "plus"){
            if(individualPedalMenuOption==2){
            	individualPedalMenuOption=0;
            	currentScreen="pedalMenu";
            	drawPedalMenu();
            }else if(individualPedalMenuOption==1){
            	individualPedalMenuOption=0;
            	choosePedalMenu=0;
            	currentScreen="choosePedalMenu";
            	drawchoosePedalMenu();
            }else if(individualPedalMenuOption==0){
                parameterMenu=0;
                currentScreen="parameterEditMenu";
                drawParameterEditMenu();
            }
        }
	}else if(currentScreen == "parameterEditMenu"){
		if(buttonPressed=="plus"){
			parameterMenu=0;
			currentScreen = "individualPedalsMenu";
			drawIndividualPedalMenu();
		}else if(buttonPressed=="right"){
			int num;
			numToArray(allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[0],&num);
			if(num!=0){
				parameterMenu++;
				if(parameterMenu>=num)parameterMenu=0;
			}
			drawParameterEditMenu();
		}else if(buttonPressed=="left"){
			int num;
			numToArray(allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[0],&num);
			if(num!=0){
				parameterMenu--;
				if(parameterMenu<0)parameterMenu=num-1;
			}
			drawParameterEditMenu();
		}else if(buttonPressed=="rotation"){
			int num;
			numToArray(allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[0],&num);
			if(rotationDirection==1&&num>0){
				allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[parameterMenu+1]++;
				if(allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[parameterMenu+1]>=100)allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[parameterMenu+1]=100;
				makeNewPedals();

			}else if(rotationDirection==0&&num>0){
				allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[parameterMenu+1]--;
				if(allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[parameterMenu+1]>240)allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[parameterMenu+1]=0;
				makeNewPedals();

			}
			drawParameterEditMenu();

		}
	}else if(currentScreen=="choosePedalMenu"){
		if(buttonPressed=="right"){
			int num = sizeof(INDIVIDUALPEDALSSTRING)/sizeof(INDIVIDUALPEDALSSTRING[0]);
			choosePedalMenu++;
			if(choosePedalMenu>=num)choosePedalMenu=0;
			drawchoosePedalMenu();
		}else if(buttonPressed == "left"){
			int num = sizeof(INDIVIDUALPEDALSSTRING)/sizeof(INDIVIDUALPEDALSSTRING[0]);
			choosePedalMenu--;
			if(choosePedalMenu<0)choosePedalMenu=num-1;
			drawchoosePedalMenu();
		}else if(buttonPressed == "plus"){
			allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[0]=choosePedalMenu;
			allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[1]=50;
			allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[2]=50;
			allPedalsData.allChains[pedalMenu].allPedals[individualPedalMenu].parameters[3]=50;
			choosePedalMenu=0;
			currentScreen="individualPedalsMenu";
			drawIndividualPedalMenu();
			makeNewPedals();

		}
	}else if(currentScreen=="editName"){
		if(buttonPressed == "right"){
            if(nameArray.length()>=PEDAL_NAME_SIZE-1){
            	strcpy(allPedalsData.allChains[pedalMenu].name,nameArray.c_str());
				currentScreen = "pedalMenu";
				currentCharacter = 'a';
				nameArray = "a";
				drawPedalMenu();
            }else{
            	currentCharacter='a';
            	nameArray.push_back(currentCharacter);
        		drawEditNameMenu();
            }
		}else if(buttonPressed == "left"){
			if(nameArray.length()>1){
				nameArray.pop_back();
				currentCharacter = nameArray.back();
        		drawEditNameMenu();
			}
        }else if(buttonPressed == "rotation"){
        	if(rotationDirection==1){
        		currentCharacter+=1;
        		if(currentCharacter>'z')currentCharacter='a';
        		nameArray.back()=currentCharacter;
        		drawEditNameMenu();
        	}else{
        		currentCharacter-=1;
        		if(currentCharacter<'a')currentCharacter='z';
        		nameArray.back()=currentCharacter;
        		drawEditNameMenu();
        	}
        }else if(buttonPressed == "plus"){
        	strcpy(allPedalsData.allChains[pedalMenu].name,nameArray.c_str());
            currentScreen = "pedalMenu";
            currentCharacter = 'a';
            nameArray = "a";
            drawPedalMenu();

        }


		/////////////////////////////////////////////////////////////////////////////////////////
	}else if(currentScreen=="pedalMenuOption"){  //chose options for the bunch(edit,delete,return)
		if(buttonPressed == "right"){
			pedalMenuOption++;
			if(pedalMenuOption>=3)pedalMenuOption=0;
			drawPedalMenuOptions();

		}else if(buttonPressed == "left"){
			pedalMenuOption--;
			if(pedalMenuOption<0)pedalMenuOption=2;
			drawPedalMenuOptions();
		}else if(buttonPressed == "plus"){
			if(pedalMenuOption==2){
				pedalMenuOption=0;
				currentScreen="menu";
				drawMenuScreen();
			}else if(pedalMenuOption == 1){
				//deletePedal(pedalMenuOption);
				pedalMenuOption=0;
				currentCharacter='a';
				nameArray="a";
				currentScreen="editName";
				drawEditNameMenu();
			}else if(pedalMenuOption == 0){
				pedalMenuOption=0;
				currentScreen="individualPedalsMenu";
				drawIndividualPedalMenu();
			}
		}
		/////////////////////////////////////////////////////////////////////////////////////////

	}else if(currentScreen=="pedalMenu"){ //this is a bunch (distort + delay + fuzz +...)
		if(buttonPressed=="right"){
			pedalMenu++;
			if(pedalMenu>=MAX_CHAINS){
				pedalMenu=0;
			}
			makeNewPedals();
			drawPedalMenu();
		}else if (buttonPressed=="left"){
			pedalMenu --;
			if(pedalMenu<0){
				pedalMenu=MAX_CHAINS-1;
			}
			makeNewPedals();
			drawPedalMenu();
		}else if(buttonPressed=="plus"){
			currentScreen="pedalMenuOption";
			drawPedalMenuOptions();
		}
		/////////////////////////////////////////////////////////////////////////////////////////

	}else if(currentScreen=="metronomeMenu"){
		if(buttonPressed=="right"){
			metronomeMenuOption++;
			if(metronomeMenuOption>4)  metronomeMenuOption=0;
			drawMetronomeMenu();
		}else if(buttonPressed=="left"){
			metronomeMenuOption--;
			if(metronomeMenuOption<0)  metronomeMenuOption=4;
			drawMetronomeMenu();
		}else if(buttonPressed=="rotation"){
			if(metronomeMenuOption==0){
				if(rotationDirection==1){
					currentBPM++;
					if(currentBPM>240)currentBPM=240;
					drawMetronomeMenu();
				}else{
					currentBPM--;
					if(currentBPM<20)currentBPM=20;
					drawMetronomeMenu();
				}
			}else if(metronomeMenuOption==4){
				if(rotationDirection==1){
					timesSigned=0;
					timeSignature++;
					if(timeSignature>19)timeSignature=19;
					drawMetronomeMenu();
				}else{
					timesSigned=0;
					timeSignature--;
					if(timeSignature<1)timeSignature=1;
					drawMetronomeMenu();
				}
			}
		}else if(buttonPressed=="plus"){
			if(metronomeMenuOption==1){
				metronomeLED=!metronomeLED;
				drawMetronomeMenu();
			}else if(metronomeMenuOption==2){
				metronomeClick=!metronomeClick;
				drawMetronomeMenu();
			}else if(metronomeMenuOption==3){
				currentScreen="menu";
				drawMenuScreen();
			}
		}

	}else if(currentScreen=="menu"){
		if(buttonPressed=="right"){
			currentMenuOption++;
            if(currentMenuOption>3)  currentMenuOption=0;
			drawMenuScreen();
		}else if(buttonPressed=="left"){
			currentMenuOption--;
            if(currentMenuOption<0)  currentMenuOption=3;
			drawMenuScreen();
		}else if(buttonPressed=="plus"){
             if(currentMenuOption==0){
            	 currentScreen="guitarChainMenu";
            	 drawChainMenu();
             }else if(currentMenuOption==1){
            	 currentScreen="metronomeMenu";
            	 drawMetronomeMenu();
             }else if(currentMenuOption==2){
            	 currentScreen="pedalMenu";
            	 drawPedalMenu();
             }else if(currentMenuOption==3){
            	 savePedals();
            	 drawMenuScreen();
             }
		}
	}
}
uint16_t bytestowrite (uint16_t size, uint16_t offset)
{
	if ((size+offset)<PAGE_SIZE) return size;
	else return PAGE_SIZE-offset;
}

void EEPROM_Write (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{

	// Find out the number of bit, where the page addressing starts
	int paddrposition = log(PAGE_SIZE)/log(2);

	// calculate the start page and the end page
	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/PAGE_SIZE);

	// number of pages to be written
	uint16_t numofpages = (endPage-startPage) + 1;
	uint16_t pos=0;

	// write the data
	for (int i=0; i<numofpages; i++)
	{
		/* calculate the address of the memory location
		 * Here we add the page address with the byte address
		 */
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(size, offset);  // calculate the remaining bytes to be written

		HAL_I2C_Mem_Write(EEPROM_I2C, EEPROM_ADDR, MemAddress, 2, &data[pos], bytesremaining, 1000);  // write the data to the EEPROM

		startPage += 1;  // increment the page, so that a new page address can be selected for further write
		offset=0;   // since we will be writing to a new page, so offset will be 0
		size = size-bytesremaining;  // reduce the size of the bytes
		pos += bytesremaining;  // update the position for the data buffer

		HAL_Delay (5);  // Write cycle delay (5ms)
	}
}


void EEPROM_Read (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	int paddrposition = log(PAGE_SIZE)/log(2);

	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/PAGE_SIZE);

	uint16_t numofpages = (endPage-startPage) + 1;
	uint16_t pos=0;

	for (int i=0; i<numofpages; i++)
	{
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(size, offset);
		HAL_I2C_Mem_Read(EEPROM_I2C, EEPROM_ADDR, MemAddress, 2, &data[pos], bytesremaining, 1000);
		startPage += 1;
		offset=0;
		size = size-bytesremaining;
		pos += bytesremaining;
	}
}






/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartBlinker */
/**
  * @brief  Function implementing the blinker thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBlinker */
void StartBlinker(void *argument)
{
  /* USER CODE BEGIN 5 */
//	int pulse=0;
//  /* Infinite loop */
//  for(;;)
//  {
//	        pulse++;
//	        if(pulse/256==1)pulse=0;
//	        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,pulse);
//	        osDelay(10);
//
//  }
  for(;;){
	  if (xSemaphoreTake(blinkSem, portMAX_DELAY)){
		  HAL_GPIO_WritePin(flashLED_GPIO_Port, flashLED_Pin,GPIO_PIN_SET);
		  osDelay(10);
		  HAL_GPIO_WritePin(flashLED_GPIO_Port, flashLED_Pin,GPIO_PIN_RESET);


	  }

  }
  osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartOled */
/**
* @brief Function implementing the oled thread.
* @param argument: Not used
* @retval None
*/
//uint8_t loop[1000];
/* USER CODE END Header_StartOled */


void StartOled(void *argument)
{
  /* USER CODE BEGIN StartOled */
  /* Infinite loop */

	  if (EE24_Init(&ee24, &hi2c2, EE24_ADDRESS_DEFAULT)){
//			for(int i = 0; i < 1000; i++){loop[i]=0;}
//			EE24_Write(&ee24, 0, loop, sizeof(loop), 1000);

	     EE24_Read(&ee24, 0, (uint8_t*)&allPedalsData, sizeof(allPedalsData), 1000);
	  }
	  makeNewPedals();

    ssd1306_SetCursor(3+16+4, 22+4);

		ssd1306_WriteString((char*)"welcome!", Font_7x10, White);
	      ssd1306_UpdateScreen();

		xSemaphoreGiveFromISR(screenChangeSem, NULL);
		osDelay(3000);
  for(;;){

   if (xSemaphoreTake(screenChangeSem, portMAX_DELAY)){
	      menuChange();

      }
  }


  osThreadTerminate(NULL);
  /* USER CODE END StartOled */
}

/* USER CODE BEGIN Header_startMetronomeSync */
/**
* @brief Function implementing the metronomeSync thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startMetronomeSync */
void startMetronomeSync(void *argument)
{
  /* USER CODE BEGIN startMetronomeSync */
  /* Infinite loop */
  for(;;)
  {
    if(metronomeLED)
	xSemaphoreGiveFromISR(blinkSem, NULL);
    if(metronomeClick)
	xSemaphoreGiveFromISR(clickSem, NULL);

    osDelay((60000.0/currentBPM));
    timesSigned = (timesSigned+1)%timeSignature;
  }
  /* USER CODE END startMetronomeSync */
}

/* USER CODE BEGIN Header_startClick */
/**
* @brief Function implementing the click thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startClick */
void startClick(void *argument)
{
  /* USER CODE BEGIN startClick */
  /* Infinite loop */
	//int i;
	for(;;){
		if (xSemaphoreTake(clickSem, portMAX_DELAY)){
//			getSinVal();
//			osDelay(50);
//			makeArray0();
			metronomeIsOn=true;
			osDelay(50);
			metronomeIsOn=false;
		}
	}


  /* USER CODE END startClick */
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
  if (htim->Instance == TIM6) {
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

#ifdef  USE_FULL_ASSERT
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
