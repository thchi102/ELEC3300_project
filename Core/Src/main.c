/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/*
 * pin setup
 * light resistor
 * IR receiver PA3
 * IR transmitter PA2
 * LED PC8 PC9
 * */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "bsp_xpt2046_lcd.h"
#include "pn532_stm32f1.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define use_HEXADECIMAL
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

//uint8_t rx_data[11];
char receiveCode[10];
uint32_t send_Code = 0;
uint32_t receive_Code = 0;
uint8_t receive[33] = {0};
uint8_t receive_Flag = 0;
uint32_t pwm_low_level_time;
uint32_t pwm_high_level_time;
uint8_t button1 = 0;
uint8_t button2 = 0;
uint8_t light_dark_mode = 0;
int tim_mode_raise_or_falling = 0;
int Index = 0;
int ok = 0,ok2=0;
int datawp[35] = {0};
int x, y;
char strx[5];
char stry[5];
uint8_t buff[255];
uint8_t uid[MIFARE_UID_MAX_LENGTH];
int32_t uid_len = 0;
PN532 pn532;
uint8_t key_a[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint32_t pn532_error = PN532_ERROR_NONE;
uint8_t power = 0;
uint8_t open = 0;
uint8_t user = 0;
uint8_t user_change = 0;

#define DEVICE 1
#define REMOTE 0
#define LIGHT 0
#define DARK 1
#define BUTTON1 0x01
#define BUTTON2 0x02
#define BUTTON3 0x03
#define BUTTON4 0x04
#define POWER 0x69
#define ADDRESS 0x11
#define NOUSER 0
#define JONATHAN 1
#define CHRIS 2


#define DLY_TIM_Handle (&htim1)//定时器延时微妙级函数
void HAL_Delay_us(uint16_t nus)
{
	__HAL_TIM_SET_COUNTER(DLY_TIM_Handle, 0);
	__HAL_TIM_ENABLE(DLY_TIM_Handle);
	while (__HAL_TIM_GET_COUNTER(DLY_TIM_Handle) < nus)
	{
	}
	__HAL_TIM_DISABLE(DLY_TIM_Handle);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DecimalToHex(uint32_t decimal, char Hex[])
{
	for(int i = 0; i < 3; i++)
	{
		int x = decimal%16;
		if(x < 10)
			Hex[2-i] = x+48;
		else
			Hex[2-i] = x+55;
		decimal = decimal/16;
	}
	Hex[3] = '\0';
}

void IntToStr(uint32_t integer, char string[])
{

	for(int i = 0; i < 4; i++)
	{
		string[3-i] = integer%10+48;
		integer = integer /10;
	}
	string[4] = '\0';
}

void IR_send(uint8_t addr, uint8_t data)
{
	uint8_t iaddr = ~addr;

	uint8_t idata = ~data;
	char sendCode[10];
	send_Code = addr<<24 | iaddr<<16 | data<<8 | idata;
	itoa(send_Code, sendCode, 16);
	LCD_DrawString(0,0,sendCode, BLACK, WHITE);

	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_Delay_us(9000);
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
	HAL_Delay_us(4500);
	for(int i=31;i>=0;i--){
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
		HAL_Delay_us(560);
		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
		if((send_Code>>i) & 0x01)
			HAL_Delay_us(1690);
		else
			HAL_Delay_us(560);
	}
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_Delay_us(560);
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
}

void PrintRemote()
{
//	LCD_DrawString(75, 0, "Remote mode", BLACK, WHITE);

	LCD_OpenWindow(40, 20, 160, 60);
	LCD_FillColor(160*60, RED);
	LCD_DrawString(100, 30, "Power", WHITE, RED);

	LCD_OpenWindow(10, 100, 100, 100);
	LCD_FillColor(100*100, BLACK);
	LCD_DrawString(30, 150, "button1", WHITE, BLACK);

	LCD_OpenWindow(130, 100, 100, 100);
	LCD_FillColor(100*100, BLACK);
	LCD_DrawString(152, 150, "button2", WHITE, BLACK);

	LCD_OpenWindow(10, 210, 100, 100);
	LCD_FillColor(100*100, BLACK);
	LCD_DrawString(30, 260, "button3", WHITE, BLACK);

	LCD_OpenWindow(130, 210, 100, 100);
	LCD_FillColor(100*100, BLACK);
	LCD_DrawString(152, 260, "button4", WHITE, BLACK);
}

void PrintDevice_light(uint8_t user, uint8_t button1, uint8_t button2)
{
	LCD_Clear(0,0,240,320,WHITE);
	LCD_DrawString(75, 0, "Device Mode", BLACK, WHITE);

	if(user == JONATHAN)
	{
		LCD_DrawString(50, 20, "Welcome! Jonathan!", BLACK, WHITE);
	}
	else if(user == CHRIS)
	{
		LCD_DrawString(60, 20, "Welcome! Chris!", BLACK, WHITE);
	}
	else
	{
		LCD_DrawString(60, 20, "Device Locked", BLACK, WHITE);
	}

	if(button1)
	{
		LCD_DrawString(50, 45, "ON", BLACK, WHITE);
		LCD_OpenWindow(15, 60, 100, 250);
		LCD_FillColor(100*250, GREEN);
	}
	else
	{
		LCD_DrawString(50, 45, "OFF", BLACK, WHITE);
		LCD_OpenWindow(15, 60, 100, 250);
		LCD_FillColor(100*250, RED);
	}
	if(button2)
	{
		LCD_DrawString(165, 45, "ON", BLACK, WHITE);
		LCD_OpenWindow(125, 60, 100, 250);
		LCD_FillColor(100*250, GREEN);
	}
	else
	{
		LCD_DrawString(165, 45, "OFF", BLACK, WHITE);
		LCD_OpenWindow(125, 60, 100, 250);
		LCD_FillColor(100*250, RED);
	}
}

void PrintDevice_dark(uint8_t user, uint8_t button1, uint8_t button2)
{
	LCD_Clear(0,0,240,320,BLACK);
	LCD_DrawString(75, 0, "Device Mode", WHITE, BLACK);

	if(user == JONATHAN)
	{
		LCD_DrawString(50, 20, "Welcome! Jonathan!", WHITE, BLACK);
	}
	else if(user == CHRIS)
	{
		LCD_DrawString(60, 20, "Welcome! Chris!", WHITE, BLACK);
	}
	else
	{
		LCD_DrawString(60, 20, "Device Locked", WHITE, BLACK);
	}

	if(button1)
	{
		LCD_DrawString(50, 45, "ON", WHITE, BLACK);
		LCD_OpenWindow(15, 60, 100, 250);
		LCD_FillColor(100*250, GREEN);
	}
	else
	{
		LCD_DrawString(50, 45, "OFF", WHITE, BLACK);
		LCD_OpenWindow(15, 60, 100, 250);
		LCD_FillColor(100*250, RED);
	}
	if(button2)
	{
		LCD_DrawString(165, 45, "ON", WHITE, BLACK);
		LCD_OpenWindow(125, 60, 100, 250);
		LCD_FillColor(100*250, GREEN);
	}
	else
	{
		LCD_DrawString(165, 45, "OFF", WHITE, BLACK);
		LCD_OpenWindow(125, 60, 100, 250);
		LCD_FillColor(100*250, RED);
	}
}

void Find_Data(uint32_t *code)
{
	*code = *code & 0x0000ff00;
	*code = *code >> 8;
}

void Update_Button1()
{
	if(button1)
	{
		Button1_Off();
	}
	else
	{
		Button1_On();
	}
}

void Update_Button2()
{
	if(button2)
	{
		Button2_Off();
	}
	else
	{
		Button2_On();
	}
}

void Button1_On()
{
	button1 = 1;
	LCD_Clear(50, 45, 25, 15, WHITE);
	if(light_dark_mode == DARK)
		LCD_DrawString(50, 45, "ON  ", WHITE, BLACK);
	else
		LCD_DrawString(50, 45, "ON  ", BLACK, WHITE);
	LCD_OpenWindow(15, 60, 100, 250);
	LCD_FillColor(100*250, GREEN);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
}

void Button2_On()
{
	button2 = 1;
	LCD_Clear(165, 45,25, 15, WHITE);
	if(light_dark_mode == DARK)
		LCD_DrawString(165, 45, "ON  ", WHITE, BLACK);
	else
		LCD_DrawString(165, 45, "ON  ", BLACK, WHITE);
	LCD_OpenWindow(125, 60, 100, 250);
	LCD_FillColor(100*250, GREEN);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
}

void Button1_Off()
{
	button1 = 0;
	LCD_Clear(50, 45, 25, 15, WHITE);
	if(light_dark_mode == DARK)
		LCD_DrawString(50, 45, "OFF ", WHITE, BLACK);
	else
		LCD_DrawString(50, 45, "OFF ", BLACK, WHITE);
	LCD_OpenWindow(15, 60, 100, 250);
	LCD_FillColor(100*250, RED);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
}

void Button2_Off()
{
	button2 = 0;
	LCD_Clear(165, 45,25, 15, WHITE);
	if(light_dark_mode == DARK)
		LCD_DrawString(165, 45, "OFF ", WHITE, BLACK);
	else
		LCD_DrawString(165, 45, "OFF ", BLACK, WHITE);
	LCD_OpenWindow(125, 60, 100, 250);
	LCD_FillColor(100*250, RED);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
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
  MX_FSMC_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  LCD_INIT();
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim5);
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  GPIO_PinState prevButtonState = GPIO_PIN_RESET;
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
  PN532_SPI_Init(&pn532);
  PN532_SamConfiguration(&pn532);
  HAL_ADC_Start(&hadc1);
  TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_4,TIM_ICPOLARITY_FALLING);
  TIM2->ARR = 1893;
  TIM2->CCR2 = TIM2->ARR/2;
  char stage[10] = "menu";
  if(DEVICE)
	  PrintDevice_light(0,0,0);
  else
	  PrintRemote();
//  LCD_Clear(0,0,240,320, BLACK);
//  PrintDevice_light(0,button1,button2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//
	  	if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
		  {
			  int ADCValue1 = HAL_ADC_GetValue(&hadc1);
			  if(ADCValue1 < 3750)
			  {
				  if(!open)
					  PrintDevice_light(user, button1, button2);
				  open = 1;
				  light_dark_mode = LIGHT;
			  }
			  else
			  {
				  if(open)
					  PrintDevice_dark(user, button1, button2);
				  open = 0;
				  light_dark_mode = DARK;
			  }
		  }

	  if(REMOTE)
	  {
			XPT2046_TouchEvenHandler(&stage, &x, &y);
			if((x>40&&x<185)&&(y>40&&y<85))
			{
				IR_send(ADDRESS, POWER);
			}
			if((x>10&&x<105)&&(y>115&&y<200))
			{
				IR_send(ADDRESS, BUTTON1);
			}
			if((x>125&&x<230)&&(y>115&&y<200))
			{
				IR_send(ADDRESS, BUTTON2);
			}
			if((x>10&&x<105)&&(y>215&&y<300))
			{
				IR_send(ADDRESS, BUTTON3);
			}
			if((x>125&&x<230)&&(y>215&&y<300))
			{
				IR_send(ADDRESS, BUTTON4);
			}
			x=0;y=0;
	  }

	  if(DEVICE)
	  {
		  if(user_change == 1)
		  {
			  if(light_dark_mode == LIGHT)
				  LCD_Clear(60, 20, 150, 20, WHITE);
			  else
				  LCD_Clear(60, 20, 150, 20, BLACK);
			  if(user == CHRIS)
			  {
				  if(light_dark_mode == LIGHT)
					  LCD_DrawString(60, 20, "Welcome! Chris!", BLACK, WHITE);
				  else
					  LCD_DrawString(60, 20, "Welcome! Chris!", WHITE, BLACK);
			  }
			  else if(user == JONATHAN)
			  {
				  if(light_dark_mode == LIGHT)
					  LCD_DrawString(60, 20, "Welcome! Jonathan!", BLACK, WHITE);
				  else
					  LCD_DrawString(60, 20, "Welcome! Jonathan!", WHITE, BLACK);
			  }
			  else if(user == NOUSER)
			  {
				  if(light_dark_mode == LIGHT)
					  LCD_DrawString(60, 20, "Device locked", BLACK, WHITE);
				  else
					  LCD_DrawString(60, 20, "Device locked", WHITE, BLACK);
			  }
			  user_change = 0;
		  }
		  if(user == NOUSER)
		  {
			  memset(uid, 0, sizeof(uid));
			  uid_len = PN532_ReadPassiveTarget(&pn532, uid, PN532_MIFARE_ISO14443A, 1000);
			  HAL_Delay(1000);
		  }
		  XPT2046_TouchEvenHandler(&stage, &x, &y);
		  if(uid[0] == 19 && uid[1] == 2 && uid[2] == 48 && uid[3] == 37)
		  {
			  user = JONATHAN;
			  user_change = 1;
			  memset(uid, 0, sizeof(uid));
		  }
		  if(uid[0] == 67 && uid[1] == 79 && uid[2] == 229 && uid[3] == 247)
		  {
			  user = CHRIS;
			  user_change = 1;
			  memset(uid, 0, sizeof(uid));
		  }
		  if(user == NOUSER)
			  continue;

		  if((x>10&&x<105)&&(y>115&&y<300))
		  {
			  Update_Button1();
		  }
		  if((x>125&&x<230)&&(y>115&&y<300))
		  {
			  Update_Button2();
		  }
			if(receive_Flag)
				{
					Find_Data(&receive_Code);
					switch(receive_Code)
					{
						case BUTTON1:
							Update_Button1();
							receive_Flag = 0;
							break;
						case BUTTON2:
							Update_Button2();
							receive_Flag = 0;
							break;
						case BUTTON3:
							Button1_Off();
							Button2_Off();
							receive_Flag = 0;
							break;
						case BUTTON4:
							Button1_On();
							Button2_On();
							receive_Flag = 0;
							break;
						case POWER:
							if(user != 0)
							{
								user = NOUSER;
							}
							receive_Flag = 0;
							user_change = 1;
							break;
					}
					receive_Flag = 0;
				}
			x=0;y=0;
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1895;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 947;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 71;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD_Touch_Din_Pin|LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PN532_SS_GPIO_Port, PN532_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PN532_RST_GPIO_Port, PN532_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LCS_BL_Pin|LCD_Touch_Select_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_Touch_CLK_GPIO_Port, LCD_Touch_CLK_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LCD_Touch_Din_Pin LCD_Touch_CLK_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_Touch_Din_Pin|LCD_Touch_CLK_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_Touch_Dout_Pin LCD_Touch_IRQ_Pin */
  GPIO_InitStruct.Pin = LCD_Touch_Dout_Pin|LCD_Touch_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PN532_SS_Pin PN532_RST_Pin */
  GPIO_InitStruct.Pin = PN532_SS_Pin|PN532_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PN532_IRQ_Pin */
  GPIO_InitStruct.Pin = PN532_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PN532_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCS_BL_Pin LCD_Touch_Select_Pin */
  GPIO_InitStruct.Pin = LCS_BL_Pin|LCD_Touch_Select_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(tim_mode_raise_or_falling == 0)
  {
    pwm_low_level_time = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_4) + 1;
    __HAL_TIM_SET_COUNTER(&htim5,0);
    TIM_RESET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_4);
    TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_4,TIM_ICPOLARITY_FALLING);
    tim_mode_raise_or_falling = 1;
		ok2 = 0;
  }
  else if(tim_mode_raise_or_falling == 1)
  {
    pwm_high_level_time = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_4) + 1;
    __HAL_TIM_SET_COUNTER(&htim5,0);
    TIM_RESET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_4);
    TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_4,TIM_ICPOLARITY_RISING);
    tim_mode_raise_or_falling = 0;
		ok2 = 1;
  }
	uint8_t Data=0;
	if(pwm_high_level_time >= 5000){
		Index=0;ok = 0;
	}
	else if(pwm_high_level_time >= 3500 && pwm_high_level_time < 5000){
		Index=0;ok = 1;
	}
	else if(pwm_high_level_time >= 1000 && pwm_high_level_time < 2000){
		Data=1;		//收到数据1
	}
	else if(pwm_high_level_time > 0 && pwm_high_level_time < 1000){
		Data=0;		//收到数据0
	}


//	pwm_high_level_time = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1) + 1;
//	__HAL_TIM_SET_COUNTER(&htim3,0);
//	TIM_RESET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1);
//	uint8_t Data=0;ok2=1;
//	if(pwm_high_level_time >= 16000){
//		Index=0;ok = 1;
//	}
////	else if(pwm_high_level_time >= 12000 && pwm_high_level_time < 16000){
////		Index=0;ok = 1;
////	}
//	else if(pwm_high_level_time >= 1500 && pwm_high_level_time < 3500){
//		Data=1;		//收到数据1
//	}
//	else if(pwm_high_level_time < 1500){
//		Data=0;		//收到数据0
//	}

	if(ok == 1 && ok2 == 1){
		receive_Code <<= 1;
		receive_Code += Data;
		receive[Index] = Data;
		datawp[Index] = pwm_high_level_time;
		Index++;
		if(Index>=32){
			receive_Flag=1;
			Index = ok = 0;
		}
	}
}


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  /* Prevent unused argument(s) compilation warning */
//  //UNUSED(huart);
//  /* NOTE: This function should not be modified, when the callback is needed,
//           the HAL_UART_RxCpltCallback could be implemented in the user file
//   */
//	if(huart->Instance == USART1)
//	{
//
//		HAL_UART_Transmit(&huart1, rx_data, strlen(rx_data), 1000);
//		HAL_UART_Receive_IT(&huart1, rx_data, 4);
//	}
//	if(huart->Instance == USART3)
//	{
//		HAL_UART_Transmit(&huart1, rx_data, strlen(rx_data), 1000);
//		HAL_UART_Receive_IT(&huart3, rx_data, 1);
//	}
//}


/* USER CODE END 4 */

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
