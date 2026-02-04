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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ST7565.h"
#include "NMEA.h"
#include "Utils.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BLINK_AMNT		10u
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LED_Blink_init(void){
	for(uint8_t i = 0u; i < BLINK_AMNT; i++){
	    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin) ^ 1);
	    HAL_Delay(100u); /* 100ms */
	}
}

void LED_Blink(void){
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin) ^ 1);
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  LED_Blink_init();
  LCD_init();
  GPS_init();
  KY040_Init(0, -255, 255);

  convert_horizontal_bitmap(lcd_bitmap, lcd_bitmap_inv, 46, 64);
  convert_horizontal_bitmap(lcd_arrow, lcd_arrow_conv, 20, 15);

  int8_t x = 10, y = 10;
  int8_t dir_x= 1, dir_y = 1;
  uint8_t R = 10;
  int cur_enc = 0u;
#if 0
  char title[25u] = "Current coordinates:";
  char testPrintVer[20u] = "Geocacher ver0.2";
  char lat[21] = {0u};
  char longitude[21] = {0u};
  char speed[21] = {0u};
  char sat_amnt[21] = {0u};
  char altitude[21] = {0u};
  char tmpStr[10] = {0u};

  dtNMEACoordinate lati, longi;
  float speed_f, alti;
  uint8_t sat_amnt_ui;

  LCD_print(18u, 0u, testPrintVer, 20u);
  LCD_print(0u, 2u, title, 20u);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  GPS_handle();
#if 0
	  GPS_get_current_latitude(&lati);
	  GPS_get_current_longitude(&longi);
	  GPS_get_current_altitude(&alti);
	  GPS_get_current_speed(&speed_f);
	  GPS_get_current_satellite_amount(&sat_amnt_ui);

	  if(lati.qualifier == DATA_NOT_AVAILABLE || longi.qualifier == DATA_NOT_AVAILABLE){
		  LCD_print(1u, 4u, "LOW SIGNAL!", strlen("Low signal!"));
		  sprintf(sat_amnt, "Satellites: %d", sat_amnt_ui);
		  LCD_print(1u, 6u, sat_amnt, strlen(sat_amnt));
	  }else{
		  print_float(tmpStr, lati.minute, 3u);
		  sprintf(lat, "%d %s' %c", lati.degree, tmpStr, lati.cardinalPoint);

		  print_float(tmpStr, longi.minute, 3u);
		  sprintf(longitude, "%d %s' %c", longi.degree, tmpStr, longi.cardinalPoint);

		  print_float(tmpStr, alti, 1u);
		  sprintf(altitude, "Altitude: %s m", tmpStr);

		  print_float(tmpStr, speed_f, 1u);
		  sprintf(speed, "Speed: %s km/h", tmpStr);
		  sprintf(sat_amnt, "Satellites: %d", sat_amnt_ui);
		  LCD_print(1u, 3u, lat, strlen(lat));
		  LCD_print(1u, 4u, longitude, strlen(longitude));
		  LCD_print(1u, 5u, altitude, strlen(altitude));
		  LCD_print(1u, 6u, speed, strlen(speed));
		  LCD_print(1u, 7u, sat_amnt, strlen(sat_amnt));
	  }
#else
	  /* Test to draw line in all directions */
#if 0
	  for(uint8_t x_start = 127; x_start > 0; x_start--){
			  LCD_draw_line(x_start,0, 127 - x_start, 63);
			  HAL_Delay(100u);
			  LCD_clearScreen();
	   }
	   for(uint8_t y_start = 63; y_start > 0; y_start--){
			  LCD_draw_line(0 ,63 - y_start, 127, y_start);
			  HAL_Delay(100u);
			  LCD_clearScreen();
	   }


	   for(uint8_t i = 0u; i < 255 ; i++){
		   static float ang = 0.0;
	       LCD_draw_rectangle(30,20,80,40, ang);
	       ang += 0.08;
	       HAL_Delay(1000u);
	       LCD_clearScreen();
	   }
#else
		/* test */
	   /*
		   for(uint8_t i = 0u; i < 255 ; i++){
			   static float ang = 0.08;
			   //LCD_draw_bitmap(40,0,48,64, lcd_bitmap_inv, ang);
			   //LCD_draw_rectangle(30,20,80,40, ang);
			   //LCD_draw_bitmap(40,16,48,32, lcd_arrow_conv, ang);
//			   LCD_draw_line_with_thikness(5,5,120, 60, i);
//			   LCD_draw_circle(64, 32, 10, 1);
			   LCD_draw_rectangle(30,20,80,40, ang, 1);
		       ang += 0.08;
//		       HAL_Delay(1000u);
		       LCD_clearScreen();
		   }
		   */
		   KY040_Hanlder();

		   if(KY040_isEncoderValueChanged() == PASS){
			   cur_enc = KY040_get_enc_current_value();
			   LCD_clearScreen();
			   static float ang = 0.0;
			   ang = 0.08/*0.785398*/ * cur_enc;

			   //LCD_draw_rectangle(30,20,80,40, ang, 1);
			   //LCD_draw_triangle(63, 31, 15, 15, ang, 1);
			   LCD_draw_arrow(48, 16, 30, 30, ang, 1);
			   KY040_encoderValueChangedClearFlag();
		   }
		   if(KY040_isButtonPressed() == PASS){
				  LED_Blink();
				  KY040_set_enc_default_value(0u);
				  KY040_buttonPressedClearFlag();
		   }


#if 0
		   LCD_draw_circle(x, y, R, 1);
		   HAL_Delay(50u);
		   LCD_clearScreen();
		   x += (dir_x * 1u);
		   y += (dir_y * 1u);

		   if((x - R) <= 0){
			   dir_x = 1;
		   }else if((x + R) >= 127){
			   dir_x = -1;
		   }

		   if((y - R) <= 0){
			   dir_y = 1;
		   }else if((y + R) >= 63){
			   dir_y = -1;
		   }
#endif

#endif
//	  LCD_draw_line(0,0, 127, 63);

#endif
//	  HAL_Delay(1000u); /* 1s */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 739;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_CS_Pin|LCD_RES_Pin|LCD_RS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_RES_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_RES_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_BUTTON_Pin */
  GPIO_InitStruct.Pin = ENC_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_DT_Pin */
  GPIO_InitStruct.Pin = ENC_DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_DT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_CLK_Pin */
  GPIO_InitStruct.Pin = ENC_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_CLK_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	EXTI_ISR_Handler(GPIO_Pin);
}
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
