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
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "DHT11.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t danger_data[9]={0xFD,0x00,0x06,0x01,0x01,0xCE,0xA3,0xCF,0xD5};
int rh_ban=60,th_ban=30,co2_ban=800;
float fire_ban=2.0,gas_ban=1.0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void ui_setting(void);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  float fire_fl,smoke_f1;
	uint16_t fire_16 = 0,smoke_16 = 0;
	char huoguang[20],smoke[20];
	int dht11_ret=0;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	OLED_ColorTurn(0);
  OLED_DisplayTurn(0);
	OLED_Clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		dht11_ret = DHT11_READ_DATA();
		HAL_ADC_Start(&hadc1);     //read fire
		HAL_ADC_PollForConversion(&hadc1,10); 
		fire_16 = (uint16_t)HAL_ADC_GetValue(&hadc1);  
		fire_fl = (float)fire_16*3.3/4096; 
		sprintf(huoguang,"Fire:%.2f",fire_fl);
		OLED_ShowString(0,16,(uint8_t*)huoguang,16,1);
		OLED_Refresh();
		
		HAL_ADC_Start(&hadc2);     //read fire
		HAL_ADC_PollForConversion(&hadc2,10); 
		smoke_16 = (uint16_t)HAL_ADC_GetValue(&hadc2);  
		smoke_f1 = (float)smoke_16*3.3/4096; 
		sprintf(smoke,"Gas:%.2f",smoke_f1);
		OLED_ShowString(0,32,(uint8_t*)smoke,16,1);
		OLED_Refresh();
		
		if((fire_fl<fire_ban)||(smoke_f1>gas_ban))
		{
			 HAL_UART_Transmit(&huart2,danger_data,9,0xffff);
				uart3_send_messageCALL();
		}
		
		if(rx1_end_flag)
		{
			 rx1_end_flag =false;
			
        if(uart1_rx[5]==((uart1_rx[0]+uart1_rx[1]+uart1_rx[2]+uart1_rx[3]+uart1_rx[4])&0xff))
				{
					sprintf(huoguang,"CO2:%d ppm",uart1_rx[1]*256+uart1_rx[2]);
					OLED_ShowString(0,48,(uint8_t*)huoguang,16,1);
				  OLED_Refresh();
					if((uart1_rx[1]*256+uart1_rx[2]>co2_ban)||(dht11_ret==1))
					{
							lay_control(true);
						  beep_control(true);
					}
					else
					{
							lay_control(false);
						  beep_control(false);
					}

				}

				rx1_count=0;
				memset(uart1_rx,0,20);
				HAL_UART_Receive_DMA(&huart1,uart1_rx,20);  //��Ҫ��������DMA
		}
		
		if(botton == RIGHT)
		{
				HAL_Delay(100);
			botton = UNPRESS;
			  OLED_Clear();
				ui_setting();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void ui_setting(void)
{
	 int position=0;
	 char chow1[20],chow2[20],chow3[20],chow4[20],chow5[20];
							OLED_ShowString(20,0,(uint8_t*)"Setting",16,1);
				  OLED_Refresh();
	while(1)
	{
				sprintf(chow1,"RH:%02d",rh_ban);
				OLED_ShowString(0,16,(uint8_t*)chow1,8,position==0?0:1);
		
		    sprintf(chow2,"TH:%02d",th_ban);
				OLED_ShowString(70,16,(uint8_t*)chow2,8,position==1?0:1);
		
		    sprintf(chow3,"Fire:%0.1f",fire_ban);
				OLED_ShowString(0,32,(uint8_t*)chow3,8,position==2?0:1);
		
		    sprintf(chow4,"Gas:%0.1f",gas_ban);		
				OLED_ShowString(70,32,(uint8_t*)chow4,8,position==3?0:1);

		    sprintf(chow5,"CO2:%03d",co2_ban);			
				OLED_ShowString(0,48,(uint8_t*)chow5,8,position==4?0:1);
				
				OLED_ShowString(70,48,(uint8_t*)"BACK",8,position==5?0:1);
				OLED_Refresh();
		
				if(botton == RIGHT)
				{
					HAL_Delay(100);
					botton = UNPRESS;
					position++;
					if(position>5)
					{
							position=0;
					}
				}
				
				if(botton == LEFT)
				{
					HAL_Delay(100);
					botton = UNPRESS;
					switch(position)
					{
						case 0:
							  rh_ban--;
						    if(rh_ban<0)
									rh_ban=99;
							break;
						case 1:
							  th_ban--;
						    if(th_ban<0)
									th_ban=99;
							break;
						case 2:
							  fire_ban=fire_ban-0.1;
						    if(fire_ban<0)
									fire_ban=3.0;
							break;
						case 3:
							  gas_ban=gas_ban-0.1;
						    if(gas_ban<0)
									gas_ban=3.0;
							break;
						case 4:
							  co2_ban=co2_ban-10;
						    if(co2_ban<400)
									co2_ban=900;
							break;
					}
				}
				
				if(botton == MIDLE)
				{
					HAL_Delay(100);
					botton = UNPRESS;
					switch(position)
					{
						case 0:
							  rh_ban++;
						    if(rh_ban>99)
									rh_ban=0;
							break;
						case 1:
							  th_ban++;
						    if(th_ban>99)
									th_ban=0;
							break;
						case 2:
							  fire_ban=fire_ban+0.1;
						    if(fire_ban>3.0)
									fire_ban=0;
							break;
						case 3:
							  gas_ban=gas_ban+0.1;
						    if(gas_ban>3.0)
									gas_ban=0;
							break;
						case 4:
							  co2_ban=co2_ban+10;
						    if(co2_ban>900)
									co2_ban=400;
							break;
						case 5:
								OLED_Clear();
								return;
							break;
					}
				}
	}
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
