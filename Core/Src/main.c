/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include "bmp280_defs.h"
#include "arm_math.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define PID_TS        0.1        /* Sampling Time [s]*/
#define PID_KP        15.5       /* Proportional */
#define PID_KI        0.7886	 /* Integral */
#define PID_KD        0          /* Derivative */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP280_SPI (&hspi4)
#define BMP280_CS1 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

arm_pid_instance_f32 PID;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int8_t bmp280_spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t bmp280_spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct bmp280_dev bmp280 = {
	.dev_id = BMP280_CS1,
	.intf = BMP280_SPI_INTF,
	.read = bmp280_spi_reg_read,
	.write = bmp280_spi_reg_write,
	.delay_ms = HAL_Delay
};

struct bmp280_uncomp_data bmp280_data;

//int32_t temp32;
double temp;
//uint32_t pres32;
//uint32_t press32;
//double pres;
int8_t rslt;
//int counter = 0;
char odebrane[4];
uint16_t PWM;
uint16_t setValue;
float uchyb = 0;
float PWM_raw = 0;
double SWV_temp = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	 if (htim->Instance == TIM3)
	 {
		 /* Reading the raw data from sensor */
		 rslt = bmp280_get_uncomp_data(&bmp280_data, &bmp280);

		 rslt = bmp280_get_comp_temp_double(&temp, bmp280_data.uncomp_temp, &bmp280);
		 SWV_temp = temp;

		 uchyb = (float)(setValue - temp);

		 PWM_raw = arm_pid_f32(&PID, uchyb);
		 PWM=(float)(PWM_raw);

		 if(PWM_raw>2000)
		 {
		 	PWM=2000;
		 	//arm_pid_reset_f32(&PID_regulator);
		 	if(PID.state[2] > 2500) PID.state[2] = 2500;
		 }
		 else if (PWM_raw<0)
		 {
		 	//arm_pid_reset_f32(&PID_regulator);
		 	PWM=0;
		 	PID.state[2] = 0;
		 }
		 if(PWM<=1000)
		 {
			 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM);
		 }
		 else
		 {
			 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);
		 }


	 }

	 if (htim->Instance == TIM4)
	 {
		 char txData[50];
		 int text = sprintf(txData, "set_temp: %u | temp: %d.%02u | uchyb: %d\r\n",
		 				 	setValue, (int32_t)temp, (int32_t)((temp-(int32_t)temp)*100),
		 					uchyb);

		 HAL_UART_Transmit(&huart3, (uint8_t*)txData, text, strlen(text));
	 }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		if(odebrane[0]=='H')
		{
			//konwersja na int od 1 elementu tablicy
			PWM = ((int8_t)odebrane[1]-'0')*100+((int8_t)odebrane[2]-'0')*10+((int8_t)odebrane[3]-'0')*1;
			if(PWM>100) PWM = 100;
			else if(PWM<0) PWM = 0;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM*10);
		}

		if(odebrane[0]=='T')
		{
			 rslt = bmp280_get_uncomp_data(&bmp280_data, &bmp280);
			 rslt = bmp280_get_comp_temp_double(&temp, bmp280_data.uncomp_temp, &bmp280);

			//konwersja na int od 1 elementu tablicy
			setValue = ((int8_t)odebrane[1]-'0')*100+((int8_t)odebrane[2]-'0')*10+((int8_t)odebrane[3]-'0')*1;
			if(setValue>45) setValue = 45;
			else if(setValue<temp) setValue = 0;
//			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, setValue*10);
		}
	}

	HAL_UART_Receive_IT(&huart3, (uint8_t*)odebrane, 4);
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
  MX_USART3_UART_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // ZAD. 4 ////////////////////////////////////
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_UART_Receive_IT(&huart3, (uint8_t*)odebrane, 4);

  PID.Kp = PID_KP;
  PID.Ki = PID_KI * PID_TS;
//  PID.Kd = PID_KD / PID_TS;

  arm_pid_init_f32(&PID, 1);

      struct bmp280_config conf;
      uint8_t rslt = bmp280_init(&bmp280);
      /* Always read the current settings before writing, especially when
      * all the configuration is not modified
      */
      rslt = bmp280_get_config(&conf, &bmp280);
      /* configuring the temperature oversampling, filter coefficient and output data rate */
      /* Overwrite the desired settings */
      conf.filter = BMP280_FILTER_OFF;
      /* Temperature oversampling set at 1x */
      conf.os_temp = BMP280_OS_1X;
      /* Pressure over sampling none (disabling pressure measurement) */
      conf.os_pres = BMP280_OS_1X;
      /* Setting the output data rate as 1HZ(1000ms) */
      conf.odr = BMP280_ODR_1000_MS;
      rslt = bmp280_set_config(&conf, &bmp280);
      /* Always set the power mode after setting the configuration */
      rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp280);
      /* Pressure over sampling 1X */
      conf.os_pres = BMP280_OS_1X;
  /////////////////////////////////////////////

  /* USER CODE END 2 */

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

#define BMP280_SPI_BUFFER_LEN 28
#define BMP280_DATA_INDEX 1

int8_t bmp280_spi_reg_read ( uint8_t cs , uint8_t reg_addr , uint8_t * reg_data , uint16_t length)
{
	/* Implement the SPI read routine according to the target machine . */
	HAL_StatusTypeDef status = HAL_OK ;
	int32_t iError = BMP280_OK ;
	uint8_t txarray [ BMP280_SPI_BUFFER_LEN ] = {0 ,};
	uint8_t rxarray [ BMP280_SPI_BUFFER_LEN ] = {0 ,};
	uint8_t stringpos ;
	txarray [0] = reg_addr ;

	/* Software slave selection procedure */
	if(cs == BMP280_CS1 ){
		HAL_GPIO_WritePin ( BMP280_CS1_GPIO_Port , BMP280_CS1_Pin , GPIO_PIN_RESET );
	}

	/* Data exchange */
	status = HAL_SPI_TransmitReceive ( BMP280_SPI , ( uint8_t *) (& txarray ), ( uint8_t *)
			(&rxarray ), length +1, 5);
	while ( BMP280_SPI -> State == HAL_SPI_STATE_BUSY ) {};

	/* Disable all slaves */
	HAL_GPIO_WritePin ( BMP280_CS1_GPIO_Port , BMP280_CS1_Pin , GPIO_PIN_SET );

	/* Copy data from rx buffer */
	for ( stringpos = 0; stringpos < length ; stringpos ++) {
		reg_data [ stringpos ] = rxarray [ stringpos + BMP280_DATA_INDEX ];
	}

	// memcpy ( reg_data , rxarray + BMP280_DATA_INDEX , length );
	if ( status != HAL_OK ) {
	// The BME280 API calls for 0 return value as a success , and -1 returned as failure
		iError = ( -1);
	}

	return ( int8_t ) iError ;
}

int8_t bmp280_spi_reg_write ( uint8_t cs , uint8_t reg_addr , uint8_t * reg_data , uint16_t length )
{
	/* Implement the SPI write routine according to the target machine . */
	HAL_StatusTypeDef status = HAL_OK ;
	int32_t iError = BMP280_OK ;
	uint8_t txarray [ BMP280_SPI_BUFFER_LEN ];
	uint8_t stringpos ;

	/* Copy register address and data to tx buffer */
	txarray [0] = reg_addr ;
	for ( stringpos = 0; stringpos < length ; stringpos ++) {
		txarray [ stringpos + BMP280_DATA_INDEX ] = reg_data [ stringpos ];
	}

	// memcpy ( txarray + BMP280_DATA_INDEX , reg_data , length );
	/* Software slave selection procedure */
	if(cs == BMP280_CS1 ){
		HAL_GPIO_WritePin ( BMP280_CS1_GPIO_Port , BMP280_CS1_Pin , GPIO_PIN_RESET );
	}

	/* Data exchange */
	status = HAL_SPI_Transmit ( BMP280_SPI , ( uint8_t *) (& txarray ), length +1, 100) ;
	while ( BMP280_SPI -> State == HAL_SPI_STATE_BUSY ) {};

	/* Disable all slaves */
	HAL_GPIO_WritePin (BMP280_CS1_GPIO_Port, BMP280_CS1_Pin, GPIO_PIN_SET);
	if ( status != HAL_OK ) {
		// The BMP280 API calls for 0 return value as a success , and -1 returned as failure
		iError = ( -1);
	}

	return ( int8_t ) iError ;
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

