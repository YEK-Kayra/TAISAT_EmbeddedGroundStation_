/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * 	-> hi2c1  ==> BMP280 pressure & temparature sensor
  * 	-> huart1 ==> LORA wireless communication module
  * 	-> huart2 ==> USB-TTL for comunicating with Ground Station PC
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SubSys_Sensor_TPGVH_Driver.h"						/*! IOT data comes from this library*/
#include "SubSys_WirelessCommunication_Setting_Driver.h"	/*! Maybe we can set our lora configuration by stm32 MCU*/
#include "SubSys_WirelessCommunication_Telemetry_Driver.h"	/*! This library provides Wireless communication between EmbeddedGroundStation(EmbeddedGS) and Payload of Satellite */
#include "SubSys_USART_ReceiveIT_CallBacks_Driver.h"		/*! Interrupts events will be managed in this library, EmbeddedGS to PC , Payload to EmbeddedGS*/
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	/**!
	 * Pressure Temperature Sensor (MS5611) Definitions
	 */
	#define MS5611_I2C_ADDRESS_H 0xEE		/*! CSB pin is HIGH */
	#define MS5611_I2C_ADDRESS_L 0xEC 		/*! CSB pin is LOW  */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;	/*! EmbeddedGS to/from PC   */
UART_HandleTypeDef huart2;	/*! Wireless Communication  */

/* USER CODE BEGIN PV */
/*
	 ===============================================================================
	                      ##### MULTIPLE VARIABLE #####
	 ===============================================================================
*/

	/**!
	 * SubSys_Sensor_TPGVH_Driver variables
	 */
	MS5611_HandleTypeDef MS5611;	/*! MS5611 object					*/
	float  MS5611_Press=0.0;		/*! Pressure data variable 			*/
	float  MS5611_Temp=0.0;			/*! Temperature data variable 		*/
	float  MS5611_Altitude;			/*! Vertical Altitude data variable */
	float  MS5611_VertSpeed;		/*! Vertical Speed data variable    */
	float  MS5611_VertAcc;			/*! Vertical Acceleration variable  */
	float  MS5611_gForce;			/*! Vertical g force data variable  */
	float  SatCar_Mass;				/*! Total mass of Satellites Carrier module */


	/*! We create 2 object for subsystem wireless communication,
	 * 	one of them is about configuration setting of wirelesscom device
	 * 	other one is about application object
	 */
	SubSys_WirelesscomConfig_HandleTypeDef dev_WirelessComConfig;
	SubSys_WirelessCom_APP_HandleTypeDef dev_WirelessComApp;

/*
	 ===============================================================================
						  ##### SINGLE VARIABLE #####
	 ===============================================================================
*/
	/*! CommandPC(GroundStation) send a packet :
	 * e.g ==> *G<3R7B+> or *G<????->
	 * Let's we explain what are they
	 * 			 '*' 			 -> CommandPC(GroundStation) wants a telepacket
	 * 			 'G' 			 -> We put 'G' char into the telemetry packet that we will send to the payload
	 * 			 '3' 'R' '7' 'B' -> Chars number and letter for Color Filtering
	 * 			 -,+ 			 ->	Manuel Deploy command, + manuel deploy active, - manuel deploy deactive
	 *
	 */
	char UsbTTL2EmbeddedGS[9];
	char EmbeddedGS2Payload[30];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /******>>> SENSOR TPGVH INITIALIZATION BEGIN >>>******/
  	#ifdef SAT_PAYLOAD_SUBSYS_DRIVERS_SENSOR_TPGVH_H
  	MS5611.I2C_ADDRESS = MS5611_I2C_ADDRESS_H;
  	MS5611.i2c = &hi2c1;
  	MS5611.Ref_Alt_Sel = 'm';
  	MS5611_Init(&MS5611);
  	#endif
  /******<<< SENSOR TPGVH INITIALIZATION END <<<******/


    /******>>> WIRELESS COMMUNICATION SETTING & TELEMETRY INITIALIZATION BEGIN >>>******/
  	/**
  	 * @brief : First of all, we upload initial settings into the wireless communication device.
  	 * 		  After that, we determine Target Address high and low byte and Target Channel.
  	 * 		  Some LoRa module has two pin as named M0 and M1. These provides selecting working mode
  	 * 		  For E220400T30 are M0 and M1 pins.
  	 * @note  : If you use dma for receiving and transmiting, fill it parameters that
  	 *		  come after channel info
  	 */
  	 #ifdef SAT_PAYLOAD_SUBSYS_DRIVERS_WIRELESSCOMMUNICATION_SETTING_H__CLOSED
  	 dev_WirelessComConfig.interface.huart = &huart2;
  	 dev_WirelessComConfig.interface.GPIOx = GPIOB;
  	 dev_WirelessComConfig.LORA_PIN_M0= GPIO_PIN_14;
  	 dev_WirelessComConfig.LORA_PIN_M1= GPIO_PIN_13;
  	 dev_WirelessComConfig.Mode_SW = DeepSleep; 		/*! Module goes to sleep, that provides you to configure settings */

  	 SubSys_WirelessCom_Config_Init(&dev_WirelessComConfig);
  	 #endif

  	 #ifdef SAT_PAYLOAD_SUBSYS_DRIVERS_WIRELESSCOMMUNICATION_TELEMETRY_H
  	 /*! Will be filled for your dev that use now*/
  	 dev_WirelessComApp.huartX = &huart2;
  	 dev_WirelessComConfig.Mode_SW = NormalMode; 		/*! UART and wireless channel are open, transparent transmission is on*/
  	 SubSys_WirelessCom_Config_WORK_MODE(&dev_WirelessComConfig);

  	 /*! Will be filled for the Payload of the Satellite(Target) Device */
  	 dev_WirelessComApp.Target_ADDH = 0x14;
  	 dev_WirelessComApp.Target_ADDL = 0x53;
  	 dev_WirelessComApp.Target_Ch   = 0x05;


  	  /*! Interrupt is active for receiving wireless data
  	   * You need to cast variable type from char to uint8_t because of the instruction of Uart Receive function*/
  	  HAL_UART_Receive_IT(dev_WirelessComApp.huartX, (uint8_t *)dev_WirelessComApp.Buffer.Rx, sizeof(dev_WirelessComApp.Buffer.Rx));
  	  #endif
    /******<<< WIRELESS COMMUNICATION SETTING & TELEMETRY INITIALIZATION END <<<******/

	/******>>> USB-TTL INITIALIZATION BEGIN >>>******/

  	 /*! Start receiving from Laptop(GroundStation) to EmbeddedGroundStation(STM32)*/
	 HAL_UART_Receive_IT(&huart1, UsbTTL2EmbeddedGS, sizeof(UsbTTL2EmbeddedGS));

	/******<<< USB-TTL INITIALIZATION END <<<******/


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /*! Read ambiance temperature for IOT mission */
	  MS5611_Read_ActVal(&MS5611);

	  /*! Do it by 1Hz*/
	  HAL_Delay(1000);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  huart1.Init.BaudRate = 115200;
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

  /* USER CODE END USART1_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
