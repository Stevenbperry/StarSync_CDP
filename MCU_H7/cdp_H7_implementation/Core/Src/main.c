/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  *
  ******************************************************************************
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

double unfiltered[6]; 	// accelerometer values
double filtered[6]; // filtered values

uint8_t error_flag;	 // any error that the accelerometer might throw
ekf_t ekf;		// ekf object

uint32_t start, end, elapsed = 0; // using these to track loop execution time

char debugMsg[100];			// variable to hold messages to be transmitted over UART
char command[RX_BUFFER_SIZE];	// rx buffer for interrupt reception
volatile uint32_t rxIndex = 0;	// variable representing what index the next UART byte goes to
volatile bool messageReady = false;	// when a message is ready, null terminate it, feed it
									// to HC05_ProcessCommand();
float mag[3];
int16_t accelX, accelY, accelZ;
double angles[2];


double BMA456_X_OFFSET = DEFAULT_BMA456_X_OFFSET;
double BMA456_Y_OFFSET = DEFAULT_BMA456_Y_OFFSET;
double BMA456_Z_OFFSET = DEFAULT_BMA456_Z_OFFSET;

Telescope_Status modeStatus =  {.currentMode = MODE_STANDBY,
								.reference_altitude = 0,
								.reference_azimuth = 0,
								.current_altitude = 0,
								.current_azimuth = 0,
							    .est_altitude = 0,
							    .est_azimuth = 0,
							    .ref_mag_x = 0,
							    .ref_mag_y = 0,
								.ref_dec = 0,
							    .current_mag_x = 0,
								.current_mag_y = 0,
								.current_mag_z = 0,
							    .encoder1 = 0,
								.encoder2 = 0,
							    .motor1_en = 0,
							    .motor2_en = 0,
								.hard_iron = {0, 0, 0},
								.soft_iron = {{1, 0, 0},
											  {0, 1, 0},
											  {0, 0, 1}}}; // initialize the telescope

int increment = 0;
															// initialize the magnetometer
MAGNETO_InitTypeDef initDef = {
	    .Register1 = LIS3MDL_MAG_OM_XY_ULTRAHIGH | LIS3MDL_MAG_ODR_80_HZ,
	    .Register2 = LIS3MDL_MAG_FS_4_GA,
	    .Register3 = LIS3MDL_MAG_CONTINUOUS_MODE,
	    .Register4 = LIS3MDL_MAG_OM_Z_ULTRAHIGH,
	    .Register5 = LIS3MDL_MAG_BDU_MSBLSB
	};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void calculateAnglesFromAcceleration(const double data[6], double *altitude, double *azimuth);
float vector_dot(const float[3], const float[3]);
void vector_normalize(float[3]);
void vector_cross(const float[3], const float[3], float[3]);
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
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  BMA456_Startup(hi2c1);	// initializes the BMA456 accelerometer
  LIS3MDL_MagInit(initDef);	// initializes the LIS3MDL magnetometer

  ekf_setup(&ekf);


  HAL_UART_Receive_IT(&huart2, (uint8_t*) command, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  start = HAL_GetTick();
	  // Based on modeStatus.currentMode, switch between different operational modes.
	  if(messageReady==true){
		  messageReady = false;
		  HC05_ProcessCommand(command, &modeStatus, &huart2);
	  }

	  switch(modeStatus.currentMode) {

	      case MODE_POINTING:
	    	 volatile float distance_traveled_alt, distance_traveled_az, alt_diff, az_diff;
	    	 distance_traveled_alt = modeStatus.encoder1 * AMT103_DPP;
	    	 distance_traveled_az = modeStatus.encoder2 * AMT103_DPP;
	    	 modeStatus.est_altitude = modeStatus.current_altitude + distance_traveled_alt;
	    	 modeStatus.est_azimuth = modeStatus.current_azimuth + distance_traveled_az;
	    	 alt_diff = modeStatus.est_altitude - modeStatus.reference_altitude;
	    	 az_diff = modeStatus.est_azimuth - modeStatus.reference_azimuth;
	    	 // altitude control
	    	 if(fabs(alt_diff) >= 1){
	    		 if(alt_diff < 0){
	    			 TMC2208_Backward(ALT_MOTOR);
	    		 } else{
	    			 TMC2208_Forward(ALT_MOTOR);
	    		 }
	    		 TMC2208_Step(ALT_MOTOR);
	    	 }
	    	 // azimuth control
	    	 if(fabs(az_diff) >= 1){
	    		 if(az_diff < 0){
	    			 TMC2208_Backward(AZ_MOTOR);
	    		 } else{
	    			 TMC2208_Forward(AZ_MOTOR);
	    		 }
	    		 TMC2208_Step(2);
	    	 }
			 if(increment >= 100 ){
				 // if the telescope isn't moving from the encoder's POV, send debug message and leave pointing
				 if(distance_traveled_alt == 0 || distance_traveled_az == 0){
					  sprintf(debugMsg, "ERROR - invalid measured distance traveled from encoders! Exiting pointing mode.\r\n");
					  HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), 100);
					  modeStatus.currentMode = MODE_STANDBY;
					  increment = 0;
				 } else{	// regular reports of how far the scope has traveled
					  sprintf(debugMsg, "Distance traveled - Altitude: %f | Azimuth: %f\r\n", distance_traveled_alt, distance_traveled_az);
					  HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), 100);
					 increment = 0;
				 }
	    	  	}
			  increment++;
	          break;

	      case MODE_STANDBY:
			    modeStatus.encoder1 = 0;
                modeStatus.encoder2 = 0;
				// Read acceleration data
				BMA456_ReadAccelData(&accelX, &accelY, &accelZ, hi2c1);
				LIS3MDL_MagReadXYZ((float*) &mag);

				// Convert to double for calculation (assuming BMA456 scale is set for +-2g and 12-bit resolution)
				unfiltered[0] = ((double)accelX / BMA456_FSR * 9.80665) - BMA456_X_OFFSET;
				unfiltered[1] = ((double)accelY / BMA456_FSR * 9.80665) - BMA456_Y_OFFSET;
				unfiltered[2] = ((double)accelZ / BMA456_FSR * 9.80665) - BMA456_Z_OFFSET;

				// Apply hard and soft iron correction to magnetometer data
				for (int i = 0; i < 3; i++){
					mag[i] -= modeStatus.hard_iron[i];
				}
				for (int i = 0; i < 3; i++){
					for (int j = 0; j < 3; j++){
						unfiltered[i+3] += mag[j] * modeStatus.soft_iron[i][j];
					}
				}
				// Calculate angles
				calculateAnglesFromAcceleration(unfiltered, &angles[0], &angles[1]);

				model(&ekf, angles);

				ekf_step(&ekf, angles);

				for (int i = 0; i < Nsta; i++) {
				    angles[i] = ekf.x[i];
				}
				modeStatus.current_altitude = angles[0];
				modeStatus.current_azimuth = angles[1] + modeStatus.ref_dec;

				// Send angles over UART to debug
	    	  	if(increment>=4500){
					sprintf(debugMsg, "Altitude: %f, Azimuth: %f\r\n", modeStatus.current_altitude, modeStatus.current_azimuth);
					HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), 100);
					increment = 0;
	    	  	}
	          break;

	      case MODE_CALIBRATION_A:
	    	  	BMA456_Calibration(hi2c1, &BMA456_X_OFFSET, &BMA456_Y_OFFSET, &BMA456_Z_OFFSET, ekf);
				sprintf(debugMsg, "Accelerometer calibration complete...\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), 100);
				modeStatus.currentMode = MODE_STANDBY;
	          break;

	      case MODE_CALIBRATION_M:
	    	    Magnetic_Calibration(&huart2);
				sprintf(debugMsg, "Magnetometer calibration complete...\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), 100);
				modeStatus.currentMode = MODE_STANDBY;

	    	  break;

	      case MODE_HEALTH_CHECK:
	    	    uint8_t error_flags = 0;
				BMA456_ReadErrorFlag(&error_flags, hi2c1);
				if (error_flags != 0){
					sprintf(debugMsg, "ERROR! BMA456 reported error: %i \r\n", error_flags);
					HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), 100);
				}
				int16_t iaccelX, iaccelY, iaccelZ, paccelX, paccelY, paccelZ;
				BMA456_ReadAccelData(&iaccelX, &iaccelY, &iaccelZ, hi2c1);
				HAL_Delay(2000);
				BMA456_ReadAccelData(&paccelX, &paccelY, &paccelZ, hi2c1);
				if (iaccelX == paccelX || iaccelY == paccelY || iaccelZ == paccelZ){
					sprintf(debugMsg, "ERROR! BMA456 not updating values.\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), 100);
				}
				float imag[3];
				LIS3MDL_MagReadXYZ((float*) &imag);
				for (int i = 0; i < 3; i++){
					if(imag[i] == 0 || imag[i] == 0xFF){
						sprintf(debugMsg, "ERROR! LIS3MDL not updating values.\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), 100);
					}
				}
				modeStatus.currentMode = MODE_STANDBY;
	          break;

	      default:
				sprintf(debugMsg, "Failure to change modes.\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), 100);
				modeStatus.currentMode = MODE_STANDBY;
				increment = 0;
	          break;

	  }
	  increment++;
	  // Disable the motors if we aren't in pointing mode.
 	 if(modeStatus.currentMode != MODE_POINTING){
 		 TMC2208_Disable();
 		 modeStatus.motor1_en = 0;
 		 modeStatus.motor2_en = 0;
 	 }

	  elapsed = HAL_GetTick() - start;	// execution time of main loop in milliseconds
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 275;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hi2c1.Init.Timing = 0x60404E72;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, TMC_2208_1_MS1_Pin|TMC_2208_STEP_1_Pin|TMC_2208_1_MS2_Pin|TMC_2208_DIR_1_Pin
                          |TMC_2208_EN_1_Pin|LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TMC_2208_EN_2_GPIO_Port, TMC_2208_EN_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_FS_PWR_EN_Pin|TMC_2208_2_MS1_Pin|TMC_2208_2_MS2_Pin|TMC_2208_STEP_2_Pin
                          |TMC_2208_DIR_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TMC_2208_1_MS1_Pin TMC_2208_STEP_1_Pin TMC_2208_1_MS2_Pin TMC_2208_DIR_1_Pin
                           TMC_2208_EN_1_Pin LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = TMC_2208_1_MS1_Pin|TMC_2208_STEP_1_Pin|TMC_2208_1_MS2_Pin|TMC_2208_DIR_1_Pin
                          |TMC_2208_EN_1_Pin|LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TMC_2208_EN_2_Pin */
  GPIO_InitStruct.Pin = TMC_2208_EN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TMC_2208_EN_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AMT103_2_B_Pin */
  GPIO_InitStruct.Pin = AMT103_2_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AMT103_2_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : AMT103_2_A_Pin */
  GPIO_InitStruct.Pin = AMT103_2_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AMT103_2_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_PWR_EN_Pin TMC_2208_2_MS1_Pin TMC_2208_2_MS2_Pin TMC_2208_STEP_2_Pin
                           TMC_2208_DIR_2_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin|TMC_2208_2_MS1_Pin|TMC_2208_2_MS2_Pin|TMC_2208_STEP_2_Pin
                          |TMC_2208_DIR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : AMT103_1_B_Pin AMT103_1_A_Pin */
  GPIO_InitStruct.Pin = AMT103_1_B_Pin|AMT103_1_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief EKF Initialization function.
  * @param ekf_t* ekf : ekf object defined in tiny_ekf_struct.h
  * @retval None
  */
void ekf_setup(ekf_t* ekf){

    // Initialize the EKF here
    ekf_init(ekf, Nsta, Mobs);

    // Set initial state estimates (e.g., to 0 if unknown)
    for (int i = 0; i < Nsta; ++i) {
        ekf->x[i] = 0.0;
    }

    // Define initial process noise covariance matrix Q
    double q = 0.01; // example process noise variance
    for (int i = 0; i < Nsta; ++i) {
        ekf->Q[i][i] = q;
    }

    // Define initial measurement noise covariance matrix R
    double r = 0.02; // example measurement noise variance
    for (int i = 0; i < Mobs; ++i) {
        ekf->R[i][i] = r;
    }
}
/**
  * @brief Creates state model for EKF.
  * @param ekf_t* ekf : ekf object defined in tiny_ekf_struct.h
  * @param double* z : input measurements
  * @retval None
  */
void model(ekf_t* ekf, double* z) {
    // Assuming direct measurement of acceleration for simplicity

    // State transition function f(x)
    for (int i = 0; i < Nsta; ++i) {
        ekf->fx[i] = ekf->x[i]; // Simple model: next state is equal to current state
    }

    // Measurement function h(x)
    for (int i = 0; i < Mobs; ++i) {
        ekf->hx[i] = ekf->x[i]; // Direct observation: measurement equals state
    }

    // F and H Jacobian matrices
    for (int i = 0; i < Nsta; ++i) {
        for (int j = 0; j < Nsta; ++j) {
            ekf->F[i][j] = (i == j) ? 1 : 0; // Identity matrix for F
        }
    }

    for (int i = 0; i < Mobs; ++i) {
        for (int j = 0; j < Nsta; ++j) {
            ekf->H[i][j] = (i == j) ? 1 : 0; // Identity matrix for H
        }
    }
}
/**
 * @brief Calculates alt az from accel data
 */
void calculateAnglesFromAcceleration(const double data[6], double *altitude, double *azimuth) {


    // Compute the East and North vectors
    float E[3], N[3], mag[3], accel[3];
    for (int i = 0;i<3;i++){
    	accel[i] = data[i];
    	mag[i+3] = data[i+3];
    }
    vector_cross(mag, accel, E); // Cross product of mag and accel vectors
    vector_normalize(E);                        // Normalize the East vector

    vector_cross(accel, E, N);             // Cross product of accel and East vectors
    vector_normalize(N);                        // Normalize the North vector

    // Compute the heading
    // Assuming 'from' vector is the front of the device and it's [1, 0, 0]
    float from[3] = {1, 0, 0};
    float heading = atan2(vector_dot(E, from), vector_dot(N, from)) * 180 / M_PI;
    if (heading < 0) heading += 360;

    *azimuth = heading;

    *altitude = asin(data[0] / sqrt(data[0] * data[0] + data[1] * data[1] + data[2] * data[2])) * (180 / M_PI);


}
/**
 * @brief Computes the cross product of two 3D vectors.
 * @param a The first input vector.
 * @param b The second input vector.
 * @param result The output vector that is the cross product of a and b.
 */
void vector_cross(const float a[3], const float b[3], float result[3]) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

/**
 * @brief Normalizes a 3D vector to unit length.
 * @param v The input vector to normalize.
 */
void vector_normalize(float v[3]) {
    // Calculate the magnitude of the vector
    float magnitude = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

    // Avoid division by zero
    if (magnitude > 0.0f) {
        // Normalize the vector by its magnitude
        v[0] /= magnitude;
        v[1] /= magnitude;
        v[2] /= magnitude;
    }
}

/**
 * @brief Computes the dot product of two 3D vectors.
 * @param a The first input vector.
 * @param b The second input vector.
 * @return The scalar dot product of vectors a and b.
 */
float vector_dot(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/**
 * @brief Retrieves data that was recorded from the interrupt
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (command[rxIndex] == '\n') { // Assuming '\n' as the delimiter
		messageReady = true; // A complete message is ready
		command[rxIndex] = '\0'; // Null-terminate the message
		rxIndex = 0; // Reset index for next message
	} else {
		rxIndex++; // Prepare for next character
		if (rxIndex >= RX_BUFFER_SIZE) {
			rxIndex = 0; // Prevent buffer overflow
		}
	}
	// Re-enable UART receive interrupt for next byte
	HAL_UART_Receive_IT(huart, (uint8_t*)&command[rxIndex], 1);

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

    MX_I2C1_Init(); // Reinitialize I2C peripheral

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
