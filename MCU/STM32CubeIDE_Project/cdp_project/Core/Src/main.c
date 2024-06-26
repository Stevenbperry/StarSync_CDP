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

/* USER CODE BEGIN PV */

double accel[3] = {0, 0, 0}; 	// accelerometer values
double filtered[3] = {0, 0, 0}; // filtered values

uint8_t error_flag;	 // any error that the accelerometer might throw
ekf_t ekf;		// ekf object

uint32_t start, end, elapsed = 0; // using these to track loop execution time

double altitude, azimuth;	// globally defined variables for alt az so I can track

char debugMsg[100];			// variable to hold messages to be transmitted over UART
char command[RX_BUFFER_SIZE];	// rx buffer for interrupt reception
volatile uint32_t rxIndex = 0;	// variable representing what index the next UART byte goes to
volatile bool messageReady = false;	// when a message is ready, null terminate it, feed it
									// to HC05_ProcessCommand();

double BMA456_1_X_OFFSET = 0;
double BMA456_1_Y_OFFSET = 0;
double BMA456_1_Z_OFFSET = 0;

HC05_ModeStatus modeStatus = {MODE_CALIBRATION};	// current status of the telescope

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void model(ekf_t*, double*);
void ekf_setup(ekf_t*);
void calculateAnglesFromAcceleration(const double accel[3], double *altitude, double *azimuth);
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
  /* USER CODE BEGIN 2 */

  BMA456_Startup(hi2c1);	// initializes the BMA456 accelerometer

  ekf_setup(&ekf);

  int increment = 0;

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
			 if(increment>=10000000){
				  sprintf(debugMsg, "Current mode: POINTING\r\nReference Vector: %f %f\r\n", modeStatus.altitude, modeStatus.azimuth);
				  HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), 100);
				  increment = 0;
	    	  	}
	          break;
	      case MODE_STANDBY:
				// Read acceleration data
				int16_t accelX, accelY, accelZ;
				BMA456_ReadAccelData(&accelX, &accelY, &accelZ, hi2c1);

				// Convert to double for calculation (assuming BMA456 scale is set for +-2g and 12-bit resolution)
				accel[0] = ((double)accelX / BMA456_FSR * 9.80665) - BMA456_1_X_OFFSET;
				accel[1] = ((double)accelY / BMA456_FSR * 9.80665) - BMA456_1_Y_OFFSET;
				accel[2] = ((double)accelZ / BMA456_FSR * 9.80665) - BMA456_1_Z_OFFSET;

				model(&ekf, accel);

				ekf_step(&ekf, accel);

				filtered[0] = ekf.x[0];
				filtered[1] = ekf.x[1];
				filtered[2] = ekf.x[2];
				// Calculate angles
				calculateAnglesFromAcceleration(filtered, &altitude, &azimuth);

				// Optionally, send these values over UART for debugging
	    	  	if(increment>=1500){
					sprintf(debugMsg, "Altitude: %f, Azimuth: %f\r\n", altitude, azimuth);
					HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), 100);
					increment = 0;
	    	  	}
	          break;
	      case MODE_CALIBRATION:
	    	  	increment = 0;
	    	  	while (increment < 10000) {
					increment++;
					int16_t accelX, accelY, accelZ;
					BMA456_ReadAccelData(&accelX, &accelY, &accelZ, hi2c1);

					// Convert to double for calculation (assuming BMA456 scale is set for +-2g and 12-bit resolution)
					accel[0] = ((double)accelX / BMA456_FSR * 9.80665);
					accel[1] = ((double)accelY / BMA456_FSR * 9.80665);
					accel[2] = ((double)accelZ / BMA456_FSR * 9.80665);


					model(&ekf, accel);

					ekf_step(&ekf, accel);

					filtered[0] = ekf.x[0];
					filtered[1] = ekf.x[1];
					filtered[2] = ekf.x[2];
	    	  	}
	    	  	BMA456_1_X_OFFSET = filtered[0] - 0;
	    	  	BMA456_1_Y_OFFSET = filtered[1] - 0;
	    	  	BMA456_1_Z_OFFSET = filtered[2] - 9.80556;
				sprintf(debugMsg, "Calibration complete...\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), 100);
				modeStatus.currentMode = MODE_STANDBY;
	          break;
	      case MODE_HEALTH_CHECK:
	    	  	if(increment>=10000000){
					sprintf(debugMsg, "Current mode: HEALTH CHECK\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), 100);
					increment = 0;
	    	  	}
	          break;
	      default:
				sprintf(debugMsg, "Failure to change modes.\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), 100);
				increment = 0;
				error_flag = 16;
				Error_Handler();
	          break;
	  }
	  increment++;
	  end = HAL_GetTick();
	  elapsed = end - start;
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
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TMC_2208_2_MS2_GPIO_Port, TMC_2208_2_MS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, TMC_2208_DIR_2_Pin|TMC_2208_STEP_2_Pin|TMC_2208_EN_2_Pin|TMC_2208_2_MS1_Pin
                          |LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TMC_2208_1_MS2_Pin|TMC_2208_EN_1_Pin|TMC_2208_DIR_1_Pin|TMC_2208_STEP_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TMC_2208_1_MS1_Pin|HC05_AT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin I2C_INTERRUPT_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|I2C_INTERRUPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin TMC_2208_1_MS2_Pin TMC_2208_EN_1_Pin TMC_2208_DIR_1_Pin
                           TMC_2208_STEP_1_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|TMC_2208_1_MS2_Pin|TMC_2208_EN_1_Pin|TMC_2208_DIR_1_Pin
                          |TMC_2208_STEP_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TMC_2208_2_MS2_Pin */
  GPIO_InitStruct.Pin = TMC_2208_2_MS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TMC_2208_2_MS2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TMC_2208_DIR_2_Pin TMC_2208_STEP_2_Pin TMC_2208_EN_2_Pin TMC_2208_2_MS1_Pin
                           LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = TMC_2208_DIR_2_Pin|TMC_2208_STEP_2_Pin|TMC_2208_EN_2_Pin|TMC_2208_2_MS1_Pin
                          |LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : TMC_2208_1_MS1_Pin HC05_AT_Pin */
  GPIO_InitStruct.Pin = TMC_2208_1_MS1_Pin|HC05_AT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AMT103_2_B_Pin AMT103_2_A_Pin AMT103_1_B_Pin */
  GPIO_InitStruct.Pin = AMT103_2_B_Pin|AMT103_2_A_Pin|AMT103_1_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : AMT103_1_A_Pin OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = AMT103_1_A_Pin|OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
 * @param double accelX, accelY, accelZ: formatted accelerometer data in m/s^2
 * @param double* altitude, azimuth: pointers that point to the variables where the results will be stored
 */
void calculateAnglesFromAcceleration(const double accel[3], double *altitude, double *azimuth) {
	double accelX = accel[0];
	double accelY = accel[1];
	double accelZ = accel[2];
    *altitude = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / M_PI;
    *azimuth = atan2(accelX, accelZ) * 180.0 / M_PI;
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
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /* Turns on all the on-board LEDs if there is a critical error */
	if (error_flag == 1){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	} else if (error_flag == 2){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	} else if (error_flag == 4){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	} else if (error_flag == 8){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	} else if (error_flag == 16){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	}

  HAL_Delay(10000); // 10 seconds pass and the controller resets
  HAL_NVIC_SystemReset();

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
