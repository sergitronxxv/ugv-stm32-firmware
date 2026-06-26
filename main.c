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
#include <stdint.h> // Para usar int16_t con tamaños fijos
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
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile uint32_t pwm1;
volatile uint32_t pwm2;
int16_t vel1;
int16_t vel2;
volatile uint32_t rising1;
volatile uint32_t rising2;
volatile uint32_t falling1;
volatile uint32_t falling2;
volatile uint8_t waiting_for_falling1;
volatile uint8_t waiting_for_falling2;
uint32_t mailbox;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
int mapPulseToSpeed(uint32_t pulse);
int map(int x, int in_min, int in_max, int out_min, int out_max);
void send_can_velocities(int16_t v1, int16_t v2);
int clamp(int value, int min_val, int max_val);
void enable_motors();
void nmt_start_nodes();
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
  MX_TIM2_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  //CAN2_ConfigFilter();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

  HAL_Delay(500);  // dar tiempo a que el bus se estabilice
  nmt_start_nodes();   // ← primero sacar los nodos de Booting
  HAL_Delay(200);
  enable_motors();     // ← luego mandar la secuencia DS402

  pwm1 = 0;
  pwm2 = 0;
  vel1 = 0;
  vel2 = 0;
  rising1 = 0;
  rising2 = 0;
  falling1 = 0;
  falling2 = 0;
  waiting_for_falling1 = 0;
  waiting_for_falling2 = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // 1. LEER MANDOS PWM (Obtenemos valores de -255 a 255)
	  int throttle = mapPulseToSpeed(pwm2);
	  int steering = mapPulseToSpeed(pwm1);

	  // Filtro de zona muerta para que no se mueva solo por ruido
	  if (throttle > -15 && throttle < 15) throttle = 0;
	  if (steering > -15 && steering < 15) steering = 0;

	  // 2. MEZCLA DIFERENCIAL (Lógica de tanque)
	  // Calculamos cuánto debe moverse cada motor de forma independiente
	  int motor_izq = throttle - steering;
	  int motor_der = throttle + steering;

	  // 3. ASEGURAR LÍMITES (Clamp)
	  // No podemos pasarnos de -255 o 255 antes de mapear
	  motor_izq = clamp(motor_izq, -255, 255);
	  motor_der = clamp(motor_der, -255, 255);

	  // 4. CONVERTIR A FORMATO ANTENA (0 a 254, donde 127 es PARADO)
	  // Ahora Byte 0 será la rueda izquierda y Byte 1 la rueda derecha
	  uint8_t byte0_izq = (uint8_t)map(motor_izq, -255, 255, 0, 254);
	  uint8_t byte1_der = (uint8_t)map(motor_der, -255, 255, 0, 254);

	  // 5. ENVIAR MENSAJE DE SEGURIDAD (ID 1E4)
	  CAN_TxHeaderTypeDef safetyHeader;
	  // Byte 0: Velocidad Izquierda, Byte 1: Velocidad Derecha, Byte 2: 0x01 (Activo)
	  uint8_t safetyData[8] = {byte0_izq, byte1_der, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};

	  safetyHeader.StdId = 0x1E4;
	  safetyHeader.DLC = 8;
	  safetyHeader.IDE = CAN_ID_STD;
	  safetyHeader.RTR = CAN_RTR_DATA;

	  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);
	  HAL_CAN_AddTxMessage(&hcan2, &safetyHeader, safetyData, &mailbox);

	  HAL_Delay(20); // Ciclo de 50Hz

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
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 12;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    /* ============================
       CANAL 2  (PWM1)
       ============================ */
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        if (!waiting_for_falling1)
        {
            // Flanco de subida
            rising1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

            // Deshabilitar canal
            TIM2->CCER &= ~TIM_CCER_CC2E;

            // Limpiar flag
            __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC2);

            // Cambiar a flanco de bajada
            TIM2->CCER |= TIM_CCER_CC2P;

            // Rehabilitar canal
            TIM2->CCER |= TIM_CCER_CC2E;

            waiting_for_falling1 = 1;
        }
        else
        {
            // Flanco de bajada
            falling1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

            if (falling1 >= rising1)
                pwm1 = falling1 - rising1;
            else
                pwm1 = (htim2.Init.Period - rising1 + falling1);

            // Deshabilitar canal
            TIM2->CCER &= ~TIM_CCER_CC2E;

            // Limpiar flag
            __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC2);

            // Cambiar a flanco de subida
            TIM2->CCER &= ~TIM_CCER_CC2P;

            // Rehabilitar canal
            TIM2->CCER |= TIM_CCER_CC2E;

            waiting_for_falling1 = 0;
        }
    }

    /* ============================
       CANAL 3  (PWM2)
       ============================ */
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
        if (!waiting_for_falling2)
        {
            // Flanco de subida
            rising2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

            // Deshabilitar canal
            TIM2->CCER &= ~TIM_CCER_CC3E;

            // Limpiar flag
            __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC3);

            // Cambiar a flanco de bajada
            TIM2->CCER |= TIM_CCER_CC3P;

            // Rehabilitar canal
            TIM2->CCER |= TIM_CCER_CC3E;

            waiting_for_falling2 = 1;
        }
        else
        {
            // Flanco de bajada
            falling2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

            if (falling2 >= rising2)
                pwm2 = falling2 - rising2;
            else
                pwm2 = (htim2.Init.Period - rising2 + falling2);

            // Deshabilitar canal
            TIM2->CCER &= ~TIM_CCER_CC3E;

            // Limpiar flag
            __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC3);

            // Cambiar a flanco de subida
            TIM2->CCER &= ~TIM_CCER_CC3P;

            // Rehabilitar canal
            TIM2->CCER |= TIM_CCER_CC3E;

            waiting_for_falling2 = 0;
        }
    }
}


int mapPulseToSpeed(uint32_t pulse) {
    if (pulse < 800 || pulse > 2200)
    	return 0;  // ← AÑADIR ESTO
    if (pulse > 1500 + 30)
        return map(pulse, 1530, 2000, 0, 255);
    if (pulse < 1500 - 30)
        return map(pulse, 1000, 1470, -255, 0);
    return 0;
}

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void send_can_velocities(int16_t v1, int16_t v2) {
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
    uint32_t mailbox;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;

    // --- MOTOR L (0x4A7) ---
    header.StdId = 0x4A7;
    // Según el log, v1 debe ir en los bytes finales y el byte 4 suele ser D2 o similar
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = (uint8_t)(v1 & 0xFF);        // Velocidad byte bajo
    data[5] = (uint8_t)((v1 >> 8) & 0xFF); // Velocidad byte alto
    data[6] = data[4];                     // El log muestra repetición en 4-5 y 6-7
    data[7] = data[5];

    // Esperar a que haya un buzón libre antes de enviar
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);
    HAL_CAN_AddTxMessage(&hcan2, &header, data, &mailbox);

    // --- MOTOR R (0x4A8) ---
    header.StdId = 0x4A8;
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = (uint8_t)(v2 & 0xFF);
    data[5] = (uint8_t)((v2 >> 8) & 0xFF);
    data[6] = 0x0E; // Valor fijo observado en el log original para movimiento
    data[7] = 0x01; // Valor fijo observado en el log original para movimiento

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);
    HAL_CAN_AddTxMessage(&hcan2, &header, data, &mailbox);
}

int clamp(int value, int min_val, int max_val) {
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

void enable_motors(void)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
    uint32_t mailbox;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;

    uint8_t nodes[2] = {39, 40};

    for (int i = 0; i < 2; i++) {
        header.StdId = 0x600 + nodes[i];
        data[0] = 0x2B; // SDO Write 2 bytes (es más seguro 0x2B para uint16)
        data[1] = 0x40; data[2] = 0x60; data[3] = 0x00;

        // 1. Fault Reset (Indispensable)
        data[4] = 0x80; data[5] = 0x00;
        HAL_CAN_AddTxMessage(&hcan2, &header, data, &mailbox);
        HAL_Delay(50);

        // 2. Shutdown
        data[4] = 0x06;
        HAL_CAN_AddTxMessage(&hcan2, &header, data, &mailbox);
        HAL_Delay(50);

        // 3. Switch On
        data[4] = 0x07;
        HAL_CAN_AddTxMessage(&hcan2, &header, data, &mailbox);
        HAL_Delay(50);

        // 4. Enable Operation
        data[4] = 0x0F;
        HAL_CAN_AddTxMessage(&hcan2, &header, data, &mailbox);
        HAL_Delay(100);
    }
}

void nmt_start_nodes(void)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[2];
    uint32_t mailbox;

    header.StdId = 0x000;
    header.DLC = 2;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;

    data[0] = 0x01; // Start Remote Node
    data[1] = 0x00; // 0x00 actúa como broadcast (todos los nodos)
    HAL_CAN_AddTxMessage(&hcan2, &header, data, &mailbox);
    HAL_Delay(100);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];

    if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data);
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