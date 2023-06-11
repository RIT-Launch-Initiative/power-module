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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "device/platforms/stm32/swdebug.h"

#include "device/platforms/stm32/HAL_GPIODevice.h"
#include "device/platforms/stm32/HAL_I2CDevice.h"
#include "device/platforms/stm32/HAL_SPIDevice.h"
#include "device/peripherals/W25Q/W25Q.h"

#include "sched/macros.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart5;
/* USER CODE BEGIN PV */

static HALI2CDevice* i2c = nullptr;
static HALSPIDevice* w25q_spi = nullptr;
static HALGPIODevice* w25q_cs = nullptr;
static HALSPIDevice* wiz_spi = nullptr;
static HALGPIODevice* wiz_cs = nullptr;

static W25Q* w25q = nullptr;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
RetType i2c_poll_task(void*) {
    RESUME();
    CALL(i2c->poll());
    RESET();
    return RET_SUCCESS;
}

RetType w25q_spi_poll_task(void*) {
    RESUME();
    CALL(w25q_spi->poll());
    RESET();
    return RET_SUCCESS;
}

RetType wiz_spi_poll_task(void*) {
    RESUME();
    CALL(wiz_spi->poll());
    RESET();
    return RET_SUCCESS;
}

RetType w25q_poll_task(void *) {
    RESUME();
    CALL(w25q->poll());
    RESET();
    return RET_SUCCESS;
}

RetType w25q_test_task(void*) {
    uint8_t out_buf[] = {0x9F};
    uint8_t in_buf[] = {0x00, 0x00, 0x00};
    RetType ret;

    RESUME();
    swprint("Starting test task\n");

    w25q_cs->set(0);
    ret = CALL(w25q_spi->write(out_buf, 1, 10));
    if (RET_SUCCESS != ret) {
        swprint("#RED#SPI write failed");
    }
    ret = CALL(w25q_spi->read(in_buf, 3, 10));
    if (RET_SUCCESS != ret) {
        swprint("#RED#SPI read failed");
    }
    w25q_cs->set(1);
    swprintx("SPI read: ", in_buf, 3);

    swprint("Initializing W25Q\n");
    if (nullptr == w25q_spi || nullptr == w25q_cs) {
        swprint("Passed nullptr SPI or CS to w25q\n");
    }
    static W25Q w25q_local("Flash memory", *w25q_spi, *w25q_cs);
    ret = CALL(w25q_local.init());
    if (RET_SUCCESS != ret) {
        swprint("#RED#Failed to initialize W25Q\n");
        goto w25q_test_end;
    }

    w25q = &w25q_local;
    swprintf("#GRN#W25Q 0x%6x with %d blocks of %d bytes each\n",
             w25q->m_dev_id, w25q->getNumBlocks(), w25q->getBlockSize());
    sched_start(&w25q_poll_task, {});

    /*
    // set up input
    uint8_t page_in[256];
    memset(page_in, '\0', sizeof(page_in));
    const char text[] = "Testing text for page write";
    strncpy((char*) page_in, text, sizeof(text));
    swprintf("Page in:\n\t%256s\n", (char*) page_in);
    // write to this block
    uint32_t address = 0xFFFF;

	swprintf("Writing to page 0x%4x\n", address);
	ret = CALL(w25q->write(address, page_in));
	if (RET_SUCCESS != ret) {
		swprint("#RED#Failed to write page\n");
		goto w25q_test_end;
	}

	// set up output
	uint8_t page_out[256];
	memset(page_out, '\0', sizeof(page_out));
	// read from page;
	swprintf("Reading from page 0x%4x\n", address);
	ret = CALL(w25q->read(address, page_out));
	if (RET_SUCCESS != ret) {
		swprint("#RED#Failed to read page\n");
		goto w25q_test_end;
	}
	swprintf("Page out:\n\t%256s\n", (char*) page_out);
*/
    w25q_test_end:
    swprint("Exiting flash test task\n");
    RESET();
    return RET_ERROR;
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
    MX_ADC1_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */
    // initialize scheduler
    if (!sched_init(&HAL_GetTick)) {
        swprint("#RED# Failed to initialize schedule;");
        return -1;
    }

    // initialize devices
    RetType status;

    static HALI2CDevice i2c_local("I2C", &hi2c1);
    status = i2c_local.init();
    if (RET_SUCCESS != status) {
        swprint("#RED#Failed to initialize I2C\n");
    } else {
        i2c = &i2c_local;
    }

    static HALSPIDevice wiz_spi_local("Wiznet SPI", &hspi1);
    status = wiz_spi_local.init();
    if (RET_SUCCESS != status) {
        swprint("#RED#Failed to initialize Wiznet SPI");
    } else {
        wiz_spi = &wiz_spi_local;
    }

    static HALSPIDevice w25q_spi_local("W25Q SPI", &hspi2);
    status = w25q_spi_local.init();
    if (RET_SUCCESS != status) {
        swprint("#RED#Failed to initialize W25Q SPI");
    } else {
        w25q_spi = &w25q_spi_local;
    }

    static HALGPIODevice wiz_cs_local("Wiznet CS", ETH_CS_GPIO_Port, ETH_CS_Pin);
    wiz_cs = &wiz_cs_local;

    static HALGPIODevice w25q_cs_local(" CS", MEM_CS_GPIO_Port, MEM_CS_Pin);
    w25q_cs = &w25q_cs_local;

    swprint("Testing power module with task\n");

    uint8_t out_buffer[] = {0x9F};
    uint8_t in_buffer[] = {0x00, 0x00, 0x00};

    swprintx("Writing: ", out_buffer, sizeof(out_buffer));

    HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, out_buffer, sizeof(out_buffer), 200);
    HAL_SPI_Receive(&hspi2, in_buffer, sizeof(in_buffer), 200);
    HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, GPIO_PIN_SET);

    swprintx("Received: ", in_buffer, 3);

//    sched_start(&i2c_poll_task, {});
//    sched_start(&wiz_spi_poll_task, {});
    sched_start(&w25q_spi_poll_task, {});
//    sched_start(&w25q_poll_task, {});
    sched_start(&w25q_test_task, {});

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        sched_dispatch();
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
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_4;
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
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

    /* USER CODE BEGIN SPI2_Init 0 */

    /* USER CODE END SPI2_Init 0 */

    /* USER CODE BEGIN SPI2_Init 1 */

    /* USER CODE END SPI2_Init 1 */
    /* SPI2 parameter configuration*/
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI2_Init 2 */

    /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

    /* USER CODE BEGIN UART5_Init 0 */

    /* USER CODE END UART5_Init 0 */

    /* USER CODE BEGIN UART5_Init 1 */

    /* USER CODE END UART5_Init 1 */
    huart5.Instance = UART5;
    huart5.Init.BaudRate = 115200;
    huart5.Init.WordLength = UART_WORDLENGTH_8B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_NONE;
    huart5.Init.Mode = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart5) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN UART5_Init 2 */

    /* USER CODE END UART5_Init 2 */

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
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, ETH_CS_Pin|ETH_RST_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, GPIO_PIN_SET);

    /*Configure GPIO pins : ADDR0_Pin ADDR1_Pin ADDR2_Pin ADDR3_Pin */
    GPIO_InitStruct.Pin = ADDR0_Pin|ADDR1_Pin|ADDR2_Pin|ADDR3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : ETH_CS_Pin ETH_RST_Pin */
    GPIO_InitStruct.Pin = ETH_CS_Pin|ETH_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : ETH_INT_Pin */
    GPIO_InitStruct.Pin = ETH_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ETH_INT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : MEM_CS_Pin */
    GPIO_InitStruct.Pin = MEM_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MEM_CS_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
