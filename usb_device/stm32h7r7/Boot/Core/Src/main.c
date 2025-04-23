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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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

UART_HandleTypeDef huart4;

PCD_HandleTypeDef hpcd_USB_OTG_HS;

/* USER CODE BEGIN PV */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart4,(uint8_t*)&ch,1,1000);
  return ch;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USB_OTG_HS_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*属性为Normal，cache的属性为 write-back，即仅更新cache，
 *在合适的时候(由cache策略决定或者软件强制更新)将数据更新到相应的SRAM空间
 *特别注意：如果要数据立即更新，写之后要SCB_CleanDCache，读数据时要SCB_InvalidateDCache
 */
#define MPU_Normal_WB        0x00


/*属性为Normal，cache的属性为 write-back，即仅更新cache，
 *在合适的时候(由cache策略决定或者软件强制更新)将数据更新到相应的SRAM空间
 *特别注意：如果要数据立即更新，写之后要SCB_CleanDCache，读数据时要SCB_InvalidateDCache
 */
#define MPU_Normal_WBWARA    0x01  //外部和内部写入无写入分配


/*属性为 normal，cache的属性为 Write-through，即更新cache的同时，
 *将数据同时写入相应的物理地址空间
 *特别注意：如果要数据立即更新，可以直接往内存写数据，但读数据时要SCB_InvalidateDCache
 */
#define MPU_Normal_WT         0x02


/*属性为 normal，禁用共享,禁用缓存
 */
#define MPU_Normal_NonCache   0x03


/*属性为 Device，共享设置无效，禁用共享,禁用缓存
 */
#define MPU_Device_NonCache   0x04

/**
  * @brief  配置MPU区域属性和大小寄存器值
    * @param  Region            MPU保护区域，取值范围（0—7）
    * @param  AccessPermission  数据访问权限，取值范围（MPU_NO_ACCESS—MPU_PRIV_RO_URO）
    * @param  TypeExtField      用于配置 Cache 策略，可以取值（0，1，2），一般只用0和1
    *
  * @param  Address             MPU保护区域基地址，特别注意配置的Address需要被Size整除
  * @param  Size                MPU保护区域大小,可以取值（MPU_1KB，MPU_4KB ...MPU_512MB）
    * @param  IsShareable       保护的存储空间是否可以共享，1=允许共享，0=禁止共享。
  * @param  IsCacheable         保护的存储空间是否可以缓存，1=允许缓存，0=禁止缓存。
  * @param  IsBufferable        使能Cache之后，策略是write-through还是write-back(bufferable)
    *                           1=允许缓冲，即回写（write-back），0=禁止缓冲，即直写（write-through）。
  * @retval None
  */
static void BSP_MPU_ConfigRegion(uint8_t    Number,
                                 uint8_t    TypeExtField,
                                 uint32_t   Address,
                                 uint32_t   Size,
                                 uint8_t    IsBufferable,
                                 uint8_t    IsCacheable)
{
    MPU_Region_InitTypeDef MPU_InitStruct;

    /* 禁用MPU */
    HAL_MPU_Disable();

    /* 配置MPU属性*/
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = Address;                      //区域基地址。
    MPU_InitStruct.Size = Size;                                //要配置的区域的区域大小。
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;  //数据访问权限允许您配置用户和特权模式的读/写访问权限。
    MPU_InitStruct.IsBufferable = IsBufferable;                //区域是可缓冲的，即使用回写缓存。 可缓存但不可缓冲的区域使用直写策略。WB
    MPU_InitStruct.IsCacheable = IsCacheable;                  //区域是否可缓存的，即其值是否可以保存在缓存中。//M7 内核只要开启了 Cache，read allocate 就是开启的
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;     //区域是否可以在多个总线主控器之间共享。H7 的应用笔记对齐的描述是开启共享基本等同于关闭 Cache
    MPU_InitStruct.Number = Number;                            //区域号。
    MPU_InitStruct.TypeExtField = TypeExtField;                //键入扩展字段，允许您配置内存访问类型。
    MPU_InitStruct.SubRegionDisable = 0x00;                    //子区域禁用字段。
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;//指令访问禁用位，0=允许指令访问，1=禁止指令访问

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* 启用MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT); //表示禁止了背景区，访问任何未使能 MPU 的区域均会造成内存异常 MemFault


}

void cpu_mpu_config(uint8_t Region, uint8_t Mode, uint32_t Address, uint32_t Size)
{
    switch (Mode)
    {
    case MPU_Normal_WB:
        /*write back,no write allocate */
        /* 设置内存为Normal类型,禁用共享, 回写模式不带写入读取分配*/
        BSP_MPU_ConfigRegion(Region, MPU_TEX_LEVEL0, Address, Size, MPU_ACCESS_BUFFERABLE, MPU_ACCESS_CACHEABLE);
        break;

    case MPU_Normal_WBWARA:
        /*write back,write and read allocate */
        /* 设置内存为Normal类型,禁用共享, 回写模式带写入读取分配*/
        BSP_MPU_ConfigRegion(Region, MPU_TEX_LEVEL1, Address, Size, MPU_ACCESS_BUFFERABLE, MPU_ACCESS_CACHEABLE);
        break;

    case MPU_Normal_WT:
        /*write through,no write allocate */
        /* 设置内存为Normal类型,禁用共享, 直写模式*/
        BSP_MPU_ConfigRegion(Region, MPU_TEX_LEVEL0, Address, Size, MPU_ACCESS_NOT_BUFFERABLE, MPU_ACCESS_CACHEABLE);
        break;

    case MPU_Normal_NonCache:
        /* 设置内存为Normal类型,禁用共享，禁用缓存模式*/
        BSP_MPU_ConfigRegion(Region, MPU_TEX_LEVEL1, Address, Size, MPU_ACCESS_NOT_BUFFERABLE, MPU_ACCESS_NOT_CACHEABLE);
        break;
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
  cpu_mpu_config(0, MPU_Normal_NonCache, 0x24050000-0x4000, MPU_REGION_SIZE_64KB);
  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_UART4_Init();
  //MX_USB_OTG_HS_PCD_Init();
  /* USER CODE BEGIN 2 */
  /* Enable the USB HS regulator. */
  HAL_PWREx_EnableUSBHSregulator();
    
  extern void cdc_acm_msc_init(uint8_t busid, uintptr_t reg_base);
  cdc_acm_msc_init(0, USB_OTG_HS_PERIPH_BASE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      extern void cdc_acm_data_send_with_dtr_test(uint8_t busid);
      cdc_acm_data_send_with_dtr_test(0);
      HAL_Delay(50);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL1.PLLM = 12;
  RCC_OscInitStruct.PLL1.PLLN = 300;
  RCC_OscInitStruct.PLL1.PLLP = 1;
  RCC_OscInitStruct.PLL1.PLLQ = 2;
  RCC_OscInitStruct.PLL1.PLLR = 2;
  RCC_OscInitStruct.PLL1.PLLS = 2;
  RCC_OscInitStruct.PLL1.PLLT = 2;
  RCC_OscInitStruct.PLL1.PLLFractional = 0;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK4|RCC_CLOCKTYPE_PCLK5;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  hpcd_USB_OTG_HS.Instance = USB_OTG_HS;
  hpcd_USB_OTG_HS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_HS.Init.speed = PCD_SPEED_HIGH;
  hpcd_USB_OTG_HS.Init.phy_itface = USB_OTG_HS_EMBEDDED_PHY;
  hpcd_USB_OTG_HS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_HS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_HS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();

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
