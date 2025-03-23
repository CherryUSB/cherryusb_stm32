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
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "usbh_core.h"
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
void usb_hc_low_level_init(uint8_t busid)
{
   RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  /* USER CODE BEGIN USB_OTG_HS_MspInit 0 */

  /* USER CODE END USB_OTG_HS_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USBPHYC;
    PeriphClkInit.UsbPhycClockSelection = RCC_USBPHYCCLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

  /** Enable USB Voltage detector
  */
    HAL_PWREx_EnableUSBVoltageDetector();
    /* Enable the USB HS regulator. */
    HAL_PWREx_EnableUSBHSregulator();
    
    /* Peripheral clock enable */
    __HAL_RCC_USB_OTG_HS_CLK_ENABLE();
    __HAL_RCC_USBPHYC_CLK_ENABLE();
    /* USB_OTG_HS interrupt Init */
    HAL_NVIC_SetPriority(OTG_HS_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
  /* USER CODE BEGIN USB_OTG_HS_MspInit 1 */

  /* USER CODE END USB_OTG_HS_MspInit 1 */
}

void usb_hc_low_level_deinit(uint8_t busid)
{
  /* USER CODE BEGIN USB_OTG_HS_MspDeInit 0 */

  /* USER CODE END USB_OTG_HS_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USB_OTG_HS_CLK_DISABLE();
    __HAL_RCC_USBPHYC_CLK_DISABLE();

    /* USB_OTG_HS interrupt DeInit */
    HAL_NVIC_DisableIRQ(OTG_HS_IRQn);
  /* USER CODE BEGIN USB_OTG_HS_MspDeInit 1 */

  /* USER CODE END USB_OTG_HS_MspDeInit 1 */
}

/*����ΪNormal��cache������Ϊ write-back����������cache��
 *�ں��ʵ�ʱ��(��cache���Ծ����������ǿ�Ƹ���)�����ݸ��µ���Ӧ��SRAM�ռ�
 *�ر�ע�⣺���Ҫ�����������£�д֮��ҪSCB_CleanDCache��������ʱҪSCB_InvalidateDCache
 */
#define MPU_Normal_WB        0x00


/*����ΪNormal��cache������Ϊ write-back����������cache��
 *�ں��ʵ�ʱ��(��cache���Ծ����������ǿ�Ƹ���)�����ݸ��µ���Ӧ��SRAM�ռ�
 *�ر�ע�⣺���Ҫ�����������£�д֮��ҪSCB_CleanDCache��������ʱҪSCB_InvalidateDCache
 */
#define MPU_Normal_WBWARA    0x01  //�ⲿ���ڲ�д����д�����


/*����Ϊ normal��cache������Ϊ Write-through��������cache��ͬʱ��
 *������ͬʱд����Ӧ�������ַ�ռ�
 *�ر�ע�⣺���Ҫ�����������£�����ֱ�����ڴ�д���ݣ���������ʱҪSCB_InvalidateDCache
 */
#define MPU_Normal_WT         0x02


/*����Ϊ normal�����ù���,���û���
 */
#define MPU_Normal_NonCache   0x03


/*����Ϊ Device������������Ч�����ù���,���û���
 */
#define MPU_Device_NonCache   0x04

/**
  * @brief  ����MPU�������Ժʹ�С�Ĵ���ֵ
    * @param  Region            MPU��������ȡֵ��Χ��0��7��
    * @param  AccessPermission  ���ݷ���Ȩ�ޣ�ȡֵ��Χ��MPU_NO_ACCESS��MPU_PRIV_RO_URO��
    * @param  TypeExtField      �������� Cache ���ԣ�����ȡֵ��0��1��2����һ��ֻ��0��1
    *
  * @param  Address             MPU�����������ַ���ر�ע�����õ�Address��Ҫ��Size����
  * @param  Size                MPU���������С,����ȡֵ��MPU_1KB��MPU_4KB ...MPU_512MB��
    * @param  IsShareable       �����Ĵ洢�ռ��Ƿ���Թ���1=������0=��ֹ����
  * @param  IsCacheable         �����Ĵ洢�ռ��Ƿ���Ի��棬1=�����棬0=��ֹ���档
  * @param  IsBufferable        ʹ��Cache֮�󣬲�����write-through����write-back(bufferable)
    *                           1=�����壬����д��write-back����0=��ֹ���壬��ֱд��write-through����
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

    /* ����MPU */
    HAL_MPU_Disable();

    /* ����MPU����*/
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = Address;                      //�������ַ��
    MPU_InitStruct.Size = Size;                                //Ҫ���õ�����������С��
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;  //���ݷ���Ȩ�������������û�����Ȩģʽ�Ķ�/д����Ȩ�ޡ�
    MPU_InitStruct.IsBufferable = IsBufferable;                //�����ǿɻ���ģ���ʹ�û�д���档 �ɻ��浫���ɻ��������ʹ��ֱд���ԡ�WB
    MPU_InitStruct.IsCacheable = IsCacheable;                  //�����Ƿ�ɻ���ģ�����ֵ�Ƿ���Ա����ڻ����С�//M7 �ں�ֻҪ������ Cache��read allocate ���ǿ�����
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;     //�����Ƿ�����ڶ������������֮�乲��H7 ��Ӧ�ñʼǶ���������ǿ������������ͬ�ڹر� Cache
    MPU_InitStruct.Number = Number;                            //����š�
    MPU_InitStruct.TypeExtField = TypeExtField;                //������չ�ֶΣ������������ڴ�������͡�
    MPU_InitStruct.SubRegionDisable = 0x00;                    //����������ֶΡ�
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;//ָ����ʽ���λ��0=����ָ����ʣ�1=��ָֹ�����

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* ����MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT); //��ʾ��ֹ�˱������������κ�δʹ�� MPU �������������ڴ��쳣 MemFault


}

void cpu_mpu_config(uint8_t Region, uint8_t Mode, uint32_t Address, uint32_t Size)
{
    switch (Mode)
    {
    case MPU_Normal_WB:
        /*write back,no write allocate */
        /* �����ڴ�ΪNormal����,���ù���, ��дģʽ����д���ȡ����*/
        BSP_MPU_ConfigRegion(Region, MPU_TEX_LEVEL0, Address, Size, MPU_ACCESS_BUFFERABLE, MPU_ACCESS_CACHEABLE);
        break;

    case MPU_Normal_WBWARA:
        /*write back,write and read allocate */
        /* �����ڴ�ΪNormal����,���ù���, ��дģʽ��д���ȡ����*/
        BSP_MPU_ConfigRegion(Region, MPU_TEX_LEVEL1, Address, Size, MPU_ACCESS_BUFFERABLE, MPU_ACCESS_CACHEABLE);
        break;

    case MPU_Normal_WT:
        /*write through,no write allocate */
        /* �����ڴ�ΪNormal����,���ù���, ֱдģʽ*/
        BSP_MPU_ConfigRegion(Region, MPU_TEX_LEVEL0, Address, Size, MPU_ACCESS_NOT_BUFFERABLE, MPU_ACCESS_CACHEABLE);
        break;

    case MPU_Normal_NonCache:
        /* �����ڴ�ΪNormal����,���ù������û���ģʽ*/
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
  printf("Start usb host task...\r\n");
                  
  usbh_initialize(0, USB_OTG_HS_PERIPH_BASE);

  vTaskStartScheduler();
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
