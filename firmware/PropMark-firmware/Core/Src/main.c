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
#include "fatfs.h"
#include "memorymap.h"
#include "quadspi.h"
#include "sdmmc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "string.h"
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

/* USER CODE BEGIN PV */
volatile uint16_t DMABufferWS2812[NUM_OF_LED * 24];
volatile char TxBuffer[250];

const uint8_t COS256[256] = {
    255, 255, 255, 255, 254, 254, 254, 253, 253, 252, 251, 250, 249, 249, 248,
    246, 245, 244, 243, 241, 240, 238, 237, 235, 233, 232, 230, 228, 226, 224,
    222, 220, 218, 215, 213, 211, 208, 206, 203, 201, 198, 195, 193, 190, 187,
    185, 182, 179, 176, 173, 170, 167, 164, 161, 158, 155, 152, 149, 146, 143,
    140, 136, 133, 130, 127, 124, 121, 118, 114, 111, 108, 105, 102, 99, 96,
    93, 90, 87, 84, 81, 78, 75, 72, 69, 67, 64, 61, 59, 56, 53,
    51, 48, 46, 43, 41, 39, 36, 34, 32, 30, 28, 26, 24, 22, 21,
    19, 17, 16, 14, 13, 11, 10, 9, 8, 6, 5, 5, 4, 3, 2,
    1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 2, 3, 4, 5, 5, 6, 8, 9, 10, 11, 13, 14, 16,
    17, 19, 21, 22, 24, 26, 28, 30, 32, 34, 36, 39, 41, 43, 46,
    48, 51, 53, 56, 59, 61, 64, 67, 69, 72, 75, 78, 81, 84, 87,
    90, 93, 96, 99, 102, 105, 108, 111, 114, 118, 121, 124, 127, 130, 133,
    136, 140, 143, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173, 176, 179,
    182, 185, 187, 190, 193, 195, 198, 201, 203, 206, 208, 211, 213, 215, 218,
    220, 222, 224, 226, 228, 230, 232, 233, 235, 237, 238, 240, 241, 243, 244,
    245, 246, 248, 249, 249, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255,
    255};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void WsSet(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void WsSetAll(uint8_t r, uint8_t g, uint8_t b);

static void SDIO_SDCard_Test(void)
{
    FATFS FatFs;
    FIL Fil;
    FRESULT FR_Status;
    FATFS *FS_Ptr;
    UINT RWC, WWC; // Read/Write Word Counter
    DWORD FreeClusters;
    uint32_t TotalSize, FreeSpace;
    char RW_Buffer[200];

    do
    {
        //------------------[ Mount The SD Card ]--------------------
        FR_Status = f_mount(&FatFs, SDPath, 1);
        if (FR_Status != FR_OK)
        {
            sprintf(TxBuffer, "Error! While Mounting SD Card, Error Code: (%i)\r\n", FR_Status);
            HAL_UART_Transmit(&huart8, (uint8_t *)TxBuffer, strlen(TxBuffer), 100);
            break;
        }
        sprintf(TxBuffer, "SD Card Mounted Successfully! \r\n\n");
        HAL_UART_Transmit(&huart8, (uint8_t *)TxBuffer, strlen(TxBuffer), 100);
        //------------------[ Get & Print The SD Card Size & Free Space ]--------------------
        f_getfree("", &FreeClusters, &FS_Ptr);
        TotalSize = (uint32_t)((FS_Ptr->n_fatent - 2) * FS_Ptr->csize * 0.5);
        FreeSpace = (uint32_t)(FreeClusters * FS_Ptr->csize * 0.5);
        sprintf(TxBuffer, "Total SD Card Size: %lu Bytes\r\n", TotalSize);
        HAL_UART_Transmit(&huart8, (uint8_t *)TxBuffer, strlen(TxBuffer), 100);
        sprintf(TxBuffer, "Free SD Card Space: %lu Bytes\r\n\n", FreeSpace);
        HAL_UART_Transmit(&huart8, (uint8_t *)TxBuffer, strlen(TxBuffer), 100);
        //------------------[ Open A Text File For Write & Write Data ]--------------------
        // Open the file
        FR_Status = f_open(&Fil, "MyTextFile.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
        if (FR_Status != FR_OK)
        {
            sprintf(TxBuffer, "Error! While Creating/Opening A New Text File, Error Code: (%i)\r\n", FR_Status);
            HAL_UART_Transmit(&huart8, (uint8_t *)TxBuffer, strlen(TxBuffer), 100);
            break;
        }
        sprintf(TxBuffer, "Text File Created & Opened! Writing Data To The Text File..\r\n\n");
        HAL_UART_Transmit(&huart8, (uint8_t *)TxBuffer, strlen(TxBuffer), 100);
        // (1) Write Data To The Text File [ Using f_puts() Function ]
        f_puts("Hello! From STM32 To SD Card Over SDMMC, Using f_puts()\n", &Fil);
        // (2) Write Data To The Text File [ Using f_write() Function ]
        strcpy(RW_Buffer, "Hello! From STM32 To SD Card Over SDMMC, Using f_write()\r\n");
        f_write(&Fil, RW_Buffer, strlen(RW_Buffer), &WWC);
        // Close The File
        f_close(&Fil);
        //------------------[ Open A Text File For Read & Read Its Data ]--------------------
        // Open The File
        FR_Status = f_open(&Fil, "MyTextFile.txt", FA_READ);
        if (FR_Status != FR_OK)
        {
            sprintf(TxBuffer, "Error! While Opening (MyTextFile.txt) File For Read.. \r\n");
            HAL_UART_Transmit(&huart8, (uint8_t *)TxBuffer, strlen(TxBuffer), 100);
            break;
        }
        // (1) Read The Text File's Data [ Using f_gets() Function ]
        f_gets(RW_Buffer, sizeof(RW_Buffer), &Fil);
        sprintf(TxBuffer, "Data Read From (MyTextFile.txt) Using f_gets():%s", RW_Buffer);
        HAL_UART_Transmit(&huart8, (uint8_t *)TxBuffer, strlen(TxBuffer), 100);
        // (2) Read The Text File's Data [ Using f_read() Function ]
        f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);
        sprintf(TxBuffer, "Data Read From (MyTextFile.txt) Using f_read():%s", RW_Buffer);
        HAL_UART_Transmit(&huart8, (uint8_t *)TxBuffer, strlen(TxBuffer), 100);
        // Close The File
        f_close(&Fil);
        sprintf(TxBuffer, "File Closed! \r\n\n");
        HAL_UART_Transmit(&huart8, (uint8_t *)TxBuffer, strlen(TxBuffer), 100);
        //------------------[ Open An Existing Text File, Update Its Content, Read It Back ]--------------------
        // (1) Open The Existing File For Write (Update)
        FR_Status = f_open(&Fil, "MyTextFile.txt", FA_OPEN_EXISTING | FA_WRITE);
        FR_Status = f_lseek(&Fil, f_size(&Fil)); // Move The File Pointer To The EOF (End-Of-File)
        if (FR_Status != FR_OK)
        {
            sprintf(TxBuffer, "Error! While Opening (MyTextFile.txt) File For Update.. \r\n");
            HAL_UART_Transmit(&huart8, (uint8_t *)TxBuffer, strlen(TxBuffer), 100);
            break;
        }
        // (2) Write New Line of Text Data To The File
        FR_Status = f_puts("This New Line Was Added During File Update!\r\n", &Fil);
        f_close(&Fil);
        memset(RW_Buffer, '\0', sizeof(RW_Buffer)); // Clear The Buffer
        // (3) Read The Contents of The Text File After The Update
        FR_Status = f_open(&Fil, "MyTextFile.txt", FA_READ); // Open The File For Read
        f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);
        sprintf(TxBuffer, "Data Read From (MyTextFile.txt) After Update:\r\n%s", RW_Buffer);
        HAL_UART_Transmit(&huart8, (uint8_t *)TxBuffer, strlen(TxBuffer), 100);
        f_close(&Fil);
        //------------------[ Delete The Text File ]--------------------
        // Delete The File
        /*
        FR_Status = f_unlink(MyTextFile.txt);
        if (FR_Status != FR_OK){
            sprintf(TxBuffer, "Error! While Deleting The (MyTextFile.txt) File.. \r\n");
            USC_CDC_Print(TxBuffer);
        }
        */
    } while (0);
    //------------------[ Test Complete! Unmount The SD Card ]--------------------
    FR_Status = f_mount(NULL, "", 0);
    if (FR_Status != FR_OK)
    {
        sprintf(TxBuffer, "\r\nError! While Un-mounting SD Card, Error Code: (%i)\r\n", FR_Status);
        HAL_UART_Transmit(&huart8, (uint8_t *)TxBuffer, strlen(TxBuffer), 100);
    }
    else
    {
        sprintf(TxBuffer, "\r\nSD Card Un-mounted Successfully! \r\n");
        HAL_UART_Transmit(&huart8, (uint8_t *)TxBuffer, strlen(TxBuffer), 100);
    }
}
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
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM3_Init();
    MX_TIM15_Init();
    MX_UART8_Init();
    MX_QUADSPI_Init();
    MX_SDMMC1_SD_Init();
    MX_FATFS_Init();
    MX_TIM6_Init();
    /* USER CODE BEGIN 2 */

    // HAL_TIM_Base_Start_IT(&htim6);

    HAL_Delay(1000);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

    // arming sequence
    // __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 5000);
    // HAL_Delay(1000);
    // __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 2500);
    // HAL_Delay(1000);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    // HAL_Delay(1000);
    // SDIO_SDCard_Test();
    // for (int i = 2500; i <= 5000; i += 2)
    // {
    //     __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, i);
    //     HAL_Delay(1);
    // }
    // for (int i = 5000; i >= 2500; i -= 2)
    // {
    //     __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, i);
    //     HAL_Delay(1);
    // }

    uint8_t index = 0;

    while (1)
    {
        for (int i = 0; i < NUM_OF_LED - 1; i++)
        {
            WsSet(i, COS256[index] >> 4, COS256[(uint8_t)(index + (256 / 3))] >> 4, COS256[(uint8_t)(index - (256 / 3))] >> 4);
            index += 3;
            WsShow();
            HAL_Delay(50);
            WsSet(i, 0, 0, 0);
            WsShow();
        }
        for (int i = NUM_OF_LED - 1; i > 0; i--)
        {
            WsSet(i, COS256[index] >> 4, COS256[(uint8_t)(index + (256 / 3))] >> 4, COS256[(uint8_t)(index - (256 / 3))] >> 4);
            index += 3;
            WsShow();
            HAL_Delay(50);
            WsSet(i, 0, 0, 0);
            WsShow();
        }

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
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
    }

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 20;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
}

void WsSet(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    index *= 24;
    uint8_t i = 0, mask;
    for (mask = 0x80; mask > 0; i++, mask >>= 1)
    {
        if (g & mask)
            DMABufferWS2812[index + i] = WS2812_PULSE_1;
        else
            DMABufferWS2812[index + i] = WS2812_PULSE_0;
    }
    for (mask = 0x80; mask > 0; i++, mask >>= 1)
    {
        if (r & mask)
            DMABufferWS2812[index + i] = WS2812_PULSE_1;
        else
            DMABufferWS2812[index + i] = WS2812_PULSE_0;
    }
    for (mask = 0x80; mask > 0; i++, mask >>= 1)
    {
        if (b & mask)
            DMABufferWS2812[index + i] = WS2812_PULSE_1;
        else
            DMABufferWS2812[index + i] = WS2812_PULSE_0;
    }
}

void WsSetAll(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < NUM_OF_LED; i++)
    {
        WsSet(i, r, g, b);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // if (htim == &htim6)
    // {
    //     WsShow();
    // }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim3)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
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
