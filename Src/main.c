// ****************************************************************************
/// \file      main.c
///
/// \brief     Main C Source File
///
/// \details   This is an example code of controlling an ws2812b led stripe, 
///            with 18 leds thus the used library is configured as 1 row with 18 
///            cols. You can change row and col in the ws2812b header file.
///            You can connect up to 16 led stripes. Data is written in
///            parallel to the stripes from a GPIO Bank (GPIO A in this example)
///            This is why up to 16 stripes can be controlled in parallel.
///            A Timer is used in which 3 DMA transfer are triggered used to 
///            write data to the gpio's on which the stripes are connected to.
///            This 3 DMA transfer are triggered as following:
///            First trigger is on each period. It set all gpios to high.
///            Second trigger is on the first capture compare event on the 8th
///            tick/pulse. The GPIOS are set accordingly if the bit for the
///            ws2812b shall be a 1 or a 0. 
///            The third trigger is the second capture compare event an sets
///            all gpio's always to 0 through a dma transfer. It doesn't matter
///            if the pins are already set to 0 by the first capture compare
///            event.
///            Please read the ws2812b datasheet to understand the communication
///            protocol with the ws2812b led chips.
///            This example is programmed in the IAR Embedded Workbench IDE for
///            a stm32f103 and tested on the famous Blue-Pill. 
///            But you can use this library for any other IDE or stm32 
///            microcontroller. Just be sure to set the correct DMA
///            streams/channels, otherwise it won't work.
///
/// \author    Nico Korn
///
/// \version   1.0.0.0
///
/// \date      24032021
/// 
/// \copyright Copyright (c) 2021 Nico Korn
/// 
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
///
/// \pre       
///
/// \bug       
///
/// \warning   
///
/// \todo      
///
// ****************************************************************************

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "ws2812b.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* Private user code ---------------------------------------------------------*/

// ----------------------------------------------------------------------------
/// \brief     Application entry point.
///
/// \param     none
///
/// \return    none
int main( void )
{
   uint32_t i=0;
  
   // Reset of all peripherals, Initializes the Flash interface and the Systick.
   HAL_Init();
   
   // Configure the system clock
   SystemClock_Config();
   
   // Initialize all configured peripherals
   MX_GPIO_Init();
   
   // init peripherals for using the ws2812b leds
   WS2812B_init();
   
   while (1)
   {
      WS2812B_clearBuffer();
      WS2812B_setPixel( 0, ++i%COL, rand()%0xFF, rand()%0xFF, rand()%0xFF );
      WS2812B_sendBuffer();
      HAL_Delay(10);
   }
}

// ----------------------------------------------------------------------------
/// \brief     System Clock configuration according to the blue pills crystals.
///            HSE is set to 8 MHz and LSE is set to 32 kHz.
///
/// \param     none
///
/// \return    none
void SystemClock_Config( void )
{
   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
   
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
}

// ----------------------------------------------------------------------------
/// \brief     Code piece automatically generated by the cube mx tool.
///
/// \param     none
///
/// \return    none
static void MX_GPIO_Init( void )
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

// ----------------------------------------------------------------------------
/// \brief     Code piece automatically generated by the cube mx tool.
///
/// \param     none
///
/// \return    none
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
