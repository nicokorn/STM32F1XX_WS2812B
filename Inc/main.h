// ****************************************************************************
/// \file      main.hh
///
/// \brief     Main C Header File
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
