# STM32F1XX_WS2812B

This is an example code of controlling an ws2812b led stripe, 
with 18 leds thus the used library is configured as 1 row with 18
cols. You can change row and col in the ws2812b header file.
You can connect up to 16 led stripes. Data is written in
parallel to the stripes from a GPIO Bank (GPIO A in this example)
This is why up to 16 stripes can be controlled in parallel.
A Timer is used in which 3 DMA transfer are triggered used to 
write data to the gpio's on which the stripes are connected to.
These 3 DMA transfers are triggered as following:
First trigger is on each period. It sets all gpios to high.
Second trigger is on the first capture compare event on the 8th
tick/pulse. The GPIOS are set accordingly if the bit for the
ws2812b shall be a 1 or a 0 in the output buffer "WS2812_Buffer". 
The third trigger is the second capture compare event an sets
all gpio's always to 0 through a dma transfer. It doesn't matter
if the pins are already set to 0 by the first capture compare
event.
Please read the ws2812b datasheet to understand the communication
protocol with the ws2812b led chips.
This example is programmed in the IAR Embedded Workbench IDE for
a stm32f103 and tested on the famous Blue-Pill. 
But you can use this library for any other IDE or stm32 
microcontroller. Just be sure to set the correct DMA
streams/channels, otherwise it won't work.
    
<html>
<body>

<h2>The ws2812b example</h2>
<img src="https://github.com/nicokorn/STM32F1XX_WS2812B/blob/main/docs/20210504_182221.jpg" alt="st32f1xx_ws2812b1">
<h2>WS2812B protocol timing and signal</h2>
<img src="https://github.com/nicokorn/STM32F1XX_WS2812B/blob/main/docs/WS2812B_Protocol_1.PNG" alt="st32f1xx_ws2812b2">
<h2>Use of peripherals to meet timing and signal requirements</h2>
<img src="https://github.com/nicokorn/STM32F1XX_WS2812B/blob/main/docs/WS2812B_Protocol_2.jpg" alt="st32f1xx_ws2812b3">

</body>
</html>
