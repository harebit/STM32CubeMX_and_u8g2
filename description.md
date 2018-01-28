# Description

I presume you have an IDE, in this example I use SW4STM32 (Eclipse), STM32CubeMX and prepared a workspace, maybe with the name "oled".
  
Start STM32CubeMX, open a new Project, select your board/MCU, activate I2C1, choose the pins, the standard configuration for I2C is right (address =0, we add the address later).
Configure the Project Settings, give your project a name e.g. "laura", choose your IDE, here is SW4STM32 with a mark at "Generate Under Root" (it's a simpler structure, but only
 useful for one project in the workspace). Start "Generate Code" and open the project.  
 
Download u8g2 from [github.com/olikraus/u8g2](https://github.com/olikraus/u8g2), we need only the folder "csrc".  
import (_don't copy with the explorer_) to the folder "inc". Select "General"--"File System" and from "\u8g2-master\csrc", select "u8g2.h" and "u8x8.h" and finish this. 
In the same way import "u8x8_stm32_HAL.h" from this repository.  
Then import all c-files from "\u8g2-master\csrc" to the folder "src", next import "main.c" and "u8x8_stm32_HAL.c" from this repository.  

If you compile the project, you will get 2 Errors. "Flash overflowed" and "RAM overflowed", don't be worry!
The Flash overflowed because of too many fonts in the file "u8g2_fonts.c", delete a lot of fonts (a simpler way is import the file "u8g2_fonts.c" from this repository).  

The RAM overflowed because of to many definitions of memory in "u8g2.h".  
The names of the funktions are "u8g2_m_a_b_c" and means.
Example: OLed with 128x64 pixels  

* a = 128 segment / 8bit = 16 Byte  
* b = 64 column / 8bit = 8 pages  
* c = number of pages by one transfer ( 1 or 2 = 1 or 2 pages, f = full = all pages at once)  

In this example we need the function "u8g2_m_16_8_1", the rest can be deleted.  

I use a [Seeed Display](https://www.seeedstudio.com/Grove-OLED-Display-0.96%22-p-781.html) with I2C-bus, i think it works similar with other monochrom displays and spi-bus.
This display have a SSD1308 device, I always use driver for SSD1306, I didn't find a difference.

Compile the project and program your controller.  

The display should look like this. 

![Nucleo-SSD1308](https://github.com/harebit/cubemx_and_u8g2/blob/master/Nucleo-SSD1308.JPG?raw=true)


harebit
