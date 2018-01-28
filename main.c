/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "u8g2.h"
#include "u8x8_stm32_HAL.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static u8g2_t u8g2; // a structure which will contain all the data for one display
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#define hare1_width 25
#define hare1_height 28
static unsigned char hare1_bits[] = {
   0x00, 0x08, 0x00, 0x00, 0x00, 0x77, 0x00, 0x00, 0x80, 0x80, 0x01, 0x00,
   0x00, 0x01, 0x02, 0x00, 0x00, 0x0e, 0x0c, 0x00, 0x00, 0x01, 0x30, 0x00,
   0x80, 0x00, 0x40, 0x00, 0x80, 0x00, 0x88, 0x00, 0x40, 0x1c, 0x1c, 0x01,
   0x80, 0x23, 0x1c, 0x01, 0x00, 0x20, 0x08, 0x01, 0x00, 0x1c, 0x80, 0x00,
   0x00, 0x03, 0x40, 0x00, 0x80, 0x00, 0x20, 0x00, 0x60, 0x00, 0x20, 0x00,
   0x10, 0x00, 0x20, 0x00, 0x08, 0x00, 0x40, 0x00, 0x04, 0x00, 0x40, 0x00,
   0x02, 0x80, 0x87, 0x00, 0x01, 0x40, 0x88, 0x00, 0x01, 0x20, 0x48, 0x00,
   0x03, 0x20, 0x30, 0x00, 0x1c, 0xc0, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00,
   0x40, 0x00, 0x01, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x67, 0x00, 0x00,
   0x00, 0x18, 0x00, 0x00 };

#define hare2_width 30
#define hare2_height 28
static unsigned char hare2_bits[] = {
   0x00, 0xc0, 0x03, 0x00, 0x00, 0x20, 0x04, 0x00, 0x00, 0x20, 0x08, 0x00,
   0x00, 0x40, 0x10, 0x00, 0x00, 0x80, 0x20, 0x00, 0x00, 0x70, 0x40, 0x00,
   0x00, 0x08, 0x80, 0x01, 0x00, 0x04, 0x00, 0x02, 0x00, 0xf8, 0x00, 0x04,
   0x00, 0x00, 0x81, 0x08, 0x00, 0x00, 0xc1, 0x11, 0x00, 0xfc, 0xc0, 0x11,
   0x00, 0x03, 0x80, 0x10, 0x80, 0x00, 0x00, 0x08, 0x70, 0x00, 0x00, 0x04,
   0x08, 0x00, 0x00, 0x03, 0x06, 0x00, 0xc0, 0x00, 0x01, 0x00, 0x20, 0x00,
   0x01, 0x00, 0x10, 0x00, 0x01, 0x00, 0x10, 0x00, 0x06, 0x00, 0x20, 0x00,
   0x08, 0x00, 0x40, 0x00, 0x08, 0xf0, 0x41, 0x00, 0x08, 0x08, 0x46, 0x00,
   0x08, 0x04, 0x48, 0x00, 0x08, 0x08, 0x30, 0x00, 0x30, 0x10, 0x00, 0x00,
   0xc0, 0x0f, 0x00, 0x00 };

/*
  Function which responds for drawing
  */
void u8g2_prepare()
{
  u8g2_SetFont(&u8g2, u8g2_font_courB10_tr);
  u8g2_SetFontRefHeightExtendedText(&u8g2);
  u8g2_SetDrawColor(&u8g2, 1);
  u8g2_SetFontPosTop(&u8g2);
  u8g2_SetFontDirection(&u8g2, 0);
}

void u8g2_drawLogo(void)
{
    u8g2_SetFontMode(&u8g2,1);	// Transparent
    u8g2_SetDrawColor(&u8g2,1);

   u8g2_SetFontDirection(&u8g2, 0);
   u8g2_SetFont(&u8g2, u8g2_font_inb24_mf);
   u8g2_DrawStr(&u8g2, 0, 5, "U");

   u8g2_SetFontDirection(&u8g2, 1);
   u8g2_SetFont(&u8g2, u8g2_font_inb30_mn);
   u8g2_DrawStr(&u8g2, 51,8,"8");

   u8g2_SetFontDirection(&u8g2, 0);
   u8g2_SetFont(&u8g2, u8g2_font_inb24_mf);
   u8g2_DrawStr(&u8g2, 51,5,"g");
   u8g2_DrawStr(&u8g2, 67,5,"\xb2");

   u8g2_DrawHLine(&u8g2, 2, 35, 47);
   u8g2_DrawHLine(&u8g2, 3, 36, 47);
   u8g2_DrawVLine(&u8g2, 45, 32, 12);
   u8g2_DrawVLine(&u8g2, 46, 33, 12);

   u8g2_SetFont(&u8g2, u8g2_font_5x8_tr);

   u8g2_DrawStr(&u8g2, 1,54,"github.com/olikraus/u8g2");
}

void u8g2_box_frame(uint8_t a)
{
  u8g2_DrawStr(&u8g2, 0,0, "DrawBox");
  u8g2_DrawBox(&u8g2,0,15,20,10);
  u8g2_DrawBox(&u8g2,0+a,15,30,7);
  u8g2_DrawStr(&u8g2, 0, 32, "DrawFrame");
  u8g2_DrawFrame(&u8g2,0,15+32,20,10);
  u8g2_DrawFrame(&u8g2,0+a,15+32,30,7);
}

void u8g2_disc_circle(uint8_t a)
{
  u8g2_DrawStr(&u8g2, 0, 0, "DrawDisc");
  u8g2_DrawDisc(&u8g2,10,22,7,U8G2_DRAW_ALL);
  u8g2_DrawDisc(&u8g2,24+a,20,5,U8G2_DRAW_ALL);
  u8g2_DrawStr(&u8g2, 0, 32, "DrawCircle");
  u8g2_DrawCircle(&u8g2,10,22+32,7,U8G2_DRAW_ALL);
  u8g2_DrawCircle(&u8g2,24+a,20+32,5,U8G2_DRAW_ALL);
}

void u8g2_string(uint8_t a)
{
  u8g2_SetFontDirection(&u8g2,0);
  u8g2_DrawStr(&u8g2,70+a,31, " 0");
  u8g2_SetFontDirection(&u8g2,1);
  u8g2_DrawStr(&u8g2,68,0+a, " 90");
  u8g2_SetFontDirection(&u8g2,2);
  u8g2_DrawStr(&u8g2,70-a,31, " 180");
  u8g2_SetFontDirection(&u8g2,3);
  u8g2_DrawStr(&u8g2,72,64-a, " 270");
}

void u8g2_line(uint8_t a)
{
  u8g2_DrawStr(&u8g2, 0, 0, "DrawLine");
  u8g2_DrawLine(&u8g2,7+a, 15, 60-a, 60);
  u8g2_DrawLine(&u8g2,7+a*2, 15, 80-a, 60);
  u8g2_DrawLine(&u8g2,7+a*3, 15, 100-a, 60);
  u8g2_DrawLine(&u8g2,7+a*4, 15, 120-a, 60);
}

void u8g2_hare(uint8_t a)
{
  u8g2_DrawStr(&u8g2, 0, 0,"DrawXBM");
  if((a&1) == 0)
	  {
	  	  u8g2_DrawXBM(&u8g2, 0+a, 30, hare1_width, hare1_height, hare1_bits);
	  }
  else
  	  {
	 	u8g2_DrawXBM(&u8g2, 0+a, 30, hare2_width, hare2_height, hare2_bits);
  	  }
}

uint8_t max = 0;
uint8_t step = 0;
uint8_t picture =0;

void draw(void)
{
	u8g2_prepare();
  switch(picture) {
  	case 0:
  		u8g2_drawLogo();

  	    break;
    case 1:
    	max = 150;
    	u8g2_box_frame(step);
    	break;
    case 2:
    	max = 150;
    	u8g2_disc_circle(step);
    	break;
    case 3:
    	max = 60;
    	u8g2_string(step);
    	break;
    case 4:
    	max = 50;
    	u8g2_line(step);
    	break;
    case 5:
    	max = 120;
    	u8g2_hase(step);
    	break;
 }
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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

  /* USER CODE BEGIN 2 */
  //device ssd1306 = ssd1308
  u8g2_Setup_ssd1306_i2c_128x64_noname_1(&u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_stm32_gpio_and_delay_cb); // init u8g2 structure
  u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
  u8g2_SetPowerSave(&u8g2, 0); // wake up display

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  //this loop correspond of drawing


	         u8g2_FirstPage(&u8g2);
	             do
	             {
	                draw();
	             }
	             while ( u8g2_NextPage(&u8g2) ); // 8 times running

	             	 if (step <= max)
	             		 step += 3;
	             	 else
	             		 {
	             		 step = 0;
	             		 picture++;
	                     	   if ( picture >= 6)
	                   		   picture = 0;
	             		 }
  }
}
  /* USER CODE END 3 */


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
