#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_adc.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_rtc.h>
#include <stm32f4xx_tim.h>

#include "./FATFS/ff.h"

#include "define.h"

#include "I2C.h"
#include "SSD1306.h"

#include "FIR.h"
#include "adaptive_algorithm.h"

#include "adc.h"

#include "tiny_printf.h"

#include "./usb_cdc_device/usbd_audio_core.h"
#include "./usb_cdc_device/usbd_usr.h"
#include "./usb_cdc_device/usbd_desc.h"
#include "./usb_cdc_device/usbd_audio_core.h"
#include "./usb_cdc_device/waverecorder.h"

#define __FPU_PRESENT
#define __FPU_USED

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

const uint32_t SECOND = 1000000;

const uint16_t PWM_Freq = 1000;

volatile uint32_t TimingDelay;

volatile uint32_t micros = 20000000;

uint32_t last_update;
uint32_t last_screen_update;

volatile uint8_t button_flag = 0;


const unsigned int taps = 19;
//100hz 0.7Hz~8.9Hz
float coeff1[19] = {-0.043047,-0.048389,-0.042549,-0.023677,0.0073069,0.046611,0.088179,0.12489,0.15008,0.15905,0.15008,0.12489,0.088179,0.046611,0.0073069,-0.023677,-0.042549,-0.048389,-0.043047};
// 100Hz 0.5Hz low pass
float coeff2[19] = {0.050186,0.050989,0.051705,0.052331,0.052864,0.053303,0.053646,0.053892,0.05404,0.054089,0.05404,0.053892,0.053646,0.053303,0.052864,0.052331,0.051705,0.050989,0.050186};
float buffer1[19];
unsigned offset1;
float buffer2[19];
unsigned offset2;

FIRInfo info1;
FIRInfo info2;

void Delay(__IO uint32_t nTime)
{
   TimingDelay = nTime;
   while(TimingDelay){
   }
}

void SysTick_Handler(void)
{
   if(TimingDelay){
      --TimingDelay;
   }
   ++micros;
}

void Init_LED()
{
   //Enable the GPIOD Clock
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);


   // GPIOD Configuration
   GPIO_InitTypeDef GPIO_InitStruct;
   GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
   GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

   GPIO_SetBits(GPIOB, GPIO_Pin_8);
   GPIO_SetBits(GPIOB, GPIO_Pin_9);
   GPIO_Init(GPIOB, &GPIO_InitStruct);   
}

void Init_AnalogSwitch()
{
   //Enable the GPIOD Clock
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);


   // GPIOD Configuration
   GPIO_InitTypeDef GPIO_InitStruct;
   GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
   GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

   GPIO_SetBits(GPIOB, GPIO_Pin_5);
   GPIO_Init(GPIOB, &GPIO_InitStruct);   



   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);


   // GPIOD Configuration
   GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
   GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

   GPIO_ResetBits(GPIOC, GPIO_Pin_13);
   GPIO_Init(GPIOC, &GPIO_InitStruct);   

}


void EXTILine6_Config(void)
{
   EXTI_InitTypeDef   EXTI_InitStructure;
   GPIO_InitTypeDef   GPIO_InitStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;

   /* Enable GPIOB clock */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
   /* Enable SYSCFG clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

   /* Configure PB0 pin as input floating */
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   /* Connect EXTI Line0 to PB0 pin */
   SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource6);

   /* Configure EXTI Line0 */
   EXTI_InitStructure.EXTI_Line = EXTI_Line6;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);

   /* Enable and set EXTI Line0 Interrupt to the lowest priority */
   NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}

void EXTILine7_Config(void)
{
   EXTI_InitTypeDef   EXTI_InitStructure;
   GPIO_InitTypeDef   GPIO_InitStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;

   /* Enable GPIOB clock */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
   /* Enable SYSCFG clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

   /* Configure PB0 pin as input floating */
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   /* Connect EXTI Line0 to PB0 pin */
   SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource7);

   /* Configure EXTI Line0 */
   EXTI_InitStructure.EXTI_Line = EXTI_Line7;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);

   /* Enable and set EXTI Line0 Interrupt to the lowest priority */
   NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}

void Init_Peripheral()
{
   USBD_Init(&USB_OTG_dev,      
	 USB_OTG_FS_CORE_ID,
	 &USR_desc, 
	 &AUDIO_cb, 
	 &USR_cb);
   Delay(10000);

   Init_LED();
   EXTILine6_Config();
   EXTILine7_Config();
   Delay(10000);

   Init_AnalogSwitch();
   Delay(10000);

   Init_I2C3();
   Init_SSD1306();
   Delay(10000);

   //Init_ADC(); //ADC3 Pooling
   ADC_4_Channel_Init(); // DMA + TIMER + FIFO, 1000Hz sampling rate
   Delay(10000);

   /* if external clock is work */
   __IO uint32_t HSEStatus = 0;
   HSEStatus = RCC->CR & RCC_CR_HSERDY;
   if(HSEStatus){
      GPIO_ResetBits(GPIOB, GPIO_Pin_8);
   }
   else{
      GPIO_SetBits(GPIOB, GPIO_Pin_8);
   }     
}

////////////////
long map(long x, long in_min, long in_max, long out_min, long out_max){
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int main(void)
{
   if(SysTick_Config(SystemCoreClock / 1000 / 1000)){
      while(1){}
   }

   Init_Peripheral();

   SSD1306_Fill(0x00);
   SSD1306_UpdateScreen();
   Delay(SECOND);
   SSD1306_Fill(0xFF);
   SSD1306_UpdateScreen();
   Delay(SECOND);
   SSD1306_Fill(0x00);
   SSD1306_UpdateScreen();  
   Delay(SECOND);   

   SSD1306_GotoXY(3, 4);
   SSD1306_Puts("Stethoscope", &Font_11x18, 0xFF);
   SSD1306_GotoXY(3, 25);
   SSD1306_Puts("20200515", &Font_11x18, 0xFF);
   SSD1306_GotoXY(3, 45);
   SSD1306_Puts("TonyGUO", &Font_11x18, 0xFF);
   SSD1306_UpdateScreen();  
   Delay(SECOND);
   SSD1306_Fill(0x00);
   SSD1306_UpdateScreen();

   SSD1306_GotoXY(3, 4);
   SSD1306_Puts("Lung Sound", &Font_11x18, 0xFF);
   SSD1306_GotoXY(3, 25);
   SSD1306_Puts("72Hz~3000Hz", &Font_11x18, 0xFF);
   SSD1306_GotoXY(3, 45);
   SSD1306_Puts(".wav file", &Font_11x18, 0xFF);
   SSD1306_UpdateScreen();     



   last_update = micros;
   last_screen_update = micros;

   FATFS fs;
   FIL fnew;	
   FRESULT res_sd; 
   UINT fnum;   

   res_sd = f_mount(&fs,"0:",1);
   if(res_sd == FR_NO_FILESYSTEM){
      res_sd=f_mkfs("0:",0,0);							

      if(res_sd == FR_OK){
	 res_sd = f_mount(NULL,"0:",1);					
	 res_sd = f_mount(&fs,"0:",1);
      }
      else{
	 GPIO_ResetBits(GPIOB, GPIO_Pin_9);
      }
   }

   res_sd = f_open(&fnew, "0:FatFs 20200515.csv",FA_CREATE_ALWAYS | FA_WRITE );
   BYTE WriteBuffer[255] = {0};
   sprintf(WriteBuffer,"%d,%d,%d\r\n",micros,12345,54321);
   res_sd=f_write(&fnew,WriteBuffer,strlen(WriteBuffer),&fnum);

   f_close(&fnew); 


   SSD1306_Fill(0x00);
   SSD1306_UpdateScreen();  
   Delay(SECOND); 

   while(1){
      /*
	 if(button_flag == 0){
	 SSD1306_Fill(0xFF);
	 SSD1306_UpdateScreen();
	 Delay(SECOND);
	 SSD1306_Fill(0x00);
	 SSD1306_UpdateScreen();  
	 Delay(SECOND); 
	 }      
	 else if(button_flag == 6){
	 SSD1306_Fill(0x00);
	 SSD1306_UpdateScreen(); 
	 SSD1306_GotoXY(3, 25);
	 SSD1306_Puts("PC6", &Font_11x18, 0xFF);
	 SSD1306_UpdateScreen();  

	 }
	 else if(button_flag == 7){
	 SSD1306_Fill(0x00);
	 SSD1306_UpdateScreen(); 
	 SSD1306_GotoXY(3, 25);
	 SSD1306_Puts("PC7", &Font_11x18, 0xFF);
	 SSD1306_UpdateScreen();  

	 }
	 */
   }

   return(0); // System will implode
}    

void EXTI9_5_IRQHandler(void)
{
   button_flag = 6;
   if(EXTI_GetITStatus(EXTI_Line6) != RESET){
      button_flag = 6;
      EXTI_ClearITPendingBit(EXTI_Line6);
      EXTI_ClearFlag(EXTI_Line6);    
   } 
   if(EXTI_GetITStatus(EXTI_Line7) != RESET){
      button_flag = 7;
      EXTI_ClearITPendingBit(EXTI_Line7);
      EXTI_ClearFlag(EXTI_Line7);    
   }

   EXTI_ClearITPendingBit(EXTI9_5_IRQn);

}
