/* Includes ------------------------------------------------------------------*/
#include "waverecorder.h" 

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Current state of the audio recorder interface intialization */
static uint32_t AudioRecInited = 0;

int16_t RecBuf0[MIC_FILTER_RESULT_LENGTH]; //buffer for filtered PCM data from MIC
int16_t RecBuf1[MIC_FILTER_RESULT_LENGTH]; //buffer for filtered PCM data from MIC
uint8_t buffer_ready = 1;//number of buffer with fitered PCM data

volatile uint16_t Mic_DMA_PDM_Buffer0[INTERNAL_BUFF_SIZE];//buffer for RAW MIC data (filled by DMA)
volatile uint16_t Mic_DMA_PDM_Buffer1[INTERNAL_BUFF_SIZE];//buffer for RAW MIC data (filled by DMA)


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
