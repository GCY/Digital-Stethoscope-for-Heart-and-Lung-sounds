#include "define.h"

//uint32_t SAMPLING_RATE = 1000;
#define ADC_BUFFER_SIZE 1

const uint32_t SAMPLING_RATE = 16000;
uint32_t ADC3_value[ADC_BUFFER_SIZE] = {0};
bool  ADC3_ready = false;

//uint32_t adc_value[ADC_BUFFER_SIZE] = {0};
