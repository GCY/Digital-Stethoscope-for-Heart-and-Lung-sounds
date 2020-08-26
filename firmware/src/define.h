#ifndef __DEFINE__
#define __DEFINE__

#include <stdint.h>
#include <stdbool.h>

#define ADC_BUFFER_SIZE 1

typedef struct
{
   float value;
   int32_t index;
}SignalPoint;

extern const uint32_t SAMPLING_RATE;
extern uint32_t ADC3_value[ADC_BUFFER_SIZE];
extern bool  ADC3_ready;
//extern uint32_t adc_value[ADC_BUFFER_SIZE];

#endif
