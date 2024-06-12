#ifndef ADC_H_
#define ADC_H_

#include <stdio.h>

// select independent operation mode for ADC module
#define ADC1_CR1_DUALMOD   ( 0x0UL << 16U )

// right alignment
#define ADC1_CR2_ALIGN     ( 0x0UL << 11U )

// continuous conversion mode
#define ADC1_CR2_CONT     ( 0x1UL << 1U )

// sample time for ADC channel (below is for 1.5 cycles)
#define ADC1_SMPR2_SMP0    ( 0x0UL << 1U )

// sequence and/or number of conversions for ADC regular channels
// 1 conversion for regular channels
#define ADC1_SQR1_L ( 0x0UL << 20U )

// channel for the first ADC conversion | PA0
#define ADC1_SQR3_SQ1   ( 0x0UL << 0U )

// ADC module enable
#define ADC1_CR2_ADON   ( 0x1UL << 0U )

// ADC module calibration start | the same hardware resets the bit after calibration is complete
#define ADC1_CR2_CAL    ( 0x1UL << 2U )

// End of conversion bit
#define ADC1_SR_EOC	( 0x1UL << 1U )

void USER_ADC1_Init( void );
uint16_t USER_ADC1_Read ( void );

#endif /* ADC_H_ */