#include <stdint.h>
#include "main.h"
#include "adc.h"

void USER_ADC1_Init( void ) {
    // Configure ADC1
    ADC1->CR1 |= ADC1_CR1_DUALMOD; // Select independent operation mode
    ADC1->CR2 |= ADC1_CR2_ALIGN; // Right alignment
    ADC1->CR2 |= ADC1_CR2_CONT; // Continuous conversion mode

    // Configure sample time for ADC channel 0
    ADC1->SMPR2 |= ADC1_SMPR2_SMP0

    // Configure sequence and/or number of conversions for ADC regular channels
    ADC1->SQR1 |= ADC1_SQR1_L; // 1 conversion for regular channels
    ADC1->SQR3 |= ADC1_SQR3_SQ1; // Channel for the first ADC conversion | PAO
    // Since we will read from PA0, we don't need to include a POT_ADC_PIN in main.c

    // Enable ADC module
    ADC1->CR2 |= ADC1_CR2_ADON;

    // Calibrate ADC module
    ADC1->CR2 |= ADC1_CR2_CAL;
    while ( ADC1->CR2 & ADC1_CR2_CAL ); // Wait for calibration to complete
}

uint16_t USER_ADC1_Read( void ) {
    // Start ADC conversion
    ADC1->CR2 |= ADC1_CR2_ADON;

    // Wait for conversion to complete
    while (!(ADC->SR & ( 0x1UL << 1U ))); // Wait until EOC (End of Conversion Flag) is set

    // Read the converted value
    return ADC1->DR;
}