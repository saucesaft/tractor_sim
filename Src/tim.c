#include <stdint.h>
#include "main.h"
#include "tim.h"

// Function that initializes TIM2 with constant values from tim.h
void USER_TIM2_Init( void ){
	TIM2->SMCR &= ~(TIM2_SMCR_SMS); 	// Enable the internal clock source
	TIM2->CR1 &= ~(TIM2_CR1_EA); 		// Set edge-aligned mode
	TIM2->CR1 &= ~(TIM2_CR1_UC); 		// Set up-counter mode
	TIM2->CR1 &= ~(TIM2_CR1_UEV); 		// Set UEV enabled

	USER_TIM2_Reset(); // Executes reset function

	USER_TIM2_Start(); // Executes start function
}

// Set/Reset the values needed to count 1s
void USER_TIM2_Reset( void ){
	TIM2->SR &= ~(TIM2_SR_UIF);			// Clear the timer update interrupt flag
	TIM2->CNT = TIM2_CNT;				// Set initial count
	TIM2->PSC = TIM2_PSC;				// Set prescaler
}

// Start the counting
void USER_TIM2_Start( void ){
	TIM2->CR1 |= TIM2_CR1_CEN;			// Start counting
}

// Wait until the timer overflows and stop the timer
void USER_TIM2_Delay( void ){
	while(!( TIM2->SR & TIM2_SR_UIF ));

	TIM2->CR1 &= ~(TIM2_CR1_CEN);		// Stop the timer
}
