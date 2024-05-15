#include <stdint.h>
#include "main.h"
#include "tim.h"
#include "lcd.h"
#include "uart.h"

/////////////
// TIMER 2 //
/////////////

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

/////////////
// TIMER 3 //
/////////////

// Function that initializes TIM3 with constant values from tim.h
void USER_TIM3_Init( void ){
	TIM3->SMCR &= ~(TIM3_SMCR_SMS); 	// Enable the internal clock source
	TIM3->CR1 &= ~(TIM3_CR1_EA); 		// Set edge-aligned mode
	TIM3->CR1 &= ~(TIM3_CR1_UC); 		// Set up-counter mode
	TIM3->CR1 &= ~(TIM3_CR1_UEV); 		// Set UEV enabled
	TIM3->DIER |= TIM3_DIER_UIE;		// Enable the update interrupt
	NVIC->ISER[0] |= NVIC_ISER_TIM3;	// Enable the TIM3 interrupt

	USER_TIM3_Reset(); // Executes reset function

	USER_TIM3_Start(); // Executes start function
}

// Set/Reset the values needed to count 1s
void USER_TIM3_Reset( void ){
	TIM3->SR &= ~(TIM3_SR_UIF);			// Clear the timer update interrupt flag
	TIM3->CNT = TIM3_CNT;				// Set initial count
	TIM3->PSC = TIM3_PSC;				// Set prescaler
}

// Start the counting
void USER_TIM3_Start( void ){
	TIM3->CR1 |= TIM3_CR1_CEN;			// Start counting
}

// Wait until the timer overflows and stop the timer
void USER_TIM3_Delay( void ){
	while(!( TIM3->SR & TIM3_SR_UIF ));

	TIM3->CR1 &= ~(TIM3_CR1_CEN);		// Stop the timer
}

/////////////
// TIMER 4 //
/////////////

// Function that initializes TIM4 with constant values from tim.h
void USER_TIM4_Init( void ){
	TIM4->SMCR &= ~(TIM4_SMCR_SMS); 	// Enable the internal clock source
	TIM4->CR1 &= ~(TIM4_CR1_EA); 		// Set edge-aligned mode
	TIM4->CR1 &= ~(TIM4_CR1_UC); 		// Set up-counter mode
	TIM4->CR1 &= ~(TIM4_CR1_UEV); 		// Set UEV enabled

	USER_TIM4_Reset(); // Executes reset function

	USER_TIM4_Start(); // Executes start function
}

// Set/Reset the values needed to count 1s
void USER_TIM4_Reset( void ){
	TIM4->SR &= ~(TIM4_SR_UIF);			// Clear the timer update interrupt flag
	TIM4->CNT = TIM4_CNT;				// Set initial count
	TIM4->PSC = TIM4_PSC;				// Set prescaler
}

// Start the counting
void USER_TIM4_Start( void ){
	TIM4->CR1 |= TIM4_CR1_CEN;			// Start counting
}

// Wait until the timer overflows and stop the timer
void USER_TIM4_Delay( void ){
	while(!( TIM4->SR & TIM4_SR_UIF ));

	TIM4->CR1 &= ~(TIM4_CR1_CEN);		// Stop the timer
}