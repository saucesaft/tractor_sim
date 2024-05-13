#include <stdint.h>
#include "main.h"
#include "tim.h"

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
	while(!( TIM2->SR & TIM3_SR_UIF ));

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

/////////////
// TIMER 5 //
/////////////

// Function that initializes TIM5 with constant values from tim.h
void USER_TIM5_Init( void ){
	TIM5->SMCR &= ~(TIM5_SMCR_SMS); 	// Enable the internal clock source
	TIM5->CR1 &= ~(TIM5_CR1_EA); 		// Set edge-aligned mode
	TIM5->CR1 &= ~(TIM5_CR1_UC); 		// Set up-counter mode
	TIM5->CR1 &= ~(TIM5_CR1_UEV); 		// Set UEV enabled

	USER_TIM5_Reset(); // Executes reset function

	USER_TIM5_Start(); // Executes start function
}

// Set/Reset the values needed to count
void USER_TIM5_Reset( void ){
	TIM5->SR &= ~(TIM5_SR_UIF);			// Clear the timer update interrupt flag
	TIM5->CNT = TIM5_CNT;				// Set initial count
	TIM5->PSC = TIM5_PSC;				// Set prescaler
}

// Start the counting
void USER_TIM5_Start( void ){
	TIM5->CR1 |= TIM5_CR1_CEN;			// Start counting
}

// Wait until the timer overflows and stop the timer
void USER_TIM5_Delay( void ){
	while(!( TIM2->SR & TIM5_SR_UIF ));

	TIM5->CR1 &= ~(TIM5_CR1_CEN);		// Stop the timer
}

/////////////
// TIMER 9 //
/////////////

// Function that initializes TIM9 with constant values from tim.h
void USER_TIM9_Init( void ){
	TIM9->SMCR &= ~(TIM9_SMCR_SMS); 	// Enable the internal clock source
	TIM9->CR1 &= ~(TIM9_CR1_UEV); 		// Set UEV enabled

	USER_TIM9_Reset(); // Executes reset function

	USER_TIM9_Start(); // Executes start function
}

// Set/Reset the values needed to count 1s
void USER_TIM9_Reset( void ){
	TIM9->SR &= ~(TIM9_SR_UIF);			// Clear the timer update interrupt flag
	TIM9->CNT = TIM9_CNT;				// Set initial count
	TIM9->PSC = TIM9_PSC;				// Set prescaler
}

// Start the counting
void USER_TIM9_Start( void ){
	TIM9->CR1 |= TIM9_CR1_CEN;			// Start counting
}

// Wait until the timer overflows and stop the timer
void USER_TIM9_Delay( void ){
	while(!( TIM9->SR & TIM9_SR_UIF ));

	TIM9->CR1 &= ~(TIM9_CR1_CEN);		// Stop the timer
}

/////////////
// TIMER 10 //
/////////////

// Function that initializes TIM10 with constant values from tim.h
void USER_TIM10_Init( void ){
	TIM10->CR1 &= ~(TIM10_CR1_UEV); 		// Set UEV enabled

	USER_TIM10_Reset(); // Executes reset function

	USER_TIM10_Start(); // Executes start function
}

// Set/Reset the values needed to count 1s
void USER_TIM10_Reset( void ){
	TIM10->SR &= ~(TIM10_SR_UIF);		// Clear the timer update interrupt flag
	TIM10->CNT = TIM10_CNT;				// Set initial count
	TIM10->PSC = TIM10_PSC;				// Set prescaler
}

// Start the counting
void USER_TIM10_Start( void ){
	TIM10->CR1 |= TIM10_CR1_CEN;			// Start counting
}

// Wait until the timer overflows and stop the timer
void USER_TIM10_Delay( void ){
	while(!( TIM10->SR & TIM10_SR_UIF ));

	TIM10->CR1 &= ~(TIM10_CR1_CEN);		// Stop the timer
}

/////////////
// TIMER 11 //
/////////////

// Function that initializes TIM11 with constant values from tim.h
void USER_TIM11_Init( void ){
	TIM11->CR1 &= ~(TIM11_CR1_UEV); 		// Set UEV enabled

	USER_TIM11_Reset(); // Executes reset function

	USER_TIM11_Start(); // Executes start function
}

// Set/Reset the values needed to count 1s
void USER_TIM11_Reset( void ){
	TIM11->SR &= ~(TIM11_SR_UIF);			// Clear the timer update interrupt flag
	TIM11->CNT = TIM11_CNT;				// Set initial count
	TIM11->PSC = TIM11_PSC;				// Set prescaler
}

// Start the counting
void USER_TIM11_Start( void ){
	TIM11->CR1 |= TIM11_CR1_CEN;			// Start counting
}

// Wait until the timer overflows and stop the timer
void USER_TIM11_Delay( void ){
	while(!( TIM11->SR & TIM11_SR_UIF ));

	TIM11->CR1 &= ~(TIM11_CR1_CEN);		// Stop the timer
}