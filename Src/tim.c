#include <stdint.h>
#include "main.h"
#include "tim.h"
#include "lcd.h"
#include "uart.h"
#include <math.h>

/////////////
// TIMER 2 //
/////////////

//Definiciones necesarias
#define SYSCLK             64000000
#define T_HCLK             ( 1.0 / SYSCLK )
#define TIM_TIME_1S        1.0
#define TIM_PRESC_1S       ( ceil( TIM_TIME_1S / ( T_HCLK * (( 65535 + 1) - 0 ))) - 1 )
#define TIM_INIT_COUNT_1S  (( 65535 + 1 ) - ( round( TIM_TIME_1S / ( T_HCLK * ( TIM_PRESC_1S + 1 )))))

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
void USER_TIM2_Delay( uint16_t prescaler, uint16_t count ){
	TIM2->SMCR &=  ~( 0x7UL <<  0U );//        select internal clock
    TIM2->CR1  &=  ~( 0x3UL <<  5U )//         edge-aligned mode
               &   ~( 0x1UL <<  4U )//         upcounter
               &   ~( 0x1UL <<  1U );//        update event (UEV) enabled
    TIM2->PSC   =    prescaler;//              prescaler
    TIM2->EGR  |=   ( 0x1UL <<  0U );//        update the prescaler
    TIM2->CNT   =    count;//                  initial count
    TIM2->SR   &=  ~( 0x1UL <<  0U );//        clear TIM overflow-event flag
    TIM2->CR1  |=   ( 0x1UL <<  0U );//        timer enabled
    while( !(TIM2->SR & ( 0x1UL <<  0U )) );// wait until overflows
    TIM2->CR1  &=  ~( 0x1UL <<  0U );//        timer disabled
}

/////////////
// TIMER 3 //
/////////////

// Function that initializes TIM3 with constant values from tim.h
void USER_TIM3_Init( void ){
    TIM3->SMCR &=  ~( 0x7UL <<  0U );//   select internal clock
    TIM3->CR1  &=  ~( 0x3UL <<  5U )//    edge-aligned mode
               &   ~( 0x1UL <<  4U )//    upcounter
               &   ~( 0x1UL <<  1U );//   update event (UEV) enabled
    TIM3->PSC   =    1;//              time range: 30.5us to 2s
    TIM3->EGR  |=   ( 0x1UL <<  0U );//   update the prescaler
    TIM3->CNT   =    0;//                 clear count
    TIM3->SR   &=  ~( 0x1UL <<  0U );//   clear TIM overflow-event flag
    TIM3->CR1  |=   ( 0x1UL <<  0U );//   timer enabled
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