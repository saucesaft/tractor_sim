#ifndef TIM_H_
#define TIM_H_

// config registers
#define TIM2_PSC	122UL
#define TIM2_CNT	495UL

// enable internal clock source
#define TIM2_SMCR_SMS   ( 0x7UL << 0U )

// edge aligned mode
#define TIM2_CR1_EA     ( 0x3UL << 5U )

// up counter (direction)
#define TIM2_CR1_UC     ( 0x1UL << 4U )

// uev enabled
#define TIM2_CR1_UEV    ( 0x1UL << 1U )

// enable the timer
#define TIM2_CR1_CEN	( 0x1UL << 0U )

// update interrupt flag
#define TIM2_SR_UIF		( 0x1UL << 0U )

void USER_TIM2_Init( void );

void USER_TIM2_Reset( void );

void USER_TIM2_Start( void );

void USER_TIM2_Delay( void );

#endif /* TIM_H_ */
