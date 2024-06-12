#ifndef TIM_H_
#define TIM_H_

///////////////
// msg state //
///////////////

// 0xff marks the start of a new message
// 0x80 signals velocity of the motor
// 0x40 signals speed of the vehicle
// 0x20 signals the gear

// config registers -> 10 ms
#define TIM2_PSC	9UL
#define TIM2_CNT	1536UL

// config registers -> 200 ms
#define TIM3_PSC	195UL
#define TIM3_CNT	230UL

// config registers -> 10 us
#define TIM4_PSC	0UL
#define TIM4_CNT    64896UL

// config registers -> 10 us
//#define TIM5_PSC	0UL
//#define TIM5_CNT    64896UL

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

// enable internal clock source
#define TIM3_SMCR_SMS   ( 0x7UL << 0U )

// edge aligned mode
#define TIM3_CR1_EA     ( 0x3UL << 5U )

// up counter (direction)
#define TIM3_CR1_UC     ( 0x1UL << 4U )

// uev enabled
#define TIM3_CR1_UEV    ( 0x1UL << 1U )

// enable the timer
#define TIM3_CR1_CEN	( 0x1UL << 0U )

// update interrupt flag
#define TIM3_SR_UIF		( 0x1UL << 0U )

// overflow interrupt enable
#define TIM3_DIER_UIE  ( 0x1UL << 0U )

// move interrupt to peripheral
#define NVIC_ISER_TIM3 ( 0x1UL << 29U )

void USER_TIM3_Init( void );

void USER_TIM3_Reset( void );

void USER_TIM3_Start( void );

void USER_TIM3_Delay( void );

void TIM3_IRQHandler( void );

// enable internal clock source
#define TIM4_SMCR_SMS   ( 0x7UL << 0U )

// edge aligned mode
#define TIM4_CR1_EA     ( 0x3UL << 5U )

// up counter (direction)
#define TIM4_CR1_UC     ( 0x1UL << 4U )

// uev enabled
#define TIM4_CR1_UEV    ( 0x1UL << 1U )

// enable the timer
#define TIM4_CR1_CEN	( 0x1UL << 0U )

// update interrupt flag
#define TIM4_SR_UIF		( 0x1UL << 0U )

void USER_TIM4_Init( void );

void USER_TIM4_Reset( void );

void USER_TIM4_Start( void );

void USER_TIM4_Delay( void );

#endif /* TIM_H_ */