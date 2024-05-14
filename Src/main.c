#include "main.h"
#include "uart.h"
#include "tim.h"
#include "adc.h"
#include "lcd.h"
#include "adc.h"
#include "lcd.h"
#include "EngTrModel.h"

/* Function prototypes */
/* Function prototypes */
void USER_RCC_Init(void);
void USER_GPIO_Init(void);
float map(float x, float in_min, float in_max, float out_min, float out_max);

int main(void) {
	uint8_t col = 16;

  	USER_RCC_Init();
	USER_GPIO_Init();
  	USER_USART1_Init();
	USER_TIM2_Init();
	USER_TIM3_Init();
	USER_TIM4_Init();
	USER_TIM5_Init();
	USER_TIM9_Init();
	USER_TIM10_Init();
	USER_TIM11_Init();

	LCD_Init();

	USER_ADC1_Init();
	USER_TIM2_Init();
	USER_TIM3_Init();
	USER_TIM4_Init();
	USER_TIM5_Init();
	USER_TIM9_Init();
	USER_TIM10_Init();
	USER_TIM11_Init();

	LCD_Init();

	USER_ADC1_Init();
	EngTrModel_initialize();

	// ROW 1 - 'A' | Right movement (not used for now)
	// ROW 2 - 'B' | Brake
	// ROW 3 - 'C' | Left movement (not used for now)


	// ROW 1 - 'A' | Right movement (not used for now)
	// ROW 2 - 'B' | Brake
	// ROW 3 - 'C' | Left movement (not used for now)

  for(;;)
	{

		GPIOA->ODR ^= ( 0x1UL << 5U );
		USER_TIM4_Delay();

		// Reads the value from the potentiometer
		uint16_t pot_value = USER_ADC1_Read();

		// Scales the potentiometer value to the range of acceleration (0 to 100)
		float acceleration = map(pot_value, 0, 4095, 0, 100);

		// Key 'B' is pressed and executes brake
		if ( !(ROW2_PIN) ) {
			
			USER_TIM2_Delay(); // 10ms delay for debounce

			if ( !(ROW2_PIN) ) {
				EngTrModel_U.Throttle = 0.0;
				EngTrModel_U.BrakeTorque = 100.0;
			}
		} else {
			EngTrModel_U.Throttle = acceleration;

		GPIOA->ODR ^= ( 0x1UL << 5U );
		USER_TIM4_Delay();

		// Reads the value from the potentiometer
		uint16_t pot_value = USER_ADC1_Read();

		// Scales the potentiometer value to the range of acceleration (0 to 100)
		float acceleration = map(pot_value, 0, 4095, 0, 100);

		// Key 'B' is pressed and executes brake
		if ( !(ROW2_PIN) ) {
			
			USER_TIM2_Delay(); // 10ms delay for debounce

			if ( !(ROW2_PIN) ) {
				EngTrModel_U.Throttle = 0.0;
				EngTrModel_U.BrakeTorque = 100.0;
			}
		} else {
			EngTrModel_U.Throttle = acceleration;
			EngTrModel_U.BrakeTorque = 0.0;
		}

		// calculate the model output values

		// calculate the model output values
		EngTrModel_step();

		// set the values in the msgs
		msg[2] = (uint8_t) EngTrModel_Y.EngineSpeed;
		msg[4] = (uint8_t) EngTrModel_Y.VehicleSpeed;
		msg[6] = (uint8_t) EngTrModel_Y.Gear;

		// printf("Vehicle Speed: %f\r\n", EngTrModel_Y.VehicleSpeed);
		// printf("Engine Speed: %f\r\n", EngTrModel_Y.EngineSpeed);
		// printf("Gear: %f\r\n", EngTrModel_Y.Gear);

		// set the values in the msgs
		msg[2] = (uint8_t) EngTrModel_Y.EngineSpeed;
		msg[4] = (uint8_t) EngTrModel_Y.VehicleSpeed;
		msg[6] = (uint8_t) EngTrModel_Y.Gear;

		// printf("Vehicle Speed: %f\r\n", EngTrModel_Y.VehicleSpeed);
		// printf("Engine Speed: %f\r\n", EngTrModel_Y.EngineSpeed);
		// printf("Gear: %f\r\n", EngTrModel_Y.Gear);
		}
	}
}

void USER_RCC_Init(void){
	
	/* System Clock (SYSCLK) configuration for 64 MHz */
	
	FLASH->ACR	&=	~( 0x5UL << 0U ); // two wait states latency, if SYSCLK > 48 MHz
	FLASH->ACR	|=	( 0x2UL << 0U ); // two wait states latency, if SYSCLK > 48 MHz
	RCC->CFGR	&=	~( 0x1UL << 16U ) // PLL HSI clock /2 selected as PLL input clock
				&	~( 0x7UL << 11U ) // APB2 prescaler /1
				&	~( 0x3UL << 8U ) // APB1 prescaler /2 (APB1 must not exceed 36 MHz)
				&	~( 0xFUL << 4U ); // AHB prescaler /1
	RCC->CFGR	|=	 ( 0xFUL << 18U ) // PLL input clock x 16 (PLLMUL bits)
				|	 ( 0x4UL << 8U ); // APB1 prescaler / 2
	RCC->CR		|=	 ( 0x1UL << 24U ); // PLL2 on
	while ( !(RCC->CR & ~( 0x1UL << 25U ))); // Wait until PLL is locked
	RCC->CFGR	&=	~( 0x1UL << 0U ); // PLL used as system clock (SW bits)
	RCC->CFGR	|=	 ( 0x2UL << 0U ); // PLL used as system clock (SW bits)
	while ( 0x8UL != ( RCC->CFGR & 0xCUL )); // Wait until PLL is switched

	RCC->CFGR	|=	 ( 0x3UL << 14U ); // PCLK2 divided by 8. PCLK2 is 64MHz and ADC clock is for 8 MHz
	// Clock for the ADC peripheral is configured. Must not exceed 14 MHz

	RCC->APB1ENR	|=	( 0x1UL << 0U ) // TIM2 clock enable
					| ( 0x1UL << 1U ) 	// TIM3 clock enable
					| ( 0x1UL << 2U ) 	// TIM4 clock enable
					| ( 0x1UL << 3U );	// TIM5 clock enable

	RCC->APB2ENR 	|= ( 0x1UL << 2U ) // IO Port A clock enable
					| ( 0x1UL << 14U ) // USART 1 clock enable
					| ( 0x1UL << 3U ) // IO Port B clock enable
					| ( 0x1UL << 4U ) // IO Port C clock enable
					| ( 0x1UL << 9U ) // Configured ADC1 clock is enabled
					| ( 0x1UL << 19U ) // TIM9 clock enable
					| ( 0x1UL << 20U ) // TIM10 clock enable
					| ( 0x1UL << 21U ); // TIM11 clock enable
	/* System Clock (SYSCLK) configuration for 64 MHz */
	
	FLASH->ACR	&=	~( 0x5UL << 0U ); // two wait states latency, if SYSCLK > 48 MHz
	FLASH->ACR	|=	( 0x2UL << 0U ); // two wait states latency, if SYSCLK > 48 MHz
	RCC->CFGR	&=	~( 0x1UL << 16U ) // PLL HSI clock /2 selected as PLL input clock
				&	~( 0x7UL << 11U ) // APB2 prescaler /1
				&	~( 0x3UL << 8U ) // APB1 prescaler /2 (APB1 must not exceed 36 MHz)
				&	~( 0xFUL << 4U ); // AHB prescaler /1
	RCC->CFGR	|=	 ( 0xFUL << 18U ) // PLL input clock x 16 (PLLMUL bits)
				|	 ( 0x4UL << 8U ); // APB1 prescaler / 2
	RCC->CR		|=	 ( 0x1UL << 24U ); // PLL2 on
	while ( !(RCC->CR & ~( 0x1UL << 25U ))); // Wait until PLL is locked
	RCC->CFGR	&=	~( 0x1UL << 0U ); // PLL used as system clock (SW bits)
	RCC->CFGR	|=	 ( 0x2UL << 0U ); // PLL used as system clock (SW bits)
	while ( 0x8UL != ( RCC->CFGR & 0xCUL )); // Wait until PLL is switched

	RCC->CFGR	|=	 ( 0x3UL << 14U ); // PCLK2 divided by 8. PCLK2 is 64MHz and ADC clock is for 8 MHz
	// Clock for the ADC peripheral is configured. Must not exceed 14 MHz

	RCC->APB1ENR	|=	( 0x1UL << 0U ) // TIM2 clock enable
					| ( 0x1UL << 1U ) 	// TIM3 clock enable
					| ( 0x1UL << 2U ) 	// TIM4 clock enable
					| ( 0x1UL << 3U );	// TIM5 clock enable

	RCC->APB2ENR 	|= ( 0x1UL << 2U ) // IO Port A clock enable
					| ( 0x1UL << 14U ) // USART 1 clock enable
					| ( 0x1UL << 3U ) // IO Port B clock enable
					| ( 0x1UL << 4U ) // IO Port C clock enable
					| ( 0x1UL << 9U ) // Configured ADC1 clock is enabled
					| ( 0x1UL << 19U ) // TIM9 clock enable
					| ( 0x1UL << 20U ) // TIM10 clock enable
					| ( 0x1UL << 21U ); // TIM11 clock enable
}

void USER_GPIO_Init(void){

	// TEST LED
	// clear bits to remove trash values
	GPIOA->CRL 	&= 	~( 0x3UL << 22U )
				& 	~( 0x2UL << 20U );

	// set bits to configure
	GPIOA->CRL	|= ( 0x1UL << 20U ); // output, 10mhz

	// 4x4 Keypad GPIOs

	// PB4: 'A' | Right movement
	GPIOB->CRL &=  ~( 0x3UL << 18U ) & ~( 0x3UL << 16U ); // Clear to CNF and MODE bits
	GPIOB->CRL |=	( 0x2UL << 18U ); // CNF of PB4: Input with pull-up/pull-down
	GPIOB->CRL |=	( 0x0UL << 16U ); // MODE of PB4: Input mode (reset state)
	GPIOB->ODR |=	( 0x1UL <<  4U); // ODR of PB4: Input pull-up
	
	// PB5: 'B' | Brake
	GPIOB->CRL &=  ~( 0x3UL << 22U ) & ~( 0x3UL << 20U ); // Clear to CNF and MODE bits
	GPIOB->CRL |=	( 0x2UL << 22U ); // CNF of PB5: Input with pull-up/pull-down
	GPIOB->CRL |=	( 0x0UL << 20U ); // MODE of PB5: Input mode (reset state)
	GPIOB->ODR |=	( 0x1UL <<  5U); // ODR of PB5: Input pull-up

	// PB3: 'C' | Left movement
	GPIOB->CRL &=  ~( 0x3UL << 14U ) & ~( 0x3UL << 12U ); // Clear to CNF and MODE bits
	GPIOB->CRL |=	( 0x2UL << 14U ); // CNF of PB3: Input with pull-up/pull-down
	GPIOB->CRL |=	( 0x0UL << 12U ); // MODE of PB3: Input mode (reset state)
	GPIOB->ODR |=	( 0x1UL <<  3U); // ODR of PB3: Input pull-up

	// // PA10: Column number for 'A', 'B' and 'C'
	// GPIOA->CRH &=  ~( 0x3UL << 10U ) & ~( 0x3UL << 8U ); // Clear to CNF and MODE bits
	// GPIOA->CRH |=	( 0x2UL << 10U ); // CNF of PA10: Input with pull-up/pull-down
	// GPIOA->CRH |=	( 0x0UL << 8U ); // MODE of PA10: Input mode (reset state)
	// GPIOA->ODR |=	( 0x1UL << 10U); // ODR of PA10: Input pull-up

	// PA0: Potenciometer Pin
	GPIOA->CRL |= 	( 0x0UL << 2U ); // CNF of PA0: Analog mode
	GPIOA->CRL |= 	( 0x0UL << 0U ); // MODE of PA0: Input mode (reset state)

}

// Function to map a value from one range to another
float map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// void USER_Delay ( void ) {
// 	__asm("	ldr r0, =2000 ");
// 	__asm("again: sub r0, r0, #1 ");
// 	__asm("	cmp r0, #0 ");
// 	__asm("	bne again ");
// }

// void USER_Debounce ( void ) {
// 	if ((!ROW1_PIN) | (!ROW2_PIN) | (!ROW3_PIN) | (!COL4_PIN)) {
// 		USER_Delay();
// 		if (!ROW1_PIN) {

// 		}
// 		else if (!ROW2_PIN) {

// 		}
// 		else if (!ROW3_PIN) {

// 		}
// 	}
// }
