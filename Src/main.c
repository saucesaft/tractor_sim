#include "main.h"
#include "uart.h"
#include "tim.h"
#include "adc.h"
#include "EngTrModel.h"

#define USER_B1		( GPIOC->IDR & ( 0x1UL << 13U ))

/* Function prototypes */
void USER_RCC_Init(void);
void USER_GPIO_Init(void);
float map(float x, float in_min, float in_max, float out_min, float out_max);

int main(void)
{
  	USER_RCC_Init();
	USER_GPIO_Init();
  	USER_USART1_Init();
	USER_ADC1_Init();
	EngTrModel_initialize();
	
  for(;;)
	{	
		// Reads the value from the potentiometer
		uint16_t pot_value = USER_ADC1_Read();

		// Scales the potentiometer value to the range of acceleration (0 to 100)
		float acceleration = map(pot_value, 0, 4095, 0, 100);

		// Checks for any key presses in column 4
		// PA10 is input pull up - normally 1. When pressed, it is 0
		if (!COL4_PIN) {
			// Debounce and checks which key is pressed
			USER_TIM2_Delay(); // 10ms delay for debounce
			if (!COL4_PIN) { // If the key is still pressed after debounce
				if (!ROW1_PIN) {
					// Key 'A' is pressed and executes right movement
					EngTrModel_U.Throttle = 1.45 * acceleration / 100.0;
					EngTrModel_U.BrakeTorque = 0.0;
				}
				else if (!ROW2_PIN) {
					// Key 'B' is pressed and executes brake
					EngTrModel_U.Throttle = 0.0;
					EngTrModel_U.BrakeTorque = 100.0 * acceleration / 100.0;
				}
				else if (!ROW3_PIN) {
					// Key 'C' is pressed and executes left movement
					EngTrModel_U.Throttle = 0.0;
					EngTrModel_U.BrakeTorque = 0.0;
				}
			}
		}
		else {
			EngTrModel_U.Throttle = 50.0 * acceleration / 100.0;
			EngTrModel_U.BrakeTorque = 0.0;
		}

		EngTrModel_step();
		printf("Vehicle Speed: %f\r\n", EngTrModel_Y.VehicleSpeed);
		printf("Engine Speed: %f\r\n", EngTrModel_Y.EngineSpeed);
		printf("Gear: %f\r\n", EngTrModel_Y.Gear);
		USER_TIM3_Delay();//  200ms
	}
}

void USER_RCC_Init(void){
	
	/* System Clock (SYSCLK) configuration for 64 MHz */
	
	FLASH->ACR	&=	~( 0x5UL << 0U ); // two wait states latency, if SYSCLK > 48 MHz
	FLASH->ACR	|=	( 0x2UL << 0U ); // two wait states latency, if SYSCLK > 48 MHz
	RCC->CFGR	&=	~( 0x1UL << 16U ); // PLL HSI clock /2 selected as PLL input clock
				&	~( 0x7UL << 11U ); // APB2 prescaler /1
				&	~( 0x3UL << 8U ); // APB1 prescaler /2 (APB1 must not exceed 36 MHz)
				&	~( 0xFUL << 4U ); // AHB prescaler /1
	RCC->CFGR	|=	 ( 0xFUL << 18U ); // PLL input clock x 16 (PLLMUL bits)
				|	 ( 0x4UL << 8U ); // APB1 prescaler / 2
	RCC->CR		|=	 ( 0x1UL << 24U ); // PLL2 on
	while ( !(RCC->CR & ~( 0x1UL << 25U ))); // Wait until PLL is locked
	RCC->CFGR	&=	~( 0x1UL << 0U ); // PLL used as system clock (SW bits)
	RCC->CFGR	|=	 ( 0x2UL << 0U ); // PLL used as system clock (SW bits)
	while ( 0x8UL != ( RCC->CFGR & 0xCUL )); // Wait until PLL is switched

	RCC->CFGR	|=	 ( 0x3UL << 14U ); // PCLK2 divided by 8. PCLK2 is 64MHz and ADC clock is for 8 MHz
	// Clock for the ADC peripheral is configured

	RCC->APB2ENR |= ( 0x1UL << 2U ) // IO Port A clock enable
	| ( 0x1UL << 14U ); // USART 1 clock enable
	| ( 0x1UL << 3U ); // IO Port B clock enable
	| ( 0x1UL << 4U ); // IO Port C clock enable
	| ( 0x1UL << 9U ); // Configured ADC1 clock is enabled
}

void USER_GPIO_Init(void){

	// 4x4 Keypad GPIOs

	// PB4: 'A' | Right movement
	GPIOB->CRL &=  ~( 0x3UL << 18U ) & ~( 0x3UL << 16U ); // Clear to CNF and MODE bits
	GPIOB->CRL |=	( 0x2UL << 18U ); // CNF of PB4: Input with pull-up/pull-down
	GPIOB->CRL |=	( 0x0UL << 16U ); // MODE of PB4: Input mode (reset state)
	
	// PB5: 'B' | Brake
	GPIOB->CRL &=  ~( 0x3UL << 22U ) & ~( 0x3UL << 20U ); // Clear to CNF and MODE bits
	GPIOB->CRL |=	( 0x2UL << 22U ); // CNF of PB5: Input with pull-up/pull-down
	GPIOB->CRL |=	( 0x0UL << 20U ); // MODE of PB5: Input mode (reset state)

	// PB3: 'C' | Left movement
	GPIOB->CRL &=  ~( 0x3UL << 14U ) & ~( 0x3UL << 12U ); // Clear to CNF and MODE bits
	GPIOB->CRL |=	( 0x2UL << 14U ); // CNF of PB3: Input with pull-up/pull-down
	GPIOB->CRL |=	( 0x0UL << 12U ); // MODE of PB3: Input mode (reset state)

	// PA10: Column number for 'A', 'B' and 'C'
	GPIOA->CRH &=  ~( 0x3UL << 10U ) & ~( 0x3UL << 8U ); // Clear to CNF and MODE bits
	GPIOA->CRH |=	( 0x2UL << 10U ); // CNF of PA10: Input with pull-up/pull-down
	GPIOA->CRH |=	( 0x0UL << 8U ); // MODE of PA10: Input mode (reset state)

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