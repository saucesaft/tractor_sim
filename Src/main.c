#include "main.h"
#include "uart.h"
#include "tim.h"
#include "adc.h"
#include "lcd.h"
#include "EngTrModel.h"

/* Function prototypes */
void USER_RCC_Init(void);
void USER_GPIO_Init(void);
float map(float x, float in_min, float in_max, float out_min, float out_max);

//// msg to send ////
// [bytes]:
// [0] 0xff marks the start of a new message
// [1] 0x80 signals velocity of the motor
// [2] lsb of value
// [3] msb of value
// [4] 0x40 signals speed of the vehicle
// [5] lsb of value
// [6] msb of value
// [7] 0x20 signals the gear
// [8] lsb of value
// [9] msb of value
// (not send to serial) [10] 0x00 signals the action to take (0: nothing, 1: right, 2: left, 3: brake)
/////////////////////

uint8_t msg[11] = {0xff, 0x80, 0x00, 0x00, 0x40, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00};
uint8_t msg_length = ( sizeof(msg) / sizeof(msg[0]) ) - 1; // we substract 1 because we don't send the last byte (the direction/brake)

/* 
	Timer 3 Interrupt Handler

	This function will run every 200 ms, it is in charge
	of sending the message through the UART and showing
	the message in the LCD.

	We also toggle the LED in the board to show that the
	interrupt is running.
 */
void TIM3_IRQHandler( void ){
	if ( TIM3->SR & TIM3_SR_UIF ){
		LCD_Clear();
		LCD_Set_Cursor(1, 1);

		GPIOA->ODR ^= ( 0x1UL << 5U );					// Toggle the LED

		USER_UART_Send_Message(msg, msg_length);		// Send the message

		USER_LCD_Send_Message(msg, msg_length);			// Show the message

		USER_TIM3_Reset();								// Reset the timer
	}
}

/*
	Main function

	This function initializes the peripherals and runs the main loop
	of the program. It reads the value from the potentiometer and
	scales it to the range of acceleration (0 to 100). It also reads
	the state of the keypad and sets the throttle and brake values
	accordingly.
*/
int main(void) {

	// Initialize the peripherals
  	USER_RCC_Init();
	USER_GPIO_Init();
  	USER_USART1_Init();
	USER_TIM2_Init();
	USER_TIM4_Init();
	USER_TIM3_Init();
	LCD_Init();
	USER_ADC1_Init();

	// Initialize the model
	EngTrModel_initialize();


  for(;;)
	{
		// Reads the value from the potentiometer
		uint16_t pot_value = USER_ADC1_Read();

		// Scales the potentiometer value to the range of acceleration (0 to 100)
		float acceleration = map(pot_value, 50, 4095, 1, 100);

		// Key 'B' is pressed and executes brake
		if ( !(ROW2_PIN) ) {
			
			USER_TIM2_Delay(); // 10ms delay for debounce

			if ( !(ROW2_PIN) ) {
				EngTrModel_U.Throttle = 2.0;
				EngTrModel_U.BrakeTorque = 100.0;
				msg[10] = (uint8_t) 3;
			}
		} else if ( !(ROW1_PIN) ) { // derecha

			USER_TIM2_Delay(); // 10ms delay for debounce

			if ( !(ROW1_PIN) ) {

				EngTrModel_U.Throttle = acceleration * 0.95;
				EngTrModel_U.BrakeTorque = 0.0;
				msg[10] = (uint8_t) 1;
			}

		} else if ( !(ROW3_PIN) ) { // izquierda

			USER_TIM2_Delay(); // 10ms delay for debounce

			if ( !(ROW3_PIN) ) {


				EngTrModel_U.Throttle = acceleration * 0.95;
				EngTrModel_U.BrakeTorque = 0.0;
				msg[10] = (uint8_t) 2;
			}

		} else {
			EngTrModel_U.Throttle = acceleration;
			EngTrModel_U.BrakeTorque = 0.0;
			msg[10] = (uint8_t) 0;
		}

		// Calculate the model output values
		EngTrModel_step();

		// set the values in the msgs
		// placeholders to send later (we also remove decimals to send them)
		// TODO send decimals to the serial as well
		uint16_t e_speed = (uint16_t) EngTrModel_Y.EngineSpeed;
		uint16_t v_speed = (uint16_t) EngTrModel_Y.VehicleSpeed;
		uint8_t gear = (uint8_t) EngTrModel_Y.Gear;

		msg[2] = (uint8_t) e_speed & 0xff;
		msg[3] = (uint8_t) (e_speed >> 8);


		msg[5] = (uint8_t) v_speed & 0xff;
		msg[6] = (uint8_t) (v_speed >> 8);

		msg[8] = (uint8_t) gear & 0xff;
		msg[9] = (uint8_t) (gear >> 8);
	}
}

void USER_RCC_Init(void){
	
	/* System Clock (SYSCLK) configuration for 64 MHz */
	FLASH->ACR	&=	~( 0x5UL << 0U ); // two wait states latency, if SYSCLK > 48 MHz
	FLASH->ACR	|=	 ( 0x2UL << 0U ); // two wait states latency, if SYSCLK > 48 MHz
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

	/* Enable clocks on the peripherals used */
	RCC->APB1ENR	|= ( 0x1UL << 0U )  // TIM2 clock enable
					|  ( 0x1UL << 1U ) 	// TIM3 clock enable
					|  ( 0x1UL << 2U ); // TIM4 clock enable

	RCC->APB2ENR 	|= ( 0x1UL << 2U )  // IO Port A clock enable
					| ( 0x1UL << 14U )  // USART 1 clock enable
					| ( 0x1UL << 3U )   // IO Port B clock enable
					| ( 0x1UL << 4U )   // IO Port C clock enable
					| ( 0x1UL << 9U );  // Configured ADC1 clock is enabled
}

void USER_GPIO_Init(void){

	// Interrupt LED GPIO
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

	// PB6: 'C' | Left movement
	GPIOB->CRL &=  ~( 0x3UL << 26U ) & ~( 0x3UL << 24U ); // Clear to CNF and MODE bits
	GPIOB->CRL |=	( 0x2UL << 26U ); // CNF of PB3: Input with pull-up/pull-down
	GPIOB->CRL |=	( 0x0UL << 24U ); // MODE of PB3: Input mode (reset state)
	GPIOB->ODR |=	( 0x1UL <<  6U); // ODR of PB3: Input pull-up

	// PA0: Potenciometer Pin
	GPIOA->CRL &= 	~( 0x1UL << 2U ); // CNF of PA0: Analog mode
	GPIOA->CRL &= 	~( 0x1UL << 0U ); // MODE of PA0: Input mode (reset state)

	GPIOA->CRH	&=	~( 0x1UL <<  6U )
				&	~( 0x2UL <<  4U );

	GPIOA->CRH	|=	 ( 0x2UL <<  6U )
				|	 ( 0x1UL <<  4U );

}

/*
	Function to map a value from one range to another
*/
float map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
