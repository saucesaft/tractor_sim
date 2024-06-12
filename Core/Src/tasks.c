#include "tasks.h"

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "adc.h"
#include "tim.h"
#include "uart.h"
#include "lcd.h"
#include "EngTrModel.h"

// #include <stdio.h>

uint8_t msg[10] = {0xaa, 0xbb, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t msg_length = ( sizeof(msg) / sizeof(msg[0]) ); // we substract 1 because we don't send the last byte (the direction/brake)

osThreadId Task1Exec;
osThreadId Task2Exec;
osThreadId Task3Exec;
osThreadId Task4Exec;
osThreadId Task5Exec;
osThreadId TaskSetup;

// int _write (int file, char *ptr, int len) {
// 	int idx;
// 	for (idx = 0; idx < len; idx++) {
// 		while( !(USART2->SR & USART_SR_TXE) );
// 		USART2->DR = *ptr++;
// 	}
// }

//// msg to send ////
// [bytes]:
// [0] 0xff marks the start of a new message
// [1] 0xff marks the start of a new message
// [2] 0xff marks the start of a new message
// [3] lsb of engine speed
// [4] msb of engine speed
// [5] lsb of vehicle speed
// [6] msb of vehicle speed
// [7] lsb of gear
// [8] msb of gear
// [9] 0x00 signals the action to take (0: nothing, 1: right, 2: left, 3: brake)
/////////////////////

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
}

void defineTasks(void) {
  /* USER CODE BEGIN RTOS_THREADS */

	osThreadDef(TaskSetup, Setup, osPriorityRealtime, 1, 256);
	TaskSetup = osThreadCreate(osThread(TaskSetup), NULL);

	osThreadDef(CreateExec, TaskCreateData2_Execute, osPriorityNormal, 1, 256);
	Task2Exec = osThreadCreate(osThread(CreateExec), NULL);

	osThreadDef(ReadExec, TaskRead1_Execute, osPriorityNormal, 1, 256);
	Task1Exec = osThreadCreate(osThread(ReadExec), NULL);

	osThreadDef(LCDExec, TaskLCD3_Execute, osPriorityNormal, 1, 256);
	Task3Exec = osThreadCreate(osThread(LCDExec), NULL);

	osThreadDef(SerialExec, TaskSerial4_Execute, osPriorityNormal, 1, 256);
	Task4Exec = osThreadCreate(osThread(SerialExec), NULL);

}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Setup( void const * argument ) {
	USER_RCC_Init();
	USER_GPIO_Init();

    USER_ADC1_Init();
    USER_TIM2_Init();

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

	EngTrModel_initialize();

	USER_TIM4_Init();
    LCD_Init(); // Initialization of LCD display

	USER_USART1_Init();

    // Serial Pins Configuration
	GPIOA->CRH	&=	~( 0x1UL <<  6U )
				&	~( 0x2UL <<  4U );

	GPIOA->CRH	|=	 ( 0x2UL <<  6U )
				|	 ( 0x1UL <<  4U );

	osThreadTerminate(TaskSetup);
}

/*
	Reads the value from the potentiometer and the keypad
*/
void TaskRead1_Execute( void const * argument ) {

	for(;;) {
    	// Reads the value from the potentiometer
		uint16_t pot_value = USER_ADC1_Read();
        
		// Scales the potentiometer value to the range of acceleration (0 to 100)
		float acceleration = map(pot_value, 50, 4095, 1, 100);

        // Key 'B' is pressed and executes brake
		if ( !(ROW2_PIN) ) {

			for (int i = 0; i < 1000; i++) {
				USER_TIM4_Delay(); // fake 10ms delay for debounce
			}

			if ( !(ROW2_PIN) ) {
				EngTrModel_U.Throttle = 2.0;
				EngTrModel_U.BrakeTorque = 100.0;
				msg[10] = (uint8_t) 3;
			}
		} else if ( !(ROW1_PIN) ) { // derecha

			for (int i = 0; i < 1000; i++) {
				USER_TIM4_Delay(); // fake 10ms delay for debounce
			}

			if ( !(ROW1_PIN) ) {
				EngTrModel_U.Throttle = acceleration * 0.95;
				EngTrModel_U.BrakeTorque = 0.0;
				msg[10] = (uint8_t) 1;
			}

		} else if ( !(ROW3_PIN) ) { // izquierda

			for (int i = 0; i < 1000; i++) {
				USER_TIM4_Delay(); // fake 10ms delay for debounce
			}

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

	// osThreadYield();
	osDelay(500);
	}	
}

void TaskCreateData2_Execute( void const * argument ){
	for(;;) {
		EngTrModel_step();

		uint16_t e_speed = (uint16_t) EngTrModel_Y.EngineSpeed;
		uint16_t v_speed = (uint16_t) EngTrModel_Y.VehicleSpeed;
		uint8_t gear = (uint8_t) EngTrModel_Y.Gear;

		msg[2] = (uint8_t) e_speed & 0xff;
		msg[3] = (uint8_t) (e_speed >> 8);


		msg[5] = (uint8_t) v_speed & 0xff;
		msg[6] = (uint8_t) (v_speed >> 8);

		msg[8] = (uint8_t) gear & 0xff;
		msg[9] = (uint8_t) (gear >> 8);

		// osDelay(50);
	}
}

void TaskLCD3_Execute( void const * argument ){
	for(;;) {
		LCD_Clear();
		LCD_Set_Cursor(1, 1);

		GPIOA->ODR ^= ( 0x1UL << 5U );					// Toggle the LED

		USER_LCD_Send_Message(msg, msg_length);			// Show the message

		osDelay(200);
	}
}

void TaskSerial4_Execute( void const * argument ){
	for(;;) {
		USER_UART_Send_Message(msg, msg_length);		// Send the message
		osDelay(200);
	}
}