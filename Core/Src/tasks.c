#include "stm32f1xx_hal.h"

void TaskRead1_Init( void ){
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

  osThreadTerminate(Task1Init);
}

/*
	Reads the value from the potentiometer and the keypad
*/
void TaskRead1_Execute( void ) {
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
		
}

void TaskCreateData2_Init( void ){
    EngTrModel_initialize();
    osThreadTerminate(Task2Init);
}

void TaskCreateData2_Execute( void ){
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
}

void TaskLCD3_Init( void ){
    USER_TIM4_Init();
    LCD_Init(); // Initialization of LCD display
    osThreadTerminate(Task3Init);
}

void TaskLCD3_Execute( void ){
		LCD_Clear();
		LCD_Set_Cursor(1, 1);

		GPIOA->ODR ^= ( 0x1UL << 5U );					// Toggle the LED

		USER_LCD_Send_Message(msg, msg_length);			// Show the message
}

void TaskSerial4_Init( void ){

    // USART communication initialization
    USER_USART1_Init();

    // Serial Pins Configuration
	GPIOA->CRH	&=	~( 0x1UL <<  6U )
				&	~( 0x2UL <<  4U );

	GPIOA->CRH	|=	 ( 0x2UL <<  6U )
				|	 ( 0x1UL <<  4U );

  osThreadTerminate(Task4Init);
}

void TaskSerial4_Execute(){
	USER_UART_Send_Message(msg, msg_length);		// Send the message
}