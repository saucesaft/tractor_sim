#include "tasks.h"

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "adc.h"
#include "tim.h"
#include "uart.h"
#include "lcd.h"
#include "EngTrModel.h"

#include <stdbool.h>

bool toggle = false;
bool ran = false;

bool read_serial_running = false;
bool read_sensors_running = false;

uint8_t msg[12] = {0xaa, 0xbb, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t msg_length = ( sizeof(msg) / sizeof(msg[0]) );

osThreadId Task1Exec;
osThreadId Task2Exec;
osThreadId Task3Exec;
osThreadId Task4Exec;
osThreadId Task5Exec;
osThreadId Task6Exec;
osThreadId TaskSetup;

// mutex to control access to msg variable
osMutexId msg_mutex;

// queue to send data between tasks
osMessageQId change_queue;

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
	// clear bits to remove trash values
	GPIOA->CRL 	&= 	~( 0x3UL << 22U )
				& 	~( 0x2UL << 20U );

	// set bits to configure
	GPIOA->CRL	|= ( 0x1UL << 20U ); // output, 10mhz
}

void defineTasks(void) {
	osMessageQDef(q1, 4, uint32_t);
	change_queue = osMessageCreate(osMessageQ(q1), NULL);

	osMutexDef(msgLock);
	msg_mutex = osMutexCreate( osMutex(msgLock) );

	osThreadDef(TaskSetup, Setup, osPriorityRealtime, 1, 256);
	TaskSetup = osThreadCreate(osThread(TaskSetup), NULL);

	osThreadDef(CreateExec, TaskCreateData2_Execute, osPriorityNormal, 1, 256);
	Task2Exec = osThreadCreate(osThread(CreateExec), NULL);

	osThreadDef(LCDExec, TaskLCD3_Execute, osPriorityAboveNormal, 1, 256);
	Task3Exec = osThreadCreate(osThread(LCDExec), NULL);

	osThreadDef(SerialExec, TaskSerial4_Execute, osPriorityAboveNormal, 1, 256);
	Task4Exec = osThreadCreate(osThread(SerialExec), NULL);

	osThreadDef(ModeExec, change_mode, osPriorityNormal, 1, 256);
	Task6Exec = osThreadCreate(osThread(ModeExec), NULL);
}

void change_mode( void const * argument ) {
	
	// PB8
	GPIOB->CRH &=  ~( 0x3UL << 2U ) & ~( 0x3UL << 0U ); // Clear to CNF and MODE bits
	GPIOB->CRH |=	( 0x2UL << 2U ); // CNF of PB4: Input with pull-up/pull-down
	GPIOB->CRH |=	( 0x0UL << 0U ); // MODE of PB4: Input mode (reset state)
	GPIOB->ODR |=	( 0x1UL <<  8U); // ODR of PB4: Input pull-up

	for(;;) {

		if ( !( GPIOB->IDR & ( 0x1UL << 8U )) && ran ) {
			
			GPIOA->ODR ^= ( 0x1UL << 5U );

			if (read_sensors_running) {
				osThreadTerminate(Task1Exec);
			}
			read_sensors_running = false;
			read_serial_running = true;

			uint16_t msg_quantity = osMessageWaiting( change_queue );
			for ( int i = 0; i < msg_quantity; i++ ) {
				osMessageGet(change_queue, osWaitForever);
			}

			osThreadDef(ReadExec, serial_read, osPriorityNormal, 1, 256);
			Task5Exec = osThreadCreate(osThread(ReadExec), NULL);

			ran = false;
		
		} else if ( ( GPIOB->IDR & ( 0x1UL << 8U )) && !ran ) {

			GPIOA->ODR ^= ( 0x1UL << 5U );

			if (read_serial_running) {
				osThreadTerminate(Task5Exec);
			}
			read_serial_running = false;
			read_sensors_running = true;

			uint16_t msg_quantity = osMessageWaiting( change_queue );
			for ( int i = 0; i < msg_quantity; i++ ) {
				osMessageGet(change_queue, osWaitForever);
			}

			osThreadDef(SensorsExec, read_sensors, osPriorityNormal, 1, 256);
			Task1Exec = osThreadCreate(osThread(SensorsExec), NULL);

			ran = true;

		}
	}

	osDelay(50);
}

/*
	reads the value from the raspberry pi's serial TX and queues up the values
*/
void serial_read( void const * argument ) {
	for(;;) {
		uint8_t pot_value = USER_USART1_Read_8bit();

		// scales the potentiometer value to the range of acceleration (0 to 100)
		queue_data data = { .acceleration = pot_value, .brake = 0x0, .direction = 0x0, };

		data.src = 0x0;

		while( !( USART1->SR & USART_SR_RXNE )) {
			osMessagePut(change_queue, (uint32_t)&data, osWaitForever);

			if ( !(ROW2_PIN) ) {
				for (int i = 0; i < 1000; i++) {
					USER_TIM4_Delay(); // fake 10ms delay for debounce
				}

				if ( !(ROW2_PIN) ) {
					// data.acceleration = 2.0;
					data.brake = 100.0;
					data.direction = 3;
				}

			} else if ( !(ROW1_PIN) ) { // derecha
				for (int i = 0; i < 1000; i++) {
					USER_TIM4_Delay(); // fake 10ms delay for debounce
				}

				if ( !(ROW1_PIN) ) {
					// data.acceleration = data.acceleration * 0.95;
					// data.brake = 0.0;
					data.direction = 1;
				}

			} else if ( !(ROW3_PIN) ) { // izquierda
				for (int i = 0; i < 1000; i++) {
					USER_TIM4_Delay(); // fake 10ms delay for debounce
				}

				if ( !(ROW3_PIN) ) {
					// data.acceleration = data.acceleration * 0.95;
					// data.brake = 0.0;
					data.direction = 2;
				}

			} else {
				// data.acceleration = data.acceleration;
				// data.brake = 0.0;
				data.direction = 0;
			}
		}

		osDelay(50);
	}
}

/*
	reads the value from the potentiometer and the keypad and queues up the values
*/
void read_sensors( void const * argument ) {

	for(;;) {
    	// reads the value from the potentiometer
		uint16_t pot_value = USER_ADC1_Read();
        
		// scales the potentiometer value to the range of acceleration (0 to 100)
		float acceleration = map(pot_value, 50, 4095, 1, 100);

		queue_data send_data = { .acceleration = 0x42, .brake = 0x42, .direction = 0x42, };

        // Key 'B' is pressed and executes brake
		if ( !(ROW2_PIN) ) {

			for (int i = 0; i < 1000; i++) {
				USER_TIM4_Delay(); // fake 10ms delay for debounce
			}

			if ( !(ROW2_PIN) ) {
				send_data.acceleration = 2.0;
				send_data.brake = 100.0;
				send_data.direction = 3;
			}
		} else if ( !(ROW1_PIN) ) { // derecha

			for (int i = 0; i < 1000; i++) {
				USER_TIM4_Delay(); // fake 10ms delay for debounce
			}

			if ( !(ROW1_PIN) ) {
				send_data.acceleration = acceleration * 0.95;
				send_data.brake = 0.0;
				send_data.direction = 1;
			}

		} else if ( !(ROW3_PIN) ) { // izquierda

			for (int i = 0; i < 1000; i++) {
				USER_TIM4_Delay(); // fake 10ms delay for debounce
			}

			if ( !(ROW3_PIN) ) {
				send_data.acceleration = acceleration * 0.95;
				send_data.brake = 0.0;
				send_data.direction = 2;
			}

		} else {
			send_data.acceleration = acceleration;
			send_data.brake = 0.0;
			send_data.direction = 0;
		}

		send_data.src = 0x1;
d
		osMessagePut(change_queue, (uint32_t)&send_data, osWaitForever);

		osDelay(50);
	}	
}

void TaskCreateData2_Execute( void const * argument ){

	// PA6 LED //
	// clear bits to remove trash values
	GPIOA->CRL 	&= 	~( 0x3UL << 26U )
				& 	~( 0x2UL << 24U );

	// set bits to configure
	GPIOA->CRL	|= ( 0x1UL << 24U ); // output, 10mhz

	// PA7 LED //
	// clear bits to remove trash values
	GPIOA->CRL 	&= 	~( 0x3UL << 30U )
				& 	~( 0x2UL << 28U );

	// set bits to configure
	GPIOA->CRL	|= ( 0x1UL << 28U ); // output, 10mhz

	// PA8 LED //
	// clear bits to remove trash values
	GPIOA->CRH 	&= 	~( 0x3UL << 2U )
				& 	~( 0x2UL << 0U );

	// set bits to configure
	GPIOA->CRH	|= ( 0x1UL << 0U ); // output, 10mhz

	for(;;) {

		osEvent evt = osMessageGet(change_queue, osWaitForever);
		if (evt.status == osEventMessage) {
			queue_data* data = (queue_data*)evt.value.v;
			// Now you can access the fields of the struct
			EngTrModel_U.Throttle = data->acceleration;
			EngTrModel_U.BrakeTorque = data->brake;

			msg[9] = data->direction;

			msg[10] = (uint8_t) data->acceleration & 0xFF;
			msg[11] = (uint8_t) (data->acceleration >> 8);
		}

		EngTrModel_step();

		// lock mutex (dont modify msg while it is being sent)
		osMutexWait(msg_mutex, osWaitForever);

		uint16_t e_speed = (uint16_t) EngTrModel_Y.EngineSpeed;
		uint16_t v_speed = (uint16_t) EngTrModel_Y.VehicleSpeed;
		uint8_t gear = (uint8_t) EngTrModel_Y.Gear;

		msg[3] = (uint8_t) e_speed & 0xff;
		msg[4] = (uint8_t) (e_speed >> 8);


		msg[5] = (uint8_t) v_speed & 0xff;
		msg[6] = (uint8_t) (v_speed >> 8);

		msg[7] = (uint8_t) gear & 0xff;
		msg[8] = (uint8_t) (gear >> 8);

		

		// release mutex
		osMutexRelease(msg_mutex);
	}
}

void TaskLCD3_Execute( void const * argument ){
	for(;;) {
		LCD_Clear();
		LCD_Set_Cursor(1, 1);

		// lock mutex (dont send msg while it is being modified)
		osMutexWait(msg_mutex, osWaitForever);

		// show the message on the LCD
		USER_LCD_Send_Message(msg, msg_length);

		// show direction on LEDs
		if (msg[9] == 3) {
			GPIOA->ODR |= ( 0x1UL << 6U );
			GPIOA->ODR &= ~( 0x1UL << 7U );
			GPIOA->ODR &= ~( 0x1UL << 8U );
		} else if (msg[9] == 1) {
			GPIOA->ODR &= ~( 0x1UL << 6U );
			GPIOA->ODR &= ~( 0x1UL << 7U );
			GPIOA->ODR |= ( 0x1UL << 8U );
		} else if (msg[9] == 2) {
			GPIOA->ODR &= ~( 0x1UL << 6U );
			GPIOA->ODR |= ( 0x1UL << 7U );
			GPIOA->ODR &= ~( 0x1UL << 8U );
		} else {
			GPIOA->ODR &= ~( 0x1UL << 6U );
			GPIOA->ODR &= ~( 0x1UL << 7U );
			GPIOA->ODR &= ~( 0x1UL << 8U );
		}

		// release mutex
		osMutexRelease(msg_mutex);

		osDelay(100);
	}
}

void TaskSerial4_Execute( void const * argument ){
	for(;;) {

		// lock mutex (dont send msg while it is being modified)
		osMutexWait(msg_mutex, osWaitForever);

		// send the message through the serial port
		USER_UART_Send_Message(msg, msg_length);

		// release mutex
		osMutexRelease(msg_mutex);

		osDelay(200);
	}
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

	GPIOA->CRH	&=	~( 0x1UL <<  10U )
				&	~( 0x2UL <<  8U );

	GPIOA->CRH	|=	 ( 0x1UL <<  10U )
				|	 ( 0x0UL <<  8U );

	osThreadTerminate(TaskSetup);
}