#ifndef TASKS_H_
#define TASKS_H_

#include "cmsis_os.h"

// extern osThreadId Task1Init;
// extern osThreadId Task1Exec;
// extern osThreadId Task2Init;
// extern osThreadId Task2Exec;
// extern osThreadId Task3Init;
// extern osThreadId Task3Exec;
// extern osThreadId Task4Init;
// extern osThreadId Task4Init;
// extern osThreadId Task5Init;
// extern osThreadId Task5Init;

float map(float x, float in_min, float in_max, float out_min, float out_max);

#define ROW1_PIN ( GPIOB->IDR & ( 0x1UL << 4U )) // Brake button 'A' | Pin PB_4
#define ROW2_PIN ( GPIOB->IDR & ( 0x1UL << 5U )) // Right movement 'B' | Pin PB_5
#define ROW3_PIN ( GPIOB->IDR & ( 0x1UL << 6U )) // Left movement 'C' | Pin PB_3
#define POT_PIN ( GPIOA->IDR & ( 0x1UL << 0U )) // Potentiometer pin | PA0

void defineTasks( void );

void Setup( void const * argument );

/* Task method initialization */
// Task 1 reads the data inputs from the potentiometer values and the keypad
void TaskRead1_Init( void const * argument );
void TaskRead1_Execute( void const * argument );

// Task 2 uses values generated in Task 1 to create the data simulating the vehicle
void TaskCreateData2_Init( void const * argument );
void TaskCreateData2_Execute( void const * argument );

// Task 3 sends and writes the data to the LCD
void TaskLCD3_Init( void const * argument );
void TaskLCD3_Execute( void const * argument );

// Task 4 sends information through the serial port to the Raspberry Pi
void TaskSerial4_Init( void const * argument );
void TaskSerial4_Execute( void const * argument );

// Task 5 recieves information from the serial port
void TaskRX5_Init( void const * argument );
void TaskRX5_Execute( void const * argument  );

#endif