#ifndef TASKS_H
#define TASKS_H

/*
    Number of each task represents Task value (task1, task 2... etc)
    Task Name represents what task is going to do
*/ 


void USER_RCC_Init(void);
void USER_GPIO_Init(void);

// Task 1 reads the data inputs from the potentiometer values and the keypad
void TaskRead1_Init( void );
void TaskRead1_Execute( void );

// Task 2 uses values generated in Task 1 to create the data simulating the vehicle
void TaskCreateData2_Init( void );
void TaskCreateData2_Execute( void );

// Task 3 sends and writes the data to the LCD
void TaskLCD3_Init( void );
void TaskLCD3_Execute( void );

// Task 4 sends information through the serial port to the Raspberry Pi
void TaskSerial4_Init( void );
void TaskSerial4_Execute( void );

#endif