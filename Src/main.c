#include "main.h"
#include "uart.h"
#include "tim.h"
#include "adc.h"
#include "lcd.h"
#include "EngTrModel.h"
#include "Tasks.h"

/*
	Main function

	This function initializes the peripherals and runs the main loop
	of the program. It reads the value from the potentiometer and
	scales it to the range of acceleration (0 to 100). It also reads
	the state of the keypad and sets the throttle and brake values
	accordingly.
*/

int main(void) {
    USER_RCC_Init();
	USER_GPIO_Init();
	
	// Initialize timer right now to time the tasks
	USER_TIM3_Init();

	volatile double t1_init_time = 0, t2_init_time = 0, t3_init_time = 0, t4_init_time = 0;
	volatile double t1_task_time = 0, t2_task_time = 0, t3_task_time = 0, t4_task_time = 0;
	
	// Initialize tasks
	TIM3->CNT = 0;
	TaskRead1_Init();
	t1_init_time = T_HCLK * TIM3->CNT * ( TIM3->PSC + 1);

	TIM3->CNT = 0;
	TaskCreateData2_Init();
	t2_init_time = T_HCLK * TIM3->CNT * ( TIM3->PSC + 1);
	
	TIM3->CNT = 0;
	TaskLCD3_Init();
	t3_init_time = T_HCLK * TIM3->CNT * ( TIM3->PSC + 1);

	TIM3->CNT = 0;
	TaskSerial4_Init();
	t4_init_time = T_HCLK * TIM3->CNT * ( TIM3->PSC + 1);


	for(;;)
	{
		// * TASK 1 * //
		TIM3->CNT = 0;

		// Task 1 reads the data inputs from the potentiometer values and the keypad
		TaskRead1_Execute();

		t1_task_time = T_HCLK * TIM3->CNT * ( TIM3->PSC + 1);

		// * TASK 2 * //

		TIM3->CNT = 0;

		// Task 2 uses values generated in Task 1 to create the data simulating the vehicle
		TaskCreateData2_Execute();

		t2_task_time = T_HCLK * TIM3->CNT * ( TIM3->PSC + 1);

		// * TASK 3 * //
		TIM3->CNT = 0;

		// Task 3 sends and writes the data to the LCD
		TaskLCD3_Execute();

		t3_task_time = T_HCLK * TIM3->CNT * ( TIM3->PSC + 1);

		// * TASK 4 * //
		TIM3->CNT = 0;

		// Task 4 sends information through the serial port to the Raspberry Pi
		TaskSerial4_Execute();

		t4_task_time = T_HCLK * TIM3->CNT * ( TIM3->PSC + 1);

		// Delay for 100 ms
		for (int i = 0; i < 5000; i++) { // Fake 50ms timer
			USER_TIM4_Delay();
		}
	}
}

