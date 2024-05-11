#include "main.h"
#include "uart.h"
#include "tim.h"
#include "EngTrModel.h"

#define USER_B1		( GPIOC->IDR & ( 0x1UL << 13U ))

void USER_RCC_Init(void);
void USER_GPIO_Init(void);

int main(void)
{
  	USER_RCC_Init();
	USER_GPIO_Init();
  	USER_USART1_Init();
	EngTrModel_initialize();
	
  for(;;)
	{
		if(!USER_B1){
      			USER_TIM2_Delay();//  10ms
      			if(!USER_B1){
        			EngTrModel_U.Throttle = 1.45;
			  	EngTrModel_U.BrakeTorque = 100.0;
      			}
		}
		else{
			EngTrModel_U.Throttle = 50.0;
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
	
	RCC->APB2ENR |= ( 0x1UL << 2U ) // IO Port A clock enable
	| ( 0x1UL << 14U ); // USART 1 clock enable
	RCC->APB2ENR |=  ( 0x1UL << 3U ); // IO Port B clock enable
}

void USER_GPIO_Init(void){

}
