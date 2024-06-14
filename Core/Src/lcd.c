#include <stdint.h>
#include "main.h"
#include "tim.h"
#include "lcd.h"

//Caracter definido por usuario para cargar en la memoria CGRAM del LCD
const int8_t UserFont[8][8] =
{
		{ 0x11, 0x0A, 0x04, 0x1B, 0x11, 0x11, 0x11, 0x0E },
		{ 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10 },
		{ 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18 },
		{ 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C },
		{ 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E },
		{ 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },
		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
};

//Funcion que inicializa el LCD a 4 bits
void LCD_Init(void){
	int8_t const *p;
/**
  * Configuracion de todos los pines hacia el LCD general purpose output push-pull, 10 MHz speed
  */
	GPIOC->CRL	&=	~( 0x3UL << 30U ) & ~( 0x2UL << 28U )
			& 	~( 0x3UL << 26U ) & ~( 0x2UL << 24U );
	GPIOC->CRL 	|= 	 ( 0x1UL << 28U )
			|  	 ( 0x1UL << 24U );
	GPIOC->CRH	&=	~( 0x3UL << 18U ) & ~( 0x2UL << 16U )
			& 	~( 0x3UL << 14U ) & ~( 0x2UL << 12U )
			&	~( 0x3UL << 10U ) & ~( 0x2UL <<  8U )
			& 	~( 0x3UL <<  6U ) & ~( 0x2UL <<  4U )
			& 	~( 0x3UL <<  2U ) & ~( 0x2UL <<  0U );
	GPIOC->CRH	|= 	 ( 0x1UL << 16U )
			|  	 ( 0x1UL << 12U )
			| 	 ( 0x1UL <<  8U )
			|  	 ( 0x1UL <<  4U )
			|  	 ( 0x1UL <<  0U );
/**
  * Inicialización del LCD
  * https://web.alfredstate.edu/faculty/weimandn/lcd/lcd_initialization/lcd_initialization_index.html
  * Power ON
  */
	GPIOC->BSRR	 =	 LCD_RS_PIN_LOW;
	GPIOC->BSRR	 =	 LCD_RW_PIN_LOW;
	GPIOC->BSRR	 =	 LCD_EN_PIN_LOW;
	GPIOC->BSRR	 =	 LCD_D4_PIN_LOW;
	GPIOC->BSRR	 =	 LCD_D5_PIN_LOW;
	GPIOC->BSRR	 =	 LCD_D6_PIN_LOW;
	GPIOC->BSRR	 =	 LCD_D7_PIN_LOW;

	for (int i = 0; i < 5000; i++) { // Fake 50 ms timer
			USER_TIM4_Delay();
		}

	/* Special case of 'Function Set' 				*/
	GPIOC->BSRR	 =	 LCD_D4_PIN_HIGH;
	GPIOC->BSRR	 =	 LCD_D5_PIN_HIGH;
	GPIOC->BSRR	 =	 LCD_D6_PIN_LOW;
	GPIOC->BSRR	 =	 LCD_D7_PIN_LOW;
	LCD_Pulse_EN( );

	for (int i = 0; i < 500; i++) { // Fake 5ms timer
		USER_TIM4_Delay();
	}

	/* Special case of 'Function Set' 				*/
	GPIOC->BSRR	 =	 LCD_D4_PIN_HIGH;
	GPIOC->BSRR	 =	 LCD_D5_PIN_HIGH;
	GPIOC->BSRR	 =	 LCD_D6_PIN_LOW;
	GPIOC->BSRR	 =	 LCD_D7_PIN_LOW;
	LCD_Pulse_EN( );

	for (int i = 0; i < 10; i++) { // Fake 10us timer
		USER_TIM4_Delay();
	}

	/* Special case of 'Function Set' 				*/
	GPIOC->BSRR	 =	 LCD_D4_PIN_HIGH;
	GPIOC->BSRR	 =	 LCD_D5_PIN_HIGH;
	GPIOC->BSRR	 =	 LCD_D6_PIN_LOW;
	GPIOC->BSRR	 =	 LCD_D7_PIN_LOW;
	LCD_Pulse_EN( );
	while( LCD_Busy( ) );//						checking the busy flag
	/* Initial 'Function Set' to change 4-bit mode 			*/
	GPIOC->BSRR	 =	 LCD_D4_PIN_LOW;
	GPIOC->BSRR	 =	 LCD_D5_PIN_HIGH;
	GPIOC->BSRR	 =	 LCD_D6_PIN_LOW;
	GPIOC->BSRR	 =	 LCD_D7_PIN_LOW;
	LCD_Pulse_EN( );
	while( LCD_Busy( ) );//						checking the busy flag
	/* 'Function Set' (I=1, N and F as required)			*/
	LCD_Write_Cmd( 0x28U );//					2-line display, 5x7 dot
	/* 'Display ON/OFF Control' (D=0, C=0, B=0)			*/
	LCD_Write_Cmd( 0x08U );//					display, cursor and blinking off
	/* 'Clear Display'						*/
	LCD_Write_Cmd( 0x01U );//
	/* 'Entry Mode Set' (I/D and S as required)			*/
	LCD_Write_Cmd( 0x06U );//					cursor increment by 1, shift off
	/* Initialization Ends						*/
	LCD_Write_Cmd( 0x0FU );//					display, cursor and blinking on

	//Cargamos el caracter definido por el usuario en la CGRAM
	LCD_Write_Cmd( 0x40 );//					establece la direccion CGRAM desde 0
	p = &UserFont[0][0];

	for( int i = 0; i < sizeof( UserFont ); i++, p++ )
		LCD_Put_Char( *p );

	/*	Set DDRAM address in address			*/
	LCD_Write_Cmd( 0x80 );//
}

//Funcion que genera un strobe en el LCD
void LCD_Out_Data4(uint8_t val){
	if( ( val & 0x01U ) == 0x01U )//				Bit[0]
		GPIOC->BSRR	=	LCD_D4_PIN_HIGH;
	else
		GPIOC->BSRR	=	LCD_D4_PIN_LOW;

	if( ( val & 0x02U ) == 0x02U )//				Bit[1]
		GPIOC->BSRR	=	LCD_D5_PIN_HIGH;
	else
		GPIOC->BSRR	=	LCD_D5_PIN_LOW;

	if( ( val & 0x04U ) == 0x04U )//				Bit[2]
		GPIOC->BSRR	=	LCD_D6_PIN_HIGH;
	else
		GPIOC->BSRR	=	LCD_D6_PIN_LOW;

	if( ( val & 0x08U ) == 0x08U )//				Bit[3]
		GPIOC->BSRR	=	LCD_D7_PIN_HIGH;
	else
		GPIOC->BSRR	=	LCD_D7_PIN_LOW;
}

//Funcion que escribe 1 byte de datos en el LCD
void LCD_Write_Byte(uint8_t val){
	LCD_Out_Data4( ( val >> 4 ) & 0x0FU );
	LCD_Pulse_EN( );
	LCD_Out_Data4( val & 0x0FU );
	LCD_Pulse_EN( );
	while( LCD_Busy( ) );
}

//Funcion que escribe un comando en el LCD
void LCD_Write_Cmd(uint8_t val){
	GPIOC->BSRR	=	LCD_RS_PIN_LOW;//			RS=0 (seleccion de comando)
	LCD_Write_Byte( val );
}

//Escribe un caracter ASCII en el LCD
void LCD_Put_Char(uint8_t c){
	GPIOC->BSRR	=	LCD_RS_PIN_HIGH;//			RS=1 (seleccion de caracteres)
	LCD_Write_Byte( c );
}

//Funcion que establece el cursor en una posicion de la pantalla del LCD
//Minimum values for line and column must be 1
void LCD_Set_Cursor(uint8_t line, uint8_t column){
	uint8_t address;
	column--;
	line--;
	address = ( line * 0x40U ) + column;
	address = 0x80U + ( address & 0x7FU );
	LCD_Write_Cmd( address );
}

//Funcion que envia una cadena de caracteres ASCII al LCD
void LCD_Put_Str(char * str){
	for( int16_t i = 0; i < 16 && str[ i ] != 0; i++ )
		LCD_Put_Char( str[ i ] );//				envia 1 byte al LCD
}

//Funcion que envia un caracter numerico al LCD
//El número debe ser entero y de 5 dígitos máximo
void LCD_Put_Num(int16_t num){
	int16_t p;
	int16_t f = 0;
	int8_t ch[ 5 ];

	for( int16_t i = 0; i < 5; i++ ){
		p = 1;
		for( int16_t j = 4 - i; j > 0; j-- )
			p = p * 10;
		ch[ i ] = ( num / p );
		if( num >= p && !f )
			f = 1;
		num = num - ch[ i ] * p;
		ch[ i ] = ch[ i ] + 48;
		if( f )
			LCD_Put_Char( ch[ i ] );
	}
}

//Funcion que provoca tiempos de espera en el LCD
char LCD_Busy(void){
/**
  * Configuracion de D7 as input floating
  */
	GPIOC->CRH	&=	~( 0x2UL << 18U ) & ~( 0x3UL << 16U );
	GPIOC->CRH	|=   	 ( 0x1UL << 18U );
	GPIOC->BSRR	 =	 LCD_RS_PIN_LOW;
	GPIOC->BSRR	 =	 LCD_RW_PIN_HIGH;
	GPIOC->BSRR	 =	 LCD_EN_PIN_HIGH;

	for (int i = 0; i < 1000; i++) { // fake 100 us timer
		USER_TIM4_Delay();
	}

//	USER_TIM4_Delay();//	100us (50ms)
	if(( GPIOC->IDR	& LCD_D7_PIN_HIGH )) {//			if D7 is set, then
		GPIOC->BSRR	= 	LCD_EN_PIN_LOW;
		GPIOC->BSRR	=	LCD_RW_PIN_LOW;
/**
  * Configuracion de D7 as output push-pull, 10 MHz speed
  */
		GPIOC->CRH	&=	~( 0x3UL << 18U ) & ~( 0x2UL << 16U );
		GPIOC->CRH	|=   	 ( 0x1UL << 16U );
		return 1;
	} else {
		GPIOC->BSRR	= 	LCD_EN_PIN_LOW;
		GPIOC->BSRR	=	LCD_RW_PIN_LOW;
/**
  * Configuracion de D7 as output push-pull, 10 MHz speed
  */
		GPIOC->CRH	&=	~( 0x3UL << 18U ) & ~( 0x2UL << 16U );
		GPIOC->CRH	|=   	 ( 0x1UL << 16U );
		return 0;
	}
}

//Funcion que genera un pulso en el pin EN del LCD
void LCD_Pulse_EN(void){
	GPIOC->BSRR	=	LCD_EN_PIN_LOW;//

	for (int i = 0; i < 100; i++) { // Fake 1ms timer
		USER_TIM4_Delay();
	}

	GPIOC->BSRR	=	LCD_EN_PIN_HIGH;//			habilita pin EN ON

	for (int i = 0; i < 100; i++) { // Fake 1ms timer
		USER_TIM4_Delay();
	}

	GPIOC->BSRR	=	LCD_EN_PIN_LOW;//			habilita pin EN OFF

	for (int i = 0; i < 100; i++) { // Fake 1ms timer
		USER_TIM4_Delay();
	}
}

/*
 * Funcion que muestra un caracter grafico en el LCD
 * en 'value' el valor de su posicion en CGRAM y
 * en 'size' especificamos su tamaño
 */
void LCD_BarGraphic(int16_t value, int16_t size){
	value = value * size / 20;//					matriz de 5x8 pixeles
	for( int16_t i = 0; i < size; i++ ){
		if( value > 5 ){
			LCD_Put_Char( 0x05U );
			value -= 5;
		} else {
			LCD_Put_Char( value );
			break;
		}
	}
}

/*
 * Funcion que muestra un caracter grafico en el LCD especificando 
 * la posicion pos_x horizontal de inicio y la posicion pos_y vertical de la pantalla LCD
 */
void LCD_BarGraphicXY(int16_t pos_x, int16_t pos_y, int16_t value){
	LCD_Set_Cursor( pos_x, pos_y );
	for( int16_t i = 0; i < 16; i++ ){
		if( value > 5 ){
			LCD_Put_Char( 0x05U );
			value -= 5;
		} else {
			LCD_Put_Char( value );
			while( i++ < 16 )
				LCD_Put_Char( 0 );
		}
	}
}

void USER_LCD_Send_Message( uint8_t *msg, uint16_t size ){

	// Following this scheme, we show the values on the LCD
	

	/////////////////////
	// [3] lsb of engine speed
	// [4] msb of engine speed
	// [5] lsb of vehicle speed
	// [6] msb of vehicle speed
	// [7] lsb of gear
	// [8] msb of gear
	/////////////////////

	// 	msg[3] = (uint8_t) e_speed & 0xff;
	//	msg[4] = (uint8_t) (e_speed >> 8);
	//
	//
	//	msg[5] = (uint8_t) v_speed & 0xff;
	//	msg[6] = (uint8_t) (v_speed >> 8);
	//
	//	msg[7] = (uint8_t) gear & 0xff;
	//	msg[8] = (uint8_t) (gear >> 8);

	LCD_Put_Str( "e:");
	LCD_Put_Num( (msg[4] << 8) | msg[3] );


	LCD_Put_Str(" v:");
	LCD_Put_Num( (msg[6] << 8) | msg[5] );

	LCD_Set_Cursor(2, 1);

	LCD_Put_Str("g:");
	LCD_Put_Num( (msg[8] << 8) | msg[7] );
	LCD_Put_Str(" ");

	LCD_Put_Str("a:");
	LCD_Put_Num( (msg[11] << 8) | msg[10] );

}