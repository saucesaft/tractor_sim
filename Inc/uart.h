#ifndef UART_H_
#define UART_H_

#include <stdio.h>

int _write(int file, char *ptr, int len);


// USARTDIV 34.722222222
// Fraction: 0.722222222 * 16 = 11.555555556 ~= 12
// Mantissa: 34

// [frac] [mantissa]
// 0x[22] [C]

// config registers
#define USARTDIV        0x22C				// 115200 baud rate
#define USART_CR1_UE    ( 0x1UL << 13U ) 	// enable usart peripheral
#define USART_CR1_M     ( 0x1UL << 12U )	// word length 8 bits
#define USART_CR1_TE    ( 0x1UL <<  3U ) 	// enable transmitter
#define USART_CR1_RE    ( 0x1UL <<  2U ) 	// enable receiver
#define USART_CR2_STOP  ( 0x3UL << 12U )	//  stop bits PREGUNTAR

// status registers
#define USART_SR_TXE    ( 0x1UL <<  7U )	// transmitter data register empty
// 0 : data is not transferred to the shift register
// 1 : data is transferred to the shift register

#define USART_SR_RNXE    ( 0x1UL <<  5U )	// transmitter data register not empty
// 0 : data is not received
// 1 : received data is ready to be read

#define USART_OVERRUN_ERR	( 0x1UL << 3U )
// this bit is set by hardware when the word currently being received in the shift register
// is ready to be transferred into the RDR register while RXNE=1

#define USART_NOISE_ERR		( 0x1UL << 2U )
// this bit is set by hardware when noise is detected on a received frame

#define USART_FRAMING_ERR	( 0x1UL << 1U )
// this bit is set by hardware when a de-synchronization, excessive noise or a break character is detected

#define USART_PARITY_ERR	( 0x1UL << 0U )
// this bit is set by hardware when a parity error occurs in receiver mode

void USER_USART1_Init( void );
uint8_t USER_USART1_Read_8bit();
void USER_UART_Send_Message( uint8_t *msg, uint16_t size);

#endif /* UART_H_ */
