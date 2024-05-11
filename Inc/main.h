#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

/* Reset and Clock Control registers */
typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
} RCC_TypeDef;

/* General Purpose I/O registers */
typedef struct
{
	volatile uint32_t CRL;
	volatile uint32_t CRH;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t BRR;
	volatile uint32_t LCKR;
} GPIO_TypeDef;

/* USART registers */
typedef struct
{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
} USART_TypeDef;

/* Timer registers */
typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	volatile uint32_t CCMR2;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	volatile uint32_t RCR;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	volatile uint32_t CCR3;
	volatile uint32_t CCR4;
	volatile uint32_t BDTR;
	volatile uint32_t DCR;
	volatile uint32_t DMAR;
} TIM_TypeDef;

/* Nested Vector Interrupt Controller */
typedef struct
{
	volatile uint32_t ISER[3U];
	volatile uint32_t RESERVED0[29U];
	volatile uint32_t ICER[3U];
	volatile uint32_t RESERVED1[29U];
	volatile uint32_t ISPR[3U];
	volatile uint32_t RESERVED2[29U];
	volatile uint32_t ICPR[3U];
	volatile uint32_t RESERVED3[29U];
	volatile uint32_t IABR[3U];
	volatile uint32_t RESERVED4[61U];
	volatile uint32_t IPR[84U];
	volatile uint32_t RESERVED5[683U];
	volatile uint32_t STIR;
} NVIC_TypeDef;

#define RCC_BASE	0x40021000UL	//	RCC base address
#define GPIOA_BASE	0x40010800UL	//	GPIO Port A base address
#define GPIOB_BASE  0x40010C00UL	//  GPIO Port B base address
#define GPIOC_BASE	0x40011000UL	//	GPIO Port A base address
#define USART_BASE	0x40013800UL	//  USART1 base address
#define TIM2_BASE	0x40000000UL 	//  TIM2 base address
#define TIM3_BASE	0x40000400UL 	//  TIM3 base address
#define NVIC_BASE	0xE000E100UL	//	NVIC base address

#define RCC         (( RCC_TypeDef		*)	RCC_BASE 	)
#define GPIOA		(( GPIO_TypeDef		*)	GPIOA_BASE 	)
#define GPIOB		(( GPIO_TypeDef		*)	GPIOB_BASE 	)
#define GPIOC		(( GPIO_TypeDef		*)	GPIOC_BASE 	)
#define USART1		(( USART_TypeDef 	*)	USART_BASE 	)
#define TIM2		(( TIM_TypeDef		*)	TIM2_BASE 	)
#define TIM3		(( TIM_TypeDef		*)	TIM3_BASE 	)
#define NVIC		(( NVIC_TypeDef		*)	NVIC_BASE	)

#endif /* MAIN_H_ */
