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

/* Flash memory interface registers */
typedef struct {
	volatile uint32_t ACR;
	volatile uint32_t KEYR;
	volatile uint32_t OPTKEYR;
	volatile uint32_t SR;
	volatile uint32_t CR;
	volatile uint32_t AR;
	volatile uint32_t reserved;
	volatile uint32_t OBR;
	volatile uint32_t WRPR;
} FLASH_TypeDef;

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

/* Analog to Digital Converter Registers */
typedef struct {
	volatile uint32_t SR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMPR1;
	volatile uint32_t SMPR2;
	volatile uint32_t JOFR1;
	volatile uint32_t JOFR2;
	volatile uint32_t JOFR3;
	volatile uint32_t JOFR4;
	volatile uint32_t HTR;
	volatile uint32_t LTR;
	volatile uint32_t SQR1;
	volatile uint32_t SQR2;
	volatile uint32_t SQR3;
	volatile uint32_t JSQR;
	volatile uint32_t JDR1;
	volatile uint32_t JDR2;
	volatile uint32_t JDR3;
	volatile uint32_t JDR4;
	volatile uint32_t DR;
} ADC_TypeDef;

#define RCC_BASE	0x40021000UL	//	RCC base address
#define GPIOA_BASE	0x40010800UL	//	GPIO Port A base address
#define GPIOB_BASE  0x40010C00UL	//  GPIO Port B base address
#define GPIOC_BASE	0x40011000UL	//	GPIO Port C base address
#define USART_BASE	0x40013800UL	//  USART1 base address
#define FLASH_BASE  0x40022000UL    // FLASH base address

#define TIM2_BASE	0x40000000UL 	//  TIM2 base address
#define TIM3_BASE	0x40000400UL 	//  TIM3 base address

#define TIM4_BASE	0x40000800UL 	//  TIM4 base address
#define TIM5_BASE	0x40000C00UL 	//  TIM5 base address

#define TIM9_BASE	0x40014C00UL 	//  TIM9 base address
#define TIM10_BASE	0x40015000UL 	//  TIM10 base address
#define TIM11_BASE	0x40015400UL 	//  TIM11 base address

#define NVIC_BASE	0xE000E100UL	//	NVIC base address
#define ADC1_BASE	0x40012400UL	// ADC1 base address

#define RCC         (( RCC_TypeDef		*)	RCC_BASE 	)
#define GPIOA		(( GPIO_TypeDef		*)	GPIOA_BASE 	)
#define GPIOB		(( GPIO_TypeDef		*)	GPIOB_BASE 	)
#define GPIOC		(( GPIO_TypeDef		*)	GPIOC_BASE 	)
#define USART1		(( USART_TypeDef 	*)	USART_BASE 	)
#define FLASH		(( FLASH_TypeDef 	*)	FLASH_BASE	)
#define TIM2		(( TIM_TypeDef		*)	TIM2_BASE 	)
#define TIM3		(( TIM_TypeDef		*)	TIM3_BASE 	)
#define TIM4		(( TIM_TypeDef		*)	TIM4_BASE 	)
#define TIM5		(( TIM_TypeDef		*)	TIM5_BASE 	)
#define TIM9		(( TIM_TypeDef		*)	TIM9_BASE 	)
#define TIM10		(( TIM_TypeDef		*)	TIM10_BASE 	)
#define TIM11		(( TIM_TypeDef		*)	TIM11_BASE 	)
#define NVIC		(( NVIC_TypeDef		*)	NVIC_BASE	)
#define ADC1		(( ADC_TypeDef		*)	ADC1_BASE	)

#define ROW1_PIN ( GPIOB->IDR & ( 0x1UL << 4U )) // Brake button 'A' | Pin PB_4
#define ROW2_PIN ( GPIOB->IDR & ( 0x1UL << 5U )) // Right movement 'B' | Pin PB_5
#define ROW3_PIN ( GPIOB->IDR & ( 0x1UL << 6U )) // Left movement 'C' | Pin PB_3

#define POT_PIN ( GPIOA->IDR & ( 0x1UL << 0U )) // Potenciometer pin | PA0

#endif /* MAIN_H_ */
