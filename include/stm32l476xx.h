/*
 * Definitions for developing on:
 * 	STM32L475xx
 * 	STM32L476xx
 * 	STM32L486xx
 */

#ifndef STM32L476XX_H
#define STM32L476XX_H

#include <stdint.h>

#define __I 	volatile const 	/* read only  */
#define __O 	volatile 				/* write only */
#define __IO 	volatile 				/* read-write */

#define PERIPH_BASE		((uint32_t)0x40000000)
#define APB1					(PERIPH_BASE) 							/* 0x4000 0000 */
#define APB2				 	(PERIPH_BASE + 0x00010000)  /* 0x4001 0000 */
#define AHB1					(PERIPH_BASE + 0x00020000)  /* 0x4002 0000 */
#define AHB2					(PERIPH_BASE + 0x08000000)  /* 0x4800 0000 */

/* Reset and Clock Control */
typedef struct {
__IO uint32_t CR;						/* 0x00 = Clock Control Register											  									 */
__IO uint32_t ICSCR;				/* 0x04 = Internal Clock Sources Calibration Register	  								   */
__IO uint32_t CFGR;					/* 0x08 = Clock Configuration Register								  									 */
__IO uint32_t PLLCFGR;			/* 0x0c = PLL Configuration Register									  									 */
__IO uint32_t PLLSAI1CFGR;	/* 0x10 = PLLSAI1 Configuration Register							  									 */
__IO uint32_t PLLSAI2CFGR;	/* 0x14 = PLLSAI2 Configuration Register							  									 */
__IO uint32_t CIER;					/* 0x18 = Clock Interrupt Enable Register							  									 */
__IO uint32_t CIFR;					/* 0x1c = Clock Interrupt Flag Register								  									 */
__IO uint32_t CICR;					/* 0x20 = Clock Interrupt Clear Register							  									 */
__IO uint32_t RESERVED0;		/* 0x24																								  									 */
__IO uint32_t AHB1RSTR;			/* 0x28 = AHB1 Peripheral Reset Register							  									 */
__IO uint32_t AHB2RSTR;			/* 0x2c = AHB2 Peripheral Reset Register							  									 */
__IO uint32_t AHB3RSTR;			/* 0x30 = AHB3 Peripheral Reset Register							  									 */
__IO uint32_t RESERVED1;		/* 0x34																								  									 */
__IO uint32_t APB1RSTR1;		/* 0x38 = APB1 Peripheral Reset Register 1						  									 */
__IO uint32_t APB1RSTR2;		/* 0x3c = APB1 Peripheral Reset Register 2						  									 */
__IO uint32_t APB2RSTR;			/* 0x40 = APB2 Peripheral Reset Register							  									 */
__IO uint32_t RESERVED2;		/* 0x44																								  									 */
__IO uint32_t AHB1ENR;			/* 0x48 = AHB1 Peripheral Clock Enable Register				  									 */
__IO uint32_t AHB2ENR;			/* 0x4c = AHB2 Peripheral Clock Enable Register				  									 */
__IO uint32_t AHB3ENR;			/* 0x50 = AHB3 Peripheral Clock Enable Register				  									 */
__IO uint32_t RESERVED3;		/* 0x54																								  									 */
__IO uint32_t APB1ENR1;			/* 0x58 = APB1 Peripheral Clock Enable Register 1			  									 */
__IO uint32_t APB1ENR2;			/* 0x5c = APB1 Peripheral Clock Enable Register 2			  									 */
__IO uint32_t APB2ENR;			/* 0x60 = APB2 Peripheral Clock Enable Register				  									 */
__IO uint32_t RESERVED4;		/* 0x64																								  									 */
__IO uint32_t AHB1SMENR;		/* 0x68 = AHB1 Peripheral Clocks Enable In Sleep and Stop Modes Register   */
__IO uint32_t AHB2SMENR;		/* 0x6C = AHB2 Peripheral Clocks Enable In Sleep and Stop Modes Register 	 */
__IO uint32_t AHB3SMENR;		/* 0x70 = AHB3 Peripheral Clocks Enable In Sleep and Stop Modes Register 	 */
__IO uint32_t RESERVED5;		/* 0x74 																																 	 */
__IO uint32_t APB1SMENR1;		/* 0x78 = APB1 Peripheral Clocks Enable In Sleep and Stop Modes Register 1 */
__IO uint32_t APB1SMENR2;		/* 0x7C = APB1 Peripheral Clocks Enable In Sleep and Stop Modes Register 2 */
__IO uint32_t APB2SMENR;		/* 0x80 = APB2 Peripheral Clocks Enable In Sleep and Stop Modes Register   */
__IO uint32_t RESERVED6;		/* 0x84 																												           */
__IO uint32_t CCIPR;				/* 0x88 = Peripheral Independent Clock Configuration Register 	           */
__IO uint32_t RESERVED7;		/* 0x8C  																												           */
__IO uint32_t BDCR;					/* 0x90 = Backup Domain Control Register 												           */
__IO uint32_t CSR;					/* 0x94 = Control/Status Register 															           */
} RCC_TypeDef;

#define RCC_AHB2ENR_GPIOAEN	(1<<0)
#define RCC_AHB2ENR_GPIOBEN	(1<<1)
#define RCC_AHB2ENR_GPIOCEN	(1<<2)
#define RCC_AHB2ENR_GPIODEN	(1<<3)
#define RCC_AHB2ENR_GPIOEEN	(1<<4)
#define RCC_AHB2ENR_GPIOFEN	(1<<5)
#define RCC_AHB2ENR_GPIOGEN	(1<<6)
#define RCC_AHB2ENR_GPIOHEN	(1<<7)
#define RCC_AHB2ENR_GPIOIEN	(1<<8)

#define RCC_BASE	(AHB1PERIPH_BASE + 0x1000)
#define RCC				((RCC_TypeDef *)(RCC_BASE))

/* General Purpose IO */
typedef struct {
__IO uint32_t MODER;		/* 0x00 = Mode Register														*/
__IO uint32_t OTYPER;   /* 0x04 = Output Type Register										*/
__IO uint32_t OSPEEDR;	/* 0x08 = Output Speed Register										*/
__IO uint32_t PUPDR;		/* 0x0C = Pull Up/Pull Down Register							*/
__IO uint32_t IDR;			/* 0x10 = Input Data Register											*/
__IO uint32_t ODR;			/* 0x14 = Output Data Register										*/
__IO uint32_t BSRR;			/* 0x18 = Bit Set/Reset Register (LOW)						*/
__IO uint32_t LCKR;			/* 0x1C = Configuration Lock Register							*/
__IO uint32_t AFR[2];	  /* 0x20 = Alternate Function (Low/High) Registers */
__IO uint32_t BRR; 			/* 0x28 = Bit Reset Register 											*/
__IO uint32_t ASCR; 		/* 0x2C = Analog Switch Controll Register 				*/
} GPIO_TypeDef;

#define GPIOA_BASE	(AHB2 + 0x0000)
#define GPIOB_BASE	(AHB2 + 0x0400)
#define GPIOC_BASE	(AHB2 + 0x0800)
#define GPIOD_BASE	(AHB2 + 0x0C00)
#define GPIOE_BASE	(AHB2 + 0x1000)

#define GPIOA ((GPIO_TypeDef*)GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef*)GPIOB_BASE)
#define GPIOC ((GPIO_TypeDef*)GPIOC_BASE)
#define GPIOD ((GPIO_TypeDef*)GPIOD_BASE)
#define GPIOE ((GPIO_TypeDef*)GPIOE_BASE)

#define MODE_INPUT	0x00
#define MODE_OUTPUT	0x01
#define MODE_ALT		0x02
#define MODE_ANALOG	0x03

#endif // STM32L476XX_H
