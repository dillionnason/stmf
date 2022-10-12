#include <stdint.h>
/* In en.DM00083560.pdf:   */

#define __I volatile const	/* read only */
#define __O volatile		/* write only */
#define __IO volatile		/* read-write */


/* p77 */
#define PERIPH_BASE	((uint32_t)0x40000000)
#define APB1PERIPH_BASE (PERIPH_BASE)
#define APB2PERIPH_BASE (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE	(PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE	(PERIPH_BASE + 0x08000000)

/* 6.4 p223 */
/* Reset and Clock Control */
typedef struct {
__IO uint32_t CR;		/* 0x00 */
__IO uint32_t ICSCR;		/* 0x04 */
__IO uint32_t CFGR;		/* 0x08 */
__IO uint32_t PLLCFGR;		/* 0x0c */
__IO uint32_t PLLSAI1CFGR;	/* 0x10 */
__IO uint32_t PLLSAI2CFGR;	/* 0x14 */
__IO uint32_t CIER;		/* 0x18 */
__IO uint32_t CIFR;		/* 0x1c */
__IO uint32_t CICR;		/* 0x20 */
__IO uint32_t RESERVED0;	/* 0x24 */
__IO uint32_t AHB1RSTR;		/* 0x28 */
__IO uint32_t AHB2RSTR;		/* 0x2c */
__IO uint32_t AHB3RSTR;		/* 0x30 */
__IO uint32_t RESERVED1;	/* 0x34 */
__IO uint32_t APB1RSTR1;	/* 0x38 */
__IO uint32_t APB1RSTR2;	/* 0x3c */
__IO uint32_t APB2RSTR;		/* 0x40 */
__IO uint32_t RESERVED2;	/* 0x44 */
__IO uint32_t AHB1ENR;		/* 0x48 */
__IO uint32_t AHB2ENR;		/* 0x4c */
__IO uint32_t AHB3ENR;		/* 0x50 */
__IO uint32_t RESERVED3;	/* 0x54 */
__IO uint32_t APB1ENR1;		/* 0x58 */
__IO uint32_t APB1ENR2;		/* 0x5c */
__IO uint32_t APB2ENR;		/* 0x60 */
__IO uint32_t RESERVED4;	/* 0x64 */
__IO uint32_t AHB1SMENR;	/* 0x68 */
__IO uint32_t AHB2SMENR;	/* 0x6C */
__IO uint32_t AHB3SMENR;	/* 0x70 */
__IO uint32_t RESERVED5;	/* 0x74 */
__IO uint32_t APB1SMENR1;	/* 0x78 */
__IO uint32_t APB1SMENR2;	/* 0x7C */
__IO uint32_t APB2SMENR;	/* 0x80 */
__IO uint32_t RESERVED6;	/* 0x84 */
__IO uint32_t CCIPR;		/* 0x88 */
__IO uint32_t RESERVED7;	/* 0x8C */
__IO uint32_t BDCR;		/* 0x90 */
__IO uint32_t CSR;		/* 0x94 */

} RCC_TypeDef;

#define RCC_CR_MSION		(1<<0)
#define RCC_CR_MSIRDY		(1<<1)
#define RCC_CR_MSIPLLEN		(1<<2)
#define RCC_CR_MSIRGSEL		(1<<3)
#define RCC_CR_MSIRANGE		(0xf<<4)
#define RCC_CR_HSION		(1<<8)
#define RCC_CR_HSIKERON		(1<<9)
#define RCC_CR_HSIRDY		(1<<10)
#define RCC_CR_HSIASFS		(1<<11)
#define RCC_CR_HSEON		(1<<16)
#define RCC_CR_HSERDY		(1<<17)
#define RCC_CR_HSEBYP		(1<<18)
#define RCC_CR_PLLON		(1<<24)
#define RCC_CR_PLLRDY		(1<<25)


#define RCC_CFGR_SW		(0x3)
#define RCC_CFGR_SW_PLL		(0x3)
#define RCC_CFGR_SW_HSE		(0x2)
#define RCC_CFGR_SW_HSI		(0x1)
#define RCC_CFGR_SW_MSI		(0x0)
#define RCC_CFGR_HPRE		(0xf<<4)

#define RCC_CFGR_SWS		(0x3<<2)
#define RCC_CFGR_SWS_PLL	(0x3<<2)

#define RCC_PLLCFGR_PLLSRC	(0x3)
#define RCC_PLLCFGR_PLLSRC_HSI	(0x2)
#define RCC_PLLCFGR_PLLN	(0xff<<8)
#define RCC_PLLCFGR_PLLM	(0x7<<4)
#define RCC_PLLCFGR_PLLR	(0x3<<25)
#define RCC_PLLCFGR_PLLREN	(1<<24)

#define RCC_AHB2ENR_GPIOAEN	(1<<0)
#define RCC_AHB2ENR_GPIOBEN	(1<<1)
#define RCC_AHB2ENR_GPIOCEN	(1<<2)
#define RCC_AHB2ENR_GPIODEN	(1<<3)
#define RCC_AHB2ENR_GPIOEEN	(1<<4)
#define RCC_AHB2ENR_GPIOFEN	(1<<5)
#define RCC_AHB2ENR_GPIOGEN	(1<<6)
#define RCC_AHB2ENR_GPIOHEN	(1<<7)
#define RCC_AHB2ENR_GPIOIEN	(1<<8)

/* 6.4.19, p253 */
#define RCC_APB1ENR1_TIM4EN (1<<2)
#define RCC_APB1ENR1_LCDEN	(1<<9)
#define RCC_APB1ENR1_PWREN	(1<<28)

/* 6.4.21 p259 */
#define RCC_APB2ENR_SDMMC1EN	(1<<10)
#define RCC_APB2ENR_TIM1EN	(1<<11)
#define RCC_APB2ENR_SPI1EN	(1<<12)
#define RCC_APB2ENR_TIM8EN	(1<<13)
#define RCC_APB2ENR_UART1EN	(1<<14)
#define RCC_APB2ENR_TIM15EN	(1<<16)
#define RCC_APB2ENR_TIM16EN	(1<<17)
#define RCC_APB2ENR_TIM17EN	(1<<18)
#define RCC_APB2ENR_SAI1EN	(1<<21)
#define RCC_APB2ENR_SAI2EN	(1<<22)
#define RCC_APB2ENR_DFSDM1EN	(1<<24)

/* 6.4.29, p272 */
#define RCC_BDCR_LSEON		(0x1<<0)	/* LSE Oscillator Enable */
#define RCC_BDCR_LSERDY		(0x1<<1)	/* LSE Oscillator Ready */
#define RCC_BDCR_LSEBYP		(0x1<<2)	/* LSE Oscillator Bypass */
#define RCC_BDCR_RTCSEL		(0x3<<8)
#define RCC_BDCR_RTCSEL_LSE	(0x1<<8)
#define RCC_BDCR_RTCSEL_LSI	(0x2<<8)
#define RCC_BDCR_RTCSEL_HSE	(0x3<<8)
#define RCC_BDCR_RTCEN		(0x1<<15)
#define RCC_BDCR_BDRST		(0x1<<16)	/* Backup domain sw reset */

#define	RCC_CSR_MSISRANGE	(0xf<<8)
#define RCC_CSR_LSION		(1<<0)


#define RCC_BASE	(AHB1PERIPH_BASE + 0x1000)
#define RCC		((RCC_TypeDef *)(RCC_BASE))

/* General Purpose IO */
typedef struct {
__IO uint32_t MODER;	/* 0x00 = Mode Register */
__IO uint32_t OTYPER;   /* 0x04 = Output Type Register */
__IO uint32_t OSPEEDR;	/* 0x08 = Output Speed Register */
__IO uint32_t PUPDR;	/* 0x0C = Pull Up/Pull Down Register */
__IO uint32_t IDR;	/* 0x10 = Input Data Register */
__IO uint32_t ODR;	/* 0x14 = Output Data Register */
__IO uint32_t BSRR;	/* 0x18 = Bit Set/Reset Register (LOW) */
__IO uint32_t LCKR;	/* 0x1C = Configuration Lock Register */
__IO uint32_t AFR[2];	/* 0x20 = Alternate Function (Low/High) Registers */
__IO uint32_t BRR;
__IO uint32_t ASCR;
} GPIO_TypeDef;

#define GPIOA_BASE	(AHB2PERIPH_BASE + 0x0000)
#define GPIOA ((GPIO_TypeDef *)GPIOA_BASE)

#define GPIOB_BASE	(AHB2PERIPH_BASE + 0x0400)
#define GPIOB ((GPIO_TypeDef *)GPIOB_BASE)

#define GPIOC_BASE	(AHB2PERIPH_BASE + 0x0800)
#define GPIOC ((GPIO_TypeDef *)GPIOC_BASE)

#define GPIOD_BASE	(AHB2PERIPH_BASE + 0x0C00)
#define GPIOD ((GPIO_TypeDef *)GPIOD_BASE)

#define GPIOE_BASE	(AHB2PERIPH_BASE + 0x1000)
#define GPIOE ((GPIO_TypeDef *)GPIOE_BASE)

#define MODER_INPUT	0x00
#define MODER_OUTPUT	0x01
#define MODER_ALT	0x02
#define MODER_ANALOG	0x03

#define MODER_SET(_s,_v)	(((_v&0x3) << (_s*2))

#define FLASH_BASE	((uint32_t)0x08000000U)

#define RESET	0


/* RTC -- Chapter 38.6 (p1238)*/
typedef struct {
__IO uint32_t TR;	/* 0x00 = Time Register */
__IO uint32_t DR;	/* 0x04 = Date Register */
__IO uint32_t CR;	/* 0x08 = Control Register */
__IO uint32_t ISR;	/* 0x0C = Init and Status Register */
__IO uint32_t PRER;	/* 0x10 = Prescalar Register */
__IO uint32_t WUTR;	/* 0x14 = Wakeup Time Register */
__IO uint32_t RESERVED0;/* 0x18 */
__IO uint32_t ALRMAR;	/* 0x1C = Alarm A register */
__IO uint32_t ALRMBR;	/* 0x20 = Alarm B register */
__IO uint32_t WPR;	/* 0x24 = Write Protect register */
} RTC_TypeDef;

#define RTC_BASE	(APB1PERIPH_BASE + 0x2800)
#define RTC ((RTC_TypeDef *)RTC_BASE)

/* LCD -- Chapter 25.6 (p767,p790) */
typedef struct {
__IO uint32_t CR;		/* 0x00 = Control Register */
__IO uint32_t FCR;		/* 0x04 = Frame Control Register */
__IO uint32_t SR;		/* 0x08 = Status Register */
__IO uint32_t CLR;		/* 0x0C = Clear Register */
__IO uint32_t RESERVED0;	/* 0x10 = Unused */
__IO uint32_t RAM[16];		/* 0x14 - 0x50 */
} LCD_TypeDef;

#define LCD_BASE	(APB1PERIPH_BASE + 0x2400)
#define LCD ((LCD_TypeDef *)LCD_BASE)

/* p790 */
#define LCD_CR_LCDEN		(0x1<<0)
#define LCD_CR_VSEL		(0x1<<1)
#define LCD_CR_DUTY_MASK	(0x7<<2)
#define LCD_CR_DUTY_1_4		(0x3<<2)
#define LCD_CR_BIAS_MASK	(0x3<<5)
#define LCD_CR_BIAS_1_3		(0x2<<5)
#define LCD_CR_MUX_SEG		(0x1<<7)


/* p791 */
#define LCD_FCR_PON_MASK	(0x7<<4)
#define LCD_FCR_PON7		(0x7<<4)
#define LCD_FCR_CC_MASK		(0x7<<10)
#define LCD_FCR_CC_LCD7		(0x7<<10)

/* p794 */
#define LCD_SR_ENS		(0x1<<0)
#define LCD_SR_UDR		(0x1<<2)
#define LCD_SR_UDD		(0x1<<3)
#define LCD_SR_RDY		(0x1<<4)
#define LCD_SR_FCRSF		(0x1<<5)




/* Power -- 5.4 p184 */
typedef struct {
__IO uint32_t CR1;	/* 0x00 = Control Register 1 */
__IO uint32_t CR2;	/* 0x04 = Control Register 2 */
__IO uint32_t CR3;	/* 0x08 = Control Register 3 */
__IO uint32_t CR4;	/* 0x0C = Control Register 4 */
} PWR_TypeDef;

/* Table 1 p80 */
#define PWR_BASE	(APB1PERIPH_BASE + 0x7000)
#define PWR ((PWR_TypeDef *)PWR_BASE)

#define PWR_CR1_DBP	(1<<8)	/* Disable Backup Write Protection */


/************************************************************/
/************************************************************/
/* Cortex-M4 stuff, see the
	Cortex-M4 Devices Generic User Guide DUI0553.pdf    */
/************************************************************/
/************************************************************/

typedef struct {
__IO uint32_t CTRL;	/* Control / Status register */
__IO uint32_t LOAD;	/* Reload value register */
__IO uint32_t VAL;	/* Current value register */
__I  uint32_t CALIB;	/* Calibration register */
} SysTick_TypeDef;

#define SysTick_BASE	0xE000E010
#define SysTick		((SysTick_TypeDef *)SysTick_BASE)

#define SysTick_CTRL_ENABLE_Msk		0x1
#define SysTick_CTRL_TICKINT_Msk	0x2
#define SysTick_CTRL_CLKSOURCE_Msk	0x4




/* Cortex-M4 Processor Exceptions Numbers */
#define NonMaskableInt_IRQn	-14	// Non Maskable Interrupt
#define	HardFault_IRQn		-13	// Memory Management Interrupt
#define	MemoryManagement_IRQn	-12	// Memory Management Interrupt
#define	BusFault_IRQn		-11	// Bus Fault Interrupt
#define UsageFault_IRQn		-10	// Usage Fault Interrupt
#define	SVCall_IRQn		-5	// SV Call Interrupt
#define	DebugMonitor_IRQn	-4	// Debug Monitor Interrupt
#define	PendSV_IRQn		-2	// Pend SV Interrupt
#define	SysTick_IRQn		-1	// System Tick Interrupt
#define TIM4_IRQn 	30	// TIM4 Interrupt

#define	__NVIC_PRIO_BITS	4       // STM32L4XX uses 4 Bits for
					// the interrupt Priority Levels


typedef struct {
__IO uint32_t 	ISER[8];	// 0x000 (R/W) Interrupt Set Enable Register
     uint32_t 	RESERVED0[24];	// Reserved
__IO uint32_t	ICER[8];	// 0x080 (R/W) Interrupt Clear Enable Register
     uint32_t 	RSERVED1[24];	// Reserved
__IO uint32_t	ISPR[8];	// 0x100 (R/W) Interrupt Set Pending Register
     uint32_t 	RESERVED2[24];	// Reserved
__IO uint32_t	ICPR[8];	// 0x180 (R/W) Interrupt Clear Pending Register
     uint32_t	RESERVED3[24];	// Reserved
__IO uint32_t   IABR[8];	// 0x200 (R/W) Interrupt Active bit Register
     uint32_t	RESERVED4[56];	// Reserved
__IO uint8_t	IP[240];	// 0x300 (R/W) Interrupt Priority Register (8Bit wide)
     uint32_t 	RESERVED5[644];	// Reserved
__O  uint32_t	STIR;		// Offset: 0xE00 ( /W) Software Trigger Interrupt Register
} NVIC_TypeDef;


#define NVIC_BASE	0xE000E100
#define NVIC		((NVIC_TypeDef *)NVIC_BASE)



typedef struct {
     uint32_t	RESERVED1[2];	// Reserved
__IO uint32_t	ACTLR;		// 0xE000E008 Auxiliary Control Register
     uint32_t   RESERVED2[0xd00-12];
__I  uint32_t	CPUID;		// 0xE000ED00 CPUID
__IO uint32_t	ICSR;		// 0xE000ED04 Interrupt Control and State Register
__IO uint32_t	VTOR;		// 0xE000ED08 Vector Table Offset Register
__IO uint32_t	AIRCR;		// 0xE000ED0C Application Interrupt and Reset Control Register
__IO uint32_t	SCR;		// 0xE000ED10 System Control Register
__IO uint32_t	CCR;		// 0xE000ED14 Configuration and Control Register
__IO uint8_t	SHP[12];	// 0xE000ED18 System Handler Priority Register
__IO uint32_t   SHCRS;		// 0xE000ED24 System Handler Control and State Register
/* ... there's more */
} SCB_TypeDef;

#define SCB_BASE	0xE000E000
#define SCB		((SCB_TypeDef *)SCB_BASE)




/* Advanced timers: TIM1/TIM8 */
/* Chapter 30, page 907 */

typedef struct {
__IO uint32_t	CR1;		// 0x00 TIM1 Control Register
__IO uint32_t	CR2;		// 0x04 TIM1 Control Register 2
__IO uint32_t	SMCR;		// 0x08 TIM1 Slave mode control register
__IO uint32_t	DIER;		// 0x0C TIM1 DMA/interrupt enable register
__IO uint32_t	SR;		// 0x10 TIM1 Status Register
__IO uint32_t	EGR;		// 0x14 TIM1 Event Generation Register
__IO uint32_t	CCMR1;		// 0x18 TIM1 Capture/Compare Mode R1
__IO uint32_t	CCMR2;		// 0x1C TIM1 Capture/Compare Mode R2
__IO uint32_t	CCER;		// 0x20 TIM1 Capture/Compare Enable Register
__IO uint32_t	CNT;		// 0x24 TIM1 Counter Register
__IO uint32_t	PSC;		// 0x28 TIM1 Prescaler
__IO uint32_t	ARR;		// 0x2C TIM1 Auto-reload
__IO uint32_t	RCR;		// 0x30 TIM1 Reptition Count
__IO uint32_t	CCR1;		// 0x34 TIM1 Capture/Compare R1
__IO uint32_t	CCR2;		// 0x38 TIM1 Capture/Compare R2
__IO uint32_t	CCR3;		// 0x3C TIM1 Capture/Compare R3
__IO uint32_t	CCR4;		// 0x40 TIM1 Capture/Compare R4
__IO uint32_t	BDTR;		// 0x44 TIM1 Break and Dead Time register
__IO uint32_t	DCR;		// 0x48 TIM1 DMA Control Register
__IO uint32_t	DMAR;		// 0x4C TIM1 DMA Address
__IO uint32_t	OR1;		// 0x50 TIM1 Option Register 1
__IO uint32_t	CCMR3;		// 0x54 TIM1 Capure/Compare Mode R3
__IO uint32_t	CCR5;		// 0x58 TIM1 Capture/Compare R5
__IO uint32_t	CCR6;		// 0x5C TIM1 Capture/Compare R6
__IO uint32_t	OR2;		// 0x60 TIM1 Option Register 2
__IO uint32_t	OR3;		// 0x64 TIM1 Option Register 3

} TIM1_TypeDef;

// p 79
#define TIM1_BASE	(APB2PERIPH_BASE + 0x2C00)
#define TIM1		((TIM1_TypeDef *)TIM1_BASE)

#define TIM8_BASE	(APB2PERIPH_BASE + 0x3400)
#define TIM8		((TIM1_TypeDef *)TIM8_BASE)

#define TIM_CR1_CEN	(1<<0)		// Counter Enable
#define TIM_CR1_UDIS	(1<<1)		// Update disable
#define TIM_CR1_URS	(1<<2)		// Update request source
#define TIM_CR1_OPM	(1<<3)		// One pulse mode
#define TIM_CR1_DIR	(1<<4)		// Direction
#define TIM_CR1_CMS_MASK (3<<5)		// Center aligned mode
#define TIM_CR1_ARPE	(1<<7)		// Auto-reload-preload enable
#define TIM_CR1_CKD	(3<<8)		// Clock division
#define TIM_CR1_UIFREMAP (1<<9)		// UIF status bit remapping

#define TIM_CCMR1_OC1M	((1<<16) | (1<<6) | (1<<5) | (1<<4))
#define TIM_CCMR1_OC1M_0	(1<<4)
#define TIM_CCMR1_OC1M_1	(1<<5)
#define TIM_CCMR1_OC1M_2	(1<<6)
#define TIM_CCMR1_OC1M_3	(1<<16)
#define TIM_CCMR1_OC2M ((1<<24) | (1<<14) | (1<<13) | (1<<12))
#define TIM_CCMR1_OC2M_0 (1<<12)
#define TIM_CCMR1_OC2M_1 (1<<13)
#define TIM_CCMR1_OC2M_2 (1<<14) 
#define TIM_CCMR1_OC2M_3 (1<<24)
#define TIM_CCMR1_OC1PE		(1<<3)	// preload enable
#define TIM_CCMR1_OC2PE		(1<<11)

#define TIM_CCER_CC1E		(1<<0)	// enable
#define TIM_CCER_CC1P		(1<<1)	// polarity
#define TIM_CCER_CC1NE		(1<<2)	// complement enable
#define TIM_CCER_CC1NP		(1<<3)	// complement polarity
#define TIM_CCER_CC2E		(1<<4)	// enable
#define TIM_CCER_CC2P		(1<<5)	// polarity
#define TIM_CCER_CC2NE		(1<<6)	// complement enable
#define TIM_CCER_CC2NP		(1<<7)	// complement polarity
#define TIM_CCER_CC3E		(1<<8)	// enable
#define TIM_CCER_CC3P		(1<<9)	// polarity
#define TIM_CCER_CC3NE		(1<<10)	// complement enable
#define TIM_CCER_CC3NP		(1<<11)	// complement polarity
#define TIM_CCER_CC4E		(1<<12)	// enable
#define TIM_CCER_CC4P		(1<<13)	// polarity
//#define TIM_CCER_CC4NE	RESERVED	// complement enable
#define TIM_CCER_CC4NP		(1<<15)	// complement polarity
#define TIM_CCER_CC5E		(1<<16)	// enable
#define TIM_CCER_CC5P		(1<<17)	// polarity
//#define TIM_CCER_CC5NE	RESERVED	// complement enable
//#define TIM_CCER_CC5NP	RESERVED	// complement polarity
#define TIM_CCER_CC6E		(1<<20)	// enable
#define TIM_CCER_CC6P		(1<<21)	// polarity
//#define TIM_CCER_CC6NE	RESERVED	// complement enable
//#define TIM_CCER_CC6NP	RESERVED	// complement polarity


#define TIM_BDTR_AOE	(1<<14) // automatic output enable
#define TIM_BDTR_MOE	(1<<15)	// main output enable

/* General purpose timers: TIM2/TIM3/TIM4/TIM5 */
/* Chapter 31, page 1012 */

typedef struct {
__IO uint32_t	CR1;		// 0x00 TIM1 Control Register
__IO uint32_t	CR2;		// 0x04 TIM1 Control Register 2
__IO uint32_t	SMCR;		// 0x08 TIM1 Slave mode control register
__IO uint32_t	DIER;		// 0x0C TIM1 DMA/interrupt enable register
__IO uint32_t	SR;		// 0x10 TIM1 Status Register
__IO uint32_t	EGR;		// 0x14 TIM1 Event Generation Register
__IO uint32_t	CCMR1;		// 0x18 TIM1 Capture/Compare Mode R1
__IO uint32_t	CCMR2;		// 0x1C TIM1 Capture/Compare Mode R2
__IO uint32_t	CCER;		// 0x20 TIM1 Capture/Compare Enable Register
__IO uint32_t	CNT;		// 0x24 TIM1 Counter Register
__IO uint32_t	PSC;		// 0x28 TIM1 Prescaler
__IO uint32_t	ARR;		// 0x2C TIM1 Auto-reload
__IO uint32_t	RCR;		// 0x30 TIM1 Reptition Count
__IO uint32_t	CCR1;		// 0x34 TIM1 Capture/Compare R1
__IO uint32_t	CCR2;		// 0x38 TIM1 Capture/Compare R2
__IO uint32_t	CCR3;		// 0x3C TIM1 Capture/Compare R3
__IO uint32_t	CCR4;		// 0x40 TIM1 Capture/Compare R4
__IO uint32_t	BDTR;		// 0x44 TIM1 Break and Dead Time register
__IO uint32_t	DCR;		// 0x48 TIM1 DMA Control Register
__IO uint32_t	DMAR;		// 0x4C TIM1 DMA Address
__IO uint32_t	OR1;		// 0x50 TIM1 Option Register 1
__IO uint32_t	CCMR3;		// 0x54 TIM1 Capure/Compare Mode R3
__IO uint32_t	CCR5;		// 0x58 TIM1 Capture/Compare R5
__IO uint32_t	CCR6;		// 0x5C TIM1 Capture/Compare R6
__IO uint32_t	OR2;		// 0x60 TIM1 Option Register 2
__IO uint32_t	OR3;		// 0x64 TIM1 Option Register 3

} TIM4_TypeDef;

#define TIM4_BASE (APB1PERIPH_BASE + 0x0800)
#define TIM4 ((TIM4_TypeDef *)TIM4_BASE)

#define TIM4_CR1_CEN (1<<0)

#define TIM4_DIER_UIE (1<<0)
#define TIM4_DIER_CC1IE (1<<1)

#define TIM4_SR_UIF (1<<0)
#define TIM4_SR_CC1IF (1<<1)

#define TIM4_CCMR1_CC1S ((1<<1) | (1<<0))
#define TIM4_CCMR1_CC1S_0 (1<<0)
#define TIM4_CCMR1_CC1S_1 (1<<1)
#define TIM4_CCMR1_IC1PSC ((1<<3) | (1<<2))
#define TIM4_CCMR1_IC1F ((1<<7) | (1<<6) | (1<<5) | (1<<4))

#define TIM4_CCER_CC1E (1<<0)
#define TIM4_CCER_CC1P (1<<1)
#define TIM4_CCER_CC1NP (1<<3)

