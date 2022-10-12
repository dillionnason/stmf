#include "stm32l476xx.h"

void TIM1_Servo_Init(void) {
	/* Counting direction: 0 = Up-counting, 1 = Down-counting */
	TIM1->CR1 &= ~TIM_CR1_DIR;	// Select up-counting

	/* Prescaler, slow down the input clock by a factor of (1 + prescaler) */
	TIM1->PSC = 39; 	// 4MHz / (1 + 39) = 100KHz

	/* Auto-reload */
	TIM1->ARR = 1999;	// PWM period = (1999 + 1) * 1/100Khz = 0.02s

	/* Clear output compare bits for channel 1 */
	TIM1->CCMR1 &= TIM_CCMR1_OC1M;

	/* Select PWM Mode 1 output on Channel 1 (OC1M = 110) */
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;

	/* Output 1 preload enable */
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE;

	/* Select output polarity: 0 = active high, 1 = active low */
	TIM1->CCER &= ~TIM_CCER_CC1NP; 	// OC1N = OCREF + CC1NP

	/* Enable complementary output of channel 1 (CH1N) */
	TIM1->CCER |= TIM_CCER_CC1NE;

	/* Main output enable (MOE): 0 = Disable, 1 = Enable */
	TIM1->BDTR |= TIM_BDTR_MOE;

	/* Output Compare Register for channel 1 */
	TIM1->CCR1 = 150;		// Initial duty cycle 7.5% (center)

	/* Enable counter */
	TIM1->CR1 |= TIM_CR1_CEN;
}

void TIM1_LED_Init(void) {
	/* Counting direction: 0 = Up-counting, 1 = Down-counting */
	TIM1->CR1 &= ~TIM_CR1_DIR;	// Select up-counting

	/* Prescaler, slow down the input clock by a factor of (1 + prescaler) */
	TIM1->PSC = 39; 	// 4MHz / (1 + 39) = 100KHz

	/* Auto-reload */
	TIM1->ARR = 999;	// PWM period = (999 + 1) * 1/100Khz = 0.01s

	/* Clear output compare bits for channel 1 */
	TIM1->CCMR1 &= TIM_CCMR1_OC1M;

	/* Select PWM Mode 1 output on Channel 1 (OC1M = 110) */
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;

	/* Output 1 preload enable */
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE;

	/* Select output polarity: 0 = active high, 1 = active low */
	TIM1->CCER &= ~TIM_CCER_CC1NP; 	// OC1N = OCREF + CC1NP

	/* Enable complementary output of channel 1 (CH1N) */
	TIM1->CCER |= TIM_CCER_CC1NE;

	/* Main output enable (MOE): 0 = Disable, 1 = Enable */
	TIM1->BDTR |= TIM_BDTR_MOE;

	/* Output Compare Register for channel 1 */
	TIM1->CCR1 = 500;		// Initial duty cycle 50%

	/* Enable counter */
	TIM1->CR1 |= TIM_CR1_CEN;
}

void TIM1_Pulse_Init(void) {
	/* Counting direction: 0 = Up-counting, 1 = Down-counting */
	TIM1->CR1 &= ~TIM_CR1_DIR;	// Select up-counting

	/* Prescaler, slow down the input clock by a factor of (1 + prescaler) */
	TIM1->PSC = 15; 	// 16MHz / (1 + 15) = 1MHz (1 microsecond)

	/* Auto-reload for largest period, largest 16-bit value */
	TIM1->ARR = 0xFFFF;

	/* Clear output compare bits for channel 2
	 * Select PWM Mode 1 output on Channel 1 (OC1M = 110) */
	TIM1->CCMR1 &= ~TIM_CCMR1_OC2M;
	TIM1->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;

	/* Output 2 preload enable */
	TIM1->CCMR1 |= TIM_CCMR1_OC2PE;

	/* Select output polarity: 0 = active high, 1 = active low */
	TIM1->CCER &= ~TIM_CCER_CC2P;

	/* Enable output of channel 2 */
	TIM1->CCER |= TIM_CCER_CC2E;

	/* Main output enable (MOE): 0 = Disable, 1 = Enable */
	TIM1->BDTR |= TIM_BDTR_MOE;

	/* Output Compare Register for channel 2 
	 * For a 10 microsecond pulse on 1MHz clock */ 
	TIM1->CCR2 = 10;		// Initial duty cycle 0.01%

	/* Enable counter */
	TIM1->CR1 |= TIM_CR1_CEN;
}


void TIM4_Init(void) {
	/* Prescalar to divide 16MHz to 1MHz */
	TIM4->PSC = 15;

	/* TIM4->ARR to maximum 16bit value */
	TIM4->ARR = 0xFFFF;

	/* Direction of channel 1 as input (CC1S = 0x01) */
	TIM4->CCMR1 &= ~TIM4_CCMR1_CC1S; // Clear capture/compare 1 selection bits
	TIM4->CCMR1 |= TIM4_CCMR1_CC1S_0;

	/* Input filter duration to 0 (TIM_CCMR1_IC1F) */
	TIM4->CCMR1 &= ~TIM4_CCMR1_IC1F;

	/* Set the capture to be on both the rising and falling (11) 
	 * Set CC1P and CC1NP in TIM4->CCER */
	TIM4->CCER |= TIM4_CCER_CC1P | TIM4_CCER_CC1NP;

	/* Clear input prescalar to capture each transition
	 * Clear IC1PSC in TIM4->CCMR1 */
	TIM4->CCMR1 &= ~TIM4_CCMR1_IC1PSC;

	/* Enable capture for Channel 1 
	 * CC1E in TIM4->CCER */
	TIM4->CCER |= TIM4_CCER_CC1E;

	/* Enable capture interrpt generation for Channel 1
	 * CC1IE in TIM4->DIER */
	TIM4->DIER |= TIM4_DIER_CC1IE;

	/* Enable overflow interrupt 
	 * UIE in TIM4->DIER */
	TIM4->DIER |= TIM4_DIER_UIE;

	/* Enable Timer4
	 * CEN in TIM4->CR1 */
	TIM4->CR1 |= TIM4_CR1_CEN;
}
