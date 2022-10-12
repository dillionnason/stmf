/* ECE271 Lab8 -- PWM */

#include <stdint.h>
#include "stm32l476xx.h"

void NVIC_SetPriority(int irq, int priority) {

	if (irq<0) {
		// for -1, 0xff -> f -> -4 = b (11)
		SCB->SHP[(((uint8_t)irq)&0xf)-4]=(priority<<4)&0xff;
	}
	else {
		NVIC->IP[irq]=(priority<<4)&0xff;
	}

	return;
}

void SysTick_Init(int ticks) {
	/* Disable SysTick */
	SysTick->CTRL = 0;

	/* Set reload register */ 
	if (ticks>0)
		SysTick->LOAD = ticks-1;

	/* Set the interrupt priority of SysTick
	 * Makes it least urgent
	 * __NVIC_PRIO_BITS: number of bits for priority levels */
	NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	
	/* Reset counter value */
	SysTick->VAL = 0;

	/* Select processor clock
	 * 1 = processor clock; 0 = external clock */
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

	/* Enable the SysTick exception request
	 * 1 = count down to zero asserts exception request
	 * 0 = count down to zero does not assert exception request */
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

	/* Enables the SysTick timer */
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}
