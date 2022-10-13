#include <stddef.h>
#include "stm32l476xx.h"

void GPIO_Init(char port, int pin, int mode, int otype, int afunction, int pupd) {
	GPIO_TypeDef* GPIO;

	switch(port) {
		case 'A':
			GPIO = GPIOA;
			break;
		case 'B':
			GPIO = GPIOB;
			break;
		case 'C':
			GPIO = GPIOC;
			break;
		case 'D':
			GPIO = GPIOD;
			break;
		case 'E':
			GPIO = GPIOE;
			break;
		default:
			GPIO = NULL;
			break;
	}

	if (GPIO == NULL) return;
	
	/* Mode setting */
	GPIO->MODER &= ~(0x3  << pin*2);
	GPIO->MODER |=  (mode << pin*2);

	/* Output type or alternate function */
	if (mode == MODER_OUTPUT) 
		GPIO->OTYPER &= ~(otype << pin);
	
	else if (mode == MODER_ALT) {
		if (pin < 8) {
			GPIO->AFR[0] &= ~(0xF 		  << pin*4);
			GPIO->AFR[0] |=  (afunction << pin*4);
		} else {
			GPIO->AFR[1] &= ~(0xF 		  << (pin%8)*4);
			GPIO->AFR[1] |=  (afunction << (pin%8)*4);
		}
	}

	/* Pull up/pull down */
	GPIO->PUPDR &= ~(pupd << pin*2);
}
