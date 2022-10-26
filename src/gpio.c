#include <stddef.h>
#include "stm32l476xx.h"
#include "gpio.h"

/*
 ******************************************************************************
 * GPIO INITIALIZATION 																												*
 ******************************************************************************
 */

void GPIO_Init(GPIO_TypeDef* port, uint8_t pin, uint8_t mode, uint8_t otype, uint8_t ospeed, uint8_t pupd, uint8_t alt_func) {
	/* input (0x00, output (0x01), alternate function (0x02), analog (0x03) */
	port->MODER &= ~(0x3	<< pin*2);
	port->MODER |=  (mode << pin*2);

	/* push-pull (0x0), open-drain (0x1) */
	port->OTYPER &= ~(0x1   << pin);
	port->OTYPER |=  (otype << pin);

	/* low speed (0x00), medium speed (0x01), high speed (0x10), very high speed (0x11) */
	port->OSPEEDR &= ~(0x3    << pin*2);
	port->OSPEEDR |=  (ospeed << pin*2);

	/* floating (0x00), pull-up (0x01), pull-down (0x10) */
	port->PUPDR &= ~(pupd << pin*2);

	if (pin < 8) {
		port->AFR[0] &= ~(0xF 		 << pin*4);
		port->AFR[0] |=  (alt_func << pin*4);
	} else {
		port->AFR[1] &= ~(0xF 		 << (pin%8)*4);
		port->AFR[1] |=  (alt_func << (pin%8)*4);
	}
}

/*
 ******************************************************************************
 * GPIO INPUT/OUTPUT 																											   	*
 ******************************************************************************
 */

uint32_t GPIO_Digital_Read(GPIO_TypeDef* port, uint32_t pin) {
	/* Return value in Input Data Register */
	return (port->IDR & ~(0x1 << pin)) >> pin;
}

void GPIO_Digital_Write(GPIO_TypeDef* port, uint32_t pin, uint32_t value) {
	if (value > 1) return;

	port->ODR &= ~(0x1   << pin);
	port->ODR |=  (value << pin);
}
