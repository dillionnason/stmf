#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include "stm32l476xx.h"

/*
 ******************************************************************************
 * GPIO INITIALIZATION 																												*
 ******************************************************************************
 */

/**
 * Function to initialize a GPIO pin in a specified port and mode 
 * @param port which port to initialize (GPIO_TypeDef*, i.e GPIOA, GPIOB, etc.)
 * @param pin integer argument for which pin
 * @param mode input (0x00), output (0x01), alternate function (0x02), analog (0x03)
 * @param otype push-pull (0x0), open-drain (0x1)
 * @param ospeed low speed (0x00), medium speed (0x01), high speed (0x10), very high speed (0x11) 
 * @param pupd floating (0x00), pull-up (0x01), pull-down (0x10)
 * @param alt_func 4 bit integer argument for alternate function (documented in datasheet) 
 */
void GPIO_Init(GPIO_TypeDef* port, uint8_t pin, uint8_t mode, uint8_t otype, uint8_t ospeed, uint8_t pupd, uint8_t alt_func);

/*
 ******************************************************************************
 * GPIO INPUT/OUTPUT 																											   	*
 ******************************************************************************
 */

/**
 * Function to return the 32-bit value in a GPIO pins Input Data Register 
 * @param port which port to initialize (GPIO_TypeDef*, i.e GPIOA, GPIOB, etc.)
 * @param pin integer argument for which pin
 * @return 32 bit integer value in Input Data Register
 */ 
uint32_t GPIO_Digital_Read(GPIO_TypeDef* port, uint32_t pin);

/**
 * Function to write a 32-bit value to a GPIO pins Output Data Register 
 * @param port which port to initialize (GPIO_TypeDef*, i.e GPIOA, GPIOB, etc.)
 * @param pin integer argument for which pin
 * @param value 32 bit integer value to write to Output Data Register
 */ 
void GPIO_Digital_Write(GPIO_TypeDef* port, uint32_t pin, uint32_t value);


#endif // GPIO_H
