#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

/*
 ******************************************************************************
 * GPIO INITIALIZATION 																												*
 ******************************************************************************
 */

/**
 * Function to initialize a GPIO pin in a specified port in digital input mode 
 * @param port character argument for which port to initialize (i.e 'A')
 * @param pin integer argument for which pin
 * @param pupd floating (0x00), pull-up (0x01), pull-down (0x10)
 */
void GPIO_Input(char port, int pin, int pupd);

/**
 * Function to initialize a GPIO pin in a specified port in digital output mode 
 * @param port character argument for which port to initialize (i.e 'A')
 * @param pin integer argument for which pin
 * @param otype push-pull (0x0), open-drain (0x1)
 * @param opseed low speed (0x00), medium speed (0x01), high speed (0x10), very high speed (0x11)
 * @param pupd floating (0x00), pull-up (0x01), pull-down (0x10)
 */
void GPIO_Output(char port, int pin, int otype, int ospeed, int pupd);

/**
 * Function to initialize a GPIO pin in a specified port in analog I/O mode 
 * @param port character argument for which port to initialize (i.e 'A')
 * @param pin integer argument for which pin
 */
void GPIO_Analog(char port, int pin);

/**
 * Function to initialize a GPIO pin in a specified port in alternate function mode 
 * @param port character argument for which port to initialize (i.e 'A')
 * @param pin integer argument for which pin
 * @param otype push-pull (0x0), open-drain (0x1)
 * @param afunction 4 bit integer argument for alternate function (documented in datasheet) 
 * @param pupd floating (0x00), pull-up (0x01), pull-down (0x10)
 */
void GPIO_Alt_Function(char port, int pin, int otype, int afunction, int pupd);

/*
 ******************************************************************************
 * GPIO INPUT/OUTPUT 																											   	*
 ******************************************************************************
 */


#endif // GPIO_H
