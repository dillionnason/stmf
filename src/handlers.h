#ifndef HANDLERS_H
#define HANDLERS_H

#include "stm32l476xx.h"

void nmi_handler(void);
void hardfault_handler(void);

void Reset_Handler(void);

#endif // HANDLERS_H
