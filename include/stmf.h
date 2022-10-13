#ifndef STMF_H
#define STMF_H

#include "../src/handlers.h"
#include "../src/gpio.h"

#define STACK_LOCATION (0x20000000+(96*1024))

/* Vector Table */
uint32_t *myvectors[128]
__attribute__ ((section(".isr_vector"))) = {
	(uint32_t *) STACK_LOCATION, 		/*   0:0  = stack pointer      */
	(uint32_t *) Reset_Handler,			/* -15:4  = code entry point   */
	(uint32_t *) nmi_handler,				/* -14:8  = NMI handler        */
	(uint32_t *) hardfault_handler,	/* -13:12 = hard fault handler */
	(uint32_t *) nmi_handler,				/* -12:16 = MemManage   	     */
	(uint32_t *) nmi_handler,				/* -11:20 = BusFault      	   */
	(uint32_t *) nmi_handler,				/* -10:24 = UsageFault       	 */
	(uint32_t *) nmi_handler,				/*  -9:28 = Reserved        	 */
	(uint32_t *) nmi_handler,				/*  -8:32 = Reserved        	 */
	(uint32_t *) nmi_handler,				/*  -7:36 = Reserved        	 */
	(uint32_t *) nmi_handler,				/*  -6:40 = Reserved        	 */
	(uint32_t *) nmi_handler,				/*  -5:44 = SVC Handler        */
	(uint32_t *) nmi_handler,				/*  -4:48 = Debugmon        	 */
	(uint32_t *) nmi_handler,				/*  -3:52 = Reserved           */
	(uint32_t *) nmi_handler,				/*  -2:54 = PendSV             */
};

#endif // STMF_H
