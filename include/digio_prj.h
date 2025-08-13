#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

#include "hwdefs.h"

/* Here you specify generic IO pins, i.e. digital input or outputs.
 * Inputs can be floating (INPUT_FLT), have a 30k pull-up (INPUT_PU)
 * or pull-down (INPUT_PD) or be an output (OUTPUT)

*/
#define DIG_IO_LIST \
    DIG_IO_ENTRY(led_out,     GPIOC, GPIO13, PinMode::OUTPUT)	\
    DIG_IO_ENTRY(P_LED,       GPIOB, GPIO3,  PinMode::OUTPUT) 	\
	DIG_IO_ENTRY(R_LED,       GPIOB, GPIO4,  PinMode::OUTPUT) 	\
	DIG_IO_ENTRY(N_LED,       GPIOB, GPIO5,  PinMode::OUTPUT) 	\
	DIG_IO_ENTRY(D_LED,       GPIOB, GPIO6,  PinMode::OUTPUT) 	\
    DIG_IO_ENTRY(BkLT_LED,    GPIOB, GPIO0,  PinMode::OUTPUT) 	\
	DIG_IO_ENTRY(M1_CW,       GPIOA, GPIO10, PinMode::OUTPUT) 	\
	DIG_IO_ENTRY(M1_CCW,      GPIOA, GPIO11, PinMode::OUTPUT) 	\
	DIG_IO_ENTRY(M2_CW,       GPIOA, GPIO8,  PinMode::OUTPUT) 	\
	DIG_IO_ENTRY(M2_CCW,      GPIOA, GPIO9,  PinMode::OUTPUT) 	\
	DIG_IO_ENTRY(CS1,     	  GPIOA, GPIO3,  PinMode::OUTPUT) 	\
	DIG_IO_ENTRY(CS2,     	  GPIOA, GPIO4,  PinMode::OUTPUT) 	\
	
#endif // PinMode_PRJ_H_INCLUDED