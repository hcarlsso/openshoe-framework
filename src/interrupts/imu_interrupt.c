/*
 * interrupt.c
 *
 * Created: 2014-03-12 11:55:11
 *  Author: jnil02
 */ 

#include "imu_interrupt.h"
#include "bt_uart_interrupt.h"
#include "user_board.h"

#if defined(OPENSHOE_CLASSIC)
#include "toggle_interrupt.h"
#elif defined(MIMU3333) || defined(MIMU22BT) || defined(MIMU4444)
#include "timer_interrupt.h"
#endif

void interrupt_init(void){
	
	// For some reason this has to be run before timer_interrupt_init.
	// Don't know why. Probably because I run either Disable_global_interrupt() or INTC_init_interrupts().
	#ifdef BT_MODULE
	bt_interrupt_init();
	#endif /* BT_MODULE */
	
	#if defined(OPENSHOE_CLASSIC)
	toggle_interrupt_init();
	#elif defined(MIMU3333) || defined(MIMU22BT) || defined(MIMU4444)
	timer_interrput_init();
	#endif
}