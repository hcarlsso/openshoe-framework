
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#include "imu_interrupt.h"
#include "user_board.h"
#include "timer_interrupt.h"

#ifdef BT_MODULE
#  include "bt_uart_interrupt.h"
#endif
#ifdef EXT_UART_MODULE
#  include "ext_uart_interrupt.h"
#endif

void interrupt_init(void){
	
	// For some reason this has to be run before timer_interrupt_init.
	// Don't know why. Probably because I run either Disable_global_interrupt() or INTC_init_interrupts().
	#ifdef BT_MODULE
	bt_interrupt_init();
	#endif /* BT_MODULE */
	#ifdef EXT_UART_MODULE
	ext_uart_interrupt_init();
	#endif /* EXT_UART_MODULE */
	
	timer_interrput_init();
}