/*
 * interrupt.c
 *
 * Created: 2014-03-12 11:55:11
 *  Author: jnil02
 */ 

#include "imu_interrupt.h"
#include "user_board.h"

#if defined(OPENSHOE_CLASSIC)
#include "toggle_interrupt.h"
#elif defined(MIMU3333) || defined(MIMU22BT)
#include "timer_interrupt.h"
#endif

void imu_interrupt_init(void){
	#if defined(OPENSHOE_CLASSIC)
	toggle_interrupt_init();
	#elif defined(MIMU3333) || defined(MIMU22BT)
	timer_interrput_init();
	#endif	
}