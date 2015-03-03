
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#ifndef TIMING_CONTROL_H_
#define TIMING_CONTROL_H_

#include <stdint.h>
#include <compiler.h>

extern uint32_t interrupt_counter;
extern uint32_t interrupt_ts;
extern uint32_t gp_dt;

void wait_for_interrupt(void);
bool time_is_up(void);
void end_of_main_loop(void);


#endif /* TIMING_CONTROL_H_ */