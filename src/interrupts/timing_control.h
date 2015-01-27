
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#ifndef TIMING_CONTROL_H_
#define TIMING_CONTROL_H_

#include <stdint.h>

extern uint32_t interrupt_counter;
extern uint32_t interrupt_ts;
extern uint32_t gp_dt;

void wait_for_interrupt(void);
void within_time_limit(void);


#endif /* TIMING_CONTROL_H_ */