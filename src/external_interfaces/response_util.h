/*
 * commands_util.h
 *
 * Created: 2015-01-18 11:34:44
 *  Author: jnil02
 */ 


#ifndef RESPONSE_UTIL_H_
#define RESPONSE_UTIL_H_

#include <stdint.h>
#include <stdbool.h>

#define OUTPUT_DIVIDER_MASK 0x07
#define OUTPUT_PULL_MASK    0x20
#define STATE_OUTPUT_IF_SIZE 10

void state_output(uint8_t from,uint8_t mode,uint8_t state_id);

void state_output_if_setup(uint8_t flag_state_id,uint8_t from,uint8_t mode,uint8_t* state_ids,uint32_t wait_cycles);
void state_output_if_state_add(uint8_t state_id,uint8_t place);
void state_output_if(void);
void state_output_once_if(void);
void state_output_once_force(void);
void state_output_if_counter(void);

void restor_proc_sequ_if_setup(uint8_t flag_state_id);
void restor_proc_sequ_if(void);

#endif /* RESPONSE_UTIL_H_ */