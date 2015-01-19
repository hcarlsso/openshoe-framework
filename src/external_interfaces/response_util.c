/*
 * commands_util.c
 *
 * Created: 2015-01-18 11:34:22
 *  Author: jnil02
 */ 

#include "response_util.h"
#include "external_interface.h"
#include "process_sequence.h"
#include "control_tables.h"
#include <string.h>

// SET OUTPUT GIVEN MODE BYTE
void state_output(uint8_t from,uint8_t mode,uint8_t state_id){
	if (mode & OUTPUT_PULL_MASK)
		set_conditional_output(state_id,from);
	else
		set_state_output(state_id,mode & OUTPUT_DIVIDER_MASK,from);
}

static uint8_t state_output_if_from;
static uint8_t state_output_if_mode;
static uint8_t state_output_if_states[8];
static bool* state_output_if_flagp;
void state_output_if_setup(uint8_t flag_state_id,uint8_t from,uint8_t mode,uint8_t* state_ids){
	state_output_if_flagp = (bool*) get_state_p(flag_state_id);
	state_output_if_from = from;
	state_output_if_mode = mode;
	if(state_ids)
		memcpy(state_output_if_states,state_ids,sizeof(state_output_if_states));
}
void state_output_if_state_add(uint8_t state_id,uint8_t place){
	if(place<sizeof(state_output_if_states))
		state_output_if_states[place]=state_id;
}
void state_output_if(void){
	if(state_output_if_flagp && *state_output_if_flagp){
		for (int i=0;i<sizeof(state_output_if_states);i++)
			state_output(state_output_if_from,state_output_if_mode,state_output_if_states[i]);
	}
}
void state_output_once_if(void){
	if(state_output_if_flagp && *state_output_if_flagp){
		state_output_if();
		*state_output_if_flagp=false;
	}
}
void state_output_once_force(void){
	for (int i=0;i<sizeof(state_output_if_states);i++)
		state_output(state_output_if_from,state_output_if_mode,state_output_if_states[i]);
}

// CONDITIONAL OUTPUT IF
// array med state_IDs
// Pekare till något tillstånd som "if->..."
//
// Setup function
// Process_sequence function

static bool* restor_proc_sequ_if_flagp;
void restor_proc_sequ_if_setup(uint8_t flag_state_id){
	restor_proc_sequ_if_flagp = (bool*) get_state_p(flag_state_id);
}
void restor_proc_sequ_if(void){
	if(restor_proc_sequ_if_flagp && *restor_proc_sequ_if_flagp)
		restore_process_sequence();
}
