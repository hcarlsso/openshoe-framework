/*
 * response_functions.c
 *
 * Created: 2015-01-18 19:33:34
 *  Author: jnil02
 */ 

// Needed include for definition of command response functions
#include "response_functions.h"
#include "response_util.h"
#include "external_interface.h"
#include "bluetooth_interface.h"
#include "process_sequence.h"
#include "package_queue.h"
#include "control_tables.h"

void do_nothing(uint8_t** arg){;}

void get_mcu_serial(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	set_conditional_output(MCU_ID_SID,from);
}

void input_imu_rd(uint8_t** cmd_arg){
	set_state(IMU_TS_SID,(void*)cmd_arg[1]);
	for (int i=0; i<NR_IMUS; i++)
		set_state(IMU0_RD_SID+i,(void*)cmd_arg[i+2]);
	restore_process_sequence(); // Presumably setup by setup_debug_processing(..);
	state_output_once_force();
}

void setup_debug_processing(uint8_t** cmd_arg){
	empty_process_sequence();
	state_output_if_setup(NOSTATE_SID,cmd_arg[3][0],OUTPUT_PULL_MASK,NULL,0);
	for (int i=0; i<NR_DEBUG_PROC; i++)
		set_elem_in_process_sequence_by_id(cmd_arg[1][i],i);
	for (int i=0; i<NR_DEBUG_OUTPUT; i++)
		state_output_if_state_add(cmd_arg[2][i],i);
	set_elem_in_process_sequence_by_id(STATE_OUTPUT_ONCE_IF,NR_DEBUG_PROC);
	set_elem_in_process_sequence_by_id(STORE_AND_EMPTY_PROC_SEQU,NR_DEBUG_PROC+1);
	store_and_empty_process_sequence();
}
static void check_size_and_set_state(uint8_t size,uint8_t** cmd_arg){
	if(get_state_size(cmd_arg[1][0])<=size)
	set_state(cmd_arg[1][0],cmd_arg[2]);
}
void set_state_max_1byte(uint8_t** cmd_arg){ check_size_and_set_state(1,cmd_arg); }
void set_state_max_4bytes(uint8_t** cmd_arg){ check_size_and_set_state(4,cmd_arg); }
void set_state_max_12bytes(uint8_t** cmd_arg){ check_size_and_set_state(12,cmd_arg); }
void set_state_max_24bytes(uint8_t** cmd_arg){ check_size_and_set_state(24,cmd_arg); }
void set_state_max_48bytes(uint8_t** cmd_arg){ check_size_and_set_state(48,cmd_arg); }
void set_state_max_254bytes(uint8_t** cmd_arg){ check_size_and_set_state(254,cmd_arg); }

#define OUTPUT_LOSSY_MASK   0x10
void output_state(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	uint8_t state_id = cmd_arg[1][0];
	uint8_t mode_select = cmd_arg[2][0];
	set_lossy_transmission(!(mode_select & OUTPUT_LOSSY_MASK),from);
	state_output(from,mode_select,state_id);
}
void output_multiple_states(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	uint8_t mode_select = cmd_arg[9][0];
	set_lossy_transmission(!(mode_select & OUTPUT_LOSSY_MASK),from);
	for (int i=1;i<=8;i++)
		state_output(from,mode_select,cmd_arg[i][0]);
}

void all_output_off(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	for(uint8_t i = 0; i<max(SID_LIMIT,0xFF); i++)
		set_state_output(i,0,from);
	uint8_t tmp[8] = {0,0,0,0,0,0,0,0};
	state_output_if_setup(NOSTATE_SID,0,0,tmp,0);
	if (from & COMMAND_FROM_BT)
		empty_package_queue();
}
void state_if_setup_resp(uint8_t** cmd_arg){
	state_output_if_setup(cmd_arg[1][0],(uint8_t)cmd_arg[0],cmd_arg[2][0],cmd_arg[3],0);
}
void state_if_add_state(uint8_t** cmd_arg){
	state_output_if_state_add(cmd_arg[1][0],cmd_arg[2][0]);
}

#define OUTPUT_INERTIAL_MASK 0x40
#define OUTPUT_TEMP_MASK 0x80
void output_imu_rd(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	uint32_t imu_selector = cmd_arg[1][0]<<24;
	imu_selector |= cmd_arg[1][1]<<16;
	imu_selector |= cmd_arg[1][2]<<8;
	imu_selector |= cmd_arg[1][3];
	uint8_t mode_select = cmd_arg[2][0];
	set_lossy_transmission(!(mode_select & OUTPUT_LOSSY_MASK),from);
	for (uint8_t i=0;i<32;i++) {
		if( (imu_selector>>i)&1 ){
			if (mode_select & OUTPUT_INERTIAL_MASK)
				state_output(from,mode_select,IMU0_RD_SID+i);
			if (mode_select & OUTPUT_TEMP_MASK)
				state_output(from,mode_select,IMU0_TEMP_SID+i);
		}
	}
	set_state_output(IMU_TS_SID,mode_select & OUTPUT_DIVIDER_MASK,from);
}

void set_proc(uint8_t** cmd_arg){
	uint8_t function_id = cmd_arg[1][0];
	uint8_t array_location = cmd_arg[2][0];
	set_elem_in_process_sequence_by_id(function_id,array_location);
}
void set_mult_proc(uint8_t** cmd_arg){
	for(int i=1;i<=8;i++)
	set_elem_in_process_sequence_by_id(cmd_arg[i][0],i-1);
}
void all_proc_off(uint8_t** no_arg){
	empty_process_sequence();
}
void restore_proc_if_setup(uint8_t** cmd_arg){
	restor_proc_sequ_if_setup(cmd_arg[1][0]);
}

void zupt_aided_ins(uint8_t** no_arg){
	// Stop whatever was going on
	empty_process_sequence();
	all_output_off(no_arg);
	// Setup stepwise dead reckoning
	set_elem_in_process_sequence_by_id(FRONTEND_PREPROC,0);
	set_elem_in_process_sequence_by_id(FRONTEND_STATDET,1);
	set_elem_in_process_sequence_by_id(FRONTEND_POSTPROC,2);
	set_elem_in_process_sequence_by_id(MECHANIZATION,4);
	set_elem_in_process_sequence_by_id(TIME_UPDATE,5);
	set_elem_in_process_sequence_by_id(ZUPT_UPDATE,6);
	// Hide away stepwise dead reckoning while initializing
	store_and_empty_process_sequence();
	// Setup initialization
	set_elem_in_process_sequence_by_id(FRONTEND_PREPROC,0);
	set_elem_in_process_sequence_by_id(FRONTEND_STATDET,1);
	set_elem_in_process_sequence_by_id(FRONTEND_POSTPROC,2);
	set_elem_in_process_sequence_by_id(FRONTEND_INITIAL_ALIGNMENT,3);
	// Setup initialization termination actions
	bool tmp=false;
	set_state(INIT_DONE_SID,(void*)&tmp);
	restor_proc_sequ_if_setup(INIT_DONE_SID);
	set_elem_in_process_sequence_by_id(RESTORE_PROC_SEQU_IF,4);
}

void stepwise_dead_reckoning(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	// Stop whatever was going on
	empty_process_sequence();
	all_output_off(cmd_arg);
	// Make sure we have error-resistant transmission
	set_lossy_transmission(false,from);
	// Reset step counter;
	uint16_t zero_value=0;
	set_state(STEP_COUNTER_SID,(void*)&zero_value);
	// Setup stepwise dead reckoning
	set_elem_in_process_sequence_by_id(FRONTEND_PREPROC,0);
	set_elem_in_process_sequence_by_id(FRONTEND_STATDET,1);
	set_elem_in_process_sequence_by_id(FRONTEND_POSTPROC,2);
	set_elem_in_process_sequence_by_id(STEPWISE_SYSTEM_RESET,3);
	set_elem_in_process_sequence_by_id(MECHANIZATION,4);
	set_elem_in_process_sequence_by_id(TIME_UPDATE,5);
	set_elem_in_process_sequence_by_id(ZUPT_UPDATE,6);
	// Setup conditional (reset) output
	state_output_if_setup(FILTER_RESET_FLAG_SID,from,OUTPUT_PULL_MASK,NULL,0);
	state_output_if_state_add(DX_SID,0);
	state_output_if_state_add(DP_SID,1);
	state_output_if_state_add(STEP_COUNTER_SID,2);
	set_elem_in_process_sequence_by_id(STATE_OUTPUT_IF,7);
	// Hide away stepwise dead reckoning while initializing
	store_and_empty_process_sequence();
	// Setup initialization
	set_elem_in_process_sequence_by_id(FRONTEND_PREPROC,0);
	set_elem_in_process_sequence_by_id(FRONTEND_STATDET,1);
	set_elem_in_process_sequence_by_id(FRONTEND_POSTPROC,2);
	set_elem_in_process_sequence_by_id(FRONTEND_INITIAL_ALIGNMENT,3);
	// Setup initialization termination actions
	bool tmp=false;
	set_state(INIT_DONE_SID,(void*)&tmp);
	restor_proc_sequ_if_setup(INIT_DONE_SID);
	set_elem_in_process_sequence_by_id(RESTORE_PROC_SEQU_IF,4);
}

uint8_t gp_id;
void stepwise_dead_reckoning_TOR(uint8_t** cmd_arg){
	set_state(GP_ID_SID,(void*)cmd_arg[1]);
	stepwise_dead_reckoning(cmd_arg);
	state_output_if_state_add(GP_ID_SID,3);
}

void start_inertial_frontend(uint8_t** no_arg){
	empty_process_sequence();
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_PREPROC]->func_p,0);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_STATDET]->func_p,1);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_POSTPROC]->func_p,2);
}

void normal_imu(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	uint8_t mode_select = cmd_arg[1][0];
	all_output_off(cmd_arg);
	empty_process_sequence();
	set_lossy_transmission(!(mode_select & OUTPUT_LOSSY_MASK),from);
	state_output(from,mode_select,IMU_TS_SID);
	state_output(from,mode_select,U_K_SID);
	set_elem_in_process_sequence_by_id(FRONTEND_PREPROC,0);
}
void normal_imu_with_bias_est(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	uint8_t mode_select = cmd_arg[1][0];
	all_output_off(cmd_arg);
	empty_process_sequence();
	set_lossy_transmission(!(mode_select & OUTPUT_LOSSY_MASK),from);
	state_output_if_setup(NOSTATE_SID,from,mode_select,NULL,128); // Wait until buffers have filled up
	state_output_if_state_add(IMU_TS_SID,0);
	state_output_if_state_add(U_K_SID,1);
	set_elem_in_process_sequence_by_id(STATE_OUTPUT_IF_COUNTER,3);
	set_elem_in_process_sequence_by_id(FRONTEND_PREPROC,0);
	set_elem_in_process_sequence_by_id(FRONTEND_STATDET,1);
	set_elem_in_process_sequence_by_id(FRONTEND_POSTPROC,2);
}
//@}