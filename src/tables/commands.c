

/** \file
	\brief Definition of user commands and command response functions.
	
	\details This file contains definition of user commands used to control
	the OpenShoe system. Each command is defined in a command info struct
	containing an ID, a pointer to a response function, and information about
	the arguemetns of the command.
	
	The command response functions are the functions which will be exectued as
	a response to the command being called. These functions are also declared
	and defined in this file.
	
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
*/ 


///	\addtogroup control_tables
///	@{

#include "control_tables.h"
#include <string.h>

// Needed include for definition of command response functions
#include "external_interface.h"
#include "bluetooth_interface.h"
#include "process_sequence.h"
#include "udi_cdc.h"
#include "package_queue.h"

#if defined(OPENSHOE_CLASSIC)
#  define NR_IMU_RD 6
#  include "classic.h"
#elif defined(MIMU3333)
#  define NR_IMU_RD 108
#  include "MIMU3333.h"
#elif defined(MIMU4444)
#  define NR_IMU_RD 192
#  include "MIMU4444.h"
#elif defined(MIMU22BT)
#  define NR_IMU_RD 24
#  include "MIMU22BT.h"
#endif

#define NR_DEBUG_PROC 8
#define NR_DEBUG_OUTPUT 8

#define MREPEAT_ARG_CONST(narg,size) size,

///  \name Command response functions
///  Functions which are executed in response to commands.  
//@{
extern void handle_ack(uint8_t**);
void do_nothing(uint8_t**);
void get_mcu_serial(uint8_t**);
void input_imu_rd(uint8_t**);
void setup_debug_processing(uint8_t**);
void output_state(uint8_t**);
void output_multiple_states(uint8_t**);
void output_imu_rd(uint8_t**);
void turn_off_output(uint8_t**);
void set_proc(uint8_t**);
void set_mult_proc(uint8_t**);
void reset_zupt_aided_ins(uint8_t**);
void reset_zupt_aided_ins2(uint8_t**);
void stepwise_dead_reckoning(uint8_t**);
void stepwise_dead_reckoning_TOR(uint8_t**);
void processing_off(uint8_t**);
void start_inertial_frontend(uint8_t**);
void normal_imu(uint8_t**);
void normal_imu_with_bias_est(uint8_t**);
//@}

///  \name Command definitions
///  Structs containing the information/definitions of the commands.
//@{
static command_info ack = {ACK_ID,&handle_ack,2,1,{2}};
static command_info ping = {PING_ID,&do_nothing,0,0,{0}};
static command_info mcu_id = {MCU_ID,&get_mcu_serial,0,0,{0}};
static command_info input_imu_rd_cmd = {INPUT_IMU_RD,&input_imu_rd,4+NR_IMUS*6*sizeof(int16_t),NR_IMUS+1,{4,MREPEAT(NR_IMUS, MREPEAT_ARG_CONST, 12)}};
static command_info setup_debug_proc_cmd = {SETUP_DEBUG_PROC,&setup_debug_processing,NR_DEBUG_PROC+NR_DEBUG_OUTPUT+1,2,{NR_DEBUG_PROC,NR_DEBUG_OUTPUT,1}};
static command_info output_onoff_state = {OUTPUT_STATE,&output_state,2,2,{1,1}};
static command_info output_multiple_states_cmd = {OUTPUT_MULTIPLE_STATES,&output_multiple_states,9,9,{1,1,1,1,1,1,1,1,1}};
static command_info output_all_off = {OUTPUT_ALL_OFF,&turn_off_output,0,0,{0}};
static command_info output_onoff_imu_rd = {OUTPUT_IMU_RD,&output_imu_rd,5,2,{4,1}};
static command_info processing_function_onoff = {RUN_PROC,&set_proc,2,2,{1,1}};
static command_info run_mult_proc_cmd = {RUN_MULT_PROC,&set_mult_proc,8,8,{1,1,1,1,1,1,1,1}};
static command_info reset_system_cmd = {RESET_ZUPT_AIDED_INS,&reset_zupt_aided_ins,0,0,{0}};
static command_info stepwise_dead_reckoning_cmd = {STEPWISE_DEAD_RECKONING,&stepwise_dead_reckoning,0,0,{0}};
static command_info stepwise_dead_reckoning_TOR_cmd = {STEPWISE_DEAD_RECKONING_TOR,&stepwise_dead_reckoning_TOR,1,1,{1}};
static command_info processing_off_cmd = {PROCESSING_OFF,&processing_off,0,0,{0}};
static command_info mimu_frontend_cmd = {MIMU_FRONTEND,&start_inertial_frontend,0,0,{0}};
static command_info normal_imu_cmd = {NORMAL_IMU,&normal_imu,1,1,{1}};
static command_info normal_imu_with_bias_est_cmd = {NORMAL_IMU_WITH_BIAS_EST,&normal_imu_with_bias_est,1,1,{1}};
//@}

// Arrays/tables to find appropriate commands
static const command_info* commands[] = {&ack,
											  &ping,
											  &mcu_id,
											  &input_imu_rd_cmd,
											  &setup_debug_proc_cmd,
											  &output_onoff_state,
											  &output_all_off,
											  &output_multiple_states_cmd,
											  &output_onoff_imu_rd,
											  &processing_function_onoff,
											  &reset_system_cmd,
											  &stepwise_dead_reckoning_cmd,
											  &stepwise_dead_reckoning_TOR_cmd,
											  &processing_off_cmd,
											  &mimu_frontend_cmd};
												  
// Arrays/tables for commands
uint8_t command_header_table[32]={0};
command_info* command_info_array[256]={NULL};
												  
void commands_init(void){
	// Initialize tables
	for(int i = 0;i<(sizeof(commands)/sizeof(commands[0]));i++){
		command_header_table[ (commands[i]->header) >> 3 ] |= 1<<( (commands[i]->header) & 7);}
		
	for(int i = 0;i<(sizeof(commands)/sizeof(commands[0]));i++){
		command_info_array[commands[i]->header] = commands[i];}
}

void get_mcu_serial(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	set_conditional_output(MCU_ID_SID,from);
}

void do_nothing(uint8_t** arg){;}
	
void input_imu_rd(uint8_t** cmd_arg){
	// Sets time stamp and imu_rd (raw data) states
	memcpy(state_info_access_by_id[IMU_TS_SID]->state_p,cmd_arg[1],state_info_access_by_id[IMU_TS_SID]->state_size);
	for (int i=0; i<NR_IMUS; i++)
		memcpy(state_info_access_by_id[IMU0_RD_SID+i]->state_p,cmd_arg[i+2],state_info_access_by_id[IMU0_RD_SID+i]->state_size);
	// Restores process sequence which has presumably been setup by setup_debug_processing(..)
	restore_process_sequence();
}

uint8_t debug_output[NR_DEBUG_OUTPUT];
uint8_t debug_output_to;
void set_debug_output(void){
	for(int i=0;i<NR_DEBUG_OUTPUT;i++)
		set_conditional_output(debug_output[i],debug_output_to);
}
void setup_debug_processing(uint8_t** cmd_arg){
	empty_process_sequence();
	for (int i=0; i<NR_DEBUG_PROC; i++)
		set_elem_in_process_sequence_by_id(cmd_arg[1][i],i);
	for (int i=0; i<NR_DEBUG_OUTPUT; i++)
		debug_output[i]=cmd_arg[2][i];
	debug_output_to = cmd_arg[3][0];
	set_elem_in_process_sequence(&set_debug_output,NR_DEBUG_PROC);
	set_last_process_sequence_element(&store_and_empty_process_sequence);
	store_and_empty_process_sequence();
}

#define OUTPUT_DIVIDER_MASK 0x0F
#define OUTPUT_PULL_MASK    0x20
#define OUTPUT_LOSSY_MASK   0x10
void output_state(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	uint8_t state_id = cmd_arg[1][0];
	uint8_t mode_select = cmd_arg[2][0];
	set_lossy_transmission(!(mode_select & OUTPUT_LOSSY_MASK),from);
	if (mode_select & OUTPUT_PULL_MASK)
		set_conditional_output(state_id,from);
	else
		set_state_output(state_id,mode_select & OUTPUT_DIVIDER_MASK,from);
}
void output_multiple_states(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	uint8_t mode_select = cmd_arg[9][0];
	set_lossy_transmission(!(mode_select & OUTPUT_LOSSY_MASK),from);
	if (mode_select & OUTPUT_PULL_MASK)
		for (int i=1;i<=9;i++)
			set_conditional_output(cmd_arg[i][0],from);
	else
		for (int i=1;i<=9;i++)
			set_state_output(cmd_arg[i][0],mode_select & OUTPUT_DIVIDER_MASK,from);
}

void turn_off_output(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	for(uint8_t i = 0; i<max(SID_LIMIT,0xFF); i++){
	set_state_output(i,0,from);}
	if (from & COMMAND_FROM_BT)
	empty_package_queue();
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
	if (mode_select & OUTPUT_PULL_MASK)	{
		for (uint8_t i=0;i<32;i++) {
			if( (imu_selector>>i)&1 ){
				if (mode_select & OUTPUT_INERTIAL_MASK)
					set_conditional_output(IMU0_RD_SID+i,from);
				if (mode_select & OUTPUT_TEMP_MASK)
					set_conditional_output(IMU0_TEMP_SID+i,from);
			}
		}
	} else {
		uint8_t output_divider = mode_select & OUTPUT_DIVIDER_MASK;
		for (uint8_t i=0;i<32;i++) {
			if( (imu_selector>>i)&1 ){
				if (mode_select & OUTPUT_INERTIAL_MASK)
					set_state_output(IMU0_RD_SID+i,output_divider,from);
				if (mode_select & OUTPUT_TEMP_MASK)
					set_state_output(IMU0_TEMP_SID+i,output_divider,from);
			}
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
void processing_off(uint8_t** no_arg){
	empty_process_sequence();
}

///\cond
extern Bool initialize_flag;
///\endcond
void start_zupt_aided_ins(void){
	if(initialize_flag==false){
		// Stop initial alignment
		empty_process_sequence();
		// Start ZUPT-aided INS
		set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_PREPROC]->func_p,0);
		set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_STATDET]->func_p,1);
		set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_POSTPROC]->func_p,2);
		set_elem_in_process_sequence(processing_functions_by_id[MECHANIZATION]->func_p,3);
		set_elem_in_process_sequence(processing_functions_by_id[TIME_UPDATE]->func_p,4);
		set_elem_in_process_sequence(processing_functions_by_id[ZUPT_UPDATE]->func_p,5);
	}
}
void reset_zupt_aided_ins(uint8_t** no_arg){
	// Stop whatever was going on
	empty_process_sequence();
	initialize_flag=true;
	// Start initial alignment
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_PREPROC]->func_p,0);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_STATDET]->func_p,1);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_POSTPROC]->func_p,2);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_INITIAL_ALIGNMENT]->func_p,3);
	// Set termination function of initial alignment which will also start INS
	set_last_process_sequence_element(&start_zupt_aided_ins);
}

///\cond
extern uint16_t step_counter;
extern Bool filter_reset_flag;
uint8_t swdr_interface=0;
///\endcond
// If filter has been reset, sets the reset states to be output. Used in process sequence.
void set_conditional_output_reset(void){
	if(filter_reset_flag){
		set_conditional_output(DX_SID,swdr_interface);
		set_conditional_output(DP_SID,swdr_interface);
		set_conditional_output(STEP_COUNTER_SID,swdr_interface);
	}
}
void start_stepwise_dead_reckoning(void){
	if(initialize_flag==false){
		// Stop initial alignment
		empty_process_sequence();
		// Reset step counter;
		step_counter=0;
		// Make sure we have error-resistant transmission
		set_lossy_transmission(false,swdr_interface);
		// Start ZUPT-aided INS
		// TODO: change to set_elem_in_process_sequence_by_id
		set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_PREPROC]->func_p,0);
		set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_STATDET]->func_p,1);
		set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_POSTPROC]->func_p,2);
		set_elem_in_process_sequence(processing_functions_by_id[STEPWISE_SYSTEM_RESET]->func_p,3);
		set_elem_in_process_sequence(processing_functions_by_id[MECHANIZATION]->func_p,4);
		set_elem_in_process_sequence(processing_functions_by_id[TIME_UPDATE]->func_p,5);
		set_elem_in_process_sequence(processing_functions_by_id[ZUPT_UPDATE]->func_p,6);
		set_elem_in_process_sequence(&set_conditional_output_reset,7);
	}
}
void stepwise_dead_reckoning(uint8_t** cmd_arg){
	swdr_interface |= (uint8_t)cmd_arg[0];
	empty_process_sequence();
	initialize_flag=true;
#if defined(OPENSHOE_CLASSIC)
	// Set filter taps in IMU (since occasionally they seem to reset themselves)
	uint8_t log2_nr_filter_taps = 0;
	low_pass_filter_setting(log2_nr_filter_taps);
#endif
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_PREPROC]->func_p,0);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_STATDET]->func_p,1);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_POSTPROC]->func_p,2);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_INITIAL_ALIGNMENT]->func_p,3);
	set_last_process_sequence_element(&start_stepwise_dead_reckoning);
}


uint8_t swdr_via2=0;
void set_conditional_output_reset_TOR(void){
	if(filter_reset_flag){
		set_conditional_output(DX_SID,swdr_via2);
		set_conditional_output(DP_SID,swdr_via2);
//		set_conditional_output(INTERRUPT_COUNTER_SID);
		set_conditional_output(STEP_COUNTER_SID,swdr_via2);
		set_conditional_output(SAMSUNG_ID_SID,swdr_via2);
	}
}
//extern uint16_t step_counter;
void start_stepwise_dead_reckoning_TOR(void){
		if(initialize_flag==false){
		// Stop initial alignment
		empty_process_sequence();
		// Reset step counter;
		step_counter=0;
		// Start ZUPT-aided INS
		set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_PREPROC]->func_p,0);
		set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_STATDET]->func_p,1);
		set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_POSTPROC]->func_p,2);
		set_elem_in_process_sequence(processing_functions_by_id[STEPWISE_SYSTEM_RESET]->func_p,3);
		set_elem_in_process_sequence(processing_functions_by_id[MECHANIZATION]->func_p,4);
		set_elem_in_process_sequence(processing_functions_by_id[TIME_UPDATE]->func_p,5);
		set_elem_in_process_sequence(processing_functions_by_id[ZUPT_UPDATE]->func_p,6);
		set_elem_in_process_sequence(&set_conditional_output_reset_TOR,8);
	}
}

uint8_t samsung_id;
void stepwise_dead_reckoning_TOR(uint8_t** cmd_arg){
	swdr_via2 |= (uint8_t)cmd_arg[0];
	samsung_id = cmd_arg[1][0];
	// Stop whatever was going on
	empty_process_sequence();
	initialize_flag=true;
#if defined(OPENSHOE_CLASSIC)
	// Set filter taps in IMU (since occasionally they seem to reset themselves)
	uint8_t log2_nr_filter_taps = 0;
	low_pass_filter_setting(log2_nr_filter_taps);
#endif
	// Start initial alignment
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_PREPROC]->func_p,0);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_STATDET]->func_p,1);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_POSTPROC]->func_p,2);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_INITIAL_ALIGNMENT]->func_p,3);
	// Set termination function of initial alignment which will also start INS
	set_last_process_sequence_element(&start_stepwise_dead_reckoning_TOR);
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
	set_lossy_transmission(!(mode_select & OUTPUT_LOSSY_MASK),from);
	if (mode_select & OUTPUT_PULL_MASK)
		set_conditional_output(U_K_SID,from);
	else
		set_state_output(U_K_SID,mode_select & OUTPUT_DIVIDER_MASK,from);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_PREPROC]->func_p,0);
}
void normal_imu_with_bias_est(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	uint8_t mode_select = cmd_arg[1][0];
	set_lossy_transmission(!(mode_select & OUTPUT_LOSSY_MASK),from);
	if (mode_select & OUTPUT_PULL_MASK)
		set_conditional_output(U_K_SID,from);
	else
		set_state_output(U_K_SID,mode_select & OUTPUT_DIVIDER_MASK,from);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_PREPROC]->func_p,0);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_STATDET]->func_p,1);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_POSTPROC]->func_p,2);
}
//@}