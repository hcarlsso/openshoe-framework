

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
#include "ADIS16367_interface.h"
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
#define NR_DEBUG_OUTPUT 5

#define MREPEAT_ARG_CONST(narg,size) size,

///  \name Command response functions
///  Functions which are executed in response to commands.  
//@{
extern void handle_ack(uint8_t**);
void get_mcu_serial(uint8_t**);
void input_imu_rd(uint8_t**);
void setup_debug_processing(uint8_t**);
void output_state(uint8_t**);
void output_imu_rd(uint8_t**);
void output_imu_temp(uint8_t**);
void turn_off_output(uint8_t**);
void toggle_inertial_output(uint8_t**);
void position_plus_zupt(uint8_t**);
void output_navigational_states(uint8_t**);
void processing_onoff(uint8_t**);
void reset_zupt_aided_ins(uint8_t**);
void reset_zupt_aided_ins2(uint8_t**);
void stepwise_dead_reckoning(uint8_t**);
void stepwise_dead_reckoning_TOR(uint8_t**);
void reset_swdr_gyrocal(uint8_t**);
void gyro_self_calibration(uint8_t**);
void acc_calibration(uint8_t**);
void set_low_pass_imu(uint8_t**);
void add_sync_output(uint8_t**);
void sync_output(uint8_t**);
void processing_off(uint8_t**);
void do_nothing(uint8_t**);
void start_inertial_frontend(uint8_t**);
//@}

///  \name Command definitions
///  Structs containing the information/definitions of the commands.
//@{
static command_structure ack = {ACK_ID,&handle_ack,2,1,{2}};
static command_structure ping = {PING_ID,&do_nothing,0,0,{0}};
static command_structure mcu_id = {MCU_ID,&get_mcu_serial,0,0,{0}};
static command_structure input_imu_rd_cmd = {INPUT_IMU_RD,&input_imu_rd,4+NR_IMUS*6*sizeof(int16_t),NR_IMUS+1,{4,MREPEAT(NR_IMUS, MREPEAT_ARG_CONST, 12)}};
static command_structure setup_debug_proc_cmd = {SETUP_DEBUG_PROC,&setup_debug_processing,NR_DEBUG_PROC+NR_DEBUG_OUTPUT,2,{NR_DEBUG_PROC,NR_DEBUG_OUTPUT}};
static command_structure output_onoff_state = {OUTPUT_STATE,&output_state,2,2,{1,1}};
static command_structure output_all_off = {OUTPUT_ALL_OFF,&turn_off_output,0,0,{0}};
static command_structure output_onoff_inert = {OUTPUT_ONOFF_INERT,&toggle_inertial_output,1,1,{1}};
static command_structure output_position_plus_zupt = {OUTPUT_POSITION_PLUS_ZUPT,&position_plus_zupt,1,1,{1}};
static command_structure output_navigational_states_cmd = {OUTPUT_NAVIGATIONAL_STATES,&output_navigational_states,1,1,{1}};
static command_structure output_onoff_imu_rd = {OUTPUT_IMU_RD,&output_imu_rd,5,2,{4,1}};
static command_structure output_onoff_imu_temp = {OUTPUT_IMU_TEMP,&output_imu_temp,5,2,{4,1}};
static command_structure processing_function_onoff = {PROCESSING_FUNCTION_ONOFF,&processing_onoff,3,3,{1,1,1}};
static command_structure reset_system_cmd = {RESET_ZUPT_AIDED_INS,&reset_zupt_aided_ins,0,0,{0}};
static command_structure stepwise_dead_reckoning_cmd = {STEPWISE_DEAD_RECKONING,&stepwise_dead_reckoning,0,0,{0}};
static command_structure stepwise_dead_reckoning_TOR_cmd = {STEPWISE_DEAD_RECKONING_TOR,&stepwise_dead_reckoning_TOR,1,1,{1}};
static command_structure reset_swdr_gyrocal_cmd = {RESET_SWDR_GYROCAL,&reset_swdr_gyrocal,1,1,{1}};
static command_structure gyro_calibration_cmd = {GYRO_CALIBRATION_INIT,&gyro_self_calibration,0,0,{0}};
static command_structure acc_calibration_cmd = {ACC_CALIBRATION_INIT,&acc_calibration,1,1,{1}};
static command_structure set_low_pass_imu_cmd = {SET_LOWPASS_FILTER_IMU,&set_low_pass_imu,1,1,{1}};
static command_structure add_sync_output_cmd = {ADD_SYNC_OUTPUT,&add_sync_output,2,2,{1,1}};
static command_structure sync_output_cmd = {SYNC_OUTPUT,&sync_output,0,0,{0}};
static command_structure processing_off_cmd = {PROCESSING_OFF,&processing_off,0,0,{0}};
static command_structure mimu_frontend_cmd = {MIMU_FRONTEND,&start_inertial_frontend,0,0,{0}};
//@}

// Arrays/tables to find appropriate commands
static const command_structure* commands[] = {&ack,
											  &ping,
											  &mcu_id,
											  &input_imu_rd_cmd,
											  &setup_debug_proc_cmd,
											  &output_onoff_state,
											  &output_all_off,
											  &output_onoff_inert,
											  &output_position_plus_zupt,
											  &output_navigational_states_cmd,
											  &output_onoff_imu_rd,
											  &output_onoff_imu_temp,
											  &processing_function_onoff,
											  &reset_system_cmd,
											  &stepwise_dead_reckoning_cmd,
											  &stepwise_dead_reckoning_TOR_cmd,
											  &reset_swdr_gyrocal_cmd,
											  &gyro_calibration_cmd,
											  &acc_calibration_cmd,
											  &set_low_pass_imu_cmd,
											  &add_sync_output_cmd,
											  &sync_output_cmd,
											  &processing_off_cmd,
											  &mimu_frontend_cmd};
												  
// Arrays/tables for commands
uint8_t command_header_table[32]={0};
command_structure* command_info_array[256]={NULL};
												  
void commands_init(void){
	// Initialize tables
	for(int i = 0;i<(sizeof(commands)/sizeof(commands[0]));i++){
		command_header_table[ (commands[i]->header) >> 3 ] |= 1<<( (commands[i]->header) & 7);}
		
	for(int i = 0;i<(sizeof(commands)/sizeof(commands[0]));i++){
		command_info_array[commands[i]->header] = commands[i];}
}

void get_mcu_serial(uint8_t** arg){
//	udi_cdc_write_buf((int*)0x80800284,0x80800292-0x80800284);
}

void do_nothing(uint8_t** arg){;}
	
void input_imu_rd(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	// Sets time stamp and imu_rd (raw data) states
	memcpy(state_info_access_by_id[IMU_TS_SID]->state_p,cmd_arg[1],state_info_access_by_id[IMU_TS_SID]->state_size);
	for (int i=0; i<NR_IMUS; i++)
		memcpy(state_info_access_by_id[IMU0_RD_SID+i]->state_p,cmd_arg[i+2],state_info_access_by_id[IMU0_RD_SID+i]->state_size);
	// Restores process sequence which has presumably been setup by setup_debug_processing(..)
	// TODO: Make sure some debug processing has been setup
	restore_process_sequence();
	// Sets one-time output of up to 4 states
	// TODO: remove this (better to get the data through a separate call, the value of the states should not change without processing anyway)
// 	set_conditional_output(cmd_arg[NR_IMUS+2][0],from);
// 	set_conditional_output(cmd_arg[NR_IMUS+2][1],from);
// 	set_conditional_output(cmd_arg[NR_IMUS+2][2],from);
// 	set_conditional_output(cmd_arg[NR_IMUS+2][3],from);
}

uint8_t debug_output[NR_DEBUG_OUTPUT];
uint8_t debug_from;
void set_debug_output(void){
	for(int i=0;i<NR_DEBUG_OUTPUT;i++)
		set_conditional_output(debug_output[i],debug_from);
}
void setup_debug_processing(uint8_t** cmd_arg){
	debug_from = (uint8_t)cmd_arg[0];
	empty_process_sequence();
	for (int i=0; i<NR_DEBUG_PROC; i++)
		set_elem_in_process_sequence_by_id(cmd_arg[1][i],i);
	for (int i=0; i<NR_DEBUG_OUTPUT; i++)
		debug_output[i]=cmd_arg[2][i];
	set_elem_in_process_sequence(&set_debug_output,NR_DEBUG_PROC);
	set_last_process_sequence_element(&store_and_empty_process_sequence);
	store_and_empty_process_sequence();
}

void output_state(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	uint8_t state_id = cmd_arg[1][0];
	uint8_t output_divider = cmd_arg[2][0];
	// TODO: remove if-statement. This is now checked in set_state_output
	if(state_info_access_by_id[state_id]){  // Valid state?
		set_state_output(state_id,output_divider,from);}}
		
void output_imu_rd(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	uint32_t imu_selector = cmd_arg[1][0]<<24;
	imu_selector |= cmd_arg[1][1]<<16;
	imu_selector |= cmd_arg[1][2]<<8;
	imu_selector |= cmd_arg[1][3];
	uint8_t output_divider = cmd_arg[2][0];
	for (uint8_t i=0;i<32;i++) {
		if( (imu_selector>>i)&1 )
		set_state_output(IMU0_RD_SID+i,output_divider,from);
	}
	set_state_output(IMU_TS_SID,output_divider,from);
}
void output_imu_temp(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	uint32_t imu_selector = cmd_arg[1][0]<<24;
	imu_selector |= cmd_arg[1][1]<<16;
	imu_selector |= cmd_arg[1][2]<<8;
	imu_selector |= cmd_arg[1][3];
	uint8_t output_divider = cmd_arg[2][0];
	for (uint8_t i=0;i<32;i++) {
		if( (imu_selector>>i)&1 )
		set_state_output(IMU0_TEMP_SID+i,output_divider,from);
	}
}

void toggle_inertial_output(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	uint8_t output_divider = cmd_arg[1][0];
	set_state_output(ANGULAR_RATE_SID,output_divider,from);
	set_state_output(SPECIFIC_FORCE_SID,output_divider,from);
	set_state_output(INTERRUPT_COUNTER_SID,output_divider,from);
	set_state_output(IMU_DT_SID,output_divider,from);}

void position_plus_zupt(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	uint8_t output_divider = cmd_arg[1][0];
	set_state_output(POSITION_SID,output_divider,from);
	set_state_output(ZUPT_SID,output_divider,from);}

void output_navigational_states(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	uint8_t output_divider = cmd_arg[1][0];
	set_state_output(POSITION_SID,output_divider,from);
	set_state_output(VELOCITY_SID,output_divider,from);
	set_state_output(QUATERNION_SID,output_divider,from);
	set_state_output(INTERRUPT_COUNTER_SID,output_divider,from);}

void turn_off_output(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	for(uint8_t i = 0; i<max(SID_LIMIT,0xFF); i++){
		set_state_output(i,0,from);}
	if (from & COMMAND_FROM_BT)
		empty_package_queue();
}

void processing_onoff(uint8_t** cmd_arg){
	uint8_t function_id = cmd_arg[1][0];
	uint8_t onoff    = cmd_arg[2][0];
	uint8_t array_location = cmd_arg[3][0];
	processing_function_p process_sequence_elem_value = onoff ? (processing_functions_by_id[function_id]->func_p) : NULL;
	set_elem_in_process_sequence(process_sequence_elem_value,array_location);
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



///\cond
extern uint32_t nr_of_inital_alignment_samples;
///\endcond
void reset_swdr_gyrocal(uint8_t** cmd_arg){
	// Stop whatever was going on
	empty_process_sequence();
	initialize_flag=true;
	nr_of_inital_alignment_samples = 100*cmd_arg[1][0];
	// Set filter taps in IMU (since occasionally they seem to reset themselves)
	uint8_t log2_nr_filter_taps = 0;
	low_pass_filter_setting(log2_nr_filter_taps);
	// Start initial alignment
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_PREPROC]->func_p,0);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_STATDET]->func_p,1);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_POSTPROC]->func_p,2);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_INITIAL_ALIGNMENT]->func_p,3);
	// Set termination function of initial alignment which will also start INS
	set_last_process_sequence_element(&start_stepwise_dead_reckoning);
}

void gyro_self_calibration(uint8_t** no_arg){
	store_and_empty_process_sequence();
	set_elem_in_process_sequence(processing_functions_by_id[GYRO_CALIBRATION]->func_p,0);
	set_last_process_sequence_element(&restore_process_sequence);
}
	
///\cond
extern Bool new_orientation_flag;
extern Bool acc_calibration_finished_flag;
///\endcond
void new_calibration_orientation(void){
	if(acc_calibration_finished_flag){
		acc_calibration_finished_flag = false;
		restore_process_sequence();
		udi_cdc_putc(103);
		//TODO: Send out acknowledgment that calibration was successful
	}
	if(new_orientation_flag==true){
		new_orientation_flag = false;
		//TODO: Send out request for new orientation position
		empty_process_sequence();
		udi_cdc_putc(102);
	}
}

///\cond
extern uint8_t nr_of_calibration_orientations;
///\endcond
void acc_calibration(uint8_t ** cmd_arg){
	uint8_t nr_orientations = cmd_arg[1][0];
	nr_of_calibration_orientations = nr_orientations;
	
	store_and_empty_process_sequence();
	set_elem_in_process_sequence(processing_functions_by_id[ACCELEROMETER_CALIBRATION]->func_p,0);
	set_last_process_sequence_element(&new_calibration_orientation);
}

void set_low_pass_imu(uint8_t ** cmd_arg){
	uint8_t log2_nr_filter_taps = cmd_arg[1][0];
	if (log2_nr_filter_taps<=4){
		low_pass_filter_setting(log2_nr_filter_taps);}
}

void add_sync_output(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	uint8_t state_id = cmd_arg[1][0];
	uint8_t output_divider    = cmd_arg[2][0];
	if(state_info_access_by_id[state_id]){  // Valid state?
		set_state_output(state_id,output_divider,from);}
	reset_output_counters(from);
}

void sync_output(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	reset_output_counters(from);
}

void processing_off(uint8_t** no_arg){
	// Stop whatever was going on
	empty_process_sequence();
}

void start_inertial_frontend(uint8_t** no_arg){
	empty_process_sequence();
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_PREPROC]->func_p,0);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_STATDET]->func_p,1);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_POSTPROC]->func_p,2);
}

//@}