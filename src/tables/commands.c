

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

// Needed include for definition of command response functions
#include "external_interface.h"
#include "process_sequence.h"
#include "ADIS16367_interface.h"
#include "udi_cdc.h"


///  \name Command response functions
///  Functions which are executed in response to commands.  
//@{
void output_state(uint8_t**);
void output_imu_rd(uint8_t**);
void turn_off_output(uint8_t**);
void get_mcu_serial(uint8_t**);
void toggle_inertial_output(uint8_t**);
void position_plus_zupt(uint8_t**);
void output_navigational_states(uint8_t**);
void processing_onoff(uint8_t**);
void reset_zupt_aided_ins(uint8_t**);
void reset_zupt_aided_ins2(uint8_t**);
void reset_stepwise_dead_reckoning(uint8_t**);
void reset_stepwise_dead_reckoning2(uint8_t**);
void reset_stepwise_dead_reckoning3(uint8_t**);
void reset_stepwise_dead_reckoning4(uint8_t**);
void reset_swdr_gyrocal(uint8_t**);
void gyro_self_calibration(uint8_t**);
void acc_calibration(uint8_t**);
void set_low_pass_imu(uint8_t**);
void add_sync_output(uint8_t**);
void sync_output(uint8_t**);
void processing_off(uint8_t**);

void mimu_frontend(uint8_t**);
//@}

///  \name Command definitions
///  Structs containing the information/definitions of the commands.
//@{
static command_structure only_ack = {ONLY_ACK,NULL,0,0,{0}};
static command_structure mcu_id = {MCU_ID,&get_mcu_serial,0,0,{0}};
static command_structure output_onoff_state = {OUTPUT_STATE,&output_state,2,2,{1,1}};
static command_structure output_all_off = {OUTPUT_ALL_OFF,&turn_off_output,0,0,{0}};
static command_structure output_onoff_inert = {OUTPUT_ONOFF_INERT,&toggle_inertial_output,1,1,{1}};
static command_structure output_position_plus_zupt = {OUTPUT_POSITION_PLUS_ZUPT,&position_plus_zupt,1,1,{1}};
static command_structure output_navigational_states_cmd = {OUTPUT_NAVIGATIONAL_STATES,&output_navigational_states,1,1,{1}};
static command_structure output_onoff_imu_rd = {OUTPUT_IMU_RD,&output_imu_rd,5,2,{4,1}};
static command_structure processing_function_onoff = {PROCESSING_FUNCTION_ONOFF,&processing_onoff,3,3,{1,1,1}};
static command_structure reset_system_cmd = {RESET_ZUPT_AIDED_INS,&reset_zupt_aided_ins,0,0,{0}};
static command_structure reset_stepwise_dead_reckoning_cmd = {RESET_STEPWISE_DEAD_RECKONING,&reset_stepwise_dead_reckoning,0,0,{0}};
static command_structure reset_stepwise_dead_reckoning2_cmd = {RESET_STEPWISE_DEAD_RECKONING2,&reset_stepwise_dead_reckoning2,0,0,{0}};
static command_structure reset_stepwise_dead_reckoning3_cmd = {RESET_STEPWISE_DEAD_RECKONING3,&reset_stepwise_dead_reckoning3,1,1,{1}};
static command_structure reset_stepwise_dead_reckoning4_cmd = {RESET_STEPWISE_DEAD_RECKONING4,&reset_stepwise_dead_reckoning4,0,0,{0}};
static command_structure reset_swdr_gyrocal_cmd = {RESET_SWDR_GYROCAL,&reset_swdr_gyrocal,1,1,{1}};
static command_structure gyro_calibration_cmd = {GYRO_CALIBRATION_INIT,&gyro_self_calibration,0,0,{0}};
static command_structure acc_calibration_cmd = {ACC_CALIBRATION_INIT,&acc_calibration,1,1,{1}};
static command_structure set_low_pass_imu_cmd = {SET_LOWPASS_FILTER_IMU,&set_low_pass_imu,1,1,{1}};
static command_structure add_sync_output_cmd = {ADD_SYNC_OUTPUT,&add_sync_output,2,2,{1,1}};
static command_structure sync_output_cmd = {SYNC_OUTPUT,&sync_output,0,0,{0}};
static command_structure processing_off_cmd = {PROCESSING_OFF,&processing_off,0,0,{0}};
static command_structure mimu_frontend_cmd = {MIMU_FRONTEND,&mimu_frontend,0,0,{0}};
//@}

// Arrays/tables to find appropriate commands
static const command_structure* commands[] = {&only_ack,
											  &mcu_id,
											  &output_onoff_state,
											  &output_all_off,
											  &output_onoff_inert,
											  &output_position_plus_zupt,
											  &output_navigational_states_cmd,
											  &output_onoff_imu_rd,
											  &processing_function_onoff,
											  &reset_system_cmd,
											  &reset_stepwise_dead_reckoning_cmd,
											  &reset_stepwise_dead_reckoning2_cmd,
											  &reset_stepwise_dead_reckoning3_cmd,
											  &reset_stepwise_dead_reckoning4_cmd,
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

void output_state(uint8_t** cmd_arg){
	uint8_t state_id = cmd_arg[0][0];
	uint8_t output_divider = cmd_arg[1][0];
	if(state_info_access_by_id[state_id]){  // Valid state?
		set_state_output(state_id,output_divider);}}
		
void output_imu_rd(uint8_t** cmd_arg){
	// TODO: redo this with something like below and a single for loop
//	uint32_t imu_selector = *((uint32_t*)cmd_arg[0]);
	uint8_t imu_selector0 = cmd_arg[0][3];
	uint8_t imu_selector1 = cmd_arg[0][2];
	uint8_t imu_selector2 = cmd_arg[0][1];
	uint8_t imu_selector3 = cmd_arg[0][0];
	uint8_t output_divider = cmd_arg[1][0];
	for (uint8_t i=0;i<8;i++)
	{
		if( (imu_selector0>>i)&1 )
			set_state_output(IMU0_RD_SID+i,output_divider);
	}
	for (uint8_t i=0;i<8;i++)
	{
		if( (imu_selector1>>i)&1 )
		set_state_output(IMU0_RD_SID+i+8,output_divider);
	}
	for (uint8_t i=0;i<8;i++)
	{
		if( (imu_selector2>>i)&1 )
		set_state_output(IMU0_RD_SID+i+16,output_divider);
	}
	for (uint8_t i=0;i<8;i++)
	{
		if( (imu_selector3>>i)&1 )
		set_state_output(IMU0_RD_SID+i+24,output_divider);
	}
	set_state_output(IMU_DT_SID,output_divider);
}

void toggle_inertial_output(uint8_t** cmd_arg){
	uint8_t output_divider = cmd_arg[0][0];
	set_state_output(ANGULAR_RATE_SID,output_divider);
	set_state_output(SPECIFIC_FORCE_SID,output_divider);
	set_state_output(INTERRUPT_COUNTER_SID,output_divider);
	set_state_output(IMU_DT_SID,output_divider);}

void position_plus_zupt(uint8_t** cmd_arg){
	uint8_t output_divider = cmd_arg[0][0];
	set_state_output(POSITION_SID,output_divider);
	set_state_output(ZUPT_SID,output_divider);}

void output_navigational_states(uint8_t** cmd_arg){
	uint8_t output_divider = cmd_arg[0][0];
	set_state_output(POSITION_SID,output_divider);
	set_state_output(VELOCITY_SID,output_divider);
	set_state_output(QUATERNION_SID,output_divider);
	set_state_output(INTERRUPT_COUNTER_SID,output_divider);}

void turn_off_output(uint8_t** cmd_arg){
	for(uint8_t i = 0; i<max(SID_LIMIT,0xFF); i++){
		set_state_output(i,0);}}

void processing_onoff(uint8_t** cmd_arg){
	uint8_t function_id = cmd_arg[0][0];
	uint8_t onoff    = cmd_arg[1][0];
	uint8_t array_location = cmd_arg[2][0];	
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
		set_elem_in_process_sequence(processing_functions_by_id[UPDATE_BUFFER]->func_p,0);
		set_elem_in_process_sequence(processing_functions_by_id[ZUPT_DETECTOR]->func_p,1);
		set_elem_in_process_sequence(processing_functions_by_id[UPDATE_ZARU_BUFFER]->func_p,2);
		set_elem_in_process_sequence(processing_functions_by_id[ZARU_DETECTOR]->func_p,3);
		set_elem_in_process_sequence(processing_functions_by_id[MECHANIZATION]->func_p,4);
		set_elem_in_process_sequence(processing_functions_by_id[TIME_UPDATE]->func_p,5);
		set_elem_in_process_sequence(processing_functions_by_id[ZUPT_UPDATE]->func_p,6);
	}
}

///\cond
extern Bool filter_reset_flag;
///\endcond
// If filter has been reset, sets the reset states to be output. Used in process sequence.
void set_conditional_output_reset(void){
	if(filter_reset_flag){
		set_conditional_output(DX_SID);
		set_conditional_output(DP_SID);
	}
}

// Once initialization is done, starts up the system in "Displacement sensor mode"
void start_stepwise_dead_reckoning(void){
	if(initialize_flag==false){
		// Stop initial alignment
		empty_process_sequence();
		// Start ZUPT-aided INS
		set_elem_in_process_sequence(processing_functions_by_id[UPDATE_ZARU_BUFFER]->func_p,0);
		set_elem_in_process_sequence(processing_functions_by_id[ZARU_DETECTOR]->func_p,1);
		set_elem_in_process_sequence(processing_functions_by_id[UPDATE_BUFFER2]->func_p,2);
		set_elem_in_process_sequence(processing_functions_by_id[ZUPT_DETECTOR]->func_p,3);
		set_elem_in_process_sequence(processing_functions_by_id[STEPWISE_SYSTEM_RESET]->func_p,4);
		set_elem_in_process_sequence(processing_functions_by_id[MECHANIZATION]->func_p,5);
		set_elem_in_process_sequence(processing_functions_by_id[TIME_UPDATE]->func_p,6);
		set_elem_in_process_sequence(processing_functions_by_id[ZUPT_UPDATE]->func_p,7);
//		set_elem_in_process_sequence(&set_conditional_output_reset,8);
	}
}

void reset_zupt_aided_ins(uint8_t** no_arg){
	// Stop whatever was going on
	empty_process_sequence();
	initialize_flag=true;
	// Start initial alignment
	set_elem_in_process_sequence(processing_functions_by_id[UPDATE_ZARU_BUFFER]->func_p,0);
	set_elem_in_process_sequence(processing_functions_by_id[UPDATE_BUFFER]->func_p,1);
	set_elem_in_process_sequence(processing_functions_by_id[INITIAL_ALIGNMENT]->func_p,2);
	// Set termination function of initial alignment which will also start INS
	set_last_process_sequence_element(&start_zupt_aided_ins);
}

void reset_stepwise_dead_reckoning(uint8_t** no_arg){
	// Stop whatever was going on
	empty_process_sequence();
	initialize_flag=true;
	// Set filter taps in IMU (since occasionally they seem to reset themselves)
	uint8_t log2_nr_filter_taps = 0;
	low_pass_filter_setting(log2_nr_filter_taps);
	// Start initial alignment
	set_elem_in_process_sequence(processing_functions_by_id[UPDATE_ZARU_BUFFER]->func_p,0);
	set_elem_in_process_sequence(processing_functions_by_id[UPDATE_BUFFER2]->func_p,1);
	set_elem_in_process_sequence(processing_functions_by_id[INITIAL_ALIGNMENT]->func_p,2);
	// Set termination function of initial alignment which will also start INS
	set_last_process_sequence_element(&start_stepwise_dead_reckoning);
}

void set_conditional_output_reset2(void){
	if(filter_reset_flag){
		set_conditional_output(DX_SID);
		set_conditional_output(DP_SID);
//		set_conditional_output(INTERRUPT_COUNTER_SID);
		set_conditional_output(STEP_COUNTER_SID);
	}
}

extern uint16_t step_counter;
void start_stepwise_dead_reckoning2(void){
	if(initialize_flag==false){
		// Stop initial alignment
		empty_process_sequence();
		// Reset step counter;
		step_counter=0;
		// Start ZUPT-aided INS
		set_elem_in_process_sequence(processing_functions_by_id[UPDATE_BUFFER]->func_p,0);
		set_elem_in_process_sequence(processing_functions_by_id[ZUPT_DETECTOR]->func_p,1);
		set_elem_in_process_sequence(processing_functions_by_id[STEPWISE_SYSTEM_RESET]->func_p,2);
		set_elem_in_process_sequence(processing_functions_by_id[MECHANIZATION]->func_p,3);
		set_elem_in_process_sequence(processing_functions_by_id[TIME_UPDATE]->func_p,4);
		set_elem_in_process_sequence(processing_functions_by_id[ZUPT_UPDATE]->func_p,5);
		set_elem_in_process_sequence(&set_conditional_output_reset2,6);
	}
}

void reset_stepwise_dead_reckoning2(uint8_t** no_arg){
	// Stop whatever was going on
	empty_process_sequence();
	initialize_flag=true;
	// Set filter taps in IMU (since occasionally they seem to reset themselves)
	uint8_t log2_nr_filter_taps = 0;
	low_pass_filter_setting(log2_nr_filter_taps);
	// Start initial alignment
	set_elem_in_process_sequence(processing_functions_by_id[UPDATE_BUFFER]->func_p,0);
	set_elem_in_process_sequence(processing_functions_by_id[INITIAL_ALIGNMENT]->func_p,1);
	// Set termination function of initial alignment which will also start INS
	set_last_process_sequence_element(&start_stepwise_dead_reckoning2);
}

void set_conditional_output_reset3(void){
	if(filter_reset_flag){
		set_conditional_output(DX_SID);
		set_conditional_output(DP_SID);
//		set_conditional_output(INTERRUPT_COUNTER_SID);
		set_conditional_output(STEP_COUNTER_SID);
		set_conditional_output(SAMSUNG_ID_SID);
	}
}

//extern uint16_t step_counter;
void start_stepwise_dead_reckoning3(void){
	if(initialize_flag==false){
		// Stop initial alignment
		empty_process_sequence();
		// Reset step counter;
		step_counter=0;
		// Start ZUPT-aided INS
		set_elem_in_process_sequence(processing_functions_by_id[UPDATE_ZARU_BUFFER]->func_p,0);
		set_elem_in_process_sequence(processing_functions_by_id[ZARU_DETECTOR]->func_p,1);
		set_elem_in_process_sequence(processing_functions_by_id[UPDATE_BUFFER2]->func_p,2);
		set_elem_in_process_sequence(processing_functions_by_id[ZUPT_DETECTOR]->func_p,3);
		set_elem_in_process_sequence(processing_functions_by_id[STEPWISE_SYSTEM_RESET]->func_p,4);
		set_elem_in_process_sequence(processing_functions_by_id[MECHANIZATION]->func_p,5);
		set_elem_in_process_sequence(processing_functions_by_id[TIME_UPDATE]->func_p,6);
		set_elem_in_process_sequence(processing_functions_by_id[ZUPT_UPDATE]->func_p,7);
		set_elem_in_process_sequence(&set_conditional_output_reset3,8);
//		set_elem_in_process_sequence(processing_functions_by_id[UPDATE_BUFFER]->func_p,0);
//		set_elem_in_process_sequence(processing_functions_by_id[ZUPT_DETECTOR]->func_p,1);
//		set_elem_in_process_sequence(processing_functions_by_id[STEPWISE_SYSTEM_RESET]->func_p,2);
//		set_elem_in_process_sequence(processing_functions_by_id[MECHANIZATION]->func_p,3);
//		set_elem_in_process_sequence(processing_functions_by_id[TIME_UPDATE]->func_p,4);
//		set_elem_in_process_sequence(processing_functions_by_id[ZUPT_UPDATE]->func_p,5);
//		set_elem_in_process_sequence(&set_conditional_output_reset3,6);
	}
}

uint8_t samsung_id;
void reset_stepwise_dead_reckoning3(uint8_t** cmd_arg){
	samsung_id = cmd_arg[0][0];
	// Stop whatever was going on
	empty_process_sequence();
	initialize_flag=true;
	// Set filter taps in IMU (since occasionally they seem to reset themselves)
	uint8_t log2_nr_filter_taps = 0;
	low_pass_filter_setting(log2_nr_filter_taps);
	// Start initial alignment
	set_elem_in_process_sequence(processing_functions_by_id[UPDATE_ZARU_BUFFER]->func_p,0);
	set_elem_in_process_sequence(processing_functions_by_id[UPDATE_BUFFER2]->func_p,1);
	set_elem_in_process_sequence(processing_functions_by_id[INITIAL_ALIGNMENT]->func_p,2);
//	set_elem_in_process_sequence(processing_functions_by_id[UPDATE_BUFFER]->func_p,0);
//	set_elem_in_process_sequence(processing_functions_by_id[INITIAL_ALIGNMENT]->func_p,1);
	// Set termination function of initial alignment which will also start INS
	set_last_process_sequence_element(&start_stepwise_dead_reckoning3);
}


void set_conditional_output_reset4(void){
	if(filter_reset_flag){
		set_conditional_output(DX_SID);
		set_conditional_output(DP_SID);
		set_conditional_output(STEP_COUNTER_SID);
	}
}
void start_stepwise_dead_reckoning4(void){
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
		set_elem_in_process_sequence(&set_conditional_output_reset4,7);
	}
}
uint32_t number_of_calls = 0;
void wait_for_filled_up_buffer(void){
	number_of_calls++;
	if (number_of_calls>64){
		empty_process_sequence();
		initialize_flag=true;
		// Start initial alignment
		set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_PREPROC]->func_p,0);
		set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_STATDET]->func_p,1);
		set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_POSTPROC]->func_p,2);
		set_elem_in_process_sequence(processing_functions_by_id[INITIAL_ALIGNMENT]->func_p,3);
		set_last_process_sequence_element(&start_stepwise_dead_reckoning4);
	}
}
void reset_stepwise_dead_reckoning4(uint8_t** no_arg){
	number_of_calls = 0;
	empty_process_sequence();
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_PREPROC]->func_p,0);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_STATDET]->func_p,1);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_POSTPROC]->func_p,2);
	set_last_process_sequence_element(&wait_for_filled_up_buffer);
}



///\cond
extern uint32_t nr_of_inital_alignment_samples;
///\endcond
void reset_swdr_gyrocal(uint8_t** cmd_arg){
	// Stop whatever was going on
	empty_process_sequence();
	initialize_flag=true;
	nr_of_inital_alignment_samples = 100*cmd_arg[0][0];
	// Set filter taps in IMU (since occasionally they seem to reset themselves)
	uint8_t log2_nr_filter_taps = 0;
	low_pass_filter_setting(log2_nr_filter_taps);
	// Start initial alignment
	set_elem_in_process_sequence(processing_functions_by_id[UPDATE_BUFFER]->func_p,0);
	set_elem_in_process_sequence(processing_functions_by_id[INITIAL_ALIGNMENT]->func_p,1);
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
	uint8_t nr_orientations = cmd_arg[0][0];
	nr_of_calibration_orientations = nr_orientations;
	
	store_and_empty_process_sequence();
	set_elem_in_process_sequence(processing_functions_by_id[ACCELEROMETER_CALIBRATION]->func_p,0);
	set_last_process_sequence_element(&new_calibration_orientation);
}

void set_low_pass_imu(uint8_t ** cmd_arg){
	uint8_t log2_nr_filter_taps = cmd_arg[0][0];
	if (log2_nr_filter_taps<=4){
		low_pass_filter_setting(log2_nr_filter_taps);}
	// Todo: set error state if above does not hold.
}

void add_sync_output(uint8_t** cmd_arg){
	uint8_t state_id = cmd_arg[0][0];
	uint8_t output_divider    = cmd_arg[1][0];
	if(state_info_access_by_id[state_id]){  // Valid state?
		set_state_output(state_id,output_divider);}
	reset_output_counters();
}

void sync_output(uint8_t** no_arg){
	reset_output_counters();
}

void processing_off(uint8_t** no_arg){
	// Stop whatever was going on
	empty_process_sequence();
}

void mimu_frontend(uint8_t** no_arg){
	empty_process_sequence();
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_PREPROC]->func_p,0);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_STATDET]->func_p,1);
	set_elem_in_process_sequence(processing_functions_by_id[FRONTEND_POSTPROC]->func_p,2);
//	set_state_output(U_K_SID,1);
}

//@}