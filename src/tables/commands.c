

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
#include "response_functions.h"
#include <compiler.h>


#if defined(MIMU3333)
#  include "MIMU3333.h"
#elif defined(MIMU4444)
#  include "MIMU4444.h"
#elif defined(MIMU22BT)
#  include "MIMU22BT.h"
#endif

#define MREPEAT_ARG_CONST(narg,size) size,

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
static command_info output_all_off = {OUTPUT_ALL_OFF,&all_output_off,0,0,{0}};
static command_info output_onoff_imu_rd = {OUTPUT_IMU_RD,&output_imu_rd,5,2,{4,1}};
static command_info processing_function_onoff = {RUN_PROC,&set_proc,2,2,{1,1}};
static command_info run_mult_proc_cmd = {RUN_MULT_PROC,&set_mult_proc,8,8,{1,1,1,1,1,1,1,1}};
static command_info reset_system_cmd = {RESET_ZUPT_AIDED_INS,&zupt_aided_ins,0,0,{0}};
static command_info stepwise_dead_reckoning_cmd = {STEPWISE_DEAD_RECKONING,&stepwise_dead_reckoning,0,0,{0}};
static command_info stepwise_dead_reckoning_TOR_cmd = {STEPWISE_DEAD_RECKONING_TOR,&stepwise_dead_reckoning_TOR,1,1,{1}};
static command_info processing_off_cmd = {ALL_PROC_OFF,&all_proc_off,0,0,{0}};
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
											  &mimu_frontend_cmd,
											  &normal_imu_cmd,
											  &normal_imu_with_bias_est_cmd};
												  
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

