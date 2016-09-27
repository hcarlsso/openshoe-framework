

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
#include "process_sequence.h"
#include "compiler.h"


#if defined(MIMU3333)
#  include "MIMU3333.h"
#elif defined(MIMU4444)
#  include "MIMU4444.h"
#elif defined(MIMU22BT)
#  include "MIMU22BT.h"
#elif defined(MIMU4444BT)
#  include "MIMU4444BT.h"
#elif defined(MIMU22BTv3)
#  include "MIMU22BTv3.h"
#endif

#define MREPEAT_ARG_CONST(narg,size) size,

///  \name Command definitions
///  Structs containing the information/definitions of the commands.
//@{
static command_info ack = {ACK_ID,&handle_ack,2,1,{2}};
static command_info ping = {PING_ID,&do_nothing,0,0,{0}};
static command_info mcu_id = {MCU_ID,&get_mcu_serial,0,0,{0}};
static command_info setup_debug_proc_cmd = {SETUP_DEBUG_PROC,&setup_debug_processing,NR_DEBUG_PROC+NR_DEBUG_OUTPUT+1,3,{NR_DEBUG_PROC,NR_DEBUG_OUTPUT,1}};
static command_info input_imu_rd_cmd = {INPUT_IMU_RD,&input_imu_rd,4+NR_IMUS*6*sizeof(int16_t),NR_IMUS+1,{4,MREPEAT(NR_IMUS, MREPEAT_ARG_CONST, 12)}};
static command_info gp_test_command_cmd = {GP_TEST_ID,&gp_test_command,8,2,{4,4}};
static command_info set_state_max_1byte_cmd = {SET_STATE_1,&set_state_max_1byte,2,2,{1,1}};
static command_info set_state_max_4bytes_cmd = {SET_STATE_4,&set_state_max_4bytes,5,2,{1,4}};
static command_info set_state_max_12bytes_cmd = {SET_STATE_12,&set_state_max_12bytes,13,2,{1,12}};
static command_info set_state_max_24bytes_cmd = {SET_STATE_24,&set_state_max_24bytes,25,2,{1,24}};
static command_info set_state_max_48bytes_cmd = {SET_STATE_48,&set_state_max_48bytes,49,2,{1,48}};
static command_info set_state_max_254bytes_cmd = {SET_STATE_254,&set_state_max_254bytes,255,2,{1,254}};
static command_info output_onoff_state = {OUTPUT_STATE,&output_state,2,2,{1,1}};
static command_info output_multiple_states_cmd = {OUTPUT_MULTIPLE_STATES,&output_multiple_states,9,9,{1,1,1,1,1,1,1,1,1}};
static command_info output_all_off = {OUTPUT_ALL_OFF,&all_output_off,0,0,{0}};
static command_info state_if_setup_cmd = {STATE_IF_SETUP,&state_if_setup_resp,10,3,{1,1,8}};
static command_info output_onoff_imu_rd = {OUTPUT_IMU_RD,&output_imu_rd,5,2,{4,1}};
static command_info set_imu_bandwidth_cmd = {SET_IMU_BANDWIDTH,&set_imu_bandwidth,1,1,{1}};
static command_info processing_function_onoff = {RUN_PROC,&set_proc,2,2,{1,1}};
static command_info run_mult_proc_cmd = {RUN_MULT_PROC,&set_mult_proc,8,8,{1,1,1,1,1,1,1,1}};
static command_info processing_off_cmd = {ALL_PROC_OFF,&all_proc_off,0,0,{0}};
static command_info reset_system_cmd = {RESET_ZUPT_AIDED_INS,&zupt_aided_ins,0,0,{0}};
#if defined(SMOOTHING)
static command_info smoothing_cmd = {SMOOTHED_ZUPT_AIDED_INS,&smoothed_zupt_aided_ins,0,0,{0}};
#endif
static command_info stepwise_dead_reckoning_cmd = {STEPWISE_DEAD_RECKONING,&stepwise_dead_reckoning,0,0,{0}};
static command_info mimu_frontend_cmd = {MIMU_FRONTEND,&start_inertial_frontend,0,0,{0}};
static command_info restore_proc_setup_cmd = {RESET_PROC_SETUP,&restore_proc_if_setup,1,1,{1}};
static command_info store_and_empty_cmd = {STORE_AND_EMPTY_CMD_ID,(void (*)(uint8_t**)) &store_and_empty_process_sequence,0,0,{0}};
static command_info restore_proc_sequ_cmd = {RESTORE_PROC_SEQU_CMD_ID,(void (*)(uint8_t**)) &restore_process_sequence,0,0,{0}};
static command_info normal_imu_cmd = {NORMAL_IMU,&normal_imu,1,1,{1}};
static command_info normal_imu_with_bias_est_cmd = {NORMAL_IMU_WITH_BIAS_EST,&normal_imu_with_bias_est,1,1,{1}};
//@}

// Arrays/tables to find appropriate commands
static const command_info* commands[] = {&ack,
										 &ping,
										 &mcu_id,
										 &setup_debug_proc_cmd,
										 &input_imu_rd_cmd,
										 &gp_test_command_cmd,
										 &set_state_max_1byte_cmd,
										 &set_state_max_4bytes_cmd,
										 &set_state_max_12bytes_cmd,
										 &set_state_max_24bytes_cmd,
										 &set_state_max_48bytes_cmd,
										 &set_state_max_254bytes_cmd,
										 &output_onoff_state,
										 &output_multiple_states_cmd,
										 &output_all_off,
										 &state_if_setup_cmd,
										 &output_onoff_imu_rd,
										 &set_imu_bandwidth_cmd,
										 &processing_function_onoff,
										 &run_mult_proc_cmd,
										 &processing_off_cmd,
										 &reset_system_cmd,
										 #if defined(SMOOTHING)
										 &smoothing_cmd,
										 #endif
										 &stepwise_dead_reckoning_cmd,
										 &mimu_frontend_cmd,
										 &restore_proc_setup_cmd,
										 &store_and_empty_cmd,
										 &restore_proc_sequ_cmd,
										 &normal_imu_cmd,
										 &normal_imu_with_bias_est_cmd};
												  
// Arrays/tables for commands
command_info* command_info_array[CID_LIMIT]={NULL};

void commands_init(void){
	for(int i = 0;i<(sizeof(commands)/sizeof(commands[0]));i++)
		if(commands[i]->header<CID_LIMIT)
			command_info_array[commands[i]->header] = commands[i];
}

