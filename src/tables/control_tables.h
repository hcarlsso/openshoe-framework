

/** \file
	\brief Header file for all tabulated information for the control and communication.
	
	\details This file contains declarations of arrays containing tabulated
	information about external system states, processing functions, and
	commands. It also contains struct typedefs of structs containing such
	information for individual states, functions, and commands, together
	with ID macros for the same.
	Delcarations of initialization functions for the arrays are also found.
	
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
*/ 

/**
	\ingroup openshoe_runtime_framework
	
	\defgroup control_tables Tabulated information	
	\brief This group contains tabulated information for the control and communication.
	@{
*/


#ifndef CONTROL_TABLES_H_
#define CONTROL_TABLES_H_


#include "compiler.h"
#include "nav_types.h"
#include "inertial_frontend.h"

/// Definition structure of commands
typedef const struct {
	uint8_t header;
	void (*cmd_response)(uint8_t**);
	uint8_t nrb_payload;
	uint8_t nr_fields;
	uint8_t field_widths[];
} command_info;

/// Information struct for processing functions
typedef const struct {
	uint8_t id;
	void (*func_p)(void);
	int max_proc_time; 
} proc_func_info;

/// State data type information struct
typedef const struct {
	uint8_t id;
	void* state_p;
	int state_size;
} state_t_info;


///  \name Processing functions IDs
///  Macros for processing functions IDs
//@{
#define STORE_AND_EMPTY_PROC_SEQU 0x01
#define RESTORE_PROC_SEQU 0x02
#define EMPTY_PROCESS_SEQUENCE 0x03
#define RESTORE_PROC_SEQU_IF 0x04
#define STATE_OUTPUT_IF 0x05
#define STATE_OUTPUT_ONCE_IF 0x06
#define STATE_OUTPUT_IF_COUNTER 0x07

#define FRONTEND_PREPROC 0x10
#define FRONTEND_STATDET 0x11
#define FRONTEND_POSTPROC 0x12

#define MECHANIZATION 0x20
#define TIME_UPDATE 0x21
#define ZUPT_UPDATE 0x22
#define FRONTEND_INITIAL_ALIGNMENT 0x23
#define STEPWISE_SYSTEM_RESET 0x24

//@}

///  \name External state IDs
///  Macros for external state IDs
//@{
// Maximum value of state ID (255)
#define SID_LIMIT 0xFF
#define NOSTATE_SID 0x00
// State IDs
// System states
#define IMU_TS_SID 0x01
#define INTERRUPT_COUNTER_SID 0x02
#define GP_DT_SID 0x03
#define MCU_ID_SID 0x04
#define GP_ID_SID 0x05
// Inertial frontend states
#define U_NEW_SID 0x10
#define U_INT_K_SID 0x11
#define T_INT_K_SID 0x12
#define U_K_SID 0x13
#define DT_SID 0x14
#define T1S2F 0x15
#define T2S2F 0x16
#define ZUPT_SID 0x17
#define ZARU_SID 0x18
// Filtering states
#define POSITION_SID 0x20
#define VELOCITY_SID 0x21
#define QUATERNION_SID 0x22
#define P_SID 0x23
#define INIT_DONE_SID 0x24
// Step-wise dead reckoning data exchange states
#define DX_SID 0x30
#define DP_SID 0x31
#define STEP_COUNTER_SID 0x32
#define	FILTER_RESET_FLAG_SID 0x33
// MIMU raw register states
#define IMU0_RD_SID 0x40
#define IMU1_RD_SID (IMU0_RD_SID+1)
#define IMU2_RD_SID (IMU0_RD_SID+2)
#define IMU3_RD_SID (IMU0_RD_SID+3)
#define IMU4_RD_SID (IMU0_RD_SID+4)
#define IMU5_RD_SID (IMU0_RD_SID+5)
#define IMU6_RD_SID (IMU0_RD_SID+6)
#define IMU7_RD_SID (IMU0_RD_SID+7)
#define IMU8_RD_SID (IMU0_RD_SID+8)
#define IMU9_RD_SID (IMU0_RD_SID+9)
#define IMU10_RD_SID (IMU0_RD_SID+10)
#define IMU11_RD_SID (IMU0_RD_SID+11)
#define IMU12_RD_SID (IMU0_RD_SID+12)
#define IMU13_RD_SID (IMU0_RD_SID+13)
#define IMU14_RD_SID (IMU0_RD_SID+14)
#define IMU15_RD_SID (IMU0_RD_SID+15)
#define IMU16_RD_SID (IMU0_RD_SID+16)
#define IMU17_RD_SID (IMU0_RD_SID+17)
#define IMU18_RD_SID (IMU0_RD_SID+18)
#define IMU19_RD_SID (IMU0_RD_SID+19)
#define IMU20_RD_SID (IMU0_RD_SID+20)
#define IMU21_RD_SID (IMU0_RD_SID+21)
#define IMU22_RD_SID (IMU0_RD_SID+22)
#define IMU23_RD_SID (IMU0_RD_SID+23)
#define IMU24_RD_SID (IMU0_RD_SID+24)
#define IMU25_RD_SID (IMU0_RD_SID+25)
#define IMU26_RD_SID (IMU0_RD_SID+26)
#define IMU27_RD_SID (IMU0_RD_SID+27)
#define IMU28_RD_SID (IMU0_RD_SID+28)
#define IMU29_RD_SID (IMU0_RD_SID+29)
#define IMU30_RD_SID (IMU0_RD_SID+30)
#define IMU31_RD_SID (IMU0_RD_SID+31)
#define IMU0_TEMP_SID 0x60
#define IMU1_TEMP_SID (IMU0_TEMP_SID+1)
#define IMU2_TEMP_SID (IMU0_TEMP_SID+2)
#define IMU3_TEMP_SID (IMU0_TEMP_SID+3)
#define IMU4_TEMP_SID (IMU0_TEMP_SID+4)
#define IMU5_TEMP_SID (IMU0_TEMP_SID+5)
#define IMU6_TEMP_SID (IMU0_TEMP_SID+6)
#define IMU7_TEMP_SID (IMU0_TEMP_SID+7)
#define IMU8_TEMP_SID (IMU0_TEMP_SID+8)
#define IMU9_TEMP_SID (IMU0_TEMP_SID+9)
#define IMU10_TEMP_SID (IMU0_TEMP_SID+10)
#define IMU11_TEMP_SID (IMU0_TEMP_SID+11)
#define IMU12_TEMP_SID (IMU0_TEMP_SID+12)
#define IMU13_TEMP_SID (IMU0_TEMP_SID+13)
#define IMU14_TEMP_SID (IMU0_TEMP_SID+14)
#define IMU15_TEMP_SID (IMU0_TEMP_SID+15)
#define IMU16_TEMP_SID (IMU0_TEMP_SID+16)
#define IMU17_TEMP_SID (IMU0_TEMP_SID+17)
#define IMU18_TEMP_SID (IMU0_TEMP_SID+18)
#define IMU19_TEMP_SID (IMU0_TEMP_SID+19)
#define IMU20_TEMP_SID (IMU0_TEMP_SID+20)
#define IMU21_TEMP_SID (IMU0_TEMP_SID+21)
#define IMU22_TEMP_SID (IMU0_TEMP_SID+22)
#define IMU23_TEMP_SID (IMU0_TEMP_SID+23)
#define IMU24_TEMP_SID (IMU0_TEMP_SID+24)
#define IMU25_TEMP_SID (IMU0_TEMP_SID+25)
#define IMU26_TEMP_SID (IMU0_TEMP_SID+26)
#define IMU27_TEMP_SID (IMU0_TEMP_SID+27)
#define IMU28_TEMP_SID (IMU0_TEMP_SID+28)
#define IMU29_TEMP_SID (IMU0_TEMP_SID+29)
#define IMU30_TEMP_SID (IMU0_TEMP_SID+30)
#define IMU31_TEMP_SID (IMU0_TEMP_SID+31)
//@}

///  \name Command IDs
///  Macros for command IDs
//@{
#define ACK_ID 0x01
#define PING_ID 0x03
#define MCU_ID 0x04
#define SETUP_DEBUG_PROC 0x10
#define INPUT_IMU_RD 0x11
#define SET_STATE_1 0x12
#define SET_STATE_4 0x13
#define SET_STATE_12 0x14
#define SET_STATE_24 0x15
#define SET_STATE_48 0x16
#define SET_STATE_254 0x18
#define OUTPUT_STATE 0x20
#define OUTPUT_MULTIPLE_STATES 0x21
#define OUTPUT_ALL_OFF 0x22
#define STATE_IF_SETUP 0x23
#define OUTPUT_IMU_RD 0x28
#define RUN_PROC 0x30
#define RUN_MULT_PROC 0x31
#define ALL_PROC_OFF 0x32
#define RESET_PROC_SETUP 0x36
#define STORE_AND_EMPTY_CMD_ID 0x37
#define RESTORE_PROC_SEQU_CMD_ID 0x38
#define RESET_ZUPT_AIDED_INS 0x33
#define STEPWISE_DEAD_RECKONING 0x34
#define MIMU_FRONTEND 0x35
#define NORMAL_IMU 0x40
#define NORMAL_IMU_WITH_BIAS_EST 0x41
#define STEPWISE_DEAD_RECKONING_TOR 0x17
//@}

// Global variables used to access command information
extern uint8_t command_header_table[32];
extern command_info* command_info_array[256];
void commands_init(void);

inline static bool is_valid_header(uint8_t header){
	return command_header_table[header>>3] & (1<<(header & 7));}
	
inline static command_info* get_command_info(uint8_t header){
	return command_info_array[header];}


// Array containing the processing functions to run
extern proc_func_info* processing_functions_by_id[256];
void processing_functions_init(void);


// Global variables used to access information about states
extern state_t_info* state_info_access_by_id[SID_LIMIT];
void system_states_init(void);
void set_state(uint8_t state_id,void* value);
void* get_state_p(uint8_t state_id);
uint8_t get_state_size(uint8_t state_id);

#endif /* CONTROL_TABLES_H_ */

//@}