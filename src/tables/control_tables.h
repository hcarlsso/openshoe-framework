

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
} command_structure;

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
#define UPDATE_BUFFER 0x04
#define INITIAL_ALIGNMENT 0x05
#define MECHANIZATION 0x06
#define TIME_UPDATE 0x07
#define ZUPT_DETECTOR 0x08
#define ZUPT_UPDATE 0x09
#define GYRO_CALIBRATION 0x10
#define ACCELEROMETER_CALIBRATION 0x11
#define STEPWISE_SYSTEM_RESET 0x13
#define UPDATE_ZARU_BUFFER 0x14
#define ZARU_DETECTOR 0x15
#define UPDATE_BUFFER2 0x16
// Frontend
#define FRONTEND_PREPROC 0x20
#define FRONTEND_STATDET 0x21
#define FRONTEND_POSTPROC 0x22
//@}

///  \name External state IDs
///  Macros for external state IDs
//@{
// Maximum value of state ID (255)
#define SID_LIMIT 0xFF
// State IDs
// IMU measurements
#define SPECIFIC_FORCE_SID 0x01
#define ANGULAR_RATE_SID 0x02
#define IMU_TEMPERATURS_SID 0x03
#define IMU_SUPPLY_VOLTAGE_SID 0x04
#define U_NEW_SID 0x05
#define U_INT_K_SID 0x06
#define U_K_SID 0x07
// Filtering states
#define POSITION_SID 0x11
#define VELOCITY_SID 0x12
#define QUATERNION_SID 0x13
#define ZUPT_SID 0x14
#define DT_SID 0x15
#define TEST_STATISTICS_SID 0x19
#define ZARU_TEST_STATISTICS_SID 0x1a
#define GYRO_BIASES 0x1b
#define ZARU_SID 0x1c
#define T1S2F 0x1d
#define T2S2F 0x1e
// Step-wise dead reckoning data exchange states
#define DX_SID 0x16
#define DP_SID 0x17
#define STEP_COUNTER_SID 0x18
// System states
#define INTERRUPT_COUNTER_SID 0x21
#define IMU_DT_SID 0x22
// "Other" states
#define ACCELEROMETER_BIASES_SID 0x35
#define SAMSUNG_ID_SID 0x36
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
#define ONLY_ACK 0x01
#define MCU_ID 0x02
#define OUTPUT_STATE 0x20
#define OUTPUT_ALL_OFF 0x21
#define OUTPUT_ONOFF_INERT 0x22
#define OUTPUT_POSITION_PLUS_ZUPT 0x23
#define OUTPUT_NAVIGATIONAL_STATES 0x24
#define OUTPUT_IMU_RD 0x28
#define OUTPUT_IMU_TEMP 0x29
#define PROCESSING_FUNCTION_ONOFF 0x30
#define RESET_ZUPT_AIDED_INS 0x10
#define GYRO_CALIBRATION_INIT 0x11
#define ACC_CALIBRATION_INIT 0x12
#define SET_LOWPASS_FILTER_IMU 0x13
#define RESET_STEPWISE_DEAD_RECKONING 0x14
#define RESET_STEPWISE_DEAD_RECKONING2 0x16
#define RESET_STEPWISE_DEAD_RECKONING3 0x17
#define RESET_STEPWISE_DEAD_RECKONING4 0x18
#define RESET_SWDR_GYROCAL 0x15
#define ADD_SYNC_OUTPUT 0x25
#define SYNC_OUTPUT 0x26
#define PROCESSING_OFF 0x27
#define MIMU_FRONTEND 0x31
//@}

// Global variables used to access command information
extern uint8_t command_header_table[32];
extern command_structure* command_info_array[256];
void commands_init(void);

inline bool is_valid_header(uint8_t header){
	return command_header_table[header>>3] & (1<<(header & 7));}
	
inline command_structure* get_command_info(uint8_t header){
	return command_info_array[header];}


// Array containing the processing functions to run
extern proc_func_info* processing_functions_by_id[256];
void processing_functions_init(void);


// Global variables used to access information about states
extern state_t_info* state_info_access_by_id[SID_LIMIT];
void system_states_init(void);


#endif /* CONTROL_TABLES_H_ */

//@}