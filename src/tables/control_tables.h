/*
 * control_tables.h
 *
 * Created: 2011-12-14 11:29:51
 *  Author: jnil02
 */ 


#ifndef CONTROL_TABLES_H_
#define CONTROL_TABLES_H_


#include "compiler.h"
#include "nav_types.h"

// Definition structure of commands
typedef const struct {
	uint8_t header;
	void (*cmd_response)(uint8_t**);
	uint8_t nrb_payload;
	uint8_t nr_fields;
	uint8_t field_widths[];
} command_structure;

// Information struct for processing functions
typedef const struct {
	uint8_t id;
	void (*func_p)(void);
	int max_proc_time; 
} proc_func_info;

// State data type information
typedef const struct {
	uint8_t id;
	void* state_p;
	int state_size;
} state_t_info;


// Processing functions ids
#define UPDATE_BUFFER 0x04
#define INITIAL_ALIGNMENT 0x05
#define MECHANIZATION 0x06
#define TIME_UPDATE 0x07
#define ZUPT_DETECTOR 0x08
#define ZUPT_UPDATE 0x09
#define GYRO_CALIBRATION 0x10
#define ACCELEROMETER_CALIBRATION 0x11

// Maximum value of state ID (255)
#define SID_LIMIT 0xFF
// State IDs
// IMU measurements
#define SPECIFIC_FORCE_SID 0x01
#define ANGULAR_RATE_SID 0x02
#define IMU_TEMPERATURS_SID 0x03
#define IMU_SUPPLY_VOLTAGE_SID 0x04
// Filtering states
#define POSITION_SID 0x11
#define VELOCITY_SID 0x12
#define QUATERNION_SID 0x13
#define ZUPT_SID 0x14
// System states
#define INTERRUPT_COUNTER_SID 0x21
#define WINDOW_SIZE_PID 0x23
#define TIME_SINCE_LAST_ZUPT_SID 0x24
#define PROCESS_CYCLE_COUNTER_SID 0x25
// "Other" states
#define ACCELEROMETER_BIASES_SID 0x35


// Global variables used to access command information
extern uint8_t command_header_table[32];
extern command_structure* command_info_array[256];
void commands_init(void);


// Array containing the processing functions to run
extern proc_func_info* processing_functions_by_id[256];
void processing_functions_init(void);


// Global variables used to access information about states
extern state_t_info* state_info_access_by_id[SID_LIMIT];
void system_states_init(void);


#endif /* CONTROL_TABLES_H_ */