
/** \file
	\brief Declarations of system states and related information tables.
	
	\details This header file contains
	1) Declarations of system states.
	2) Definitions of tables containing information about the system states.
	The system states can be defined across the program. This file just gather
	the declarations together such that the related information tables can be
	filled in. The information tables are used for output functions in
	external_interfaces.c.
	This file contians multiple static variables so it should only be included
	where necessary.
	
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)	
*/ 

/**
	\ingroup openshoe_software
	
	\defgroup tables System tables	
	\brief This group contain table of system definitions.
	@{
*/


#ifndef SYSTEM_STATES_H_
#define SYSTEM_STATES_H_

#include "compiler.h"
#include "nav_types.h"

// State data type information
typedef const struct {
	uint8_t id;
	void* state_p;
	int state_size;
} state_t_info;

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

// Global variables used to access information about states
extern state_t_info* state_info_access_by_id[SID_LIMIT];


void system_states_init(void);


#endif /* SYSTEM_STATES_H_ */

//@}