
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

// State data type information
typedef const struct {
	uint8_t id;
	void* state_p;
	int state_size;
} state_t_info;

// Maximum value of state ID (255)
#define SID_LIMIT 0xFF

// IMU measurements
extern vec3 accelerations_in;
extern vec3 angular_rates_in;
extern vec3 imu_temperaturs;
extern precision imu_supply_voltage;
// State IDs, IMU measurements
#define SPECIFIC_FORCE_SID 0x01
#define ANGULAR_RATE_SID 0x02
#define IMU_TEMPERATURS_SID 0x03
#define IMU_SUPPLY_VOLTAGE_SID 0x04
// State information
static state_t_info specific_force_sti = {SPECIFIC_FORCE_SID, (void*) accelerations_in, sizeof(vec3)};
static state_t_info angular_rate_sti = {ANGULAR_RATE_SID, (void*) angular_rates_in, sizeof(vec3)};
static state_t_info imu_temperaturs_sti = {IMU_TEMPERATURS_SID, (void*) imu_temperaturs, sizeof(vec3)};
static state_t_info imu_supply_voltage_sti = {IMU_SUPPLY_VOLTAGE_SID, (void*) &imu_supply_voltage, sizeof(precision)};

// Filtering states
extern vec3 position;
extern vec3 velocity;
extern quat_vec quaternions;
extern bool zupt;
// State IDs, filtering states
#define POSITION_SID 0x11
#define VELOCITY_SID 0x12
#define QUATERNION_SID 0x13
#define ZUPT_SID 0x14
// State information
static state_t_info position_sti = {POSITION_SID, (void*) position, sizeof(vec3)};
static state_t_info velocity_sti = {VELOCITY_SID, (void*) velocity, sizeof(vec3)};
static state_t_info quaternions_sti = {QUATERNION_SID, (void*) quaternions, sizeof(quat_vec)};
static state_t_info zupt_sti = {ZUPT_SID, (void*) &zupt, sizeof(bool)};
	
// System states
extern uint32_t interrupt_counter;
extern int window_size;
extern int time_since_last_zupt;
extern uint32_t process_cycle_counter;
// State IDs, system states
#define INTERRUPT_COUNTER_SID 0x21
#define WINDOW_SIZE_PID 0x23
#define TIME_SINCE_LAST_ZUPT_SID 0x24
#define PROCESS_CYCLE_COUNTER_SID 0x25
// State information
static state_t_info interrupt_counter_sti = {INTERRUPT_COUNTER_SID, (void*) &interrupt_counter, sizeof(uint32_t)};
static state_t_info window_size_sti = {WINDOW_SIZE_PID, (void*) &window_size, sizeof(int)};
static state_t_info time_since_last_zupt_sti = {TIME_SINCE_LAST_ZUPT_SID, (void*) &time_since_last_zupt, sizeof(int)};
static state_t_info process_cycle_counter_sti = {PROCESS_CYCLE_COUNTER_SID, (void*) &process_cycle_counter, sizeof(uint32_t)};

// "Other" states
extern vec3 accelerometer_biases;
#define ACCELEROMETER_BIASES_SID 0x35
static state_t_info accelerometer_biases_sti = {ACCELEROMETER_BIASES_SID, (void*) &accelerometer_biases, sizeof(vec3)};

// Array of state data type struct pointers
const static state_t_info* state_struct_array[] = {&interrupt_counter_sti,
												   &window_size_sti,
												   &time_since_last_zupt_sti,
												   &process_cycle_counter_sti,
												   &specific_force_sti,
												   &angular_rate_sti,
												   &imu_temperaturs_sti,
												   &imu_supply_voltage_sti,
												   &position_sti,
								 	               &velocity_sti,
												   &quaternions_sti,
												   &zupt_sti,
												   &accelerometer_biases_sti};


#endif /* SYSTEM_STATES_H_ */

//@}