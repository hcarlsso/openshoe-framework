

#include "control_tables.h"

// IMU measurements
extern vec3 accelerations_in;
extern vec3 angular_rates_in;
extern vec3 imu_temperaturs;
extern precision imu_supply_voltage;
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
// State information
static state_t_info interrupt_counter_sti = {INTERRUPT_COUNTER_SID, (void*) &interrupt_counter, sizeof(uint32_t)};
static state_t_info window_size_sti = {WINDOW_SIZE_PID, (void*) &window_size, sizeof(int)};
static state_t_info time_since_last_zupt_sti = {TIME_SINCE_LAST_ZUPT_SID, (void*) &time_since_last_zupt, sizeof(int)};
static state_t_info process_cycle_counter_sti = {PROCESS_CYCLE_COUNTER_SID, (void*) &process_cycle_counter, sizeof(uint32_t)};
	

// "Other" states
extern vec3 accelerometer_biases;
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


state_t_info* state_info_access_by_id[SID_LIMIT];

void system_states_init(void){
	for(int i = 0;i<(sizeof(state_struct_array)/sizeof(state_struct_array[0])); i++){
		state_info_access_by_id[state_struct_array[i]->id] = state_struct_array[i];}
}	
	
	
	
	
	
	