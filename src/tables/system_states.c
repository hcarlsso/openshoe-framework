
/** \file
	\brief Declaration of external system states.
	
	\details This files contain extern declaration and definition of static state
	structs for all external states. External states mean that these states
	might be requested from the system. Consequently, if any states are to be
	output from the system, they should be added here.
	
	The file also contain an initialization function to fill up some arrays of
	states structs. This initialization function must be called before the
	arrays are used. This should eventually be replaced by some code generating
	script which explicitly declear the arrays.
	
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
*/

/**
	\addtogroup control_tables
	@{
*/

#include "control_tables.h"

///\cond
// IMU measurements
extern vec3 accelerations_in;
extern vec3 angular_rates_in;
extern vec3 imu_temperaturs;
extern precision imu_supply_voltage;
	
// Filtering states
extern vec3 position;
extern vec3 velocity;
extern quat_vec quaternions;
extern bool zupt;
extern bool zaru;
extern precision dt;
extern precision Test_statistics;
extern precision zaru_Test_statistics;
extern vec3 gyroscope_biases;

// Step-wise dead reckoning data exchange states
extern vec4 dx;
extern mat4sym dP;
extern uint16_t step_counter;

// System states
extern uint32_t interrupt_counter;
extern uint32_t imu_dt;

// "Other" states
extern vec3 accelerometer_biases;
extern uint8_t samsung_id;

// IMU register states
extern uint16_t imu0_rd[6];
extern uint16_t imu1_rd[6];
extern uint16_t imu2_rd[6];
extern uint16_t imu3_rd[6];
extern uint16_t imu4_rd[6];
extern uint16_t imu5_rd[6];
extern uint16_t imu6_rd[6];
extern uint16_t imu7_rd[6];
extern uint16_t imu8_rd[6];
extern uint16_t imu9_rd[6];
extern uint16_t imu10_rd[6];
extern uint16_t imu11_rd[6];
extern uint16_t imu12_rd[6];
extern uint16_t imu13_rd[6];
extern uint16_t imu14_rd[6];
extern uint16_t imu15_rd[6];
extern uint16_t imu16_rd[6];
extern uint16_t imu17_rd[6];
extern uint16_t imu18_rd[6];
extern uint16_t imu19_rd[6];
extern uint16_t imu20_rd[6];
extern uint16_t imu21_rd[6];
extern uint16_t imu22_rd[6];
extern uint16_t imu23_rd[6];
extern uint16_t imu24_rd[6];
extern uint16_t imu25_rd[6];
extern uint16_t imu26_rd[6];
extern uint16_t imu27_rd[6];
extern uint16_t imu28_rd[6];
extern uint16_t imu29_rd[6];
extern uint16_t imu30_rd[6];
extern uint16_t imu31_rd[6];
///\endcond

///  \name External state information
///  Structs containing information and pointers to the externally accessible system states.
//@{
static state_t_info specific_force_sti = {SPECIFIC_FORCE_SID, (void*) accelerations_in, sizeof(vec3)};
static state_t_info angular_rate_sti = {ANGULAR_RATE_SID, (void*) angular_rates_in, sizeof(vec3)};
static state_t_info imu_temperaturs_sti = {IMU_TEMPERATURS_SID, (void*) imu_temperaturs, sizeof(vec3)};
static state_t_info imu_supply_voltage_sti = {IMU_SUPPLY_VOLTAGE_SID, (void*) &imu_supply_voltage, sizeof(precision)};
static state_t_info position_sti = {POSITION_SID, (void*) position, sizeof(vec3)};
static state_t_info velocity_sti = {VELOCITY_SID, (void*) velocity, sizeof(vec3)};
static state_t_info quaternions_sti = {QUATERNION_SID, (void*) quaternions, sizeof(quat_vec)};
static state_t_info zupt_sti = {ZUPT_SID, (void*) &zupt, sizeof(bool)};
static state_t_info zaru_sti = {ZARU_SID, (void*) &zaru, sizeof(bool)};
static state_t_info test_statistics_sti = {TEST_STATISTICS_SID, (void*) &Test_statistics, sizeof(precision)};
static state_t_info zaru_test_statistics_sti = {ZARU_TEST_STATISTICS_SID, (void*) &zaru_Test_statistics, sizeof(precision)};
static state_t_info gyroscope_biases_sti = {GYRO_BIASES, (void*) gyroscope_biases, sizeof(vec3)};
static state_t_info dt_sti = {DT_SID, (void*) &dt, sizeof(dt)};
static state_t_info dx_sti = {DX_SID, (void*) dx, sizeof(vec4)};
static state_t_info dP_sti = {DP_SID, (void*) dP, sizeof(mat4sym)};
static state_t_info step_counter_sti = {STEP_COUNTER_SID, (void*) &step_counter, sizeof(uint16_t)};
static state_t_info interrupt_counter_sti = {INTERRUPT_COUNTER_SID, (void*) &interrupt_counter, sizeof(uint32_t)};
static state_t_info imu_dt_sti = {IMU_DT_SID, (void*) &imu_dt, sizeof(uint32_t)};
static state_t_info accelerometer_biases_sti = {ACCELEROMETER_BIASES_SID, (void*) accelerometer_biases, sizeof(vec3)};
static state_t_info samsung_id_sti = {SAMSUNG_ID_SID, (void*) &samsung_id, sizeof(uint8_t)};

static state_t_info imu0_rd_sti = {IMU0_RD_SID, (void*) imu0_rd, sizeof(imu0_rd)};
static state_t_info imu1_rd_sti = {IMU1_RD_SID, (void*) imu1_rd, sizeof(imu1_rd)};
static state_t_info imu2_rd_sti = {IMU2_RD_SID, (void*) imu2_rd, sizeof(imu2_rd)};
static state_t_info imu3_rd_sti = {IMU3_RD_SID, (void*) imu3_rd, sizeof(imu3_rd)};
static state_t_info imu4_rd_sti = {IMU4_RD_SID, (void*) imu4_rd, sizeof(imu4_rd)};
static state_t_info imu5_rd_sti = {IMU5_RD_SID, (void*) imu5_rd, sizeof(imu5_rd)};
static state_t_info imu6_rd_sti = {IMU6_RD_SID, (void*) imu6_rd, sizeof(imu6_rd)};
static state_t_info imu7_rd_sti = {IMU7_RD_SID, (void*) imu7_rd, sizeof(imu7_rd)};
static state_t_info imu8_rd_sti = {IMU8_RD_SID, (void*) imu8_rd, sizeof(imu8_rd)};
static state_t_info imu9_rd_sti = {IMU9_RD_SID, (void*) imu9_rd, sizeof(imu9_rd)};
static state_t_info imu10_rd_sti = {IMU10_RD_SID, (void*) imu10_rd, sizeof(imu10_rd)};
static state_t_info imu11_rd_sti = {IMU11_RD_SID, (void*) imu11_rd, sizeof(imu11_rd)};
static state_t_info imu12_rd_sti = {IMU12_RD_SID, (void*) imu12_rd, sizeof(imu12_rd)};
static state_t_info imu13_rd_sti = {IMU13_RD_SID, (void*) imu13_rd, sizeof(imu13_rd)};
static state_t_info imu14_rd_sti = {IMU14_RD_SID, (void*) imu14_rd, sizeof(imu14_rd)};
static state_t_info imu15_rd_sti = {IMU15_RD_SID, (void*) imu15_rd, sizeof(imu15_rd)};
static state_t_info imu16_rd_sti = {IMU16_RD_SID, (void*) imu16_rd, sizeof(imu16_rd)};
static state_t_info imu17_rd_sti = {IMU17_RD_SID, (void*) imu17_rd, sizeof(imu17_rd)};
static state_t_info imu18_rd_sti = {IMU18_RD_SID, (void*) imu18_rd, sizeof(imu18_rd)};
static state_t_info imu19_rd_sti = {IMU19_RD_SID, (void*) imu19_rd, sizeof(imu19_rd)};
static state_t_info imu20_rd_sti = {IMU20_RD_SID, (void*) imu20_rd, sizeof(imu20_rd)};
static state_t_info imu21_rd_sti = {IMU21_RD_SID, (void*) imu21_rd, sizeof(imu21_rd)};
static state_t_info imu22_rd_sti = {IMU22_RD_SID, (void*) imu22_rd, sizeof(imu22_rd)};
static state_t_info imu23_rd_sti = {IMU23_RD_SID, (void*) imu23_rd, sizeof(imu23_rd)};
static state_t_info imu24_rd_sti = {IMU24_RD_SID, (void*) imu24_rd, sizeof(imu24_rd)};
static state_t_info imu25_rd_sti = {IMU25_RD_SID, (void*) imu25_rd, sizeof(imu25_rd)};
static state_t_info imu26_rd_sti = {IMU26_RD_SID, (void*) imu26_rd, sizeof(imu26_rd)};
static state_t_info imu27_rd_sti = {IMU27_RD_SID, (void*) imu27_rd, sizeof(imu27_rd)};
static state_t_info imu28_rd_sti = {IMU28_RD_SID, (void*) imu28_rd, sizeof(imu28_rd)};
static state_t_info imu29_rd_sti = {IMU29_RD_SID, (void*) imu29_rd, sizeof(imu29_rd)};
static state_t_info imu30_rd_sti = {IMU30_RD_SID, (void*) imu30_rd, sizeof(imu30_rd)};
static state_t_info imu31_rd_sti = {IMU31_RD_SID, (void*) imu31_rd, sizeof(imu31_rd)};
//@}
	
// Array of state data type struct pointers
const static state_t_info* state_struct_array[] = {&interrupt_counter_sti,
	                                               &imu_dt_sti,
												   &specific_force_sti,
												   &angular_rate_sti,
												   &imu_temperaturs_sti,
												   &imu_supply_voltage_sti,
												   &position_sti,
												   &dx_sti,
												   &dP_sti,
												   &step_counter_sti,
								 	               &velocity_sti,
												   &quaternions_sti,
												   &zupt_sti,
												   &zaru_sti,
												   &test_statistics_sti,
												   &zaru_test_statistics_sti,
												   &gyroscope_biases_sti,
												   &dt_sti,
												   &accelerometer_biases_sti,
												   &samsung_id_sti,
												   &imu0_rd_sti,
												   &imu1_rd_sti,
												   &imu2_rd_sti,
												   &imu3_rd_sti,
												   &imu4_rd_sti,
												   &imu5_rd_sti,
												   &imu6_rd_sti,
												   &imu7_rd_sti,
												   &imu8_rd_sti,
												   &imu9_rd_sti,
												   &imu10_rd_sti,
												   &imu11_rd_sti,
												   &imu12_rd_sti,
												   &imu13_rd_sti,
												   &imu14_rd_sti,
												   &imu15_rd_sti,
												   &imu16_rd_sti,
												   &imu17_rd_sti,
												   &imu18_rd_sti,
												   &imu19_rd_sti,
												   &imu20_rd_sti,
												   &imu21_rd_sti,
												   &imu22_rd_sti,
												   &imu23_rd_sti,
												   &imu24_rd_sti,
												   &imu25_rd_sti,
												   &imu26_rd_sti,
												   &imu27_rd_sti,
												   &imu28_rd_sti,
												   &imu29_rd_sti,
												   &imu30_rd_sti,
												   &imu31_rd_sti};


state_t_info* state_info_access_by_id[SID_LIMIT];

void system_states_init(void){
	for(int i = 0;i<(sizeof(state_struct_array)/sizeof(state_struct_array[0])); i++){
		state_info_access_by_id[state_struct_array[i]->id] = state_struct_array[i];}
}	


//@}