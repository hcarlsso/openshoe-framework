
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
extern precision dt;

// Front-end variables
extern inert_int32 u_new;
extern inert_int32 u_int_k;
extern inert_float u_k;
extern uint32_t T1s2f;
extern uint32_t T2s2f;
extern bool zupt;
extern bool zaru;
	
// Filtering states
extern vec3 pos;
extern vec3 vel;
extern quat_vec quat;
extern mat9sym P;

// Step-wise dead reckoning data exchange states
extern vec4 dx;
extern mat4sym dP;
extern uint16_t step_counter;

// System states
extern uint32_t interrupt_counter;
extern uint32_t gp_dt;

// "Other" states
extern vec3 accelerometer_biases;
extern uint8_t samsung_id;

// IMU register states
// The matrix is split up into 32 stats for the user to access (one for each IMU, see below)
extern int16_t mimu_data[32][7];
extern uint32_t ts_u;
///\endcond

///  \name External state information
///  Structs containing information and pointers to the externally accessible system states.
//@{
// TODO: change such that it's sizeof(variable) rather than sizeof(type)
static state_t_info specific_force_sti = {SPECIFIC_FORCE_SID, (void*) accelerations_in, sizeof(vec3)};
static state_t_info angular_rate_sti = {ANGULAR_RATE_SID, (void*) angular_rates_in, sizeof(vec3)};
static state_t_info imu_temperaturs_sti = {IMU_TEMPERATURS_SID, (void*) imu_temperaturs, sizeof(vec3)};
static state_t_info imu_supply_voltage_sti = {IMU_SUPPLY_VOLTAGE_SID, (void*) &imu_supply_voltage, sizeof(precision)};
static state_t_info u_new_sti = {U_NEW_SID, (void*) &u_new, sizeof(inert_int32)};
static state_t_info u_int_k_sti = {U_INT_K_SID, (void*) &u_int_k, sizeof(inert_int32)};
static state_t_info u_k_sti = {U_K_SID, (void*) &u_k, sizeof(inert_float)};
static state_t_info T1s2f_sti = {T1S2F, (void*) &T1s2f, sizeof(uint32_t)};
static state_t_info T2s2f_sti = {T2S2F, (void*) &T2s2f, sizeof(uint32_t)};
static state_t_info position_sti = {POSITION_SID, (void*) pos, sizeof(vec3)};
static state_t_info velocity_sti = {VELOCITY_SID, (void*) vel, sizeof(vec3)};
static state_t_info quaternions_sti = {QUATERNION_SID, (void*) quat, sizeof(quat_vec)};
static state_t_info P_sti = {P_SID, (void*) P, sizeof(P)};
static state_t_info zupt_sti = {ZUPT_SID, (void*) &zupt, sizeof(bool)};
static state_t_info zaru_sti = {ZARU_SID, (void*) &zaru, sizeof(bool)};
static state_t_info dt_sti = {DT_SID, (void*) &dt, sizeof(dt)};
static state_t_info dx_sti = {DX_SID, (void*) dx, sizeof(vec4)};
static state_t_info dP_sti = {DP_SID, (void*) dP, sizeof(mat4sym)};
static state_t_info step_counter_sti = {STEP_COUNTER_SID, (void*) &step_counter, sizeof(uint16_t)};
static state_t_info imu_ts_sti = {IMU_TS_SID, (void*) &ts_u, sizeof(ts_u)};
static state_t_info interrupt_counter_sti = {INTERRUPT_COUNTER_SID, (void*) &interrupt_counter, sizeof(uint32_t)};
static state_t_info gp_dt_sti = {GP_DT_SID, (void*) &gp_dt, sizeof(uint32_t)};
static state_t_info accelerometer_biases_sti = {ACCELEROMETER_BIASES_SID, (void*) accelerometer_biases, sizeof(vec3)};
static state_t_info samsung_id_sti = {SAMSUNG_ID_SID, (void*) &samsung_id, sizeof(uint8_t)};
static state_t_info imu0_rd_sti = {IMU0_RD_SID, (void*) mimu_data[0], 12};
static state_t_info imu1_rd_sti = {IMU1_RD_SID, (void*) mimu_data[1], 12};
static state_t_info imu2_rd_sti = {IMU2_RD_SID, (void*) mimu_data[2], 12};
static state_t_info imu3_rd_sti = {IMU3_RD_SID, (void*) mimu_data[3], 12};
static state_t_info imu4_rd_sti = {IMU4_RD_SID, (void*) mimu_data[4], 12};
static state_t_info imu5_rd_sti = {IMU5_RD_SID, (void*) mimu_data[5], 12};
static state_t_info imu6_rd_sti = {IMU6_RD_SID, (void*) mimu_data[6], 12};
static state_t_info imu7_rd_sti = {IMU7_RD_SID, (void*) mimu_data[7], 12};
static state_t_info imu8_rd_sti = {IMU8_RD_SID, (void*) mimu_data[8], 12};
static state_t_info imu9_rd_sti = {IMU9_RD_SID, (void*) mimu_data[9], 12};
static state_t_info imu10_rd_sti = {IMU10_RD_SID, (void*) mimu_data[10], 12};
static state_t_info imu11_rd_sti = {IMU11_RD_SID, (void*) mimu_data[11], 12};
static state_t_info imu12_rd_sti = {IMU12_RD_SID, (void*) mimu_data[12], 12};
static state_t_info imu13_rd_sti = {IMU13_RD_SID, (void*) mimu_data[13], 12};
static state_t_info imu14_rd_sti = {IMU14_RD_SID, (void*) mimu_data[14], 12};
static state_t_info imu15_rd_sti = {IMU15_RD_SID, (void*) mimu_data[15], 12};
static state_t_info imu16_rd_sti = {IMU16_RD_SID, (void*) mimu_data[16], 12};
static state_t_info imu17_rd_sti = {IMU17_RD_SID, (void*) mimu_data[17], 12};
static state_t_info imu18_rd_sti = {IMU18_RD_SID, (void*) mimu_data[18], 12};
static state_t_info imu19_rd_sti = {IMU19_RD_SID, (void*) mimu_data[19], 12};
static state_t_info imu20_rd_sti = {IMU20_RD_SID, (void*) mimu_data[20], 12};
static state_t_info imu21_rd_sti = {IMU21_RD_SID, (void*) mimu_data[21], 12};
static state_t_info imu22_rd_sti = {IMU22_RD_SID, (void*) mimu_data[22], 12};
static state_t_info imu23_rd_sti = {IMU23_RD_SID, (void*) mimu_data[23], 12};
static state_t_info imu24_rd_sti = {IMU24_RD_SID, (void*) mimu_data[24], 12};
static state_t_info imu25_rd_sti = {IMU25_RD_SID, (void*) mimu_data[25], 12};
static state_t_info imu26_rd_sti = {IMU26_RD_SID, (void*) mimu_data[26], 12};
static state_t_info imu27_rd_sti = {IMU27_RD_SID, (void*) mimu_data[27], 12};
static state_t_info imu28_rd_sti = {IMU28_RD_SID, (void*) mimu_data[28], 12};
static state_t_info imu29_rd_sti = {IMU29_RD_SID, (void*) mimu_data[29], 12};
static state_t_info imu30_rd_sti = {IMU30_RD_SID, (void*) mimu_data[30], 12};
static state_t_info imu31_rd_sti = {IMU31_RD_SID, (void*) mimu_data[31], 12};

static state_t_info imu0_temp_sti = {IMU0_TEMP_SID, (void*) &mimu_data[0][6], 2};
static state_t_info imu1_temp_sti = {IMU1_TEMP_SID, (void*) &mimu_data[1][6], 2};
static state_t_info imu2_temp_sti = {IMU2_TEMP_SID, (void*) &mimu_data[2][6], 2};
static state_t_info imu3_temp_sti = {IMU3_TEMP_SID, (void*) &mimu_data[3][6], 2};
static state_t_info imu4_temp_sti = {IMU4_TEMP_SID, (void*) &mimu_data[4][6], 2};
static state_t_info imu5_temp_sti = {IMU5_TEMP_SID, (void*) &mimu_data[5][6], 2};
static state_t_info imu6_temp_sti = {IMU6_TEMP_SID, (void*) &mimu_data[6][6], 2};
static state_t_info imu7_temp_sti = {IMU7_TEMP_SID, (void*) &mimu_data[7][6], 2};
static state_t_info imu8_temp_sti = {IMU8_TEMP_SID, (void*) &mimu_data[8][6], 2};
static state_t_info imu9_temp_sti = {IMU9_TEMP_SID, (void*) &mimu_data[9][6], 2};
static state_t_info imu10_temp_sti = {IMU10_TEMP_SID, (void*) &mimu_data[10][6], 2};
static state_t_info imu11_temp_sti = {IMU11_TEMP_SID, (void*) &mimu_data[11][6], 2};
static state_t_info imu12_temp_sti = {IMU12_TEMP_SID, (void*) &mimu_data[12][6], 2};
static state_t_info imu13_temp_sti = {IMU13_TEMP_SID, (void*) &mimu_data[13][6], 2};
static state_t_info imu14_temp_sti = {IMU14_TEMP_SID, (void*) &mimu_data[14][6], 2};
static state_t_info imu15_temp_sti = {IMU15_TEMP_SID, (void*) &mimu_data[15][6], 2};
static state_t_info imu16_temp_sti = {IMU16_TEMP_SID, (void*) &mimu_data[16][6], 2};
static state_t_info imu17_temp_sti = {IMU17_TEMP_SID, (void*) &mimu_data[17][6], 2};
static state_t_info imu18_temp_sti = {IMU18_TEMP_SID, (void*) &mimu_data[18][6], 2};
static state_t_info imu19_temp_sti = {IMU19_TEMP_SID, (void*) &mimu_data[19][6], 2};
static state_t_info imu20_temp_sti = {IMU20_TEMP_SID, (void*) &mimu_data[20][6], 2};
static state_t_info imu21_temp_sti = {IMU21_TEMP_SID, (void*) &mimu_data[21][6], 2};
static state_t_info imu22_temp_sti = {IMU22_TEMP_SID, (void*) &mimu_data[22][6], 2};
static state_t_info imu23_temp_sti = {IMU23_TEMP_SID, (void*) &mimu_data[23][6], 2};
static state_t_info imu24_temp_sti = {IMU24_TEMP_SID, (void*) &mimu_data[24][6], 2};
static state_t_info imu25_temp_sti = {IMU25_TEMP_SID, (void*) &mimu_data[25][6], 2};
static state_t_info imu26_temp_sti = {IMU26_TEMP_SID, (void*) &mimu_data[26][6], 2};
static state_t_info imu27_temp_sti = {IMU27_TEMP_SID, (void*) &mimu_data[27][6], 2};
static state_t_info imu28_temp_sti = {IMU28_TEMP_SID, (void*) &mimu_data[28][6], 2};
static state_t_info imu29_temp_sti = {IMU29_TEMP_SID, (void*) &mimu_data[29][6], 2};
static state_t_info imu30_temp_sti = {IMU30_TEMP_SID, (void*) &mimu_data[30][6], 2};
static state_t_info imu31_temp_sti = {IMU31_TEMP_SID, (void*) &mimu_data[31][6], 2};
//@}
	
// Array of state data type struct pointers
const static state_t_info* state_struct_array[] = {&imu_ts_sti,
												   &interrupt_counter_sti,
												   &specific_force_sti,
												   &angular_rate_sti,
												   &imu_temperaturs_sti,
												   &imu_supply_voltage_sti,
												   &u_new_sti,
												   &u_int_k_sti,
												   &u_k_sti,
												   &T1s2f_sti,
												   &T2s2f_sti,
												   &position_sti,
												   &dx_sti,
												   &dP_sti,
												   &step_counter_sti,
								 	               &velocity_sti,
												   &quaternions_sti,
												   &P_sti,
												   &zupt_sti,
												   &zaru_sti,
												   &dt_sti,
												   &gp_dt_sti,
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
												   &imu31_rd_sti,
												   &imu0_temp_sti,
												   &imu1_temp_sti,
												   &imu2_temp_sti,
												   &imu3_temp_sti,
												   &imu4_temp_sti,
												   &imu5_temp_sti,
												   &imu6_temp_sti,
												   &imu7_temp_sti,
												   &imu8_temp_sti,
												   &imu9_temp_sti,
												   &imu10_temp_sti,
												   &imu11_temp_sti,
												   &imu12_temp_sti,
												   &imu13_temp_sti,
												   &imu14_temp_sti,
												   &imu15_temp_sti,
												   &imu16_temp_sti,
												   &imu17_temp_sti,
												   &imu18_temp_sti,
												   &imu19_temp_sti,
												   &imu20_temp_sti,
												   &imu21_temp_sti,
												   &imu22_temp_sti,
												   &imu23_temp_sti,
												   &imu24_temp_sti,
												   &imu25_temp_sti,
												   &imu26_temp_sti,
												   &imu27_temp_sti,
												   &imu28_temp_sti,
												   &imu29_temp_sti,
												   &imu30_temp_sti,
												   &imu31_temp_sti};


state_t_info* state_info_access_by_id[SID_LIMIT];

void system_states_init(void){
	for(int i = 0;i<(sizeof(state_struct_array)/sizeof(state_struct_array[0])); i++){
		state_info_access_by_id[state_struct_array[i]->id] = state_struct_array[i];}
}	


//@}