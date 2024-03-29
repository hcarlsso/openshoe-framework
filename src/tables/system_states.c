
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

#include <stdint.h>
#include <string.h>

#include "control_tables.h"

///\cond
// Front-end variables
#include "inertial_frontend.h"
#include "nav_eq.h"
#include "timing_control.h"
#include "imu_interface.h"
#include "process_sequence.h"
#if defined(SMOOTHING)
#  include "smoothing.h"
#endif


// General purpose ID
uint8_t gp_id;

///\endcond

///  \name External state information
///  Structs containing information and pointers to the externally accessible system states.
//@{
static state_t_info imu_ts_sti = {IMU_TS_SID, (void*) &ts_u, sizeof(ts_u)};
static state_t_info interrupt_counter_sti = {INTERRUPT_COUNTER_SID, (void*) &interrupt_counter, sizeof(interrupt_counter)};
static state_t_info gp_dt_sti = {GP_DT_SID, (void*) &gp_dt, sizeof(gp_dt)};
static state_t_info mcu_id_sti = {MCU_ID_SID, (void*) 0x80800284, 0x80800292-0x80800284+1};
static state_t_info gp_id_sti = {GP_ID_SID, (void*) &gp_id, sizeof(gp_id)};
static state_t_info u_new_sti = {U_NEW_SID, (void*) &u_new, sizeof(u_new)};
static state_t_info u_int_k_sti = {U_INT_K_SID, (void*) &u_int_k, sizeof(u_int_k)};
static state_t_info t_int_k_sti = {T_INT_K_SID, (void*) &t_int_k, sizeof(t_int_k)};
static state_t_info u_k_sti = {U_K_SID, (void*) &u_k, sizeof(u_k)};
static state_t_info dt_sti = {DT_SID, (void*) &dt_k, sizeof(dt_k)};
static state_t_info T1s2f_sti = {T1S2F, (void*) &T1s2f, sizeof(T1s2f)};
static state_t_info T2s2f_sti = {T2S2F, (void*) &T2s2f, sizeof(T2s2f)};
static state_t_info zupt_sti = {ZUPT_SID, (void*) &zupt, sizeof(zupt)};
static state_t_info zaru_sti = {ZARU_SID, (void*) &zaru, sizeof(zaru)};
static state_t_info th_zupt_sti = {TH_ZUPT_SID, (void*) &th_zupt, sizeof(th_zupt)};
static state_t_info th_zaru_sti = {TH_ZARU_SID, (void*) &th_zaru, sizeof(th_zaru)};
static state_t_info u_int16_k_sti = {U_INT16_K_SID, (void*) &u_int16_k, sizeof(u_int16_k)};
static state_t_info pos_sti = {POSITION_SID, (void*) pos, sizeof(pos)};
static state_t_info vel_sti = {VELOCITY_SID, (void*) vel, sizeof(vel)};
static state_t_info quat_sti = {QUATERNION_SID, (void*) quat, sizeof(quat)};
static state_t_info P_sti = {P_SID, (void*) P, sizeof(P)};
static state_t_info init_done_sti = {INIT_DONE_SID, (void*) &init_done, sizeof(init_done)};
static state_t_info dx_sti = {DX_SID, (void*) dx, sizeof(dx)};
static state_t_info dP_sti = {DP_SID, (void*) dP, sizeof(dP)};
static state_t_info step_counter_sti = {STEP_COUNTER_SID, (void*) &step_counter, sizeof(step_counter)};
static state_t_info filter_reset_flag_sti = {FILTER_RESET_FLAG_SID, (void*) &filter_reset_flag, sizeof(filter_reset_flag)};
#if defined(SMOOTHING)
static state_t_info Pnn_sti = {PNN_SID, (void*) Pnn, sizeof(Pnn)};
static state_t_info Pr_sti = {PR_SID, (void*) Pr, sizeof(Pr)};
static state_t_info smoothing_data_sti = {SMOOTHING_DATA_SID, (void*) &send_buf, sizeof(send_buf)};
static state_t_info smoothing_done_sti = {SMOOTHING_DONE_SID, (void*) &smoothing_done, sizeof(smoothing_done)};
#endif
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

static state_t_info ps_dt0_sti = {PS_DT0_SID, (void*) &ps_dt[0], sizeof(ps_dt[0])};
static state_t_info ps_dt1_sti = {PS_DT1_SID, (void*) &ps_dt[1], sizeof(ps_dt[1])};
static state_t_info ps_dt2_sti = {PS_DT2_SID, (void*) &ps_dt[2], sizeof(ps_dt[2])};
static state_t_info ps_dt3_sti = {PS_DT3_SID, (void*) &ps_dt[3], sizeof(ps_dt[3])};
static state_t_info ps_dt4_sti = {PS_DT4_SID, (void*) &ps_dt[4], sizeof(ps_dt[4])};
static state_t_info ps_dt5_sti = {PS_DT5_SID, (void*) &ps_dt[5], sizeof(ps_dt[5])};
static state_t_info ps_dt6_sti = {PS_DT6_SID, (void*) &ps_dt[6], sizeof(ps_dt[6])};
static state_t_info ps_dt7_sti = {PS_DT7_SID, (void*) &ps_dt[7], sizeof(ps_dt[7])};
static state_t_info ps_dt8_sti = {PS_DT8_SID, (void*) &ps_dt[8], sizeof(ps_dt[8])};
static state_t_info ps_dt9_sti = {PS_DT9_SID, (void*) &ps_dt[9], sizeof(ps_dt[9])};
static state_t_info ps_dt10_sti = {PS_DT10_SID, (void*) &ps_dt[10], sizeof(ps_dt[10])};
static state_t_info ps_dt11_sti = {PS_DT11_SID, (void*) &ps_dt[11], sizeof(ps_dt[11])};
	
static state_t_info imu0_mag_sti = {IMU0_MAG_SID, (void*) &mimu_data[0][7], 6};
static state_t_info imu1_mag_sti = {IMU1_MAG_SID, (void*) &mimu_data[1][7], 6};
static state_t_info imu2_mag_sti = {IMU2_MAG_SID, (void*) &mimu_data[2][7], 6};
static state_t_info imu3_mag_sti = {IMU3_MAG_SID, (void*) &mimu_data[3][7], 6};
static state_t_info imu4_mag_sti = {IMU4_MAG_SID, (void*) &mimu_data[4][7], 6};
static state_t_info imu5_mag_sti = {IMU5_MAG_SID, (void*) &mimu_data[5][7], 6};
static state_t_info imu6_mag_sti = {IMU6_MAG_SID, (void*) &mimu_data[6][7], 6};
static state_t_info imu7_mag_sti = {IMU7_MAG_SID, (void*) &mimu_data[7][7], 6};
static state_t_info imu8_mag_sti = {IMU8_MAG_SID, (void*) &mimu_data[8][7], 6};
static state_t_info imu9_mag_sti = {IMU9_MAG_SID, (void*) &mimu_data[9][7], 6};
static state_t_info imu10_mag_sti = {IMU10_MAG_SID, (void*) &mimu_data[10][7], 6};
static state_t_info imu11_mag_sti = {IMU11_MAG_SID, (void*) &mimu_data[11][7], 6};
static state_t_info imu12_mag_sti = {IMU12_MAG_SID, (void*) &mimu_data[12][7], 6};
static state_t_info imu13_mag_sti = {IMU13_MAG_SID, (void*) &mimu_data[13][7], 6};
static state_t_info imu14_mag_sti = {IMU14_MAG_SID, (void*) &mimu_data[14][7], 6};
static state_t_info imu15_mag_sti = {IMU15_MAG_SID, (void*) &mimu_data[15][7], 6};
static state_t_info imu16_mag_sti = {IMU16_MAG_SID, (void*) &mimu_data[16][7], 6};
static state_t_info imu17_mag_sti = {IMU17_MAG_SID, (void*) &mimu_data[17][7], 6};
static state_t_info imu18_mag_sti = {IMU18_MAG_SID, (void*) &mimu_data[18][7], 6};
static state_t_info imu19_mag_sti = {IMU19_MAG_SID, (void*) &mimu_data[19][7], 6};
static state_t_info imu20_mag_sti = {IMU20_MAG_SID, (void*) &mimu_data[20][7], 6};
static state_t_info imu21_mag_sti = {IMU21_MAG_SID, (void*) &mimu_data[21][7], 6};
static state_t_info imu22_mag_sti = {IMU22_MAG_SID, (void*) &mimu_data[22][7], 6};
static state_t_info imu23_mag_sti = {IMU23_MAG_SID, (void*) &mimu_data[23][7], 6};
static state_t_info imu24_mag_sti = {IMU24_MAG_SID, (void*) &mimu_data[24][7], 6};
static state_t_info imu25_mag_sti = {IMU25_MAG_SID, (void*) &mimu_data[25][7], 6};
static state_t_info imu26_mag_sti = {IMU26_MAG_SID, (void*) &mimu_data[26][7], 6};
static state_t_info imu27_mag_sti = {IMU27_MAG_SID, (void*) &mimu_data[27][7], 6};
static state_t_info imu28_mag_sti = {IMU28_MAG_SID, (void*) &mimu_data[28][7], 6};
static state_t_info imu29_mag_sti = {IMU29_MAG_SID, (void*) &mimu_data[29][7], 6};
static state_t_info imu30_mag_sti = {IMU30_MAG_SID, (void*) &mimu_data[30][7], 6};
static state_t_info imu31_mag_sti = {IMU31_MAG_SID, (void*) &mimu_data[31][7], 6};
//@}
	
// Array of state data type struct pointers
const static state_t_info* state_struct_array[] = {&imu_ts_sti,
												   &interrupt_counter_sti,
												   &dt_sti,
												   &gp_dt_sti,
												   &gp_id_sti,
												   &mcu_id_sti,
												   &u_new_sti,
												   &u_int_k_sti,
												   &t_int_k_sti,
												   &u_k_sti,
												   &T1s2f_sti,
												   &T2s2f_sti,
												   &zupt_sti,
												   &zaru_sti,
												   &th_zupt_sti,
												   &th_zaru_sti,
												   &u_int16_k_sti,
												   &pos_sti,
								 	               &vel_sti,
												   &quat_sti,
												   &P_sti,
												   &init_done_sti,
												   &dx_sti,
												   &dP_sti,
												   &step_counter_sti,
												   &filter_reset_flag_sti,
												   #if defined(SMOOTHING)
												   &Pnn_sti,
												   &Pr_sti,
												   &smoothing_data_sti,
												   &smoothing_done_sti,
												   #endif
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
												   &imu31_temp_sti,
												   &ps_dt0_sti,
												   &ps_dt1_sti,
												   &ps_dt2_sti,
												   &ps_dt3_sti,
												   &ps_dt4_sti,
												   &ps_dt5_sti,
												   &ps_dt6_sti,
												   &ps_dt7_sti,
												   &ps_dt8_sti,
												   &ps_dt9_sti,
												   &ps_dt10_sti,
												   &ps_dt11_sti,
												   &imu0_mag_sti,
												   &imu1_mag_sti,
												   &imu2_mag_sti,
												   &imu3_mag_sti,
												   &imu4_mag_sti,
												   &imu5_mag_sti,
												   &imu6_mag_sti,
												   &imu7_mag_sti,
												   &imu8_mag_sti,
												   &imu9_mag_sti,
												   &imu10_mag_sti,
												   &imu11_mag_sti,
												   &imu12_mag_sti,
												   &imu13_mag_sti,
												   &imu14_mag_sti,
												   &imu15_mag_sti,
												   &imu16_mag_sti,
												   &imu17_mag_sti,
												   &imu18_mag_sti,
												   &imu19_mag_sti,
												   &imu20_mag_sti,
												   &imu21_mag_sti,
												   &imu22_mag_sti,
												   &imu23_mag_sti,
												   &imu24_mag_sti,
												   &imu25_mag_sti,
												   &imu26_mag_sti,
												   &imu27_mag_sti,
												   &imu28_mag_sti,
												   &imu29_mag_sti,
												   &imu30_mag_sti,
												   &imu31_mag_sti};


state_t_info* state_info_access_by_id[SID_LIMIT];

void system_states_init(void){
	for(int i = 0;i<(sizeof(state_struct_array)/sizeof(state_struct_array[0])); i++)
		if(state_struct_array[i]->id<SID_LIMIT)
			state_info_access_by_id[state_struct_array[i]->id] = state_struct_array[i];
}

void set_state(uint8_t state_id,void* value){
	if(state_id<SID_LIMIT && state_info_access_by_id[state_id])
		memcpy(state_info_access_by_id[state_id]->state_p,value,state_info_access_by_id[state_id]->state_size);
}

void get_state(uint8_t state_id,void* value){
	if(state_id<SID_LIMIT && state_info_access_by_id[state_id])
		memcpy(value,state_info_access_by_id[state_id]->state_p,state_info_access_by_id[state_id]->state_size);
}

void* get_state_p(uint8_t state_id){
	if(state_id<SID_LIMIT && state_info_access_by_id[state_id])
		return state_info_access_by_id[state_id]->state_p;
	return NULL;
}

uint8_t get_state_size(uint8_t state_id){
	if(state_id<SID_LIMIT && state_info_access_by_id[state_id])
		return state_info_access_by_id[state_id]->state_size;
	return 0;
}


//@}