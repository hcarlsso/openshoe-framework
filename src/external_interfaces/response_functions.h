/*
 * response_functions.h
 *
 * Created: 2015-01-18 19:33:52
 *  Author: jnil02
 */ 


#ifndef RESPONSE_FUNCTIONS_H_
#define RESPONSE_FUNCTIONS_H_

#include <stdint.h>

#if defined(MIMU3333)
#  include "MIMU3333.h"
#elif defined(MIMU4444)
#  include "MIMU4444.h"
#elif defined(MIMU22BT)
#  include "MIMU22BT.h"
#elif defined(MIMU4444BT)
#  include "MIMU4444BT.h"
#endif

#define NR_DEBUG_PROC 8
#define NR_DEBUG_OUTPUT 8

///  \name Command response functions
///  Functions which are executed in response to commands.
//@{
extern void handle_ack(uint8_t**);
void do_nothing(uint8_t**);
void get_mcu_serial(uint8_t**);
void input_imu_rd(uint8_t**);
void setup_debug_processing(uint8_t**);
void set_state_max_1byte(uint8_t**);
void set_state_max_4bytes(uint8_t**);
void set_state_max_12bytes(uint8_t**);
void set_state_max_24bytes(uint8_t**);
void set_state_max_48bytes(uint8_t**);
void set_state_max_254bytes(uint8_t**);
void output_state(uint8_t**);
void output_multiple_states(uint8_t**);
void output_imu_rd(uint8_t**);
void all_output_off(uint8_t**);
void state_if_setup_resp(uint8_t**);
void state_if_add_state(uint8_t**);
void set_proc(uint8_t**);
void set_mult_proc(uint8_t**);
void all_proc_off(uint8_t**);
void restore_proc_if_setup(uint8_t** cmd_arg);
void zupt_aided_ins(uint8_t**);
void reset_zupt_aided_ins2(uint8_t**);
void stepwise_dead_reckoning(uint8_t**);
void stepwise_dead_reckoning_TOR(uint8_t**);
void start_inertial_frontend(uint8_t**);
void normal_imu(uint8_t**);
void normal_imu_with_bias_est(uint8_t**);
//@}


#endif /* RESPONSE_FUNCTIONS_H_ */