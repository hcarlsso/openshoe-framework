/*
 * response_functions.h
 *
 * Created: 2015-01-18 19:33:52
 *  Author: jnil02
 */ 


#ifndef RESPONSE_FUNCTIONS_H_
#define RESPONSE_FUNCTIONS_H_

#include <stdint.h>

#include "user_board.h"

#define NR_DEBUG_PROC 10
#define NR_DEBUG_OUTPUT 10

///  \name Command response functions
///  Functions which are executed in response to commands.
//@{
void handle_ack(uint8_t**);
void do_nothing(uint8_t**);
void get_mcu_serial(uint8_t**);
void input_imu_rd(uint8_t**);
void gp_test_command(uint8_t**);
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
void set_imu_bandwidth(uint8_t**);
void set_proc(uint8_t**);
void set_mult_proc(uint8_t**);
void all_proc_off(uint8_t**);
void restore_proc_if_setup(uint8_t**);
void zupt_aided_ins(uint8_t**);
#if defined(SMOOTHING)
void smoothed_zupt_aided_ins(uint8_t**);
#endif
void reset_zupt_aided_ins2(uint8_t**);
void stepwise_dead_reckoning(uint8_t**);
void start_inertial_frontend(uint8_t**);
void normal_imu(uint8_t**);
void normal_imu_with_bias_est(uint8_t**);
//@}


#endif /* RESPONSE_FUNCTIONS_H_ */