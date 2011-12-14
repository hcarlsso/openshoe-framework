/*
 * commands.c
 *
 * Created: 2011-12-14 10:15:54
 *  Author: jnil02
 */ 

#include "commands.h"

// Command response functions
void retransmit_header(uint8_t**);
void retransmit_command_info(uint8_t**);
void output_state(uint8_t**);
void turn_off_output(uint8_t**);
void get_mcu_serial(uint8_t**);
void toggle_inertial_output(uint8_t**);
void position_plus_zupt(uint8_t**);
void output_navigational_states(uint8_t**);
void processing_onoff(uint8_t**);
void reset_zupt_aided_ins(uint8_t**);
void gyro_self_calibration(uint8_t**);
void acc_calibration(uint8_t **);
void set_low_pass_imu(uint8_t **);

// Definition of received commands
static command_structure only_ack = {0x01,NULL,0,0,{0}};
static command_structure mcu_id = {0x02,&get_mcu_serial,0,0,{0}};
static command_structure header_info = {0x03,&retransmit_command_info,1,1,{1}};
static command_structure command4 = {0x04,&retransmit_header,4,2,{2,2}};
static command_structure output_onoff_state = {0x20,&output_state,2,2,{1,1}};
static command_structure output_all_off = {0x21,&turn_off_output,0,0,{0}};
static command_structure output_onoff_inert = {0x22,&toggle_inertial_output,1,1,{1}};
static command_structure output_position_plus_zupt = {0x23,&position_plus_zupt,1,1,{1}};
static command_structure output_navigational_states_cmd = {0x24,&output_navigational_states,1,1,{1}};
static command_structure processing_function_onoff = {0x30,&processing_onoff,3,3,{1,1,1}};
static command_structure reset_system_cmd = {0x10,&reset_zupt_aided_ins,0,0,{0}};
static command_structure gyro_calibration_cmd = {0x11,&gyro_self_calibration,0,0,{0}};
static command_structure acc_calibration_cmd = {0x12,&acc_calibration,1,1,{1}};
static command_structure set_low_pass_imu_cmd = {0x13,&set_low_pass_imu,1,1,{1}};

// Arrays/tables to find appropriate commands
static const command_structure* commands[] = {&only_ack,
												  &mcu_id,
												  &header_info,
												  &command4,
												  &output_onoff_state,
												  &output_all_off,
												  &output_onoff_inert,
												  &output_position_plus_zupt,
												  &output_navigational_states_cmd,
												  &processing_function_onoff,
												  &reset_system_cmd,
												  &gyro_calibration_cmd,
												  &acc_calibration_cmd,
												  &set_low_pass_imu_cmd};
												  
// Arrays/tables for commands
uint8_t command_header_table[32]={0};
command_structure* command_info_array[256]={NULL};
												  
void commands_init(void){
	// Initialize tables
	for(int i = 0;i<(sizeof(commands)/sizeof(commands[0]));i++){
		command_header_table[ (commands[i]->header) >> 3 ] |= 1<<( (commands[i]->header) & 7);}
		
	for(int i = 0;i<(sizeof(commands)/sizeof(commands[0]));i++){
		command_info_array[commands[i]->header] = commands[i];}
}