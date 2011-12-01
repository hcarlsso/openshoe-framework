
/** \file
	\brief Definitions of commands and declarations of command response functions.
	
	\details This header file contains
	1) Declarations of the command response functions.
	2) Definitions of commands
	The command response functions are defined in external_interfaces.c.
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


#ifndef COMMANDS_H_
#define COMMANDS_H_


// Definition structure of commands
typedef const struct {
	uint8_t header;
	void (*cmd_response)(uint8_t**);
	uint8_t nrb_payload;
	uint8_t nr_fields;
	uint8_t field_widths[];
} command_structure;

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

// Arrays/tables to find appropriate commands
const static command_structure* commands[] = {&only_ack,
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
												  &acc_calibration_cmd};


#endif /* COMMANDS_H_ */

//@}