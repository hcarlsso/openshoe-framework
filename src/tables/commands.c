

#include "control_tables.h"

// Needed include for definition of command response functions
#include "external_interface.h"
#include "process_sequence.h"
#include "imu_interface.h"

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


void retransmit_header(uint8_t** command){
//	udi_cdc_putc(*(*command-1));
	return;}

void retransmit_command_info(uint8_t** header_p){
	uint8_t header = **header_p;
	// Check that header is valid
	if(is_valid_header(header)){
		command_structure* cmd_info = get_command_info(header);
/*
		udi_cdc_putc(cmd_info->header);
//		udi_cdc_putc(cmd_info->cmd_response);
		udi_cdc_putc(cmd_info->nrb_payload);
		udi_cdc_putc(cmd_info->nr_fields);
		udi_cdc_write_buf((int*)cmd_info->field_width,cmd_info->nr_fields);*/
		}
	else {
//		udi_cdc_putc(header);
	}
}


void get_mcu_serial(uint8_t** arg){
//	udi_cdc_write_buf((int*)0x80800284,0x80800292-0x80800284);
}

void output_state(uint8_t** cmd_arg){
	uint8_t state_id = cmd_arg[0][0];
	uint8_t output_divider    = cmd_arg[1][0];
	uint8_t id_bit = 1 << (state_id % 8);
	if(state_info_access_by_id[state_id]){  // Valid state?
		set_state_output(state_id,output_divider);}}

void toggle_inertial_output(uint8_t** cmd_arg){
	uint8_t output_divider = cmd_arg[0][0];
	set_state_output(ANGULAR_RATE_SID,output_divider);
	set_state_output(SPECIFIC_FORCE_SID,output_divider);}

void position_plus_zupt(uint8_t** cmd_arg){
	uint8_t output_divider = cmd_arg[0][0];
	set_state_output(POSITION_SID,output_divider);
	set_state_output(ZUPT_SID,output_divider);}

void output_navigational_states(uint8_t** cmd_arg){
	uint8_t output_divider = cmd_arg[0][0];
	set_state_output(POSITION_SID,output_divider);
	set_state_output(VELOCITY_SID,output_divider);
	set_state_output(QUATERNION_SID,output_divider);
	set_state_output(INTERRUPT_COUNTER_SID,output_divider);}

void turn_off_output(uint8_t** cmd_arg){
	for(uint8_t i = 0; i<max(SID_LIMIT,0xFF); i++){
		set_state_output(i,0);}}

void processing_onoff(uint8_t** cmd_arg){
	uint8_t function_id = cmd_arg[0][0];
	uint8_t onoff    = cmd_arg[1][0];
	uint8_t array_location = cmd_arg[2][0];	
	processing_function_p process_sequence_elem_value = onoff ? (processing_functions_by_id[function_id]->func_p) : NULL;
	set_elem_in_process_sequence(process_sequence_elem_value,array_location);
}

extern bool initialize_flag;
void stop_initial_alignement(void){
	if(initialize_flag==false){
		// Stop initial alignement
		empty_process_sequence();
		// Start ZUPT aided INS
		set_elem_in_process_sequence(processing_functions_by_id[UPDATE_BUFFER]->func_p,0);
		set_elem_in_process_sequence(processing_functions_by_id[MECHANIZATION]->func_p,1);
		set_elem_in_process_sequence(processing_functions_by_id[TIME_UPDATE]->func_p,2);
		set_elem_in_process_sequence(processing_functions_by_id[ZUPT_DETECTOR]->func_p,3);
		set_elem_in_process_sequence(processing_functions_by_id[ZUPT_UPDATE]->func_p,4);
	}
}

void reset_zupt_aided_ins(uint8_t** no_arg){
	// Stop whatever was going on
	empty_process_sequence();
	initialize_flag=true;
	// Start initial alignment
	set_elem_in_process_sequence(processing_functions_by_id[UPDATE_BUFFER]->func_p,0);
	set_elem_in_process_sequence(processing_functions_by_id[INITIAL_ALIGNMENT]->func_p,1);
	// Set termination function of initial alignment which will also start INS
	set_last_process_sequence_element(&stop_initial_alignement);
}

void gyro_self_calibration(uint8_t** no_arg){
	store_and_empty_process_sequence();
	set_elem_in_process_sequence(processing_functions_by_id[GYRO_CALIBRATION]->func_p,0);
	set_last_process_sequence_element(&restore_process_sequence);
}
	
	
extern Bool new_orientation_flag;
extern Bool acc_calibration_finished_flag;
void new_calibration_orientation(void){
	if(acc_calibration_finished_flag){
		acc_calibration_finished_flag = false;
		restore_process_sequence();
		udi_cdc_putc(103);
		//TODO: Send out acknowledgment that calibration was successful
	}
	if(new_orientation_flag==true){
		new_orientation_flag = false;
		//TODO: Send out request for new orientation position
		empty_process_sequence();
		udi_cdc_putc(102);
	}
}

extern uint8_t nr_of_calibration_orientations;
void acc_calibration(uint8_t ** cmd_arg){
	uint8_t nr_orientations = cmd_arg[0][0];
	nr_of_calibration_orientations = nr_orientations;
	
	store_and_empty_process_sequence();
	set_elem_in_process_sequence(processing_functions_by_id[ACCELEROMETER_CALIBRATION]->func_p,0);
	set_last_process_sequence_element(&new_calibration_orientation);
}

void set_low_pass_imu(uint8_t ** cmd_arg){
	uint8_t nr_filter_taps = cmd_arg[0][0];
	low_pass_filter_setting(nr_filter_taps);
}