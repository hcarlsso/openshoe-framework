
/** \file
	\brief High level external user interface (USB).
	
	\details This file contains the functions for 1) receiving and parsing
	commands and executing commands responses 2) determining what to transmit
	and to transmit data to the system user via USB.
	
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
*/ 

#include <string.h>
#include <math.h>
#include "compiler.h"
#include "external_interface.h"
#include "udi_cdc.h"
#include "gpio.h"

#define RX_BUFFER_SIZE 20
#define TX_BUFFER_SIZE 60
#define SINGLE_TX_BUFFER_SIZE 10
#define MIN_DATA 4
#define MAX_RX_NRB 10

#define NO_EXPECTED_BYTES 0
#define SINGLE_BYTE_EXPECTED 1

#define MAX_COMMAND_ARGS 10
#define USB_TIMEOUT_COUNT 200000000
#define CHECKSUM_BYTES 2
#define HEADER_BYTES 1
#define NO_INITIATED_TRANSMISSION 0
#define COUNTER_RESET_VALUE 0
#define NO_BYTES_RECEIVED_YET 0

#define STATE_OUTPUT_HEADER 0xAA

static struct rxtx_buffer{
	uint8_t* buffer;
	uint8_t* write_position;
	uint8_t* read_position;
	int  nrb;
};

// Single transmit buffer
static uint8_t single_tx_buffer_array[SINGLE_TX_BUFFER_SIZE];
static struct rxtx_buffer single_tx_buffer = {single_tx_buffer_array,single_tx_buffer_array,single_tx_buffer_array,0};

// Error variables
uint8_t error_signal=0;							//Error signaling vector. If zero no error has occurred.

/************************ Command definitions and auxiliary objects ***************************/
struct command_structure {
	uint8_t header;
	void (*cmd_response)(uint8_t**);
	uint8_t nrb_payload;
	uint8_t nr_fields;
	uint8_t field_widths[];};

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
const static struct command_structure only_ack = {0x01,NULL,0,0,{0}};
const static struct command_structure mcu_id = {0x02,&get_mcu_serial,0,0,{0}};
const static struct command_structure header_info = {0x03,&retransmit_command_info,1,1,{1}};
const static struct command_structure command4 = {0x04,&retransmit_header,4,2,{2,2}};
const static struct command_structure output_onoff_state = {0x20,&output_state,2,2,{1,1}};
const static struct command_structure output_all_off = {0x21,&turn_off_output,0,0,{0}};
const static struct command_structure output_onoff_inert = {0x22,&toggle_inertial_output,1,1,{1}};
const static struct command_structure output_position_plus_zupt = {0x23,&position_plus_zupt,1,1,{1}};
const static struct command_structure output_navigational_states_cmd = {0x24,&output_navigational_states,1,1,{1}};
const static struct command_structure processing_function_onoff = {0x30,&processing_onoff,3,3,{1,1,1}};
const static struct command_structure reset_system_cmd = {0x10,&reset_zupt_aided_ins,0,0,{0}};
const static struct command_structure gyro_calibration_cmd = {0x11,&gyro_self_calibration,0,0,{0}};
const static struct command_structure acc_calibration_cmd = {0x12,&acc_calibration,1,1,{1}};

// Arrays/tables to find appropriate commands
const static struct command_structure* commands[] = {&only_ack,
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
// Arrays/tables for 
static uint8_t command_header_table[32]={0};
static struct command_structure* command_info_array[256]={NULL};
/**********************************************************************************************/

/********************* System output definitions and auxiliary objects ************************/
// State data type information
struct state_t_info {
	uint8_t id;
	void* state_p;
	int state_size; };

// State variables defined in system
extern vec3 position;
extern vec3 velocity;
extern vec3 accelerations_in;
extern vec3 angular_rates_in;
extern int window_size;
extern int time_since_last_zupt;
extern uint32_t process_cycle_counter;
extern quat_vec quaternions;
extern bool zupt;
extern vec3 imu_temperaturs;
extern precision imu_supply_voltage;
extern vec3 accelerometer_biases;


// State ids (arbitrary number between 0x00 and 0xFF)
#define POSITION_SID 0x01
#define VELOCITY_SID 0x02
#define QUATERNION_SID 0x03
#define SPECIFIC_FORCE_SID 0x11
#define ANGULAR_RATE_SID 0x12
#define WINDOW_SIZE_PID 0x03
#define TIME_SINCE_LAST_ZUPT_SID 0x04
#define PROCESS_CYCLE_COUNTER_SID 0x05
#define ZUPT_SID 0x32
#define IMU_TEMPERATURS_SID 0x33
#define IMU_SUPPLY_VOLTAGE_SID 0x34
#define ACCELEROMETER_BIASES_SID 0x35

// State pointer and size information
const static struct state_t_info position_sti = {POSITION_SID, (void*) position, sizeof(vec3)};
const static struct state_t_info velocity_sti = {VELOCITY_SID, (void*) velocity, sizeof(vec3)};
const static struct state_t_info quaternions_sti = {QUATERNION_SID, (void*) quaternions, sizeof(quat_vec)};
const static struct state_t_info specific_force_sti = {SPECIFIC_FORCE_SID, (void*) accelerations_in, sizeof(vec3)};
const static struct state_t_info angular_rate_sti = {ANGULAR_RATE_SID, (void*) angular_rates_in, sizeof(vec3)};
const static struct state_t_info window_size_sti = {WINDOW_SIZE_PID, (void*) &window_size, sizeof(int)};
const static struct state_t_info time_since_last_zupt_sti = {TIME_SINCE_LAST_ZUPT_SID, (void*) &time_since_last_zupt, sizeof(int)};
const static struct state_t_info process_cycle_counter_sti = {PROCESS_CYCLE_COUNTER_SID, (void*) &process_cycle_counter, sizeof(int)};
const static struct state_t_info zupt_sti = {ZUPT_SID, (void*) &zupt, sizeof(bool)};
const static struct state_t_info imu_temperaturs_sti = {IMU_TEMPERATURS_SID, (void*) imu_temperaturs, sizeof(vec3)};
const static struct state_t_info imu_supply_voltage_sti = {IMU_SUPPLY_VOLTAGE_SID, (void*) &imu_supply_voltage, sizeof(precision)};
const static struct state_t_info accelerometer_biases_sti = {ACCELEROMETER_BIASES_SID, (void*) &accelerometer_biases, sizeof(vec3)};

// Array of state data type struct pointers
const struct state_t_info* state_struct_array[] = {&position_sti,
								 	               &velocity_sti,
												   &specific_force_sti,
												   &angular_rate_sti,
												   &window_size_sti,
												   &time_since_last_zupt_sti,
												   &process_cycle_counter_sti,
												   &quaternions_sti,
												   &zupt_sti,
												   &imu_temperaturs_sti,
												   &imu_supply_voltage_sti,
												   &accelerometer_biases_sti};

#define SID_LIMIT 256

// Bit-field of states which are to be included in the data transmission
static struct state_t_info* state_info_access_by_id[SID_LIMIT];
static uint8_t state_output_rate_divider[SID_LIMIT] = {0};
static uint8_t state_output_rate_counter[SID_LIMIT] = {0};
/**********************************************************************************************/

/*********************************** Processing functions *************************************/
// Information struct for processing functions
struct proc_func_info {
	uint8_t id;
	void (*func_p)(void);
	int max_proc_time; };

// Externally declared processing functions
extern void update_imu_data_buffers(void);
extern void initialize_navigation_algorithm(void);
extern void strapdown_mechanisation_equations(void);
extern void time_up_data(void);
extern void ZUPT_detector(void);
extern void zupt_update(void);
extern void precision_gyro_bias_null_calibration(void);
extern void calibrate_accelerometers(void);

// Processing functions ids
#define UPDATE_BUFFER 0x04
#define INITIAL_ALIGNMENT 0x05
#define MECHANIZATION 0x06
#define TIME_UPDATE 0x07
#define ZUPT_DETECTOR 0x08
#define ZUPT_UPDATE 0x09
#define GYRO_CALIBRATION 0x10
#define ACCELEROMETER_CALIBRATION 0x11

const static struct proc_func_info update_imu_data_buffers_info = {UPDATE_BUFFER,&update_imu_data_buffers,0};
const static struct proc_func_info initialize_navigation_algorithm_info = {INITIAL_ALIGNMENT,&initialize_navigation_algorithm,0};
const static struct proc_func_info strapdown_mechanisation_equations_info = {MECHANIZATION,&strapdown_mechanisation_equations,0};
const static struct proc_func_info time_up_data_info = {TIME_UPDATE,&time_up_data,0};
const static struct proc_func_info ZUPT_detector_info = {ZUPT_DETECTOR,&ZUPT_detector,0};
const static struct proc_func_info zupt_update_info = {ZUPT_UPDATE,&zupt_update,0};
const static struct proc_func_info precision_gyro_bias_null_calibration_info = {GYRO_CALIBRATION,&precision_gyro_bias_null_calibration,0};
const static struct proc_func_info calibrate_accelerometers_info = {ACCELEROMETER_CALIBRATION,&calibrate_accelerometers,0};

const struct proc_func_info* processing_functions[] = {&update_imu_data_buffers_info,
													   &initialize_navigation_algorithm_info,
													   &strapdown_mechanisation_equations_info,
													   &time_up_data_info,
													   &ZUPT_detector_info,
													   &zupt_update_info,
													   &precision_gyro_bias_null_calibration_info,
													   &calibrate_accelerometers_info};

// Array containing the processing functions to run
static struct proc_func_info* processing_functions_by_id[256];
/**********************************************************************************************/

void com_interface_init(void){
	// Start usb controller	
	udc_start();
	
	// Initialize tables
	for(int i = 0;i<(sizeof(commands)/sizeof(commands[0]));i++){
		command_header_table[ (commands[i]->header) >> 3 ] |= 1<<( (commands[i]->header) & 7);}
		
	for(int i = 0;i<(sizeof(commands)/sizeof(commands[0]));i++){
		command_info_array[commands[i]->header] = commands[i];}
		
	for(int i = 0;i<(sizeof(state_struct_array)/sizeof(state_struct_array[0])); i++){
		state_info_access_by_id[state_struct_array[i]->id] = state_struct_array[i];}
		
	for(int i = 0;i<(sizeof(processing_functions)/sizeof(processing_functions[0])); i++){
		processing_functions_by_id[processing_functions[i]->id] = processing_functions[i];}
}

/***************** Define and inline functions to improve readability of code *****************/
#define receive_limit_not_reached(rx_nrb_counter) ((rx_nrb_counter)<MAX_RX_NRB)
#define reset_timer(timer) ((timer) = Get_system_register(AVR32_COUNT))
#define is_new_header(exp_nrb) ((exp_nrb) == NO_EXPECTED_BYTES)
#define is_end_of_command(exp_nrb) (exp_nrb) == SINGLE_BYTE_EXPECTED
#define has_timed_out(timeout_counter, exp_nrb) ((timeout_counter) + USB_TIMEOUT_COUNT < Get_system_register(AVR32_COUNT) && (exp_nrb) > 0)
#define increment_counter(counter) ((counter)++)
#define decrement_counter(counter) ((counter)--)

static inline void reset_buffer(struct rxtx_buffer* buffer){
	buffer->write_position = buffer->buffer;
	buffer->read_position = buffer->buffer;
	buffer->nrb = NO_INITIATED_TRANSMISSION;}

static inline bool is_usb_attached(void){
	return !Is_udd_detached();}

static inline bool is_data_available(void){
	return udi_cdc_is_rx_ready();}

static inline void get_byte_from_usb(uint8_t* write_position){
	*write_position = udi_cdc_getc();}
	
inline bool is_valid_header(uint8_t header){
	return command_header_table[header>>3] & (1<<(header & 7));}
	
inline static struct command_structure* get_command_info(uint8_t header){
	return command_info_array[header];}
	
static inline int get_payload_size(struct command_structure* cmd_info){
	return cmd_info->nrb_payload;}

inline static int get_expected_nrb(struct command_structure* cmd_info){
	return get_payload_size(cmd_info) + CHECKSUM_BYTES;}
	
static inline uint16_t calc_checksum(uint8_t* first, uint8_t* last){
	int length = last-first+1;
	uint16_t checksum = 0;
	while (length-- > 0){
		checksum += *first++;}
	return checksum;}

static inline bool has_valid_checksum(struct rxtx_buffer* buffer){
	uint16_t checksum = calc_checksum(buffer->buffer,buffer->write_position-2);
	return (MSB(checksum)==*(buffer->write_position-1) && LSB(checksum)==*buffer->write_position);}

static inline void send_ak(struct rxtx_buffer* buffer){
	*single_tx_buffer.write_position = 0xa0;
	increment_counter(single_tx_buffer.write_position);
	*single_tx_buffer.write_position = *buffer->buffer;
	uint16_t chk = calc_checksum(single_tx_buffer.write_position-1,single_tx_buffer.write_position);
	increment_counter(single_tx_buffer.write_position);
	*single_tx_buffer.write_position = MSB(chk);
	increment_counter(single_tx_buffer.write_position);
	*single_tx_buffer.write_position = LSB(chk);
	increment_counter(single_tx_buffer.write_position);
	
//	uint8_t ak[4] = { 0xa0, *buffer->buffer, 0, *buffer->buffer };
//	udi_cdc_write_buf((int*) ak,4);
	}
static inline void send_nak(void){
	*single_tx_buffer.write_position = 0xa1;
	increment_counter(single_tx_buffer.write_position);
	*single_tx_buffer.write_position = 0x0;
	increment_counter(single_tx_buffer.write_position);
	*single_tx_buffer.write_position = 0xa1;
	increment_counter(single_tx_buffer.write_position);
//	uint8_t nak[3] = { 0xa1, 0, 0 };
//	udi_cdc_write_buf((int*) nak,3);
}

static inline void parse_and_execute_command(struct rxtx_buffer* buffer,struct command_structure* cmd_info){
	static uint8_t* command_arg[MAX_COMMAND_ARGS];
	uint8_t* arg_p = buffer->buffer+1;	
	// Parse command arguments
	for(int i=0;i<cmd_info->nr_fields;i++){
		command_arg[i]=arg_p;
		arg_p+=cmd_info->field_widths[i];}		
	// Execute command response
	cmd_info->cmd_response(command_arg);}


/*! \brief This functions collect the data which is to be transmitted and stores it in the argument buffer.

	\detail This function collect single output data (e.g. acks) from the \#single_tx_buffer and continual output data
	from the state variables based on the values of the \#state_output_rate_divider and the \#state_output_rate_counter.
	The output data (single output data followed by state outputs) is stored in the argument buffer.
	The argument buffer is reset before any data is written to it.
	
	@param[out] buffer						The buffer in which that output data is stored in.
	@param[in]  single_tx_buffer			Buffer containing the data to be output once.
	@param[in]  state_output_rate_divider	Array containing the dividers of the state output frequencies (and if they are to be output at all)
	@param[in]  state_output_rate_counter	Array for keeping track of when to output data next
*/
static inline void assemble_output_data(struct rxtx_buffer* buffer){
	// Clear buffer
	reset_buffer(buffer);
	
	// Copy one time transmit data to buffer
	if(single_tx_buffer.write_position!=single_tx_buffer.buffer){
		memcpy(buffer->write_position,single_tx_buffer.buffer,single_tx_buffer.write_position-single_tx_buffer.buffer);
		buffer->write_position+=single_tx_buffer.write_position-single_tx_buffer.buffer;
		reset_buffer(&single_tx_buffer);
	}

	// Save position of header
	uint8_t* state_output_header_p = buffer->write_position;
	// Add header for state data output
	*buffer->write_position = STATE_OUTPUT_HEADER;
	increment_counter(buffer->write_position);
	// Copy all enabled states to buffer	
	for(int i = 0; i<SID_LIMIT; i++){
		if( state_output_rate_divider[i] ){
			//TODO: Let counter count down instead of up
			state_output_rate_counter[i]++;
			if( state_output_rate_counter[i] == state_output_rate_divider[i]){
				state_output_rate_counter[i] = 0;
				//TODO: ensure no buffer overflow occur
				memcpy(buffer->write_position,state_info_access_by_id[i]->state_p,state_info_access_by_id[i]->state_size);
				buffer->write_position+=state_info_access_by_id[i]->state_size;
			}
		}
	}
	// If any data was added, calculate and add checksum
	if(state_output_header_p+1!=buffer->write_position){
		uint16_t checksum = calc_checksum(state_output_header_p,buffer->write_position-1);
		*buffer->write_position = MSB(checksum);
		increment_counter(buffer->write_position);
		*buffer->write_position = LSB(checksum);
		increment_counter(buffer->write_position);}
	else {
//		reset_buffer(buffer);
		buffer->write_position = state_output_header_p;
	}
}

/**
	\brief Main function for receiving commands from user.

	\detail In case the USB is attached (vbus is high) and there is data available the function
	reads in at most MAX_RX_NRB number of bytes, parses the commands, and execute the command
	response. In case the functions encounters a invalid header or checksum, it resets the
	buffer and starts over reading a new command assuming the next byte is a header. The
	function can split commands over different calls to the function.
*/
void receive_command(void){
	static uint8_t rx_buffer_array[RX_BUFFER_SIZE];
	static struct rxtx_buffer rx_buffer = {rx_buffer_array,rx_buffer_array,rx_buffer_array,0};
	static int command_tx_timer;
	static struct command_structure* info_last_command;
	int rx_nrb_counter = NO_BYTES_RECEIVED_YET;
	
	//If USB is attached and data is available, receive data (command)
	if(is_usb_attached()){
		while(is_data_available() && receive_limit_not_reached(rx_nrb_counter)){
			
			get_byte_from_usb(rx_buffer.write_position);
			increment_counter(rx_nrb_counter);
			reset_timer(command_tx_timer);
			
			// Do we have a potential new header?
			if(is_new_header(rx_buffer.nrb)){
				if(is_valid_header(*rx_buffer.write_position)){
					info_last_command = get_command_info(*rx_buffer.write_position);
					rx_buffer.nrb = get_expected_nrb(info_last_command);
					increment_counter(rx_buffer.write_position);}
				else{
					reset_buffer(&rx_buffer);}
				continue;}
				
			// Or a full command?
			else if(is_end_of_command(rx_buffer.nrb)){
				if(has_valid_checksum(&rx_buffer)){
					send_ak(&rx_buffer);
					parse_and_execute_command(&rx_buffer,info_last_command);}
				else{
					send_nak();
				}
				reset_buffer(&rx_buffer);
				continue;}
				
			// Otherwise we are in the middle of a command transmission
			increment_counter(rx_buffer.write_position);
			decrement_counter(rx_buffer.nrb);
		}
		// Reset buffer if initiated command transmission do not complete within timeout limit
		if(has_timed_out(command_tx_timer,rx_buffer.nrb)){
			reset_buffer(&rx_buffer);}
	}
	// If USB detached, reset buffers
	else{
		reset_buffer(&rx_buffer);
	}
	// Return if: USB detached, no more data available, or receive-limit reached
	return;	
}

/**
	\brief Main function to output data to user.
	
	\detail The function calls /#assemble_output_data with its internal output buffer as an argument.
	This stores the relevant data (single output messages and continual state output) in the buffer.
	Subsequently the data is written over byte by byte to the USB output buffer.
*/
void transmit_data(void){
	static uint8_t tx_buffer_array[TX_BUFFER_SIZE];
	static struct rxtx_buffer tx_buffer = {tx_buffer_array,tx_buffer_array,tx_buffer_array,0};
	static uint8_t downsampling_tx_counter = 0;

	if(is_usb_attached()){
//		bool any_data = false;
		// Generate output
		assemble_output_data(&tx_buffer);
//		if((tx_buffer.write_position-tx_buffer.buffer)!=0){
//			any_data = true;}
		// Transmit output
		while(tx_buffer.read_position<tx_buffer.write_position && udi_cdc_is_tx_ready()){
			udi_cdc_putc(*tx_buffer.read_position);
			tx_buffer.read_position++;
		}
//		udi_cdc_write_buf((int*)tx_buffer.buffer,tx_buffer.write_position-tx_buffer.buffer);
	}	
}


/******************************** Command callback functions **********************************/
void retransmit_header(uint8_t** command){
	udi_cdc_putc(*(*command-1));
	return;}

void retransmit_command_info(uint8_t** header_p){
	uint8_t header = **header_p;
	// Check that header is valid
	if(is_valid_header(header)){
		struct command_structure* cmd_info = get_command_info(header);
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
	udi_cdc_write_buf((int*)0x80800284,0x80800292-0x80800284);}

void output_state(uint8_t** cmd_arg){
	uint8_t state_id = cmd_arg[0][0];
	uint8_t output_divider    = cmd_arg[1][0];
	uint8_t id_bit = 1 << (state_id % 8);
	if(state_info_access_by_id[state_id]){  // Valid state?
		state_output_rate_divider[state_id] = output_divider;
		state_output_rate_counter[state_id] = 0;}}

void toggle_inertial_output(uint8_t** cmd_arg){
	uint8_t output_divider = cmd_arg[0][0];
	state_output_rate_divider[ANGULAR_RATE_SID] = output_divider;
	state_output_rate_counter[ANGULAR_RATE_SID] = 0;
	state_output_rate_divider[SPECIFIC_FORCE_SID] = output_divider;
	state_output_rate_counter[SPECIFIC_FORCE_SID] = 0;}

void position_plus_zupt(uint8_t** cmd_arg){
	uint8_t output_divider = cmd_arg[0][0];
	state_output_rate_divider[POSITION_SID] = output_divider;
	state_output_rate_counter[POSITION_SID] = 0;
	state_output_rate_divider[ZUPT_SID] = output_divider;
	state_output_rate_counter[ZUPT_SID] = 0;}

void output_navigational_states(uint8_t** cmd_arg){
	uint8_t output_divider = cmd_arg[0][0];
	state_output_rate_divider[POSITION_SID] = output_divider;
	state_output_rate_counter[POSITION_SID] = 0;
	state_output_rate_divider[VELOCITY_SID] = output_divider;
	state_output_rate_counter[VELOCITY_SID] = 0;
	state_output_rate_divider[QUATERNION_SID] = output_divider;
	state_output_rate_counter[QUATERNION_SID] = 0;}

void turn_off_output(uint8_t** cmd_arg){
	for(int i = 0; i<SID_LIMIT; i++){
		state_output_rate_divider[i]=0;
		state_output_rate_counter[i]=0;}}

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
/**********************************************************************************************/

// Callback function for usb-vbus event (see conf_usb.h)
void vbus_event_callback(bool b_high){
	b_high ? udc_attach() : udc_detach();}