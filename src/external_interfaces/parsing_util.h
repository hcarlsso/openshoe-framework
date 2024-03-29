﻿
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#ifndef PARSING_UTIL_H_
#define PARSING_UTIL_H_

#include "control_tables.h"
#include <stdint.h>
#include <stdbool.h>

///\cond
// Macros for improved readability. Excluded from doxygen.
#define CHECKSUM_BYTES 2
#define HEADER_BYTES 1
#define MAX_COMMAND_ARGS 255
#define NO_EXPECTED_BYTES 0
#define SINGLE_BYTE_EXPECTED 1
#define COM_TIMEOUT_LIMIT 200000000u
#define NO_INITIATED_TRANSMISSION 0
#define COUNTER_RESET_VALUE 0
#define NO_BYTES_RECEIVED_YET 0
#define STATE_OUTPUT_HEADER 0xAA
///\endcond

/// Receive and transmit buffer
typedef struct rxtx_buffer{
	uint8_t* const buffer;
	uint8_t* write_position;
	uint8_t* read_position;
	int  nrb;
} rxtx_buffer;

// Define and inline functions for improved readability of code
///\cond
#define receive_limit_not_reached(rx_nrb_counter) ((rx_nrb_counter)<MAX_RX_NRB)
#define reset_timer(timer) ((timer) = Get_system_register(AVR32_COUNT))
#define is_new_header(exp_nrb) ((exp_nrb) == NO_EXPECTED_BYTES)
#define is_end_of_command(exp_nrb) (exp_nrb) == SINGLE_BYTE_EXPECTED
#define has_timed_out(timeout_counter, exp_nrb) (Get_system_register(AVR32_COUNT) - (timeout_counter) > COM_TIMEOUT_LIMIT && (exp_nrb) > 0)
#define increment_counter(counter) ((counter)++)
#define decrement_counter(counter) ((counter)--)
#define FIRST_PAYLOAD_BYTE (state_output_header_p+4)
#define PAYLOAD_SIZE_BYTE (state_output_header_p+3)
///\endcond

static inline bool data_in_buffer(rxtx_buffer *buffer) {
	return buffer->read_position<buffer->write_position;
}

static inline uint8_t get_command_header(rxtx_buffer* buffer_p) {
	return buffer_p->buffer[0];}

static inline void reset_buffer(rxtx_buffer* buffer){
	buffer->write_position = buffer->buffer;
	buffer->read_position = buffer->buffer;
buffer->nrb = NO_INITIATED_TRANSMISSION;}

static inline int get_payload_size(command_info* cmd_info){
return cmd_info->nrb_payload;}

static inline int get_expected_nrb(command_info* cmd_info){
return get_payload_size(cmd_info) + CHECKSUM_BYTES;}

/// Calculates the 16-bit sum of all bytes between arguments.
static inline uint16_t calc_checksum(const uint8_t* first,const uint8_t* last){
	int length = last-first+1;
	uint16_t checksum = 0;
	while (length-- > 0){
	checksum += *first++;}
return checksum;}

static inline bool has_valid_checksum(const rxtx_buffer* buffer){
	uint16_t checksum = calc_checksum(buffer->buffer,buffer->write_position-CHECKSUM_BYTES);
return (MSB(checksum)==*(buffer->write_position-1) && LSB(checksum)==*buffer->write_position);}

static inline void parse_and_execute_command(const rxtx_buffer* buffer,command_info* cmd_info,uint8_t gp_arg){
	static uint8_t* command_arg[MAX_COMMAND_ARGS+1];
	command_arg[0] = gp_arg;
	const uint8_t* arg_p = buffer->buffer+1;
	// Parse command arguments
	for(int i=0;i<cmd_info->nr_fields;i++){
		command_arg[i+1]=arg_p;
	arg_p+=cmd_info->field_widths[i];}
	// Execute command response
cmd_info->cmd_response(command_arg);}

#endif /* PARSING_UTIL_H_ */