
/** \file
	\brief High level external user interface (USB).
	
	\details This file contains the functions for 1) receiving and parsing
	commands and executing commands responses 2) determining what to transmit
	and to transmit data to the system user via USB.
	
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
*/

///	\addtogroup user_interface
///	@{

// Needed for memcpy
#include <string.h>
#include "usb_interface.h"
#include "parsing_util.h"
#include "control_tables.h"

// All USB include files (possibly not all needed)
#include "conf_usb.h"
#include "udd.h"
#include "udc.h"
#include "udi_cdc.h"
#include "usbc_device.h"

///\name Buffer settings
//@{
#define RX_BUFFER_SIZE 512
#define TX_BUFFER_SIZE 512
#define SINGLE_TX_BUFFER_SIZE 10
#define MAX_RX_NRB 10
//@}

///\name State output divider limits
//@{
#define MAX_LOG2_DIVIDER 15
#define MIN_LOG2_DIVIDER 1
//@}

// Single transmit buffer
static uint8_t single_tx_buffer_array[SINGLE_TX_BUFFER_SIZE];
static struct rxtx_buffer single_tx_buffer = {single_tx_buffer_array,single_tx_buffer_array,single_tx_buffer_array,0};

///\name State output control variables
//@{
static uint16_t state_output_rate_divider[SID_LIMIT] = {0};
static uint16_t state_output_rate_counter[SID_LIMIT] = {0};
static bool state_output_cond[SID_LIMIT] = {0};
//@}

/// Initialization function for communication interface
void usb_interface_init(void){
	// Start usb controller	
	udc_start();
}

static inline bool is_usb_attached(void){
	return !Is_udd_detached();}

static inline bool is_data_available(void){
	return udi_cdc_is_rx_ready();}

static inline void get_byte_from_usb(uint8_t* write_position){
	*write_position = udi_cdc_getc();}
	
static inline void send_ak(struct rxtx_buffer* buffer){
	*single_tx_buffer.write_position = 0xa0;
	increment_counter(single_tx_buffer.write_position);
	*single_tx_buffer.write_position = *buffer->buffer;
	uint16_t chk = calc_checksum(single_tx_buffer.write_position-1,single_tx_buffer.write_position);
	increment_counter(single_tx_buffer.write_position);
	*single_tx_buffer.write_position = MSB(chk);
	increment_counter(single_tx_buffer.write_position);
	*single_tx_buffer.write_position = LSB(chk);
	increment_counter(single_tx_buffer.write_position);}
	
static inline void send_nak(void){
	*single_tx_buffer.write_position = 0xa1;
	increment_counter(single_tx_buffer.write_position);
	*single_tx_buffer.write_position = 0x0;
	increment_counter(single_tx_buffer.write_position);
	*single_tx_buffer.write_position = 0xa1;
	increment_counter(single_tx_buffer.write_position);}


/*! \brief This functions collect the data which is to be transmitted and stores it in the argument buffer.

	\details This function collect single output data (e.g. acks) from the \#single_tx_buffer and continual output data
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
	// Leave one blank buffer slot for payload size (see below)
	increment_counter(buffer->write_position);
	// Copy all enabled states to buffer	
	for(int i = 0; i<SID_LIMIT; i++){
		if( state_output_rate_divider[i] ){
			if( state_output_rate_counter[i] == 0){
				state_output_rate_counter[i] = state_output_rate_divider[i];
				state_output_cond[i]=true;
			}
			// The counter counts down since then the comparison at each procedure call can be done with a constant (0)
			state_output_rate_counter[i]--;
		}
		if(state_output_cond[i]){
			if (buffer->write_position-buffer->buffer+state_info_access_by_id[i]->state_size <= TX_BUFFER_SIZE-CHECKSUM_BYTES) {
				memcpy(buffer->write_position,state_info_access_by_id[i]->state_p,state_info_access_by_id[i]->state_size);
				buffer->write_position+=state_info_access_by_id[i]->state_size;
			}
			state_output_cond[i]=false;
		}
	}
	// If any data was added, add payload size and calculate and add checksum
	if(FIRST_PAYLOAD_BYTE!=buffer->write_position){
		*PAYLOAD_SIZE_BYTE=buffer->write_position-FIRST_PAYLOAD_BYTE;
		uint16_t checksum = calc_checksum(state_output_header_p,buffer->write_position-1);
		*buffer->write_position = MSB(checksum);
		increment_counter(buffer->write_position);
		*buffer->write_position = LSB(checksum);
		increment_counter(buffer->write_position);}
	else {
		buffer->write_position = state_output_header_p;
	}
}

/**
	\brief Main function for receiving commands from user.

	\details In case the USB is attached (vbus is high) and there is data available the function
	reads in at most MAX_RX_NRB number of bytes, parses the commands, and execute the command
	response. In case the functions encounters a invalid header or checksum, it resets the
	buffer and starts over reading a new command assuming the next byte is a header. The
	function can split commands over different calls to the function.
*/
void usb_receive_command(void){
	static uint8_t rx_buffer_array[RX_BUFFER_SIZE];
	static struct rxtx_buffer rx_buffer = {rx_buffer_array,rx_buffer_array,rx_buffer_array,0};
	static uint32_t command_tx_timer;
	static command_info* info_last_command;
	int rx_nrb_counter = NO_BYTES_RECEIVED_YET;
	
	// If USB is attached and data is available, receive data (command)
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
					parse_and_execute_command(&rx_buffer,info_last_command,COMMAND_FROM_USB);}
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
			reset_buffer(&rx_buffer);
			send_nak();
		}
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
	
	\details The function calls /#assemble_output_data with its internal output buffer as an argument.
	This stores the relevant data (single output messages and continual state output) in the buffer.
	Subsequently the data is written over byte by byte to the USB output buffer.
*/
void usb_transmit_data(void){
	static uint8_t tx_buffer_array[TX_BUFFER_SIZE];
	static struct rxtx_buffer tx_buffer = {tx_buffer_array,tx_buffer_array,tx_buffer_array,0};
//	static uint8_t downsampling_tx_counter = 0;

	if(is_usb_attached()){
		// Generate output
		assemble_output_data(&tx_buffer);
		// Transmit output
		while(tx_buffer.read_position<tx_buffer.write_position && udi_cdc_is_tx_ready()){
			udi_cdc_putc(*tx_buffer.read_position);
			tx_buffer.read_position++;
		}
		// This does not work cause it will hang if user does not clear his buffers sufficiently fast.
		// Equivalent but non-blocking function should be written.
//		udi_cdc_write_buf((int*)tx_buffer.buffer,tx_buffer.write_position-tx_buffer.buffer);
	}
}

/**
	\brief Sets state_id state to be output with interrupt frequency divided by 2^(divider-1). Divider=0 turns off output.
	
	\details The function checks that state_id is a valid state ID and that
	divider is within the allowable range.
*/
void usb_set_state_output(uint8_t state_id, uint8_t divider){
	if(state_id<=SID_LIMIT && state_info_access_by_id[state_id]){
		if (divider>=MIN_LOG2_DIVIDER){
			uint16_t rate_divider = 1<<( (divider&MAX_LOG2_DIVIDER) - 1 );
			uint16_t rate_divider_reminder_mask = rate_divider - 1;
			uint16_t min_counter = 0;
			if (rate_divider>1)
			for(int i=0;i<SID_LIMIT;i++) // Synchronize output with remaining output
				if(state_output_rate_divider[i])
					min_counter = max(min_counter,state_output_rate_counter[i]&rate_divider_reminder_mask);
			state_output_rate_divider[state_id] = rate_divider;
			state_output_rate_counter[state_id] = min_counter;
			} else {
			state_output_rate_divider[state_id] = 0;
			state_output_rate_counter[state_id] = 0;
		}
	}
}

void usb_set_conditional_output(uint8_t state_id){
	if(state_info_access_by_id[state_id])
		state_output_cond[state_id]=true;
}


// Callback function for usb-vbus event (see conf_usb.h)
///\cond
void vbus_event_callback(bool b_high){
	b_high ? udc_attach() : udc_detach();}
///\endcond
	
// @}