﻿/*
 * bluetooth_interface.c
 *
 * Created: 2014-03-18 10:11:26
 *  Author: jnil02
 */ 

#include <gpio.h>
#include <string.h>
#include "bluetooth_interface.h"
#include "parsing_util.h"
#include "control_tables.h"

#include "bt_uart_interrupt.h"
#include <usart.h>
#include "conf_clock.h"

//#if defined(MIMU22BT)
#  include "MIMU22BT.h"
//#endif

///\name Buffer settings
//@{
#define RX_BUFFER_SIZE 20
#define TX_BUFFER_SIZE 255
#define SINGLE_TX_BUFFER_SIZE 10
#define MAX_RX_NRB 10
//@}

///\name State output divider limits
//@{
#define MAX_LOG2_DIVIDER 14
#define MIN_LOG2_DIVIDER 0
//@}

///\name State output control variables
//@{
static uint16_t state_output_rate_divider[SID_LIMIT] = {0};
static uint16_t state_output_rate_counter[SID_LIMIT] = {0};
static bool state_output_cond[SID_LIMIT] = {0};
//@}

void bt_interface_init(void){
	
	gpio_enable_pin_pull_down(BT_PAIRED);
	
	static const gpio_map_t USART_GPIO_MAP =
	{
		{BT_UART_RX, BT_UART_RX_FUNC},
		{BT_UART_TX, BT_UART_TX_FUNC}
	};

	// USART options.
	static const usart_options_t USART_OPTIONS =
	{
		.baudrate     = 115200,
		.charlength   = 8,
		.paritytype   = USART_NO_PARITY,
		.stopbits     = USART_1_STOPBIT,
		.channelmode  = USART_NORMAL_CHMODE
	};
	
	// Assign GPIO to USART.
	gpio_enable_module(USART_GPIO_MAP, sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));

	// Initialize USART in RS232 mode.
	usart_init_rs232(BT_UART_P, &USART_OPTIONS, CLOCK_FREQ);
}

static inline bool is_bluetooth_paired(void){
return gpio_get_pin_value(BT_PAIRED);}

static inline void send_ak(struct rxtx_buffer* buffer){
	uint8_t ack[4];
	ack[0] = 0xa0;
	ack[1] = *buffer->buffer;
	uint16_t chk = calc_checksum(ack,ack+1);
	ack[2] = MSB(chk);
	ack[3] = LSB(chk);
	bt_send_buf(ack,4);
}

static inline void send_nak(void){
	uint8_t nack[3] = {161,0,161};
	bt_send_buf(nack,3);
}

static inline void assemble_output_data(struct rxtx_buffer* buffer){
	// Clear buffer
	reset_buffer(buffer);
	
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
		if (state_output_cond[i]){
			//TODO: ensure no buffer overflow occur
			memcpy(buffer->write_position,state_info_access_by_id[i]->state_p,state_info_access_by_id[i]->state_size);
			buffer->write_position+=state_info_access_by_id[i]->state_size;
			state_output_cond[i]=false;
		}
	}
	// If any data was added, add payload size and calculate and add checksum
	if(FIRST_PAYLOAD_BYTE!=buffer->write_position){
		*PAYLOAD_SIZE_BYTE=buffer->write_position-FIRST_PAYLOAD_BYTE;
		uint16_t checksum = calc_checksum(state_output_header_p,buffer->write_position-1);
		//TODO: ensure no buffer overflow occur
		*buffer->write_position = MSB(checksum);
		increment_counter(buffer->write_position);
		*buffer->write_position = LSB(checksum);
	increment_counter(buffer->write_position);}
	else {
		buffer->write_position = state_output_header_p;
	}
}

void bt_receive_command(void){
	static uint8_t rx_buffer_array[RX_BUFFER_SIZE];
	static struct rxtx_buffer rx_buffer = {rx_buffer_array,rx_buffer_array,rx_buffer_array,0};
	static int command_tx_timer;
	static command_structure* info_last_command;
	int rx_nrb_counter = NO_BYTES_RECEIVED_YET;
	
	// If Bluetooth paired and data is available, receive data (command)
	if(is_bluetooth_paired()) {
		while(bt_is_data_available() && receive_limit_not_reached(rx_nrb_counter)){
			
			bt_get_byte(rx_buffer.write_position);
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
					parse_and_execute_command(&rx_buffer,info_last_command,COMMAND_FROM_BT);
					}
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
	} else{
		reset_buffer(&rx_buffer);
	}
	return;
}

void bt_transmit_data(void){
	static uint8_t tx_buffer_array[TX_BUFFER_SIZE];
	static struct rxtx_buffer tx_buffer = {tx_buffer_array,tx_buffer_array,tx_buffer_array,0};
	//	static uint8_t downsampling_tx_counter = 0;

	if(is_bluetooth_paired()){
		gpio_set_pin_high(LED0);
		// Generate output
		assemble_output_data(&tx_buffer);
		// Transmit output
		
		
		if (tx_buffer.read_position<tx_buffer.write_position) {
			bt_send_buf(tx_buffer.read_position,tx_buffer.write_position-tx_buffer.read_position);
//			udi_cdc_putc(*tx_buffer.read_position);
//			tx_buffer.read_position++;
		}
		
		
		// This does not work cause it will hang if user does not clear his buffers sufficiently fast.
		// Equivalent but non-blocking function should be written.
		//		udi_cdc_write_buf((int*)tx_buffer.buffer,tx_buffer.write_position-tx_buffer.buffer);
	} else {
		gpio_set_pin_low(LED0);
	}
}

void bt_set_state_output(uint8_t state_id, uint8_t divider){
	if(state_id<=SID_LIMIT && divider<=MAX_LOG2_DIVIDER){	
		if (divider>MIN_LOG2_DIVIDER){
			uint16_t rate_divider = 1<<(divider-1);
			uint16_t min_divider = 1<<(MAX_LOG2_DIVIDER-1);
			uint16_t min_counter = 1; // 1 (instead of 0) to ensure that ACK and new output does not coincide
			// Synchronize output with remaining output
			// TODO: Test if this works!!! (could be a problem with output_imu_rd (would take to long time)?)
/*			if (rate_divider>1)
			{
				min_counter = 0xFFFF - 0xFFFF%rate_divider;
				for(int i=0;i<SID_LIMIT;i++)
				{
					if(state_output_rate_divider[i] && state_output_rate_divider[i]=<min_divider){
						if (state_output_rate_counter[i]<min_counter)
						{
							min_counter=state_output_rate_counter[i];
							min_divider=state_output_rate_divider[i];
						}
					}
				}
				min_counter = min_counter%rate_divider;
			}*/
			state_output_rate_divider[state_id] = rate_divider;
			state_output_rate_counter[state_id] = min_counter;
		} else {
			state_output_rate_divider[state_id] = 0;
			state_output_rate_counter[state_id] = 0;
		}
	}	
	// TODO: Set some error state if the above does not hold
}

/**
	\brief Reset the output counter such that the output become synchronized.
	
	\details Resets all output counters to zero. Since the output rate dividers
	will always be a power of two. This mean that all outputs will be
	synchronized independent of their dividers.
*/
void bt_reset_output_counters(void){
	for(int i=0;i<SID_LIMIT;i++){
		state_output_rate_counter[i] = 0;
	}
}

void bt_set_conditional_output(uint8_t state_id){
	state_output_cond[state_id]=true;
}
