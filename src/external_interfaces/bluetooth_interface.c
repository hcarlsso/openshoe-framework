
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#include <gpio.h>
#include <string.h>
#include "bluetooth_interface.h"
#include "parsing_util.h"
#include "control_tables.h"
#include "package_queue.h"
#include "bt_uart_interrupt.h"
#include <usart.h>
#include "conf_clock.h"

//#if defined(MIMU22BT)
#  include "MIMU22BT.h"
//#endif

///\name Buffer settings
//@{
#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 255
#define SINGLE_TX_BUFFER_SIZE 10
#define MAX_RX_NRB 10
//@}

///\name State output divider limits
//@{
#define MAX_LOG2_DIVIDER 15
#define MIN_LOG2_DIVIDER 2
//@}

#define	LOSSY_TRANSMISSION_BIT_MASK 16

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
	if ( gpio_get_pin_value(BT_PAIRED) ) {
		gpio_set_pin_high(LED0);
		return true;
	} else {
		gpio_set_pin_low(LED0);
		return false;
	}
}

static inline void send_ak(struct rxtx_buffer* buffer){
	uint8_t ack[4];
	ack[0] = 0xa0;
	ack[1] = *buffer->buffer;
	uint16_t chk = calc_checksum(ack,ack+1);
	ack[2] = MSB(chk);
	ack[3] = LSB(chk);
	bt_send_buf(ack,4);
}

// TODO: Remove these. Both USB and BT uses the same now.
#define BT_FIRST_PAYLOAD_BYTE (state_output_header_p+4)
#define BT_PAYLOAD_SIZE_BYTE (state_output_header_p+3)

static inline uint16_t assemble_output_data(struct rxtx_buffer* buffer){
	static uint16_t package_number = 0;
	
	// Clear buffer
	reset_buffer(buffer);
	
	// Save position of header
	uint8_t* state_output_header_p = buffer->write_position;
	// Add header for state data output
	*buffer->write_position = STATE_OUTPUT_HEADER;
	increment_counter(buffer->write_position);
	// Leave two blank buffer slot for package number (see below)
	increment_counter(buffer->write_position);
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
	if(BT_FIRST_PAYLOAD_BYTE!=buffer->write_position) {
		package_number++;
		*(state_output_header_p+1) = (package_number & 0xFF00) >> 8;
		*(state_output_header_p+2) =  package_number & 0xFF;
		*BT_PAYLOAD_SIZE_BYTE=buffer->write_position-BT_FIRST_PAYLOAD_BYTE;
		uint16_t checksum = calc_checksum(state_output_header_p,buffer->write_position-1);
		//TODO: ensure no buffer overflow occur
		*buffer->write_position = MSB(checksum);
		increment_counter(buffer->write_position);
		*buffer->write_position = LSB(checksum);
		increment_counter(buffer->write_position);
	} else {
		buffer->write_position = state_output_header_p;
	}
	return package_number;
}

void handle_ack(uint8_t** cmd_arg){
	uint8_t from = (uint8_t)cmd_arg[0];
	if (from & COMMAND_FROM_BT)	{
		uint16_t package_number = (cmd_arg[1][0]<<8) | cmd_arg[1][1];
		remove_package_from_queue(package_number);
	}
}

void bt_receive_command(void){
	static uint8_t rx_buffer_array[RX_BUFFER_SIZE];
	static struct rxtx_buffer rx_buffer = {rx_buffer_array,rx_buffer_array,rx_buffer_array,0};
	static int command_tx_timer;
	static command_info* info_last_command;
	int rx_nrb_counter = NO_BYTES_RECEIVED_YET;
	
	// If Bluetooth paired and data is available, receive data (command)
	if(is_bluetooth_paired()) {
		while(bt_is_data_available() && receive_limit_not_reached(rx_nrb_counter)){
			
			bt_get_byte(rx_buffer.write_position);
			increment_counter(rx_nrb_counter);
			reset_timer(command_tx_timer);
			
			// TODO: On wireless we should use a common starting header.
			
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
					// TODO: change order ot send_ak/send_nack. If parsing fails due to invalid arguments, we should send nack.
					if (get_command_header(&rx_buffer) != ACK_ID)
						send_ak(&rx_buffer);
					parse_and_execute_command(&rx_buffer,info_last_command,COMMAND_FROM_BT);
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
		}
	} else{
		reset_buffer(&rx_buffer);
	}
	return;
}

bool lossy_transmission = false;

void bt_transmit_data(void){
	static uint8_t tx_buffer_array[TX_BUFFER_SIZE];
	static struct rxtx_buffer tx_buffer = {tx_buffer_array,tx_buffer_array,tx_buffer_array,0};
	//	static uint8_t downsampling_tx_counter = 0;

	if(is_bluetooth_paired()){
		// Generate output
		uint16_t package_number = assemble_output_data(&tx_buffer);

		// Transmit output
		if (tx_buffer.read_position<tx_buffer.write_position) {
			if (lossy_transmission) {
				bt_send_buf(tx_buffer.read_position,tx_buffer.write_position-tx_buffer.read_position);
			} else {
				add_package_to_queue(tx_buffer.read_position,tx_buffer.write_position-tx_buffer.read_position,package_number);
			}
		}
		if (!lossy_transmission) {
			send_package_from_queue();
		}
	}
}

void bt_set_state_output(uint8_t state_id, uint8_t divider){
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

void bt_set_conditional_output(uint8_t state_id){
	if(state_info_access_by_id[state_id])
		state_output_cond[state_id]=true;
}

void bt_set_lossy_transmission(bool onoff){
	lossy_transmission = onoff;
}