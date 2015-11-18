
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#include <string.h>
#include <board.h>

#include "external_interface.h"
#include "parsing_util.h"
#include "usb_interface.h"
#include "bluetooth_interface.h"
#include "can_interface.h"
#include "package_queue.h"

#include "control_tables.h"

#define COMMAND_FROM_USB 1
#define COMMAND_FROM_BT 2

///\name Buffer settings
//@{
#define RX_BUFFER_SIZE 512
#define TX_BUFFER_SIZE 512
#define MAX_RX_NRB 400
//@}

///\name State output divider limits
//@{
#define MAX_LOG2_DIVIDER 15
#define MIN_LOG2_DIVIDER 1
//@}

// external interface struct used to keep all resources for an interface
typedef struct extif {
	uint8_t intern_arg;
	uint16_t state_output_rate_divider[SID_LIMIT];
	uint16_t state_output_rate_counter[SID_LIMIT];
	bool state_output_cond[SID_LIMIT];
	pkg_queue queue;
	rxtx_buffer rx_buffer;
	int command_rx_timer;
	command_info* info_last_command;
	rxtx_buffer tx_buffer;
	uint16_t pkg_number;
	bool lossless_trans;
	bool (*is_connected)(void);
	bool (*is_data_available)(void);
	uint8_t (*get_byte)(uint8_t* dest);
} extif;

// USB interface resources
static uint8_t usb_rx_array[RX_BUFFER_SIZE];
static uint8_t usb_tx_array[TX_BUFFER_SIZE];
static extif usbif = {COMMAND_FROM_USB,{0},{0},{0}, {{0},{0},0,0,0,&usb_write_buf_nonblocking,&usb_write_buf_nonblocking_allornothing}, {usb_rx_array,usb_rx_array,usb_rx_array,0}, 0,NULL, {usb_tx_array,usb_tx_array,usb_tx_array,0}, 0,false,&is_usb_attached,&is_data_available,&get_byte_from_usb};
// Bluetooth interface resources
#ifdef BT_MODULE
static uint8_t bt_rx_array[RX_BUFFER_SIZE];
static uint8_t bt_tx_array[TX_BUFFER_SIZE];
static extif btif = {COMMAND_FROM_BT,{0},{0},{0}, {{0},{0},0,0,0,&bt_send_buf,&bt_send_buf_allornothing}, {bt_rx_array,bt_rx_array,bt_rx_array,0}, 0,NULL, {bt_tx_array,bt_tx_array,bt_tx_array,0}, 0,false,&is_bluetooth_paired,&bt_is_data_available,&bt_get_byte};
#endif
// CAN interface resources
#ifdef CAN_INTERFACE
static uint8_t can_rx_array[RX_BUFFER_SIZE];
static uint8_t can_tx_array[TX_BUFFER_SIZE];
static extif canif = {COMMAND_FROM_CAN,{0},{0},{0}, {{0},{0},0,0,0,&can_send_buf,&can_send_buf_allornothing}, {can_rx_array,can_rx_array,can_rx_array,0}, 0,NULL, {can_tx_array,can_tx_array,can_tx_array,0}, 0,false,&is_can_ready,&can_is_data_available,&can_get_byte};
#endif

// Communication logics
void rxif_cmd(extif* extif_p);
void txif_data(extif* extif_p);

// Helper functions for response functions
void if_set_state_output(uint8_t state_id, uint8_t divider,extif* extif_p);
void if_set_cond_output(uint8_t state_id,extif* extif_p);
void if_set_lossless_trans(bool onoff,extif* extif_p);
void if_remove_pkg_from_queue(uint16_t package_number,extif* extif_p);
void if_empty_pkg_queue(extif* extif_p);



void external_interface_init(void){
	// TODO: These initialization functions should be replaced by a code generating script/preprocessors
	// They only initialize constant arrays.
	commands_init();	
	system_states_init();
	processing_functions_init();
	
	usb_interface_init();
	
	#ifdef BT_MODULE
	bt_interface_init();
	#endif
	#ifdef CAN_INTERFACE
	can_interface_init();
	#endif
}

void transmit_data(void){
	txif_data(&usbif);
	
	#ifdef BT_MODULE
	txif_data(&btif);
	#endif
	#ifdef CAN_INTERFACE
	txif_data(&canif);
	#endif
}

void receive_command(void){
	rxif_cmd(&usbif);
	
	#ifdef BT_MODULE
	rxif_cmd(&btif);
	#endif
	#ifdef CAN_INTERFACE
	rxif_cmd(&canif);
	#endif
}

void set_state_output(uint8_t state_id, uint8_t divider,uint8_t intern_arg){
	if(intern_arg & COMMAND_FROM_USB)
		if_set_state_output(state_id,divider,&usbif);
	
	#ifdef BT_MODULE
	if(intern_arg & COMMAND_FROM_BT)
		if_set_state_output(state_id,divider,&btif);
	#endif
	#ifdef CAN_INTERFACE
	if(intern_arg & COMMAND_FROM_CAN)
	if_set_state_output(state_id,divider,&canif);
	#endif	
}
void set_cond_output(uint8_t state_id,uint8_t intern_arg){
	if(intern_arg & COMMAND_FROM_USB)
		if_set_cond_output(state_id,&usbif);
	
	#ifdef BT_MODULE
	if(intern_arg & COMMAND_FROM_BT)
		if_set_cond_output(state_id,&btif);
	#endif
	#ifdef CAN_INTERFACE
	if(intern_arg & COMMAND_FROM_CAN)
	if_set_cond_output(state_id,&	canif);
	#endif
}
void set_lossless_trans(bool onoff,uint8_t intern_arg){
	if(intern_arg & COMMAND_FROM_USB)
		if_set_lossless_trans(onoff,&usbif);
	
	#ifdef BT_MODULE
	if(intern_arg & COMMAND_FROM_BT)
		if_set_lossless_trans(onoff,&btif);
	#endif
	#ifdef CAN_INTERFACE
	if(intern_arg & COMMAND_FROM_CAN)
	if_set_lossless_trans(onoff,&canif);
	#endif
}

void empty_pkg_queues(uint8_t intern_arg){
	if(intern_arg & COMMAND_FROM_USB)
		if_empty_pkg_queue(&usbif);

	#ifdef BT_MODULE
	if(intern_arg & COMMAND_FROM_BT)
		if_empty_pkg_queue(&btif);
	#endif
	#ifdef CAN_INTERFACE
	if(intern_arg & COMMAND_FROM_CAN)
	if_empty_pkg_queue(&canif);
	#endif
}

void receive_pkg_ack(uint16_t package_number, uint8_t intern_arg){
	if(intern_arg & COMMAND_FROM_USB)
		if_remove_pkg_from_queue(package_number,&usbif);
	
	#ifdef BT_MODULE
	if(intern_arg & COMMAND_FROM_BT)
		if_remove_pkg_from_queue(package_number,&btif);
	#endif
	#ifdef CAN_INTERFACE
	if(intern_arg & COMMAND_FROM_CAN)
	if_remove_pkg_from_queue(package_number,&canif);
	#endif
}


static inline void send_ak(struct rxtx_buffer* buffer,extif* extif_p){
	uint8_t ack[4];
	ack[0] = 0xa0;
	ack[1] = *buffer->buffer;
	uint16_t chk = calc_checksum(ack,ack+1);
	ack[2] = MSB(chk);
	ack[3] = LSB(chk);
	add_package_to_queue(ack,4,0,SINGLE_TRANSMIT,&extif_p->queue);
}

static inline void assemble_output_data(rxtx_buffer* tx_buf,extif* extif_p){
	reset_buffer(tx_buf);
	uint8_t* const state_output_header_p = tx_buf->write_position;
	// Leave one blank buffer slot for package header (see below)
	increment_counter(tx_buf->write_position);
	// Leave two blank buffer slot for package number (see below)
	increment_counter(tx_buf->write_position);
	increment_counter(tx_buf->write_position);
	// Leave one blank buffer slot for payload size (see below)
	increment_counter(tx_buf->write_position);
	// Copy all enabled states to buffer
	for(int i = 0; i<SID_LIMIT; i++){
		if( extif_p->state_output_rate_divider[i] ){
			if( extif_p->state_output_rate_counter[i] == 0){
				extif_p->state_output_rate_counter[i] = extif_p->state_output_rate_divider[i];
				extif_p->state_output_cond[i]=true;
			}
			// The counter counts down since then the comparison at each procedure call can be done with a constant (0)
			extif_p->state_output_rate_counter[i]--;
		}
		if (extif_p->state_output_cond[i]){
			// Ensure no overflow occur and copy state to buffer
			if (tx_buf->write_position-tx_buf->buffer+state_info_access_by_id[i]->state_size <= TX_BUFFER_SIZE-CHECKSUM_BYTES) {
				memcpy(tx_buf->write_position,state_info_access_by_id[i]->state_p,state_info_access_by_id[i]->state_size);
				tx_buf->write_position+=state_info_access_by_id[i]->state_size;
			}
			extif_p->state_output_cond[i]=false;
		}
	}
	// If any data was added, add payload size and calculate and add checksum
	if(FIRST_PAYLOAD_BYTE!=tx_buf->write_position) {
		extif_p->pkg_number++;
		*state_output_header_p = STATE_OUTPUT_HEADER;
		*(state_output_header_p+1) = (extif_p->pkg_number & 0xFF00) >> 8;
		*(state_output_header_p+2) =  extif_p->pkg_number & 0xFF;
		*PAYLOAD_SIZE_BYTE=tx_buf->write_position-FIRST_PAYLOAD_BYTE;
		uint16_t checksum = calc_checksum(state_output_header_p,tx_buf->write_position-1);
		*tx_buf->write_position = MSB(checksum);
		increment_counter(tx_buf->write_position);
		*tx_buf->write_position = LSB(checksum);
		increment_counter(tx_buf->write_position);
	} else
		tx_buf->write_position = state_output_header_p;
}

void rxif_cmd(extif* extif_p){
	if(extif_p->is_connected()) {
		int rx_nrb_counter = NO_BYTES_RECEIVED_YET;
		while(extif_p->is_data_available() && receive_limit_not_reached(rx_nrb_counter)) {
			
			extif_p->get_byte(extif_p->rx_buffer.write_position);
			increment_counter(rx_nrb_counter);
			reset_timer(extif_p->command_rx_timer);
			
			if(is_new_header(extif_p->rx_buffer.nrb)) {
				if(is_valid_header(*extif_p->rx_buffer.write_position)) {
					extif_p->info_last_command = get_command_info(*extif_p->rx_buffer.write_position);
					extif_p->rx_buffer.nrb = get_expected_nrb(extif_p->info_last_command);
					increment_counter(extif_p->rx_buffer.write_position);
				} else
					reset_buffer(&extif_p->rx_buffer);
				continue;
			}
			else if(is_end_of_command(extif_p->rx_buffer.nrb)){
				if(has_valid_checksum(&extif_p->rx_buffer)){
					parse_and_execute_command(&extif_p->rx_buffer,extif_p->info_last_command,extif_p->intern_arg);
					if (get_command_header(&extif_p->rx_buffer) != ACK_ID)
						send_ak(&extif_p->rx_buffer,extif_p);
				}
				reset_buffer(&extif_p->rx_buffer);
				continue;
			}
			increment_counter(extif_p->rx_buffer.write_position);
			decrement_counter(extif_p->rx_buffer.nrb);
		}
		if(!extif_p->is_data_available() && has_timed_out(extif_p->command_rx_timer,extif_p->rx_buffer.nrb))
			reset_buffer(&extif_p->rx_buffer);
	} else
		reset_buffer(&extif_p->rx_buffer);
}

void txif_data(extif* extif_p){
	if(extif_p->is_connected()){
		// Push potential old data into the send buffers before preparing new (for improved performance)
		if(!extif_p->lossless_trans)
			send_and_remove_data_from_queue(&extif_p->queue);
		assemble_output_data(&extif_p->tx_buffer,extif_p);
		add_package_to_queue(extif_p->tx_buffer.buffer,extif_p->tx_buffer.write_position-extif_p->tx_buffer.buffer,extif_p->pkg_number, extif_p->lossless_trans ? 0 : SINGLE_TRANSMIT,&extif_p->queue);
		if (extif_p->lossless_trans)
			send_package_from_queue(&extif_p->queue);
		else
			send_and_remove_data_from_queue(&extif_p->queue);
	}
}

void if_set_state_output(uint8_t state_id, uint8_t divider,extif* extif_p){
	if(state_id<SID_LIMIT && state_info_access_by_id[state_id]){
		if (divider>=MIN_LOG2_DIVIDER){
			uint16_t rate_divider = 1<<( (divider&MAX_LOG2_DIVIDER) - 1 );
			uint16_t rate_divider_reminder_mask = rate_divider - 1;
			uint16_t min_counter = 0;
			if (rate_divider>1)
				for(int i=0;i<SID_LIMIT;i++) // Synchronize output with remaining output
					if(extif_p->state_output_rate_divider[i])
						min_counter = max(min_counter,extif_p->state_output_rate_counter[i]&rate_divider_reminder_mask);
			extif_p->state_output_rate_divider[state_id] = rate_divider;
			extif_p->state_output_rate_counter[state_id] = min_counter;
		} else {
			extif_p->state_output_rate_divider[state_id] = 0;
			extif_p->state_output_rate_counter[state_id] = 0;
		}
	}
}

void if_set_cond_output(uint8_t state_id,extif* extif_p){
	if(state_id<SID_LIMIT && state_info_access_by_id[state_id])
		extif_p->state_output_cond[state_id]=true;
}

void if_set_lossless_trans(bool onoff,extif* extif_p){
	extif_p->lossless_trans = onoff;
}

void if_remove_pkg_from_queue(uint16_t package_number,extif* extif_p){
	remove_pkg_from_queue(package_number,&extif_p->queue);
}

void if_empty_pkg_queue(extif* extif_p){
	empty_pkg_queue(&extif_p->queue);
}