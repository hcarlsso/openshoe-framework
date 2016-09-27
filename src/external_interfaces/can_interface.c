
#include <string.h>

#include "can_interface.h"
#include "gpio.h"
#include "can.h"
#include "canif.h"
#include "user_board.h"
#include "scif_uc3c.h"

#ifdef CAN_INTERFACE


#define LOG2_SIZE_CAN_BUF 10
#define SIZE_CAN_BUF (1<<LOG2_SIZE_CAN_BUF)
#define CAN_BUF_MASK (SIZE_CAN_BUF-1)

static uint8_t can_rx_buf[SIZE_CAN_BUF];
static volatile uint32_t can_rx_buf_write=0;
static volatile uint32_t can_rx_buf_read=0;

static uint8_t can_tx_buf[SIZE_CAN_BUF];
static volatile uint32_t can_tx_buf_write=0;
static volatile uint32_t can_tx_buf_read=0;

#define MAX_CAN_MSG_SIZE 8


/* Local allocation for MOB buffer in HSB_RAM */
#if defined (__GNUC__) && defined (__AVR32__)
volatile can_msg_t mob_ram_ch0[NB_MOB_CHANNEL] __attribute__ ((section (".hsb_ram_loc")));
#elif defined (__ICCAVR32__)
volatile __no_init can_msg_t mob_ram_ch0[NB_MOB_CHANNEL] @0xA0000000;
#endif

/*
 * Mail Box Definition
 */
// -----------------------------------------------------------------
// CAN Message Definition: Tx Message
#if defined (__ICCAVR32__)
can_msg_t msg_tx_sot = {
  0x130,                    // Identifier
  0x1ff,                    // Mask
  0x0102030405060708LL,     // Data
};
#else
can_msg_t msg_tx_sot = {
  {
    {
      .id = 0x700,                    // Identifier
      .id_mask  = 0x1ff,              // Mask
    },
  },
 .data.u64 = 0x0102030405060708LL,    // Data
};
#endif

// MOB Message Definition: Tx Message
can_mob_t tx_msg = {
  CAN_MOB_NOT_ALLOCATED,            // Handle: by default CAN_MOB_NOT_ALLOCATED
  &msg_tx_sot,                      // Pointer on CAN Message
  0,                                // Data length DLC
  CAN_DATA_FRAME,                   // Request type : CAN_DATA_FRAME or CAN_REMOTE_FRAME
  CAN_STATUS_COMPLETED          // Status: by default CAN_STATUS_NOT_COMPLETED
};

// -----------------------------------------------------------------
// CAN Message Definition: Rx Message
#if defined (__ICCAVR32__)
can_msg_t msg_rx_listening = {
  0,                // Identifier
  0,                // Mask
  0x0LL,            // Data
};
#else
can_msg_t msg_rx_listening = {
  {
    {
      .id = 0,                      // Identifier
      .id_mask  = 0,                // Mask
    },
  },
 .data.u64 = 0x0LL,                 // Data
};
#endif

// MOB Message Definition: Tx Message
can_mob_t rx_msg = {
  CAN_MOB_NOT_ALLOCATED,            // Handle: by default CAN_MOB_NOT_ALLOCATED
  &msg_rx_listening,                // Pointer on CAN Message
  0,                                // Data length DLC
  CAN_DATA_FRAME,                   // Request type : CAN_DATA_FRAME or CAN_REMOTE_FRAME
  CAN_STATUS_NOT_COMPLETED          // Status: by default CAN_STATUS_NOT_COMPLETED
};


/** Call Back prototype */
void can_event_callback(U8 handle, U8 event);
/** Call Back called by can_drv */
void can_event_callback(U8 handle, U8 event)
{
	if (tx_msg.handle == handle) {
		switch (event) {
		case CAN_STATUS_COMPLETED:
			// Free and reset tx msg
			can_mob_free(CAN_CHANNEL,handle);
			tx_msg.handle = CAN_MOB_NOT_ALLOCATED;
			tx_msg.dlc = 0;
			tx_msg.req_type = CAN_DATA_FRAME;
			tx_msg.status = event;
			// More to send?
			if (can_tx_buf_write!=can_tx_buf_read) {
				// Try to allocate message object (mob)
				tx_msg.handle = can_mob_alloc(CAN_CHANNEL);
				tx_msg.status = CAN_STATUS_NOT_COMPLETED;
				if (tx_msg.handle!=CAN_CMD_REFUSED) {
					// Fill mob
					do {
						tx_msg.can_msg->data.u8[tx_msg.dlc] = can_tx_buf[can_tx_buf_read];
						can_tx_buf_read=(can_tx_buf_read+1)&CAN_BUF_MASK;
						tx_msg.dlc++;
					} while (tx_msg.dlc<MAX_CAN_MSG_SIZE && can_tx_buf_write!=can_tx_buf_read);
					// Push data
					can_tx(CAN_CHANNEL, tx_msg.handle, tx_msg.dlc, tx_msg.req_type, tx_msg.can_msg);
				}
				// Handle case when there is no mob
				// TODO
			}
			break;
		case CAN_STATUS_WAKEUP:
			;// Sleep not supported
		case CAN_STATUS_BUSOFF:
			;// Hardware error?
		case CAN_STATUS_ERROR:
			;// We have no error handling
		}
	} else if (rx_msg.handle == handle) {
		switch (event) {
		case (CAN_STATUS_COMPLETED):
			// Handle handle
			rx_msg.can_msg->data.u64 = can_get_mob_data(CAN_CHANNEL,handle).u64;
			rx_msg.can_msg->id = can_get_mob_id(CAN_CHANNEL,handle);
			rx_msg.dlc = can_get_mob_dlc(CAN_CHANNEL,handle);
			rx_msg.status = event;
			can_mob_free(CAN_CHANNEL,handle);
			// Filter messages
			// TODO
			// Copy message to internal buffers
			for (int i=0;i<rx_msg.dlc;i++) {
				can_rx_buf[can_rx_buf_write] = rx_msg.can_msg->data.u8[i];
				can_rx_buf_write=(can_rx_buf_write+1)&CAN_BUF_MASK;
			}
			// Set up for new reception
			rx_msg.handle = can_mob_alloc(CAN_CHANNEL);
			can_rx(CAN_CHANNEL, rx_msg.handle, rx_msg.req_type, rx_msg.can_msg);
			// Handle case when there is no mob
			// TODO
			break;
		case CAN_STATUS_WAKEUP:
			;// Sleep not supported
		case CAN_STATUS_BUSOFF:
			;// Hardware error?
		case CAN_STATUS_ERROR:
			;// We have no error handling
		}
	}
}
void can_request_tx(void) {
	// THIS IS NOT SAFE SINCE THE SAME THING IS DONE IN THE INTERRUPT ROUTINE
	if (can_tx_buf_write!=can_tx_buf_read && tx_msg.status == CAN_STATUS_COMPLETED) {
		tx_msg.handle = can_mob_alloc(CAN_CHANNEL);
		tx_msg.status = CAN_STATUS_NOT_COMPLETED;
		if (tx_msg.handle!=CAN_CMD_REFUSED) {
			// Fill mob
			do {
				tx_msg.can_msg->data.u8[tx_msg.dlc] = can_tx_buf[can_tx_buf_read];
				can_tx_buf_read=(can_tx_buf_read+1)&CAN_BUF_MASK;
				tx_msg.dlc++;
			} while (tx_msg.dlc<MAX_CAN_MSG_SIZE && can_tx_buf_write!=can_tx_buf_read);
			// Push data
			can_tx(CAN_CHANNEL, tx_msg.handle, tx_msg.dlc, tx_msg.req_type, tx_msg.can_msg);
		}
	}
}


void can_interface_init(void){
	// Setup the generic clock for CAN
	scif_gc_setup(AVR32_SCIF_GCLK_CANIF,
				SCIF_GCCTRL_OSC0,
				AVR32_SCIF_GC_NO_DIV_CLOCK,
				0);
	scif_gc_enable(AVR32_SCIF_GCLK_CANIF);

	// Assign GPIOs to CAN
	static const gpio_map_t CAN_GPIO_MAP = {
		{CAN_RX_PIN, CAN_RX_FUNC},
		{CAN_TX_PIN, CAN_TX_FUNC}
	};
	gpio_enable_module(CAN_GPIO_MAP,
	sizeof(CAN_GPIO_MAP) / sizeof(CAN_GPIO_MAP[0]));

	// Initialize CAN channel 0
	can_init(CAN_CHANNEL, ((uint32_t)&mob_ram_ch0[0]), CANIF_CHANNEL_MODE_NORMAL, can_event_callback);

	// Setup rx
	rx_msg.handle = can_mob_alloc(CAN_CHANNEL);
	can_rx(CAN_CHANNEL, rx_msg.handle, rx_msg.req_type, rx_msg.can_msg);
}

bool is_can_ready(void){
	return true;
}

uint32_t can_send_buf(const uint8_t* buf,uint32_t nob){
	uint32_t space_in_buf = (can_tx_buf_read-can_tx_buf_write)&CAN_BUF_MASK;
	space_in_buf = space_in_buf ? space_in_buf-1 : (SIZE_CAN_BUF-1);
	if(!space_in_buf)
		return nob;
	uint32_t nr_bytes_left = 0;
	if(space_in_buf<nob){
		nr_bytes_left = nob-space_in_buf;
		nob = space_in_buf;
	}
	int nob_to_end_of_buf = min(nob,SIZE_CAN_BUF-can_tx_buf_write);
	memcpy(can_tx_buf+can_tx_buf_write,buf,nob_to_end_of_buf);
	memcpy(can_tx_buf,buf+nob_to_end_of_buf,max(0,nob-nob_to_end_of_buf));
	can_tx_buf_write = (can_tx_buf_write+nob) & CAN_BUF_MASK;
//	BT_UART.ier = AVR32_USART_IER_TXRDY_MASK;

	// Run callback function
	can_request_tx();
	return nr_bytes_left;
	
	/* Allocate one mob for TX */
//	appli_tx_msg.handle = can_mob_alloc(0);

	/* Check return if no mob are available */
//	if (appli_tx_msg.handle==CAN_CMD_REFUSED)
//		while(1);

//	can_tx(0, appli_tx_msg.handle, appli_tx_msg.dlc, appli_tx_msg.req_type, appli_tx_msg.can_msg);
//	return 0;
}
uint32_t can_send_buf_allornothing(const uint8_t* buf,uint32_t nob){
	uint32_t space_in_buf = (can_tx_buf_read-can_tx_buf_write)&CAN_BUF_MASK;
	space_in_buf = space_in_buf ? space_in_buf-1 : (SIZE_CAN_BUF-1);
	if(space_in_buf<nob)
		return nob;
	int nob_to_end_of_buf = min(nob,SIZE_CAN_BUF-can_tx_buf_write);
	memcpy(can_tx_buf+can_tx_buf_write,buf,nob_to_end_of_buf);
	memcpy(can_tx_buf,buf+nob_to_end_of_buf,max(0,nob-nob_to_end_of_buf));
	can_tx_buf_write = (can_tx_buf_write+nob) & CAN_BUF_MASK;
//	BT_UART.ier = AVR32_USART_IER_TXRDY_MASK;

	// Run callback function
	can_request_tx();
	return 0;
}
bool can_is_data_available(void){
	return can_rx_buf_write!=can_rx_buf_read;
}
uint8_t can_get_byte(uint8_t* dest){
	if (can_rx_buf_write!=can_rx_buf_read) {
		*dest = can_rx_buf[can_rx_buf_read];
		can_rx_buf_read = (can_rx_buf_read+1) & CAN_BUF_MASK;
		return 1;
	}
	return 0;
}

#endif