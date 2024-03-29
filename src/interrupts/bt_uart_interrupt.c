﻿
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#include <usart.h>
#include <string.h>

#include "bt_uart_interrupt.h"

#if defined(MIMU22BT)
#  include "MIMU22BT.h"
#elif defined(MIMU4444BT)
#  include "MIMU4444BT.h"
#elif defined(MIMU22BTv3)
#  include "MIMU22BTv3.h"
#else
#  include "MIMU22BT.h"
#endif

#ifdef BT_MODULE

#define LOG2_SIZE_BT_UART_TX_BUF 10
#define LOG2_SIZE_BT_UART_RX_BUF 9
#define SIZE_BT_UART_TX_BUF (1<<LOG2_SIZE_BT_UART_TX_BUF)
#define SIZE_BT_UART_RX_BUF (1<<LOG2_SIZE_BT_UART_RX_BUF)
#define BUF_MASK_TX (SIZE_BT_UART_TX_BUF-1)
#define BUF_MASK_RX (SIZE_BT_UART_RX_BUF-1)

static uint8_t uart_rx_buf[SIZE_BT_UART_RX_BUF];
static volatile uint32_t uart_rx_buf_write=0;
static volatile uint32_t uart_rx_buf_read=0;

static uint8_t uart_tx_buf[SIZE_BT_UART_TX_BUF];
static volatile uint32_t uart_tx_buf_write=0;
static volatile uint32_t uart_tx_buf_read=0;

bool uart_is_data_available(void) {
	return uart_rx_buf_write!=uart_rx_buf_read;
}

uint8_t uart_get_byte(uint8_t* dest) {
	if (uart_rx_buf_write!=uart_rx_buf_read) {
		*dest = uart_rx_buf[uart_rx_buf_read];
		uart_rx_buf_read = (uart_rx_buf_read+1) & BUF_MASK_RX;
		return 1;
	}
	return 0;
}

uint32_t uart_send_buf_allornothing(const uint8_t* buf,uint32_t nob) {
	uint32_t space_in_buf = (uart_tx_buf_read-uart_tx_buf_write)&BUF_MASK_TX;
	space_in_buf = space_in_buf ? space_in_buf-1 : (SIZE_BT_UART_TX_BUF-1);
	if(space_in_buf<nob)
		return nob;
	int nob_to_end_of_buf = min(nob,SIZE_BT_UART_TX_BUF-uart_tx_buf_write);
	memcpy(uart_tx_buf+uart_tx_buf_write,buf,nob_to_end_of_buf);
	memcpy(uart_tx_buf,buf+nob_to_end_of_buf,max(0,nob-nob_to_end_of_buf));
	uart_tx_buf_write = (uart_tx_buf_write+nob) & BUF_MASK_TX;
	BT_UART.ier = AVR32_USART_IER_TXRDY_MASK;
	return 0;
}

uint32_t uart_send_buf(const uint8_t* buf,uint32_t nob){
	uint32_t space_in_buf = (uart_tx_buf_read-uart_tx_buf_write)&BUF_MASK_TX;
	space_in_buf = space_in_buf ? space_in_buf-1 : (SIZE_BT_UART_TX_BUF-1);
	if(!space_in_buf)
		return nob;
	uint32_t nr_bytes_left = 0;
	if(space_in_buf<nob){
		nr_bytes_left = nob-space_in_buf;
		nob = space_in_buf;
	}
	int nob_to_end_of_buf = min(nob,SIZE_BT_UART_TX_BUF-uart_tx_buf_write);
	memcpy(uart_tx_buf+uart_tx_buf_write,buf,nob_to_end_of_buf);
	memcpy(uart_tx_buf,buf+nob_to_end_of_buf,max(0,nob-nob_to_end_of_buf));
	uart_tx_buf_write = (uart_tx_buf_write+nob) & BUF_MASK_TX;
	BT_UART.ier = AVR32_USART_IER_TXRDY_MASK;
	return nr_bytes_left;
}


// Fast and dirty uart interrupt handler.
//  - No uart communication error checks when receiving data.
//  - If we overrun the internal ring buffers, the whole respective buffers are discarded.
#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
__interrupt
#endif
static void usart_int_handler(void)
{
	if (BT_UART.csr & AVR32_USART_CSR_RXRDY_MASK) { // Something HAS BEEN read
		uart_rx_buf[uart_rx_buf_write] = (uint8_t) ( (BT_UART.rhr & AVR32_USART_RHR_RXCHR_MASK) >> AVR32_USART_RHR_RXCHR_OFFSET );
		uart_rx_buf_write++;
		uart_rx_buf_write &= BUF_MASK_RX;
	}
	if (BT_UART.csr & BT_UART.imr & AVR32_USART_CSR_TXRDY_MASK) { // Something CAN be sent
		if (uart_tx_buf_read!=uart_tx_buf_write) {  // Something should be sent
			BT_UART.thr = ( (int) uart_tx_buf[uart_tx_buf_read] << AVR32_USART_THR_TXCHR_OFFSET) & AVR32_USART_THR_TXCHR_MASK;
			uart_tx_buf_read=(uart_tx_buf_read+1)&BUF_MASK_TX;
		}
		if (uart_tx_buf_read==uart_tx_buf_write) BT_UART.idr = AVR32_USART_IER_TXRDY_MASK;
	}
}

void bt_interrupt_init(void){
	Disable_global_interrupt();
	INTC_register_interrupt(&usart_int_handler, BT_UART_IRQ, AVR32_INTC_INT0);
	BT_UART.ier = AVR32_USART_IER_RXRDY_MASK;
	Enable_global_interrupt();
}

#endif /* BT_MODULE */
