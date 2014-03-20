/*
 * bt_uart_interrupt.c
 *
 * Created: 2014-03-20 09:38:54
 *  Author: jnil02
 */ 


#include <usart.h>

#include "bt_uart_interrupt.h"

//#if defined(MIMU22BT)
#  include "MIMU22BT.h"
//#endif

uint8_t uart_rx_buf[256];
volatile uint8_t uart_rx_buf_write=0;
volatile uint8_t uart_rx_buf_read=0;

uint8_t uart_tx_buf[256];
volatile uint8_t uart_tx_buf_write=0;
volatile uint8_t uart_tx_buf_read=0;

bool bt_is_data_available(void) {
	return uart_rx_buf_write!=uart_rx_buf_read;
}

uint8_t bt_get_byte(uint8_t* dest) {
	if (uart_rx_buf_write!=uart_rx_buf_read) {
		*dest = uart_rx_buf[uart_rx_buf_read];
		uart_rx_buf_read++;
		return 1;
	}
	return 0;
}

void bt_send_buf(uint8_t* buf,uint8_t nob) {
	uint8_t i;
	for (i=0;i<nob;i++)	uart_tx_buf[uart_tx_buf_write+i]=buf[i];
	uart_tx_buf_write+=nob;
	BT_UART.ier = AVR32_USART_IER_TXRDY_MASK;
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
	}
	if (BT_UART.csr & BT_UART.imr & AVR32_USART_CSR_TXRDY_MASK) { // Something CAN be sent
		if (uart_tx_buf_read!=uart_tx_buf_write) {  // Something should be sent
			BT_UART.thr = ( (int) uart_tx_buf[uart_tx_buf_read] << AVR32_USART_THR_TXCHR_OFFSET) & AVR32_USART_THR_TXCHR_MASK;
			uart_tx_buf_read++;
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