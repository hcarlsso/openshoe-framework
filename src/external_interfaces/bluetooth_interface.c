
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#include <gpio.h>
#include <string.h>
#include "bluetooth_interface.h"
//#include "parsing_util.h"
#include "control_tables.h"
#include "package_queue.h"
#include "bt_uart_interrupt.h"
#include <usart.h>
#include "conf_clock.h"

#if defined(MIMU22BT)
#  include "MIMU22BT.h"
#elif defined(MIMU4444BT)
#  include "MIMU4444BT.h"
#else
#  include "MIMU22BT.h"
#endif

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

bool is_bluetooth_paired(void) {
	if ( gpio_get_pin_value(BT_PAIRED) ) {
		gpio_set_pin_high(LED0);
		return true;
	} else {
		gpio_set_pin_low(LED0);
		return false;
	}
}

uint32_t bt_send_buf(const uint8_t* buf,uint32_t nob) {
	return uart_send_buf(buf,nob);
}

uint32_t bt_send_buf_allornothing(const uint8_t* buf,uint32_t nob) {
	return uart_send_buf_allornothing(buf,nob);
}

bool bt_is_data_available(void) {
	return uart_is_data_available();
}

uint8_t bt_get_byte(uint8_t* dest) {
	return uart_get_byte(dest);
}