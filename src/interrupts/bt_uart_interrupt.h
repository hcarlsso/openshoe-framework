
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#ifndef BT_UART_INTERRUPT_H_
#define BT_UART_INTERRUPT_H_

#include <stdint.h>
#include <stdbool.h>

bool bt_is_data_available(void);
uint8_t bt_get_byte(uint8_t* dest);
uint8_t space_in_bt_uart_buf(void);
void bt_send_buf(uint8_t* buf,uint32_t nob);

void bt_interrupt_init(void);

#endif /* BT_UART_INTERRUPT_H_ */