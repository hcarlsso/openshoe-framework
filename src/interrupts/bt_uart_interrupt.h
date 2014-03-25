/*
 * bt_uart_interrupt.h
 *
 * Created: 2014-03-20 09:39:15
 *  Author: jnil02
 */ 


#ifndef BT_UART_INTERRUPT_H_
#define BT_UART_INTERRUPT_H_

#include <stdint.h>
#include <stdbool.h>

bool bt_is_data_available(void);
uint8_t bt_get_byte(uint8_t* dest);
uint8_t space_in_bt_uart_buf(void);
void bt_send_buf(uint8_t* buf,uint8_t nob);

void bt_interrupt_init(void);

#endif /* BT_UART_INTERRUPT_H_ */