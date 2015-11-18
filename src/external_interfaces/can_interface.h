
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2015 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/


#ifndef CAN_INTERFACE_H_
#define CAN_INTERFACE_H_

#include <stdint.h>
#include <stdbool.h>

#define COMMAND_FROM_CAN 4

void can_interface_init(void);
bool is_can_ready(void);

uint32_t can_send_buf(const uint8_t* buf,uint32_t nob);
uint32_t can_send_buf_allornothing(const uint8_t* buf,uint32_t nob);
bool     can_is_data_available(void);
uint8_t  can_get_byte(uint8_t* dest);


#endif /* CAN_INTERFACE_H_ */