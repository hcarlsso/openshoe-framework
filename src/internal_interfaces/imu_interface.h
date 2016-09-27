
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#ifndef IMU_INTERFACE_H_
#define IMU_INTERFACE_H_

#include <stdint.h>

extern int16_t mimu_data[32][10];
extern uint32_t ts_u;

void imu_interface_init(void);
void imu_read(void);

#endif /* IMU_INTERFACE_H_ */