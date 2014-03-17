/*
 * imu_interface.c
 *
 * Created: 2014-03-12 14:21:11
 *  Author: jnil02
 */ 

#include "imu_interface.h"
#include <stdint.h>
#include <asf.h>

#if defined(OPENSHOE_CLASSIC)
#  include "ADIS16367_interface.h"
#elif defined(MIMU3333) || defined(MIMU22BT)
#  include "MPU9150_interface_2_ports.h"
#endif

void imu_interface_init(void){
	#if defined(OPENSHOE_CLASSIC)
	ADIS16367_interface_init();
	#elif defined(MIMU3333) || defined(MIMU22BT)
	mpu9150_interface_init();
	#endif
}

int16_t mimu_data[32][7];
uint32_t ts_u;

void imu_read(void){
	// We time stamp here rather than in the interrupt routines because it's not sure we read when we get the interrupt. It could be much later.
	ts_u = Get_system_register(AVR32_COUNT);
	#if defined(OPENSHOE_CLASSIC)
	imu_burst_read();
	#elif defined(MIMU3333) || defined(MIMU22BT)
	mpu9150_read();
	#endif
}