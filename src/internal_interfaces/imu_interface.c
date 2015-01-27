
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#include "imu_interface.h"
#include <stdint.h>
#include <asf.h>

#if defined(OPENSHOE_CLASSIC)
#  include "ADIS16367_interface.h"
#elif defined(MIMU3333)
#  include "MPU9150_interface_2_ports.h"
#elif defined(MIMU4444) || defined(MIMU22BT) || defined(MIMU4444BT)
#  include "MPU9150_interface.h"
#endif

void imu_interface_init(void){
	#if defined(OPENSHOE_CLASSIC)
	ADIS16367_interface_init();
	#elif defined(MIMU3333)
	mpu9150_2_port_interface_init();
	#elif defined(MIMU4444) || defined(MIMU22BT) || defined(MIMU4444BT)
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
	#elif defined(MIMU3333)
	mpu9150_2_port_read();
	#elif defined(MIMU4444) || defined(MIMU22BT) || defined(MIMU4444BT)
	mpu9150_read();
	#endif
}