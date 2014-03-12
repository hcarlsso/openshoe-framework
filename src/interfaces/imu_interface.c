/*
 * imu_interface.c
 *
 * Created: 2014-03-12 14:21:11
 *  Author: jnil02
 */ 

#include "imu_interface.h"

#if defined(OPENSHOE_CLASSIC)
#include "ADIS16367_interface.h"
#elif defined(MIMU3333) || defined(MIMU22BT)
#include "MPU9150_interface.h"
#endif

void imu_interface_init(void){
	#if defined(OPENSHOE_CLASSIC)
	ADIS16367_interface_init();
	#elif defined(MIMU3333) || defined(MIMU22BT)
	mpu9150_interface_init();
	#endif
}

void imu_read(void){
	#if defined(OPENSHOE_CLASSIC)
	imu_burst_read();
	#elif defined(MIMU3333) || defined(MIMU22BT)
	mpu9150_read();
	#endif
}