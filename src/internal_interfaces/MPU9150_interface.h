
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#ifndef MPU9150_INTERFACE_H_
#define MPU9150_INTERFACE_H_

/// Initialization routine for the IMU to MCU interface
void mpu9150_interface_init(void);
// Routine for fast reading of vcc, acc, gyro, and temp from IMU
void mpu9150_read(void);

#endif /* MPU9150_INTERFACE_H_ */