/*
 * MPU9150_interface.h
 *
 * Created: 2014-03-17 12:18:06
 *  Author: jnil02
 */ 


#ifndef MPU9150_INTERFACE_H_
#define MPU9150_INTERFACE_H_

/// Initialization routine for the IMU to MCU interface
void mpu9150_interface_init(void);
// Routine for fast reading of vcc, acc, gyro, and temp from IMU
void mpu9150_read(void);

#endif /* MPU9150_INTERFACE_H_ */