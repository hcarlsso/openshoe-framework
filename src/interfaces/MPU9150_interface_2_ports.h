/*
 * MPU9150_interface.h
 *
 * Created: 2013-08-13 17:56:35
 *  Author: jnil02
 */ 


#ifndef MPU9150_INTERFACE_2_PORTS_H_
#define MPU9150_INTERFACE_2_PORTS_H_

/// Initialization routine for the IMU to MCU interface
void mpu9150_2_port_interface_init(void);
// Routine for fast reading of vcc, acc, gyro, and temp from IMU
void mpu9150_2_port_read(void);

#endif /* MPU9150_INTERFACE_2_PORTS_H_ */