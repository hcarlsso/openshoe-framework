
/** \file
 *
 * \brief High level IMU interface.
 *
 * \details This file declares the high level interface functions used by the
 * rest of the system to interact with the IMU. The functions work for all IMUs
 * in the Analog Devices iSensor (R) serie. The communication is done via a SPI
 * interface. Before the functions are called the initialization routine
 * (imu_interface_init()) must be called to set up the SPI interface. The SPI
 * interface settings are found in conf_spi_master.h.
 *
 * \authors John-Olof Nilsson, Isaac Skog
 * \copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
 */ 


#ifndef IMU_SPI_INTERFACE_H_
#define IMU_SPI_INTERFACE_H_

//void imu_interupt_init(void);

/// Initialization routine for the IMU to MCU interface
void imu_interface_init(void);
/// Routine for fast reading of vcc, acc, gyro, and temp from IMU
void imu_burst_read(void);
/// Routine for reading only acc and gryo from IMU
void imu_read_acc_and_gyro(void);



#endif /* IMU_SPI_INTERFACE_H_ */