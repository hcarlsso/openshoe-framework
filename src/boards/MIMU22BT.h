/*
 * MIMU22BT.h
 *
 * Created: 2014-03-17 09:36:31
 *  Author: jnil02
 */ 


#ifndef MIMU22BT_H_
#define MIMU22BT_H_

#define LED0 AVR32_PIN_PC04

#define BT_MODULE
#define BT_PAIRED AVR32_PIN_PD13

// IMU clock pins
#define CLK_PORT 1
#define CLK_PINS (1<<1)
// IMU data pins
#define SDA_PORT 0
#define IMU0_SDA 4
#define IMU1_SDA 5
#define IMU2_SDA 6
#define IMU3_SDA 7
#define NR_IMUS 4
#define SDA_PINS ( (1<<IMU0_SDA)|(1<<IMU1_SDA)|(1<<IMU2_SDA)|(1<<IMU3_SDA) )

#endif /* MIMU22BT_H_ */