/*
 * MIMU4444.h
 *
 * Created: 2014-03-17 09:35:56
 *  Author: jnil02
 */ 


#ifndef MIMU4444_H_
#define MIMU4444_H_


// IMU clock pins
#define CLK_PORT 3
#define CLK_PINS ( (1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7)|(1<<8)|(1<<9)|(1<<10) )
// IMU data pins
#define SDA_PORT 0
#define NR_IMUS 32
#define SDA_PINS 0xFFFFFFFF

#endif /* MIMU4444_H_ */