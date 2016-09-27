
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#ifndef MIMU4444_H_
#define MIMU4444_H_


// IMU clock pins
#define CLK_PORT 3
#define CLK_PINS ( (1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7)|(1<<8)|(1<<9)|(1<<10) )
// IMU data pins
#define SDA_PORT 2
#define NR_IMUS 32
#define SDA_PINS 0xFFFFFFFF

#define IMU_POS {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31}

#define RX_BUFFER_SIZE 1024
#define TX_BUFFER_SIZE 1024
#define MAX_RX_NRB 400


#endif /* MIMU4444_H_ */