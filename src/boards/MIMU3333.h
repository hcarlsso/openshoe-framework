﻿
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#ifndef MIMU3333_H_
#define MIMU3333_H_

//#define LED0 AVR32_PIN_PB01

// IMU pins
#define CLK_PORT_NUM 3
#define CLK_PINS_2PORT   ((1<<21)|(1<<27)|(1<<28)|(1<<29)|(1<<30))
#define DATA_PORTA 0
#define DATA_PORTC 2
#define IMU0_PORTC 4
#define IMU1_PORTA 19
#define IMU2_PORTA 16
#define IMU3_PORTA 9
#define IMU4_PORTA 8
#define IMU5_PORTA 7
#define IMU6_PORTA 6
#define IMU7_PORTA 5
#define IMU8_PORTA 4
#define IMU9_PORTC 18
#define IMU10_PORTC 19
#define IMU11_PORTC 20
#define IMU12_PORTC 2
#define IMU13_PORTC 3
#define IMU14_PORTC 17
#define IMU15_PORTC 5
#define IMU16_PORTC 15
#define IMU17_PORTC 16
#define NR_IMUS_PORTA 8
#define NR_IMUS_PORTC 10
#define DATA_PINSA ( (1<<IMU1_PORTA)|(1<<IMU2_PORTA)|(1<<IMU3_PORTA)|(1<<IMU4_PORTA)|(1<<IMU5_PORTA)|(1<<IMU6_PORTA)|(1<<IMU7_PORTA)|(1<<IMU8_PORTA) )
#define DATA_PINSC ( (1<<IMU0_PORTC)|(1<<IMU9_PORTC)|(1<<IMU10_PORTC)|(1<<IMU11_PORTC)|(1<<IMU12_PORTC)|(1<<IMU13_PORTC)|(1<<IMU14_PORTC)|(1<<IMU15_PORTC)|(1<<IMU16_PORTC)|(1<<IMU17_PORTC) )

#define RX_BUFFER_SIZE 512
#define TX_BUFFER_SIZE 512
#define MAX_RX_NRB 400


#endif /* MIMU3333_H_ */