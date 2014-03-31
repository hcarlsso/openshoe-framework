
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
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

#define IMU_POS {IMU0_SDA,IMU1_SDA,IMU2_SDA,IMU3_SDA}

#define BT_UART          AVR32_USART1
#define BT_UART_P        (&AVR32_USART1)
#define BT_UART_IRQ      AVR32_USART1_IRQ
#define BT_UART_RX       AVR32_USART1_RXD_0_1_PIN
#define BT_UART_RX_FUNC  AVR32_USART1_RXD_0_1_FUNCTION
#define BT_UART_TX       AVR32_USART1_TXD_0_1_PIN
#define BT_UART_TX_FUNC  AVR32_USART1_TXD_0_1_FUNCTION

#endif /* MIMU22BT_H_ */