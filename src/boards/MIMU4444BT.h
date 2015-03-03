/*
 * MIMU4444BT.h
 *
 * Created: 2015-01-26 13:08:25
 *  Author: jnil02
 */ 


#ifndef MIMU4444BT_H_
#define MIMU4444BT_H_

#define LED0 AVR32_PIN_PB18

#define BT_MODULE
#define BT_PAIRED AVR32_PIN_PD13

// IMU clock pins
#define CLK_PORT 3
#define CLK_PINS ( (1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7)|(1<<8)|(1<<9)|(1<<10) )
// IMU data pins
#define SDA_PORT 2
#define NR_IMUS 32
#define SDA_PINS 0xFFFFFFFF

#define IMU_POS {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31}

#define BT_UART          AVR32_USART1
#define BT_UART_P        (&AVR32_USART1)
#define BT_UART_IRQ      AVR32_USART1_IRQ
#define BT_UART_RX       AVR32_USART1_RXD_0_1_PIN
#define BT_UART_RX_FUNC  AVR32_USART1_RXD_0_1_FUNCTION
#define BT_UART_TX       AVR32_USART1_TXD_0_1_PIN
#define BT_UART_TX_FUNC  AVR32_USART1_TXD_0_1_FUNCTION


#endif /* MIMU4444BT_H_ */