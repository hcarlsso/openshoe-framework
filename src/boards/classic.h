
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#ifndef CLASSIC_H_
#define CLASSIC_H_


// IMU interrupt pin defines
#define IMU_INTERUPT_PIN		AVR32_EIC_EXTINT_0_1_PIN
#define IMU_INTERUPT_FUNCTION	AVR32_EIC_EXTINT_0_1_FUNCTION
#define IMU_INTERUPT_LINE1		EXT_NMI
#define IMU_INTERUPT_NB_LINES	1
// IMU SPI communication pins
#define SPI_IMU                 (&AVR32_SPI0)
#define SPI_IMU_ID			    AVR32_SPI0_NPCS_0_PIN
#define IMU_SPI_SCK_PIN         AVR32_SPI0_SCK_PIN
#define IMU_SPI_SCK_FUNCTION    AVR32_SPI0_SCK_FUNCTION
#define IMU_SPI_MISO_PIN        AVR32_SPI0_MISO_PIN
#define IMU_SPI_MISO_FUNCTION   AVR32_SPI0_MISO_FUNCTION
#define IMU_SPI_MOSI_PIN        AVR32_SPI0_MOSI_PIN
#define IMU_SPI_MOSI_FUNCTION   AVR32_SPI0_MOSI_FUNCTION
#define IMU_SPI_NPCS_0_PIN      AVR32_SPI0_NPCS_0_PIN
#define IMU_SPI_NPCS_0_FUNCTION AVR32_SPI0_NPCS_0_FUNCTION


#endif /* CLASSIC_H_ */