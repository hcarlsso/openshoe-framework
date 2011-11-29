
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

#include "imu_interface.h"
#include "conf_spi_master.h"
#include "nav_types.h"

#include <gpio.h>
#include <spi.h>
#include <spi_master.h>

#define XGYRO_OUT 0x0400
#define YGYRO_OUT 0x0600
#define ZGYRO_OUT 0x0800
#define XACC_OUT 0x0A00
#define YACC_OUT 0x0C00
#define ZACC_OUT 0x0E00

#define BURST_READ 0x3E00
#define PRECISION_GYRO_BIAS_CALIBRATION 0xBE10

struct spi_device SPI_DEVICE_IMU = {
	// SPI bus 0
	.id = 0 };

#define SUPPLY_SCALE 0.002418f
#define GYRO_SCALE 0.00087266f
#define ACC_SCALE 0.0081643275f
#define TEMP_SCALE 0.0085f
#define SUPPLY_SCALE 0.000151125f

// Local variables to store imu raw output
static uint16_t supply;
static int16_t xgyro;
static int16_t ygyro;
static int16_t zgyro;
static int16_t xacc;
static int16_t yacc;
static int16_t zacc;
static int16_t xtemp;
static int16_t ytemp;
static int16_t ztemp;
static int16_t aux_adc;

/** \name IMU output variables
 * Global variable used for making sensor readings available to other parts of
 * the program. The variables will contain the latest read out sensor readings.
 * The values are scaled to SI units apart from the temperature which is in
 * \f$^circ C\f$. Angels are given in radians.
 */
//@{
vec3 accelerations_in;			///< \f$[m/s^2]\f$
vec3 angular_rates_in;				///< \f$[rad/s]\f$
vec3 imu_temperaturs;			///< \f$[^circ C]\f$
precision imu_supply_voltage;	///< \f$[V]\f$
//@}

/**
 * /brief Initialization routine for the IMU to MCU interface
 *
 * /details The initialization functions calls asf spi master initialization
 * and setup routines with arguments defined by setting macros found in
 * conf_spi_master.h. The routine selects the IMU SPI for communication and
 * the interface functions will assume that the IMU SPI is still selected.
 */
void imu_interface_init(void){	
	spi_master_init(SPI_IMU);
	spi_master_setup_device(SPI_IMU, &SPI_DEVICE_IMU, 3, SPI_IMU_BAUDRATE, 0);
	spi_enable(SPI_IMU);
	spi_select_device(SPI_IMU,&SPI_DEVICE_IMU);
}

void convert_inert_readings(void){
	// Shift out status bits
	xgyro = xgyro << 2;
	ygyro = ygyro << 2;
	zgyro = zgyro << 2;
	xacc = xacc << 2;
	yacc = yacc << 2;
	zacc = zacc << 2;
	// Convert to float
	angular_rates_in[0] = GYRO_SCALE * xgyro;
	angular_rates_in[1] = GYRO_SCALE * ygyro;
	angular_rates_in[2] = GYRO_SCALE * zgyro;
	accelerations_in[0] = ACC_SCALE * xacc;
	accelerations_in[1] = ACC_SCALE * yacc;
	accelerations_in[2] = ACC_SCALE * zacc;
}

void convert_auxiliary_data(void){
	// Shift out status bits
	xtemp = xtemp << 4;
	ytemp = ytemp << 4;
	ztemp = ztemp << 4;
	supply = supply << 4;
	// Convert to float
	imu_temperaturs[0] = TEMP_SCALE * xtemp + 25.0f;
	imu_temperaturs[1] = TEMP_SCALE * ytemp + 25.0f;
	imu_temperaturs[2] = TEMP_SCALE * ztemp + 25.0f;
	imu_supply_voltage = SUPPLY_SCALE * supply;
}

void imu_burst_read(void){
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,BURST_READ);
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,CONFIG_SPI_MASTER_DUMMY);
	while (!spi_is_rx_ready(SPI_IMU)) {;}
	supply = spi_get(SPI_IMU);
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,CONFIG_SPI_MASTER_DUMMY);
	while (!spi_is_rx_ready(SPI_IMU)) {;}
	xgyro = spi_get(SPI_IMU);
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,CONFIG_SPI_MASTER_DUMMY);
	while (!spi_is_rx_ready(SPI_IMU)) {;}
	ygyro = spi_get(SPI_IMU);
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,CONFIG_SPI_MASTER_DUMMY);
	while (!spi_is_rx_ready(SPI_IMU)) {;}
	zgyro = spi_get(SPI_IMU);
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,CONFIG_SPI_MASTER_DUMMY);
	while (!spi_is_rx_ready(SPI_IMU)) {;}
	xacc = spi_get(SPI_IMU);
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,CONFIG_SPI_MASTER_DUMMY);
	while (!spi_is_rx_ready(SPI_IMU)) {;}
	yacc = spi_get(SPI_IMU);
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,CONFIG_SPI_MASTER_DUMMY);
	while (!spi_is_rx_ready(SPI_IMU)) {;}
	zacc = spi_get(SPI_IMU);
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,CONFIG_SPI_MASTER_DUMMY);
	while (!spi_is_rx_ready(SPI_IMU)) {;}
	xtemp = spi_get(SPI_IMU);
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,CONFIG_SPI_MASTER_DUMMY);
	while (!spi_is_rx_ready(SPI_IMU)) {;}
	ytemp = spi_get(SPI_IMU);
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,CONFIG_SPI_MASTER_DUMMY);
	while (!spi_is_rx_ready(SPI_IMU)) {;}
	ztemp = spi_get(SPI_IMU);
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,CONFIG_SPI_MASTER_DUMMY);
	while (!spi_is_rx_ready(SPI_IMU)) {;}
	aux_adc = spi_get(SPI_IMU);
	
	convert_inert_readings();
	convert_auxiliary_data();
}

#warning If this function is used the CONFIG_SPI_MASTER_DELAY_BCT macro must be set to >=9
void imu_read_acc_and_gyro(void){
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,XGYRO_OUT);
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,YGYRO_OUT);
	while (!spi_is_rx_ready(SPI_IMU)) {;}
	xgyro = spi_get(SPI_IMU);
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,ZGYRO_OUT);
	while (!spi_is_rx_ready(SPI_IMU)) {;}
	ygyro = spi_get(SPI_IMU);
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,XACC_OUT);
	while (!spi_is_rx_ready(SPI_IMU)) {;}
	zgyro = spi_get(SPI_IMU);
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,YACC_OUT);
	while (!spi_is_rx_ready(SPI_IMU)) {;}
	xacc = spi_get(SPI_IMU);
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,ZACC_OUT);
	while (!spi_is_rx_ready(SPI_IMU)) {;}
	yacc = spi_get(SPI_IMU);
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,CONFIG_SPI_MASTER_DUMMY);
	while (!spi_is_rx_ready(SPI_IMU)) {;}
	zacc = spi_get(SPI_IMU);
	
	convert_inert_readings();
}

void precision_gyro_bias_null_calibration(void){
	while (!spi_is_tx_ready(SPI_IMU)) {;}
	spi_put(SPI_IMU,PRECISION_GYRO_BIAS_CALIBRATION);
	// After this the IMU will be off-line for ~15s
}