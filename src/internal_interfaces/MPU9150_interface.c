
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#include "MPU9150_interface.h"
#include "mpu6150.h"
#include "conf_clock.h"
#include <stdint.h>
#include <string.h>
#include <asf.h>

#if defined(MIMU22BT)
#   include "MIMU22BT.h"
#elif defined(MIMU4444)
#	include "MIMU4444.h"
#elif defined(MIMU4444BT)
#  include "MIMU4444BT.h"
#else
#   include "MIMU22BT.h"
#endif


// Functions pacing I2C bitbanging
static int32_t last_tick;
//#define QUATER_CC_COUNT 35
#define QUATER_CC_COUNT 10
#define QUATER_TICK_ADJ 10
// I2C bus speed is approximately CLOCK_FREQ/QUATER_CC_COUNT * 220 [kHz]
// 35 gives 400kHz
__always_inline static void start_tick(void){
	last_tick = Get_system_register(AVR32_COUNT);
}
__always_inline static void half_tick(void){
	int32_t wait_to = last_tick + 2*QUATER_CC_COUNT;
	while (Get_system_register(AVR32_COUNT) - wait_to < 0);
	last_tick = Get_system_register(AVR32_COUNT);
}
__always_inline static void quater_tick(void){
	int32_t wait_to = last_tick + QUATER_CC_COUNT-QUATER_TICK_ADJ;
	while (Get_system_register(AVR32_COUNT) - wait_to < 0);
	last_tick = Get_system_register(AVR32_COUNT);
}

// Function that reads all the values of a port
//#define gpio_get_port_value(port) (AVR32_GPIO.port[port].pvr)
// TODO: Use above function instead
static uint32_t gpio_get_port_value(uint32_t port,uint32_t mask)
{
	volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[port];
	return (gpio_port->pvr ) & mask;
	// TODO: Remove this masking. It is not used.
}

// Bit-wise transposition of a 32x32 bit-matrix
static void transpose32(uint32_t A[32]){
	int32_t j, k;
	uint32_t m, t;
	
	m = 0x0000FFFF;
	for (j = 16; j != 0; j = j >> 1, m = m ^ (m << j)) {
		for (k = 0; k < 32; k = (k + j + 1) & ~j) {
			t = (A[k] ^ (A[k+j] >> j)) & m;
			A[k] = A[k] ^ t;
			A[k+j] = A[k+j] ^ (t << j);
		}
	}
}

// Function that initializes the I2C communication pins
static void I2C_init(void){
	// Initialize the IMU pins to open-drain outputs
	gpio_configure_group(CLK_PORT,CLK_PINS,(GPIO_OPEN_DRAIN | GPIO_DIR_OUTPUT));
	gpio_configure_group(SDA_PORT,SDA_PINS,(GPIO_OPEN_DRAIN | GPIO_DIR_OUTPUT));
}

// Function that sends the I2C start command
static void I2C_start(void){
	
	start_tick();

	// Make sure all communication lines are in the start state
//	gpio_set_group_high(CLK_PORT_NUM,CLK_PINS);
//	gpio_set_group_high(DATA_PORT0,DATA_PINS0);
//	gpio_set_group_high(DATA_PORT1,DATA_PINS1);

	// Now pull the SDA lines low will keeping the SCL high
	gpio_set_group_low(SDA_PORT,SDA_PINS);
	quater_tick();
}

static void I2C_start_read(void){
	
	// Short delay (1/2 clock cycle)
	gpio_set_group_low(CLK_PORT,CLK_PINS);
	half_tick();
	
	// Make sure all communication lines are in the start state
	gpio_set_group_high(CLK_PORT,CLK_PINS);
	gpio_set_group_high(SDA_PORT,SDA_PINS);
	
	// Short delay (1/4 clock cycle)
	quater_tick();

	// Now pull the SDA lines low will keeping the SCL high
	gpio_set_group_low(SDA_PORT,SDA_PINS);
	quater_tick();
}

// Function that sends the I2C start command
static void I2C_stop(void)
{
	// Pull the SCL lines low
	gpio_set_group_low(CLK_PORT,CLK_PINS);
	quater_tick();

	// Pull the SDA lines low
	gpio_set_group_low(SDA_PORT,SDA_PINS);
	quater_tick();

	// Pull the SCL lines high
	gpio_set_group_high(CLK_PORT,CLK_PINS);
	quater_tick();

	// Pull the SDA lines high
	gpio_set_group_high(SDA_PORT,SDA_PINS);
}

// Function that clocks out a single byte on the I2C bus and that returns ACK (NACK) responds of the sensors
static void I2C_write_byte(uint8_t data)
{
//	uint32_t ack_vec;
	uint8_t ctr;
	
	for (ctr=8;ctr>0;ctr--)
	{
		// Pull clk low
		gpio_set_group_low(CLK_PORT,CLK_PINS);
		quater_tick();

		// Set SDA 1
		if ((data>>(ctr-1)) & 0x01)
		{
			gpio_set_group_high(SDA_PORT,SDA_PINS);
		}
		else
		{
			gpio_set_group_low(SDA_PORT,SDA_PINS);
		}
		quater_tick();

		// Pull clk high
		gpio_set_group_high(CLK_PORT,CLK_PINS);
		half_tick();
	}

	// ACK
	// Pull clk low
	gpio_set_group_low(CLK_PORT,CLK_PINS);
	quater_tick();

	// Set SDA 1
	gpio_set_group_high(SDA_PORT,SDA_PINS);
	quater_tick();

	// Pull clk high
	gpio_set_group_high(CLK_PORT,CLK_PINS);
	half_tick();
//	quater_tick();

	// Read the state of the SLA lines
//	ack_vec=gpio_get_port_value(DATA_PORT0,DATA_PINS0);
//	ack_vec=gpio_get_port_value(DATA_PORT1,DATA_PINS1);
//	quater_tick();

	// Pull SCL low (gives more even clock at higher rates)
	gpio_set_group_low(CLK_PORT,CLK_PINS);
}

// Function that reads a single byte on the I2C bus. The input flag signals if a ACK should be read or a NACK outputted.
static void I2C_read_byte(uint32_t *data_port0,Bool ack_flag){
	
	uint8_t ctr;
	
	
	for (ctr=0;ctr<8;ctr++)
	{		
		// Pull clk low
		gpio_set_group_low(CLK_PORT,CLK_PINS);
		half_tick();
		
		// Pull clk high
		gpio_set_group_high(CLK_PORT,CLK_PINS);
		quater_tick();
		data_port0[ctr]=gpio_get_port_value(SDA_PORT,SDA_PINS);

		quater_tick();
	}

/*	uint32_t tmp0, tmp1;
	// Pull clk low
	gpio_set_group_low(CLK_PORT_NUM,CLK_PINS);
	half_tick();
		
	// Pull clk high
	gpio_set_group_high(CLK_PORT_NUM,CLK_PINS);
	quater_tick();
	tmp0 = gpio_get_port_value(DATA_PORT0,DATA_PINS0);
	tmp1 = gpio_get_port_value(DATA_PORT1,DATA_PINS1);

	quater_tick();

	for (ctr=0;ctr<7;ctr++)
	{
		// Pull clk low
		gpio_set_group_low(CLK_PORT_NUM,CLK_PINS);
		data_port0[ctr]=tmp0;
		data_port1[ctr]=tmp1;
		half_tick();
		
		// Pull clk high
		gpio_set_group_high(CLK_PORT_NUM,CLK_PINS);
		quater_tick();
		tmp0 = gpio_get_port_value(DATA_PORT0,DATA_PINS0);
		tmp1 = gpio_get_port_value(DATA_PORT1,DATA_PINS1);

		quater_tick();
	}
	data_port0[ctr]=tmp0;
	data_port1[ctr]=tmp1;
*/
	
	if (ack_flag)
	{
		// ACK
		// Pull clk low
		gpio_set_group_low(CLK_PORT,CLK_PINS);
		quater_tick();

		// Set SDA 1
		gpio_set_group_low(SDA_PORT,SDA_PINS);
		quater_tick();

		// Pull clk high
		gpio_set_group_high(CLK_PORT,CLK_PINS);
		half_tick();
	}
	else
	{
		// Send NACK
		// Pull clk low
		gpio_set_group_low(CLK_PORT,CLK_PINS);
		quater_tick();

		// Set SDA 1
		gpio_set_group_high(SDA_PORT,SDA_PINS);
		quater_tick();

		// Pull clk high
		gpio_set_group_high(CLK_PORT,CLK_PINS);
		half_tick();
	}

	// Pull SCL low
	gpio_set_group_low(CLK_PORT,CLK_PINS);
	// Release data pins
	gpio_set_group_high(SDA_PORT,SDA_PINS);
}

//Function thats writes a single byte to the specified register address.
static void single_byte_write(uint8_t address,uint8_t data)
{
	// Send I2C start command
	I2C_start();

	// Send device address with write command
	I2C_write_byte(((MPU6150_ADDRESS_AD0_HIGH<<1) & 0xFE));

	// Send register address
	I2C_write_byte(address);

	// Send data
	I2C_write_byte(data);

	// Send stop
	I2C_stop();
}

//Function that reads a single byte from the specified register address
static void single_byte_read(uint8_t address,uint32_t *data_port0)
{
	// Send I2C start command
	I2C_start();
	
	// Send device address with write command
	I2C_write_byte(((MPU6150_ADDRESS_AD0_HIGH<<1) & 0xFE));
		
	// Send register address
	I2C_write_byte(address);
	
	// Send I2C start command
	I2C_start_read();
	
	// Send device address with read command
	I2C_write_byte(((MPU6150_ADDRESS_AD0_HIGH<<1) | 0x01));
	
	// Read byte (false=NACK)
	I2C_read_byte(data_port0,false);

	// Send stop
	I2C_stop();
}

//Function that reads multiple bytes in a burst
static void burst_read(uint8_t address,uint32_t *data_port0,uint8_t nr_of_bytes)
{
	uint8_t ctr;
	
	// Send I2C start command
	I2C_start();

	// Send device address with write command
	I2C_write_byte(((MPU6150_ADDRESS_AD0_HIGH<<1) & 0xFE));
	
	// Send register address
	I2C_write_byte(address);
	
	// Send I2C start command
	I2C_start_read();
	
	// Send device address with read command
	I2C_write_byte(((MPU6150_ADDRESS_AD0_HIGH<<1) | 0x01));
	
	// Read bytes (true=ACK)
	for (ctr=0;ctr<nr_of_bytes-1;ctr++)
	{
		I2C_read_byte(data_port0+ctr*8,true);
	}
	// Read last byte (false=NACK)
	I2C_read_byte(data_port0+ctr*8,false);
	
	// Send stop
	I2C_stop();
}

#define AK8963_ADDRESS  0x0C
void mpu9150_interface_init(void){
	
	// Configure IMU pins for I2C communication
	I2C_init();
	
	// Wake up the devices and set the x-axis gyroscope as the reference oscillator
	single_byte_write(MPU6150_RA_PWR_MGMT_1,0x01);

	// Set the bandwidth of the sensors to 260/256 Hz.
	single_byte_write(MPU6150_RA_CONFIG,0x00);

	// Set the accelerometer scale ranges to max (+-16g) (FS_SEL=3)
	single_byte_write(MPU6150_RA_ACCEL_CONFIG,0x18);

	// Set the gyroscope scale ranges to max (+-2000 deg/s) (AFS_SEL=3)
	single_byte_write(MPU6150_RA_GYRO_CONFIG,0x18);
	
	// Configure internal sampling rate to 1kHz
	single_byte_write(25,0x07);
	
//	single_byte_write(INT_PIN_CFG,0x02);
//	delay_ms(10);
//	uint32_t calib_values[32];
//	mag_single_byte_write(0x0A,0x00);
//	delay_ms(10);
//	mag_single_byte_write(0x0A,0x0F);
//	delay_ms(10);
//	mag_single_byte_read(0x10,calib_values);
//	mag_single_byte_read(0x11,calib_values+8);
//	mag_single_byte_read(0x12,calib_values+16);
//	mag_single_byte_write(0x0A,0x00);
//	delay_ms(10);
//	single_byte_write(INT_PIN_CFG,0x00);
	
	// Disable I2C master
	single_byte_write(106,0x00);
	// 400kHz slave bus speed
	single_byte_write(36,0x0D);

	// Configure slave0 to be read and set address
	single_byte_write(37,0x80|AK8963_ADDRESS);
	// Set start register of slave0
//	single_byte_write(38,0x02);
	single_byte_write(38,0x03);
	// Enable slave0 and set number of bytes to be read (8)
	// 1 Enable slave0
	// 0 No byte swapping
	// 0 Write (specify) register address to slave
	// 0 Word grouping
	// 8 (0-15) Number of bytes read from slave
//	single_byte_write(39,0xD8);
	single_byte_write(39,0xD6);

	// 0 write to slave1
	// Slave address (0x0C)
	single_byte_write(40,AK8963_ADDRESS);
	// Start register at slave1 (0x0A)
	single_byte_write(41,0x0A);
	// 1 Enable slave0
	// 0 No byte swapping
	// 0 Write (specify) register address to slave
	// 0 Word grouping
	// 1 (0-15) Number of bytes write to slave (WRITE WHAT?)
	single_byte_write(42,0x81);
	// Slave1 data out (0x01) "Single measurement mode"
	single_byte_write(100,0x01);
	// 1 Shadowing of data (external sensor data held until all data available)
	// -
	// -
	// 0
	// 0
	// 1
	// 1 reduced sample rate of slave1 to 1/(1+I2C_MST_DLY) (reg52,0 currently) of that
	// determined by Sample rate = gyroscope output rate/(1+ SMPLRT_DIV) (reg25)
	// determined by DLPF_CFG (reg26), 8kHz (disabled DLPFG_CFG=0 or 7) and 1kHz if enabled
	// I have DLPFG_CFG=0, therefore I should set SMPLRT_DIV to 7)
	single_byte_write(103,0x83);
	// Slave1 data out (0x01)
//	single_byte_write(100,0x01);
	
	// Delay access rate to slaves (I2C_MST_DLY=9)
	single_byte_write(52,0x09);
	// Enable I2C master
	single_byte_write(106,0x20);
}

void mpu9150_set_bandwidth(uint8_t DLPF_CFG){
	// Set config (nr 26) register
	if(DLPF_CFG<7)
		single_byte_write(MPU6150_RA_CONFIG,DLPF_CFG);
}


// IMU array data
extern int16_t mimu_data[32][10];

// Buffers for reading data (128 bits) from the IMUs
//static uint32_t data_matrix[192];
static uint32_t data_matrix[160];
// Map of the I/O-pin positions of the IMUs
static const uint8_t imu_map[NR_IMUS]=IMU_POS;

// Read inertial measurements from IMU
void mpu9150_read(void)
{
	static uint32_t last_read;
	static int16_t previous_acc[32][3];
	uint32_t this_read = Get_system_register(AVR32_COUNT);
	
	// Read all sensor registers (acc,temp,gyro)
//	burst_read(0x3B,data_matrix,14);
	burst_read(0x3B,data_matrix,20);
	
	// Transpose 32x32 bit-blocks
	transpose32(data_matrix);
	transpose32(data_matrix+32);
	transpose32(data_matrix+64);
	transpose32(data_matrix+96);
	transpose32(data_matrix+128);

	// Copy to data state arrays
	// Delay acc measurements one sample if sampling is >750Hz.
	// Because there is a built in delay of the gyro measurements of 0.98ms.
	if (this_read-last_read < (CLOCK_FREQ/(750u))) {
		for (uint8_t i=0; i<NR_IMUS; i++) {
			// Previous acc measurements
			memcpy(mimu_data[i],previous_acc[i],6);
			// Store acc measurements
			memcpy(previous_acc[i],data_matrix+(32-1)-imu_map[i],4);
			memcpy(previous_acc[i]+2,data_matrix+(64-1)-imu_map[i],2);
			// Copy gyro and temp measurements
			memcpy(mimu_data[i]+3,data_matrix+(96-1)-imu_map[i],4);
			memcpy(mimu_data[i]+5,data_matrix+(128-1)-imu_map[i],2);
			memcpy(mimu_data[i]+6,(uint16_t*)(data_matrix+(64-1)-imu_map[i])+1,2);
			memcpy(mimu_data[i]+7,(uint16_t*)(data_matrix+(128-1)-imu_map[i])+1,2);
			memcpy(mimu_data[i]+8,data_matrix+(160-1)-imu_map[i],4);
		}
		
	} else {
		for (uint8_t i=0; i<NR_IMUS; i++) {
			memcpy(previous_acc[i],data_matrix+(32-1)-imu_map[i],4);
			memcpy(previous_acc[i]+2,data_matrix+(64-1)-imu_map[i],2);
			memcpy(mimu_data[i],data_matrix+(32-1)-imu_map[i],4);
			memcpy(mimu_data[i]+2,data_matrix+(64-1)-imu_map[i],2);
			memcpy(mimu_data[i]+3,data_matrix+(96-1)-imu_map[i],4);
			memcpy(mimu_data[i]+5,data_matrix+(128-1)-imu_map[i],2);
			memcpy(mimu_data[i]+6,(uint16_t*)(data_matrix+(64-1)-imu_map[i])+1,2);
			memcpy(mimu_data[i]+7,(uint16_t*)(data_matrix+(128-1)-imu_map[i])+1,2);
			memcpy(mimu_data[i]+8,data_matrix+(160-1)-imu_map[i],4);
		}
	}
	last_read = this_read;
}

/*
void mpu9150_read_mag(void){

	// Read all sensor registers (acc,temp,gyro)
	burst_read(0x3B,data_matrix,20);
	
	// Transpose 32x32 bit-blocks
	transpose32(data_matrix);
	transpose32(data_matrix+32);
	transpose32(data_matrix+64);
	transpose32(data_matrix+96);
	transpose32(data_matrix+128);
//	transpose32(data_matrix+160);
	
	for (uint8_t i=0; i<NR_IMUS; i++) {
		memcpy(mimu_data[i],data_matrix+(32-1)-imu_map[i],4);
		memcpy(mimu_data[i]+2,data_matrix+(64-1)-imu_map[i],2);
		memcpy(mimu_data[i]+3,data_matrix+(96-1)-imu_map[i],4);
		memcpy(mimu_data[i]+5,data_matrix+(128-1)-imu_map[i],2);
		memcpy(mimu_data[i]+6,(uint16_t*)(data_matrix+(64-1)-imu_map[i])+1,2);
		
		memcpy(mimu_data[i]+7,(uint16_t*)(data_matrix+(128-1)-imu_map[i])+1,2);
		memcpy(mimu_data[i]+8,data_matrix+(160-1)-imu_map[i],4);

//		if ((*((uint8_t*)(data_matrix+(128-1)-imu_map[i])+2)) && !(*((uint8_t*)(data_matrix+(192-1)-imu_map[i])+1)&0x04) ) {
//			memcpy((uint8_t*)mag_mimu_data[i]+0,(uint8_t*)(data_matrix+(128-1)-imu_map[i])+3,1);
//			memcpy((uint8_t*)mag_mimu_data[i]+1,(uint8_t*)(data_matrix+(160-1)-imu_map[i])+0,1);
//			memcpy((uint8_t*)mag_mimu_data[i]+2,(uint8_t*)(data_matrix+(160-1)-imu_map[i])+1,1);
//			memcpy((uint8_t*)mag_mimu_data[i]+3,(uint8_t*)(data_matrix+(160-1)-imu_map[i])+2,1);
//			memcpy((uint8_t*)mag_mimu_data[i]+4,(uint8_t*)(data_matrix+(160-1)-imu_map[i])+3,1);
//			memcpy((uint8_t*)mag_mimu_data[i]+5,(uint8_t*)(data_matrix+(192-1)-imu_map[i])+0,1);
//		}
//		if (!(*((uint8_t*)(data_matrix+(128-1)-imu_map[i])+2))){
//			memset((uint8_t*)mag_mimu_data[i]+0,0,2);
//		}
//		if ((*((uint8_t*)(data_matrix+(192-1)-imu_map[i])+1)&0x04)){
//			memset((uint8_t*)mag_mimu_data[i]+2,0,2);
//		}

	}
}
*/
