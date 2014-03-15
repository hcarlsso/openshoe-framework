/*
 * MPU9150_interface.c
 *
 * Created: 2013-08-13 17:56:13
 *  Author: jnil02
 */ 

#include "MPU9150_interface.h"
//#include "i2c_com.h"
#include "mpu6150.h"
#include <stdint.h>
#include <string.h>
#include <asf.h>


//#define CLK_PORT_NUM  2
//#define CLK_PIN0 2
//#define CLK_PIN1 5
//#define CLK_PINS ((1<<CLK_PIN0)|(1<<CLK_PIN1))

/*Framsida
SDA9 PC04
SDA8 PA19
SDA7 PA16
SDA6 PA09
SDA5 PA08
SDA4 PA07
SDA3 PA06
SDA2 PA05
SDA1 PA04

Baksida
SDA16 PC18
SDA17 PC19
SDA18 PC20
SDA13 PC02
SDA14 PC03
SDA15 PC17
SDA10 PC05
SDA11 PC15
SDA12 PC16*/



//#define DATA_PORT1 1
//#define PORT1_IMU0 1
//#define PORT1_IMU1 0
//#define DATA_PINS1 ((1<<PORT1_IMU0)|(1<<PORT1_IMU1))
//#define NR_IMUS_PORT1 2

//#define PORT1_IMU0 3
//#define PORT1_IMU1 4
	
//#define DATA_PORT2  2
//#define DATA_PINS2 ((1<<2)|(1<<5))
//#define DATA_PINS2 ((1<<3)|(1<<4))

// Functions pacing I2C bitbanging
int32_t last_tick;
#define QUATER_CC_COUNT 57
// TODO: define this based on CPU freq and desired sampling rate
// 115 gives ~100kHz (with 48MHz clock)
__always_inline void start_tick(void){
	last_tick = Get_system_register(AVR32_COUNT);
}
__always_inline void half_tick(void){
	int32_t wait_to = last_tick + 2*QUATER_CC_COUNT;
	while (Get_system_register(AVR32_COUNT) - wait_to < 0);
	last_tick = Get_system_register(AVR32_COUNT);
}
__always_inline void quater_tick(void){
	int32_t wait_to = last_tick + QUATER_CC_COUNT;
	while (Get_system_register(AVR32_COUNT) - wait_to < 0);
	last_tick = Get_system_register(AVR32_COUNT);
}

// Function that reads all the values of a port
//#define gpio_get_port_value(port) (AVR32_GPIO.port[port].pvr)
// TODO: Use above function instead
uint32_t gpio_get_port_value(uint32_t port,uint32_t mask)
{
	volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[port];
	return (gpio_port->pvr ) & mask;
	// TODO: Remove this masking. It is not used.
}

// Bit-wise transposition of a 32x32 bit-matrix
void transpose32(uint32_t A[32]){
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
void I2C_init(void){
	// Initialize the IMU pins to open-drain outputs
	gpio_configure_group(CLK_PORT_NUM,CLK_PINS,(GPIO_OPEN_DRAIN | GPIO_DIR_OUTPUT));
	gpio_configure_group(DATA_PORTA,DATA_PINSA,(GPIO_OPEN_DRAIN | GPIO_DIR_OUTPUT));
	gpio_configure_group(DATA_PORTC,DATA_PINSC,(GPIO_OPEN_DRAIN | GPIO_DIR_OUTPUT));
}

// Function that sends the I2C start command
void I2C_start(void){
	
	start_tick();

	// Make sure all communication lines are in the start state
//	gpio_set_group_high(CLK_PORT_NUM,CLK_PINS);
//	gpio_set_group_high(DATA_PORT0,DATA_PINS0);
//	gpio_set_group_high(DATA_PORT1,DATA_PINS1);

	// Now pull the SDA lines low will keeping the SCL high
	gpio_set_group_low(DATA_PORTA,DATA_PINSA);
	gpio_set_group_low(DATA_PORTC,DATA_PINSC);
	quater_tick();
}

void I2C_start_read(void){
	
	// Short delay (1/2 clock cycle)
	gpio_set_group_low(CLK_PORT_NUM,CLK_PINS);
	half_tick();
	
	// Make sure all communication lines are in the start state
	gpio_set_group_high(CLK_PORT_NUM,CLK_PINS);
	gpio_set_group_high(DATA_PORTA,DATA_PINSA);
	gpio_set_group_high(DATA_PORTC,DATA_PINSC);
	
	// Short delay (1/4 clock cycle)
	quater_tick();

	// Now pull the SDA lines low will keeping the SCL high
	gpio_set_group_low(DATA_PORTA,DATA_PINSA);
	gpio_set_group_low(DATA_PORTC,DATA_PINSC);
	quater_tick();
}

// Function that sends the I2C start command
void I2C_stop(void)
{
	// Pull the SCL lines low
	gpio_set_group_low(CLK_PORT_NUM,CLK_PINS);
	quater_tick();

	// Pull the SDA lines low
	gpio_set_group_low(DATA_PORTA,DATA_PINSA);
	gpio_set_group_low(DATA_PORTC,DATA_PINSC);
	quater_tick();

	// Pull the SCL lines high
	gpio_set_group_high(CLK_PORT_NUM,CLK_PINS);
	quater_tick();

	// Pull the SDA lines high
	gpio_set_group_high(DATA_PORTA,DATA_PINSA);
	gpio_set_group_high(DATA_PORTC,DATA_PINSC);
}

// Function that clocks out a single byte on the I2C bus and that returns ACK (NACK) responds of the sensors
void I2C_write_byte(uint8_t data)
{
//	uint32_t ack_vec;
	uint8_t ctr;
	
	for (ctr=8;ctr>0;ctr--)
	{
		// Pull clk low
		gpio_set_group_low(CLK_PORT_NUM,CLK_PINS);
		quater_tick();

		// Set SDA 1
		if ((data>>(ctr-1)) & 0x01)
		{
			gpio_set_group_high(DATA_PORTA,DATA_PINSA);
			gpio_set_group_high(DATA_PORTC,DATA_PINSC);
		}
		else
		{
			gpio_set_group_low(DATA_PORTA,DATA_PINSA);
			gpio_set_group_low(DATA_PORTC,DATA_PINSC);			
		}
		quater_tick();

		// Pull clk high
		gpio_set_group_high(CLK_PORT_NUM,CLK_PINS);
		half_tick();
	}

	// ACK
	// Pull clk low
	gpio_set_group_low(CLK_PORT_NUM,CLK_PINS);
	quater_tick();

	// Set SDA 1
	gpio_set_group_high(DATA_PORTA,DATA_PINSA);
	gpio_set_group_high(DATA_PORTC,DATA_PINSC);
	quater_tick();

	// Pull clk high
	gpio_set_group_high(CLK_PORT_NUM,CLK_PINS);
	quater_tick();

	// Read the state of the SLA lines
//	ack_vec=gpio_get_port_value(DATA_PORT0,DATA_PINS0);
//	ack_vec=gpio_get_port_value(DATA_PORT1,DATA_PINS1);
	quater_tick();

	// Pull SCL low (gives more even clock at higher rates)
	gpio_set_group_low(CLK_PORT_NUM,CLK_PINS);
}

// Function that reads a single byte on the I2C bus. The input flag signals if a ACK should be read or a NACK outputted.
void I2C_read_byte(uint32_t *data_port0,uint32_t *data_port1,Bool ack_flag){
	
	uint8_t ctr;
	
	
	for (ctr=0;ctr<8;ctr++)
	{		
		// Pull clk low
		gpio_set_group_low(CLK_PORT_NUM,CLK_PINS);
		half_tick();
		
		// Pull clk high
		gpio_set_group_high(CLK_PORT_NUM,CLK_PINS);
		quater_tick();
		data_port0[ctr]=gpio_get_port_value(DATA_PORTA,DATA_PINSA);
		data_port1[ctr]=gpio_get_port_value(DATA_PORTC,DATA_PINSC);

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
		gpio_set_group_low(CLK_PORT_NUM,CLK_PINS);
		quater_tick();

		// Set SDA 1
		gpio_set_group_low(DATA_PORTA,DATA_PINSA);
		gpio_set_group_low(DATA_PORTC,DATA_PINSC);
		quater_tick();

		// Pull clk high
		gpio_set_group_high(CLK_PORT_NUM,CLK_PINS);
		half_tick();
	}
	else
	{
		// Send NACK
		// Pull clk low
		gpio_set_group_low(CLK_PORT_NUM,CLK_PINS);
		quater_tick();

		// Set SDA 1
		gpio_set_group_high(DATA_PORTA,DATA_PINSA);
		gpio_set_group_high(DATA_PORTC,DATA_PINSC);
		quater_tick();

		// Pull clk high
		gpio_set_group_high(CLK_PORT_NUM,CLK_PINS);
		half_tick();
	}

	// Pull SCL low
	gpio_set_group_low(CLK_PORT_NUM,CLK_PINS);
	// Release data pins
	gpio_set_group_high(DATA_PORTA,DATA_PINSA);
	gpio_set_group_high(DATA_PORTC,DATA_PINSC);
}

//Function thats writes a single byte to the specified register address.
void single_byte_write(uint8_t address,uint8_t data)
{
	// Send I2C start command
	I2C_start();

	// Send device address with write command
	I2C_write_byte(((MPU6150_ADDRESS_AD0_LOW<<1) & 0xFE));

	// Send register address
	I2C_write_byte(address);

	// Send data
	I2C_write_byte(data);

	// Send stop
	I2C_stop();
}

//Function that reads a single byte from the specified register address
void single_byte_read(uint8_t address,uint32_t *data_port0,uint32_t *data_port1)
{
	// Send I2C start command
	I2C_start();
	
	// Send device address with write command
	I2C_write_byte(((MPU6150_ADDRESS_AD0_LOW<<1) & 0xFE));
		
	// Send register address
	I2C_write_byte(address);
	
	// Send I2C start command
	I2C_start_read();
	
	// Send device address with read command
	I2C_write_byte(((MPU6150_ADDRESS_AD0_LOW<<1) | 0x01));
	
	// Read byte (false=NACK)
	I2C_read_byte(data_port0,data_port1,false);

	// Send stop
	I2C_stop();
}

//Function that reads multiple bytes in a burst
void burst_read(uint8_t address,uint32_t *data_port0,uint32_t *data_port1,uint8_t nr_of_bytes)
{
	uint8_t ctr;
	
	// Send I2C start command
	I2C_start();

	// Send device address with write command
	I2C_write_byte(((MPU6150_ADDRESS_AD0_LOW<<1) & 0xFE));
	
	// Send register address
	I2C_write_byte(address);
	
	// Send I2C start command
	I2C_start_read();
	
	// Send device address with read command
	I2C_write_byte(((MPU6150_ADDRESS_AD0_LOW<<1) | 0x01));
	
	// Read bytes (true=ACK)
	for (ctr=0;ctr<nr_of_bytes-1;ctr++)
	{
		I2C_read_byte(data_port0+ctr*8,data_port1+ctr*8,true);
	}
	// Read last byte (false=NACK)
	I2C_read_byte(data_port0+ctr*8,data_port1+ctr*8,false);
	
	// Send stop
	I2C_stop();
}

void mpu9150_interface_init(void){
	
	// Configure IMU pins for I2C communication
	I2C_init();
	
	// Wake up the devices and set the x-axis gyroscope as the reference oscillator
	single_byte_write(MPU6150_RA_PWR_MGMT_1,0x01);

	// Set the bandwidth of the sensors to 100 Hz (OBS we should sample with at least 200 Hz then). According to the data sheet
	// this seems to set the internal sample rate of the sensors to 1kHz.l
	single_byte_write(MPU6150_RA_CONFIG,0x02);

	// Set the accelerometer scale ranges to max (+-16g) (FS_SEL=3)
	single_byte_write(MPU6150_RA_ACCEL_CONFIG,0x18);

	// Set the gyroscope scale ranges to max (+-2000 deg/s) (AFS_SEL=3)
	single_byte_write(MPU6150_RA_GYRO_CONFIG,0x18);
}


// IMU array data
extern int16_t mimu_data[32][7];

// Buffers for reading data from the IMUs
uint32_t data_array_port0[128];
uint32_t data_array_port1[128];

const uint8_t imus0_pos[NR_IMUS_PORTA]={IMU1_PORTA,IMU2_PORTA,IMU3_PORTA,IMU4_PORTA,IMU5_PORTA,IMU6_PORTA,IMU7_PORTA,IMU8_PORTA};
const uint8_t imus1_pos[NR_IMUS_PORTC]={IMU0_PORTC,IMU9_PORTC,IMU10_PORTC,IMU11_PORTC,IMU12_PORTC,IMU13_PORTC,IMU14_PORTC,IMU15_PORTC,IMU16_PORTC,IMU17_PORTC};

// Read inertial measurements from IMU
void mpu9150_read(void)
{
	// Read all sensor registers (acc,temp,gyro)
	burst_read(0x3B,data_array_port0,data_array_port1,14);
	
	// Transpose 32x32 bit-blocks
	transpose32(data_array_port0);
	transpose32(data_array_port0+32);
	transpose32(data_array_port0+64);
	transpose32(data_array_port0+96);
	transpose32(data_array_port1);
	transpose32(data_array_port1+32);
	transpose32(data_array_port1+64);
	transpose32(data_array_port1+96);

	// Copy to data state arrays
	uint8_t i;
	for (i=0; i<NR_IMUS_PORTA; i++) {
		memcpy(mimu_data[i],data_array_port0+(32-1)-imus0_pos[i],4);
		memcpy(mimu_data[i]+2,data_array_port0+(64-1)-imus0_pos[i],2);
		memcpy(mimu_data[i]+3,data_array_port0+(96-1)-imus0_pos[i],4);
		memcpy(mimu_data[i]+5,data_array_port0+(128-1)-imus0_pos[i],2);
		memcpy(mimu_data[i]+6,(uint16_t*)(data_array_port0+(64-1)-imus0_pos[i])+1,2);
	}
	uint8_t j;
	for (j=0; j<NR_IMUS_PORTC; j++) {
		memcpy(mimu_data[i+j],data_array_port1+(32-1)-imus1_pos[j],4);
		memcpy(mimu_data[i+j]+2,data_array_port1+(64-1)-imus1_pos[j],2);
		memcpy(mimu_data[i+j]+3,data_array_port1+(96-1)-imus1_pos[j],4);
		memcpy(mimu_data[i+j]+5,data_array_port1+(128-1)-imus1_pos[j],2);
		memcpy(mimu_data[i+j]+6,(uint16_t*)(data_array_port1+(64-1)-imus1_pos[j])+1,2);
	}
}
