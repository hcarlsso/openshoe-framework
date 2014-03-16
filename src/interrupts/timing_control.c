/*
 * timing_control.c
 *
 * Created: 2014-01-14 14:28:41
 *  Author: jnil02
 */ 

#include "timing_control.h"
#include <compiler.h>


// Interrupt counter (essentially a time stamp)
uint32_t interrupt_counter = 0;
// Global IMU interrupt (data) time-stamp
uint32_t imu_interrupt_ts;
// Input variable to Navigation algorithm for time differentials
uint32_t imu_dt;
// Variable that used to signal if an external interrupt occurs.
volatile bool imu_interrupt_flag = false;

// General purpose time differential. If not used for anything else it will contain the time of the main loop.
uint32_t gp_t;
uint32_t gp_dt;


/// Wait for the interrupt flag to be set, toggle it, increase interrupt counter, and return.
void wait_for_interrupt(void){
	static uint32_t imu_interrupt_ts_old = 0;
	while(true){
		if(imu_interrupt_flag==true){
			imu_interrupt_flag=false;
			imu_dt = imu_interrupt_ts - imu_interrupt_ts_old;
			imu_interrupt_ts_old = imu_interrupt_ts;
			gp_t = Get_system_register(AVR32_COUNT);
			return;
		}
	}
}

/// Checks that the main loop has finished before next interrupt
void within_time_limit(void){
	gp_dt = Get_system_register(AVR32_COUNT) - gp_t;
	if (imu_interrupt_flag!=false){
		// Todo: Set some error state
	}
}