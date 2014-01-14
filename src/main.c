

/** \file
	\brief Main function and interrupt control
	
	\details This file contains the main function and interrupt control.
	The main function control the execution of the program. The single
	interrupt routine only toggles a flag which the main function is polling.
	Each time the flag is toggled, the following will be executed in the main
	loop: 1) data is read from the IMU 2) functions in the process sequence
	are executed 3) commands are received from the user 4) data are
	transmitted back 5) and it is checked that a new interrupt did not arrived
	while the program was executing the main loop.
	
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
*/ 

/**
  \defgroup openshoe_runtime_framework OpenShoe runtime framework
  \brief This module collect all software written for OpenShoe.
  
  \ingroup openshoe_runtime_framework
  @{
*/

/*
 * Include header files for all drivers that have been imported from
 * AVR Software Framework (ASF).
 */
#include <asf.h>

#include "process_sequence.h"
#include "external_interface.h"
#include "ADIS16367_interface.h"
#include "MPU9150_interface.h"
#include "timer_interrupt.h"
#include "toggle_interrupt.h"


// Interrupt counter (essentially a time stamp)
uint32_t interrupt_counter = 0;
// Global IMU interrupt (data) time-stamp
uint32_t imu_interrupt_ts;
// Input variable to Navigation algorithm for time differentials
uint32_t imu_dt;
// Variable that used to signal if an external interrupt occurs.
volatile bool imu_interrupt_flag = false;


/// Initialize hardware and communication interfaces
void system_init(void){
	board_init();
	sysclk_init();
	irq_initialize_vectors();
	cpu_irq_enable();
	
//	timer_interrput_init();
//	mpu9150_interface_init();

	toggle_interrupt_init();
	ADIS16367_interface_init();
	
	com_interface_init();
}

/// Wait for the interrupt flag to be set, toggle it, increase interrupt counter, and return.
void wait_for_interrupt(void){
	static uint32_t imu_interrupt_ts_old = 0;
	while(true){
		if(imu_interrupt_flag==true){
			imu_interrupt_flag=false;
			imu_dt = imu_interrupt_ts - imu_interrupt_ts_old;
			imu_interrupt_ts_old = imu_interrupt_ts;
			return;
		}
	}	
}

/// Checks that the main loop has finished before next interrupt
void within_time_limit(void){
	if (imu_interrupt_flag!=false){
		// Todo: Set some error state
	}
}

int main (void) {
	
	// Initialize system
	system_init();
	
	// Main executing loop: Loops indefinitely
	while (true) {
		
		// Check if interrupt has occurred
		wait_for_interrupt();
		
//		gpio_tgl_gpio_pin(AVR32_PIN_PB01);
		
		// Read data from IMU			
		imu_burst_read();
//		mpu9150_read();

		// Execute all processing functions (filtering)
		run_process_sequence();

		// Check if any command has been sent and respond accordingly
		receive_command();
			
		// Transmit requested data to user
		transmit_data();
		
		// Ensure the loop was finished within time limit
		within_time_limit();
	}
}

//! @}