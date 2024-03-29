

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
#include "imu_interface.h"
#include "imu_interrupt.h"


// Initialize system
void system_init(void);

int main (void) {
	
	// Initialize system
	system_init();
	
	// Main executing loop: Loops indefinitely
	while (true) {
		
		// Check if interrupt has occurred
		wait_for_interrupt();
		
		// Check if any command has been sent and respond accordingly
		receive_command();

		// Execute all processing functions (filtering)
		run_process_sequence();
			
		// Transmit requested data to user
		transmit_data();
		
		// Whatever need to be done at the end of the loop
		end_of_main_loop();
	}
	
}


/// Initialize hardware and communication interfaces
void system_init(void){
	sysclk_init();
	irq_initialize_vectors();
	cpu_irq_enable();
	INTC_init_interrupts();  //Don't know if this one is needed at all
	board_init();
	
	imu_interface_init();
	external_interface_init();
	process_sequence_init();
	
	interrupt_init();
}

//! @}