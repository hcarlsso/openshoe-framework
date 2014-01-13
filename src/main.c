

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
#include "MPU9150_interface.h"

//******* TC0 module is used to generate the interrupts *******//
// Address to TC
#define ADDRESS_TC                 (&AVR32_TC0)
//! \note IRQ Group of TC0 module
#define TC_IRQ_GROUP       AVR32_TC0_IRQ_GROUP
//! \note Interrupt priority 0 is used for TC in this example.
#define TC_IRQ_PRIORITY    AVR32_INTC_INT0


//! \note TC Channel 0 is used for the sampling
#define SAMPLE_TC_CHANNEL         0
//! \note IRQ0 line of TC0 module channel 0 is used.
#define SAMPLE_TC_IRQ             AVR32_TC0_IRQ0
//********************************************************//

// Interrupt counter (essentially a time stamp)
uint32_t interrupt_counter = 0;
// Global IMU interrupt (data) time-stamp
uint32_t imu_interrupt_ts;
// Input variable to Navigation algorithm for time differentials
extern uint32_t imu_dt;
// Variable that used to signal if an external interrupt occurs.
static volatile bool imu_interrupt_flag = false;



// Structure holding the configuration parameters of the EIC module.
static eic_options_t eic_options;

void imu_interrupt_init(void){
	// EIC settings
	eic_options.eic_mode   = EIC_MODE_EDGE_TRIGGERED ;
	eic_options.eic_edge   = EIC_EDGE_RISING_EDGE ;
	eic_options.eic_async  = EIC_SYNCH_MODE;
	eic_options.eic_line   = IMU_INTERUPT_LINE1;
	eic_options.eic_filter = AVR32_EIC_FILTER_ON;
	
	Disable_global_interrupt();
	eic_init(&AVR32_EIC, &eic_options,IMU_INTERUPT_NB_LINES);
	eic_enable_line(&AVR32_EIC, eic_options.eic_line);
	eic_enable_interrupt_line(&AVR32_EIC, eic_options.eic_line);
	Enable_global_interrupt();
}

#if __GNUC__
__attribute__((__naked__))
#elif __ICCAVR32__
#pragma shadow_registers = full
#endif
// This handler is connected to the interrupt in "exception.S"
void eic_nmi_handler( void )
{
	// Save registers not saved upon NMI exception.
	__asm__ __volatile__ ("pushm   r0-r12, lr\n\t");
	
	eic_clear_interrupt_line(&AVR32_EIC, IMU_INTERUPT_LINE1);
	imu_interrupt_ts = Get_system_register(AVR32_COUNT);
	imu_interrupt_flag = true;
	interrupt_counter++;
	
	// Significant amount of processing should not be done inside this routine
	// since the USB communication will be blocked for its duration.
	
	// Restore the registers and leaving the exception handler.
	__asm__ __volatile__ ("popm   r0-r12, lr\n\t" "rete");
}



// Variable to contain the time ticks occurred
volatile static avr32_tc_t *tc = ADDRESS_TC;


/**
 * \brief TC interrupt.
 *
 * The ISR handles RC compare interrupt and sets the update_timer flag to
 * update the timer value.
 */
#if defined (__GNUC__)
__attribute__((__interrupt__))
#elif defined (__ICCAVR32__)
#pragma handler = TC_IRQ_GROUP, 1
__interrupt
#endif
static void tc_sample_irq(void)
{
	// Clear the interrupt flag. This is a side effect of reading the TC SR.
	tc_read_sr(ADDRESS_TC, SAMPLE_TC_CHANNEL);
	//gpio_tgl_gpio_pin(AVR32_PIN_PA08);
	imu_interrupt_ts = Get_system_register(AVR32_COUNT);
	imu_interrupt_flag=true;
	interrupt_counter++;
}


/**
 * \brief TC Initialization
 *
 * Initializes and start the TC module with the following:
 * - Counter in Up mode with automatic reset on RC compare match.
 * - fPBA/8 is used as clock source for TC
 * - Enables RC compare match interrupt
 * \param tc Base address of the TC module
 */
static void tc_init(volatile avr32_tc_t *tc)
{
	// Options for waveform generation.
	static const tc_waveform_opt_t sample_waveform_opt = {
		// Channel selection.
		.channel  = SAMPLE_TC_CHANNEL,
		// Software trigger effect on TIOB.
		.bswtrg   = TC_EVT_EFFECT_NOOP,
		// External event effect on TIOB.
		.beevt    = TC_EVT_EFFECT_NOOP,
		// RC compare effect on TIOB.
		.bcpc     = TC_EVT_EFFECT_NOOP,
		// RB compare effect on TIOB.
		.bcpb     = TC_EVT_EFFECT_NOOP,
		// Software trigger effect on TIOA.
		.aswtrg   = TC_EVT_EFFECT_NOOP,
		// External event effect on TIOA.
		.aeevt    = TC_EVT_EFFECT_NOOP,
		// RC compare effect on TIOA.
		.acpc     = TC_EVT_EFFECT_NOOP,
		/*
		 * RA compare effect on TIOA.
		 * (other possibilities are none, set and clear).
		 */
		.acpa     = TC_EVT_EFFECT_NOOP,
		/*
		 * Waveform selection: Up mode with automatic trigger(reset)
		 * on RC compare.
		 */
		.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,
		// External event trigger enable.
		.enetrg   = false,
		// External event selection.
		.eevt     = 0,
		// External event edge selection.
		.eevtedg  = TC_SEL_NO_EDGE,
		// Counter disable when RC compare.
		.cpcdis   = false,
		// Counter clock stopped with RC compare.
		.cpcstop  = false,
		// Burst signal selection.
		.burst    = false,
		// Clock inversion.
		.clki     = false,
		// Internal source clock 1, connected to fPBA / 128.
		.tcclks   = TC_CLOCK_SOURCE_TC3
	};


	// Options for enabling TC interrupts
	static const tc_interrupt_t tc_interrupt = {
		.etrgs = 0,
		.ldrbs = 0,
		.ldras = 0,
		.cpcs  = 1, // Enable interrupt on RC compare alone
		.cpbs  = 0,
		.cpas  = 0,
		.lovrs = 0,
		.covfs = 0
	};
	// Initialize the timer/counter.
	tc_init_waveform(tc, &sample_waveform_opt);

	/*
	 * Set the compare triggers.
	 * We configure it to count every 1 milliseconds.
	 * We want: (1 / (fPBA / 8)) * RC = 1 ms, hence RC = (fPBA / 8) / 200
	 * to get an interrupt every 2 ms.
	 */
	tc_write_rc(tc, SAMPLE_TC_CHANNEL, 20000);
	// configure the timer interrupt
	tc_configure_interrupts(tc, SAMPLE_TC_CHANNEL, &tc_interrupt);
	// Start the timer/counter.
	tc_start(tc, SAMPLE_TC_CHANNEL);
}


/// Initialize hardware and communication interfaces
void system_init(void){
	board_init();
	sysclk_init();

	irq_initialize_vectors();
	INTC_register_interrupt(&tc_sample_irq, SAMPLE_TC_IRQ, TC_IRQ_PRIORITY);
	sysclk_enable_peripheral_clock(ADDRESS_TC);
	cpu_irq_enable();
	
	com_interface_init();

	imu_interrupt_init();
	imu_interface_init();

//	mpu9150_interface_init();
//	tc_init(tc);
	
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