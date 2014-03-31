
/** \file
	\authors John-Olof Nilsson
	\copyright Copyright (c) 2014 OpenShoe, Cre­ative Com­mons Attri­bu­tion 4.0 License
*/

#include "timer_interrupt.h"
#include <asf.h>


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
extern uint32_t interrupt_counter;
// Global IMU interrupt (data) time-stamp
extern uint32_t interrupt_ts;
// Variable that used to signal if an external interrupt occurs.
extern volatile bool imu_interrupt_flag;


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
void tc_sample_irq(void)
{
	// Clear the interrupt flag. This is a side effect of reading the TC SR.
	tc_read_sr(ADDRESS_TC, SAMPLE_TC_CHANNEL);
	//gpio_tgl_gpio_pin(AVR32_PIN_PA08);
	interrupt_ts = Get_system_register(AVR32_COUNT);
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
void tc_init(volatile avr32_tc_t *tc)
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

// Variable to contain the time ticks occurred
volatile static avr32_tc_t *tc = ADDRESS_TC;

void timer_interrput_init(void){
	Disable_global_interrupt();
	INTC_register_interrupt(&tc_sample_irq, SAMPLE_TC_IRQ, TC_IRQ_PRIORITY);
	sysclk_enable_peripheral_clock(ADDRESS_TC);
	tc_init(tc);
	Enable_global_interrupt();
}