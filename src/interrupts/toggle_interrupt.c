/*
 * toggle_interrupt.c
 *
 * Created: 2014-01-14 11:49:52
 *  Author: jnil02
 */ 

#include "toggle_interrupt.h"
#include "classic.h"
#include <asf.h>

// Interrupt counter (essentially a time stamp)
extern uint32_t interrupt_counter;
// Global IMU interrupt (data) time-stamp
extern uint32_t imu_interrupt_ts;
// Input variable to Navigation algorithm for time differentials
extern uint32_t imu_dt;
// Variable that used to signal if an external interrupt occurs.
extern volatile bool imu_interrupt_flag;


// Structure holding the configuration parameters of the EIC module.
static eic_options_t eic_options;

void toggle_interrupt_init(void){
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
