
/**
 * \mainpage The OpenShoe project
 * 
 * \section sec_intro Introduction
 * 
 * blablabla
 *
 */


/// \defgroup openshoe_software OpenShoe software
///	This module collect all software written for OpenShoe.
///	\ingroup openshoe_software
///	@{

/*
 * Include header files for all drivers that have been imported from
 * AVR Software Framework (ASF).
 */
#include <asf.h>

#include "process_sequence.h"
#include "external_interface.h"
#include "imu_interface.h"

// Delay functions
void mdelay(unsigned int ms);
void ccdelay(unsigned int cc);

// These should be removed
int window_size;
int time_since_last_zupt;
// Iteration counter
uint32_t process_cycle_counter = 0;


// Interrupt counter (essentially a time stamp)
uint32_t interrupt_counter = 0;
// Global IMU interrupt (data) time-stamp variable
uint32_t imu_interrupt_ts;
// Variable that used to signal if an external interrupt occurs.
static volatile bool imu_interrupt_flag = false;
// Structure holding the configuration parameters of the EIC module.
static eic_options_t eic_options;

void imu_interupt_init(void){
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
	
	// Restore the registers and leaving the exception handler.
	__asm__ __volatile__ ("popm   r0-r12, lr\n\t" "rete");
}

inline void wait_for_interrupt(void){
	while(true){
		if(imu_interrupt_flag==true){
			imu_interrupt_flag=false;
			interrupt_counter++;
		}
	}	
}	

inline void system_init(void){
	// Initialize hardware and communication interfaces
	irq_initialize_vectors();
	cpu_irq_enable();
	board_init();
	sysclk_init();
	com_interface_init();
	imu_interupt_init();
	imu_interface_init();
	// Any new initialization function of the system should be added here or
	// under any of the above initialization functions.
}

int main (void) {
	
	// Initialize system
	system_init();
	
	// Loop indefinately
	while (true) {
		
		// Check if interrupt has occured
		wait_for_interrupt();
/*
		if(imu_interupt_flag==true){
			imu_interupt_flag=false;
			interrupt_counter++;*/

			// Read data from IMU			
			imu_burst_read();

			// Execute all processing functions (filtering)
			run_process_sequence();

			// Check if any command has been sent and respond accordingly
			receive_command();
			
			// Transmit requested data to user
			transmit_data();
			
			// TODO: Check that the imu_interupt_flag is still false (otherwise the calculations has not completed in time and the system send a warning)
//		}		
	}
}


void mdelay(unsigned int ms)
{
	int32_t count, count_end;

	count = Get_system_register(AVR32_COUNT);
	count_end = count + ((sysclk_get_cpu_hz() + 999) / 1000) * ms;
	while ((count_end - count) > 0)
		count = Get_system_register(AVR32_COUNT);
}

void ccdelay(unsigned int cc)
{
	int32_t count, count_end;

	count = Get_system_register(AVR32_COUNT);
	count_end = count + cc;
	while ((count_end - count) > 0)
		count = Get_system_register(AVR32_COUNT);
}

//! @}