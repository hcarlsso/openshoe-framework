
/** \file
	\brief Functions for manipulating and running process sequence.
	
	\details An internal arrary of functions pointers (\#process_sequence) are
	use to store pointers to functions which are to be executed for some
	processing sequence. The pointers are of (void) (f*)(void) type meaning
	that they have to communicate via some global or c-file internal variables.
	The functions in this file are used for manipulating the \#process_sequence
	and to execute the whole processing function sequence.
		
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
*/

///\addtogroup proc_sequence
//@{

#include "process_sequence.h"
#include "control_tables.h"

/// Process sequence
static processing_function_p process_sequence[PROCESS_SEQUENCE_SIZE] = {NULL};	
/// Temporary storage for copy of processing sequence.
static processing_function_p process_sequence_storage[PROCESS_SEQUENCE_SIZE] = {NULL};
	
uint32_t ps_dt[PROCESS_SEQUENCE_SIZE] = {0};

void process_sequence_init(void){
	//set_elem_in_process_sequence(READ_INERTIAL,0);
}

/// Execute all non-NULL functions in the processing sequence.
void run_process_sequence(void){
	uint32_t ps_ts0 = Get_system_register(AVR32_COUNT);
	uint32_t ps_ts1;
	for(int i=0;i<PROCESS_SEQUENCE_SIZE;i++){
		if(process_sequence[i])
			process_sequence[i]();
		ps_ts1 = Get_system_register(AVR32_COUNT);
		ps_dt[i] = ps_ts1-ps_ts0;
		ps_ts0 = ps_ts1;
	}
}

/// Sets alla elements in processing sequence to NULL.
void empty_process_sequence(void){
	for(int i=0;i<PROCESS_SEQUENCE_SIZE;i++)
		process_sequence[i]=NULL;
}

/// Copy processing sequence to temporary storage and sets all elements to NULL.
void store_and_empty_process_sequence(void){
	for(int i = 0;i<PROCESS_SEQUENCE_SIZE;i++){
		process_sequence_storage[i] = process_sequence[i];
		process_sequence[i]=NULL;
	}
}


/**	
	\brief Copy the content of the \#process_sequence_storage to the \#process_sequence.

	\details This assumes that the store_and_empty_process_sequence routine has
	been run prior to this routine call.
*/
void restore_process_sequence(void){
	for(int i = 0;i<PROCESS_SEQUENCE_SIZE;i++)
		process_sequence[i] = process_sequence_storage[i];
}
	
		
/**	
	\brief Sets last element in the process sequence to the argument function pointer
	
	\details Sets last element in the process sequence to the argument function pointer.
	This is often used for clean-up functions which will in turn empty, restore,
	or manipulate the process_sequence.
	
	@param[in] elem_value Function pointer to insert into \#process_sequence.
*/
void set_last_process_sequence_element(uint8_t id){
	if(id<PID_LIMIT)
		process_sequence[PROCESS_SEQUENCE_SIZE-1] = processing_functions_by_id[id] ? processing_functions_by_id[id]->func_p : NULL;
}

/**
	\brief Sets process sequence element number to elem_value
	
	\details If elem_nr is less than \#process_sequence length (less than \#PROCESS_SEQUENCE_SIZE),
	sets the element value to elem_value. If the value is larger no action is taken.
	It is left up to the user to ensure that the elem_nr is valid. For this
	purpose the \#PROCESS_SEQUENCE_SIZE macro can be used.
	
	@param[in] elem_value Function pointer to insert into \#process_sequence.
	@param[in] elem_nr    \#process_sequence element index
*/	

void set_elem_in_process_sequence(uint8_t id, uint8_t elem_nr){
	if(elem_nr<PROCESS_SEQUENCE_SIZE && id<PID_LIMIT)
		process_sequence[elem_nr] = processing_functions_by_id[id] ? processing_functions_by_id[id]->func_p : NULL;
}

//@}