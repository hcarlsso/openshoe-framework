
/** \file
	\brief Declarations of functions intended for the processing sequence and related information tables.
	
	\details This header file contains
	1) Declarations of functions intended for the processing sequence.
	2) Definitions of tables containing information about the functions
	intended for the processing sequence. The functions can be defined across
	the program. This file just gather the declarations together such that the
	related information tables can be filled in. The information tables are
	used for output functions in external_interfaces.c.
	This file contians multiple static variables so it should only be included
	where necessary.
	
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)	
*/ 

/**
	\ingroup openshoe_software
	
	\defgroup tables System tables	
	\brief This group contain table of system definitions.
	@{
*/

#ifndef PROCESSING_FUNCTIONS_H_
#define PROCESSING_FUNCTIONS_H_


#include "compiler.h"

/*********************************** Processing functions *************************************/
// Information struct for processing functions
typedef const struct {
	uint8_t id;
	void (*func_p)(void);
	int max_proc_time; 
} proc_func_info;



// Processing functions ids
#define UPDATE_BUFFER 0x04
#define INITIAL_ALIGNMENT 0x05
#define MECHANIZATION 0x06
#define TIME_UPDATE 0x07
#define ZUPT_DETECTOR 0x08
#define ZUPT_UPDATE 0x09
#define GYRO_CALIBRATION 0x10
#define ACCELEROMETER_CALIBRATION 0x11

// Array containing the processing functions to run
extern proc_func_info* processing_functions_by_id[256];

void processing_functions_init(void);


#endif /* PROCESSING_FUNCTIONS_H_ */