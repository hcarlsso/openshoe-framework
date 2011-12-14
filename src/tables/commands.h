
/** \file
	\brief Definitions of commands and declarations of command response functions.
	
	\details This header file contains
	1) Declarations of the command response functions.
	2) Definitions of commands
	The command response functions are defined in external_interfaces.c.
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


#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "compiler.h"

// Definition structure of commands
typedef const struct {
	uint8_t header;
	void (*cmd_response)(uint8_t**);
	uint8_t nrb_payload;
	uint8_t nr_fields;
	uint8_t field_widths[];
} command_structure;


// Global variables used to access command information
extern uint8_t command_header_table[32];
extern command_structure* command_info_array[256];

void commands_init(void);


#endif /* COMMANDS_H_ */

//@}