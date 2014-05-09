

/** \file
	\brief Declarations of externally controlled process sequence frunctions.
	
	\details This file contain declarations of functions which can be added
	to the process sequence and definitions of information structs to these
	functions. The functions are declared and defined elsewhere in the system.
	This is	just a place where the are all gathered. This will facilitate the
	management of all processing functions.
	Any function which should be inserted in to the process sequence should be
	added to this file.
	
	The file also contain an initialization function to fill up some arrays of
	states structs. This initialization function must be called before the
	arrays are used. This should eventually be replaced by some code generating
	script which explicitly declear the arrays.
	
	\authors John-Olof Nilsson, Isaac Skog
	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
*/

/**
	\addtogroup control_tables
	@{
*/

#include "control_tables.h"
#include "inertial_frontend.h"
#include "nav_eq.h"

extern void precision_gyro_bias_null_calibration(void);

///  \name Processing functions information
///  Structs containing information and pointers to functions intended for the process sequence
//@{
static proc_func_info frontend_preproc_info = {FRONTEND_PREPROC,&frontend_preproc,0};
static proc_func_info frontend_statdet_info = {FRONTEND_STATDET,&frontend_statdet,0};
static proc_func_info frontend_postproc_info = {FRONTEND_POSTPROC,&frontend_postproc,0};
static proc_func_info initial_alignment_info = {FRONTEND_INITIAL_ALIGNMENT,&frontend_initial_alignment,0};
static proc_func_info strapdown_mechanisation_equations_info = {MECHANIZATION,&strapdown_mechanisation_equations,0};
static proc_func_info time_up_data_info = {TIME_UPDATE,&time_up_data,0};
static proc_func_info zupt_update_info = {ZUPT_UPDATE,&zupt_update,0};
static proc_func_info precision_gyro_bias_null_calibration_info = {GYRO_CALIBRATION,&precision_gyro_bias_null_calibration,0};
static proc_func_info calibrate_accelerometers_info = {ACCELEROMETER_CALIBRATION,&calibrate_accelerometers,0};
static proc_func_info stepwise_system_reset_info = {STEPWISE_SYSTEM_RESET,&stepwise_system_reset,0};
//@}

static const proc_func_info* processing_functions[] = {&strapdown_mechanisation_equations_info,
													   &time_up_data_info,
													   &zupt_update_info,
													   &precision_gyro_bias_null_calibration_info,
													   &calibrate_accelerometers_info,
													   &stepwise_system_reset_info,
													   &frontend_preproc_info,
													   &frontend_statdet_info,
													   &frontend_postproc_info,
													   &initial_alignment_info};

// Array containing the processing functions to run
proc_func_info* processing_functions_by_id[256];

void processing_functions_init(void){
	for(int i = 0;i<(sizeof(processing_functions)/sizeof(processing_functions[0])); i++){
		processing_functions_by_id[processing_functions[i]->id] = processing_functions[i];}
}