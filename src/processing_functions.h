
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


/*********************************** Processing functions *************************************/
// Information struct for processing functions
typedef const struct {
	uint8_t id;
	void (*func_p)(void);
	int max_proc_time; 
} proc_func_info;

// Externally declared processing functions
extern void update_imu_data_buffers(void);
extern void initialize_navigation_algorithm(void);
extern void strapdown_mechanisation_equations(void);
extern void time_up_data(void);
extern void ZUPT_detector(void);
extern void zupt_update(void);
extern void precision_gyro_bias_null_calibration(void);
extern void calibrate_accelerometers(void);

// Processing functions ids
#define UPDATE_BUFFER 0x04
#define INITIAL_ALIGNMENT 0x05
#define MECHANIZATION 0x06
#define TIME_UPDATE 0x07
#define ZUPT_DETECTOR 0x08
#define ZUPT_UPDATE 0x09
#define GYRO_CALIBRATION 0x10
#define ACCELEROMETER_CALIBRATION 0x11

static proc_func_info update_imu_data_buffers_info = {UPDATE_BUFFER,&update_imu_data_buffers,0};
static proc_func_info initialize_navigation_algorithm_info = {INITIAL_ALIGNMENT,&initialize_navigation_algorithm,0};
static proc_func_info strapdown_mechanisation_equations_info = {MECHANIZATION,&strapdown_mechanisation_equations,0};
static proc_func_info time_up_data_info = {TIME_UPDATE,&time_up_data,0};
static proc_func_info ZUPT_detector_info = {ZUPT_DETECTOR,&ZUPT_detector,0};
static proc_func_info zupt_update_info = {ZUPT_UPDATE,&zupt_update,0};
static proc_func_info precision_gyro_bias_null_calibration_info = {GYRO_CALIBRATION,&precision_gyro_bias_null_calibration,0};
static proc_func_info calibrate_accelerometers_info = {ACCELEROMETER_CALIBRATION,&calibrate_accelerometers,0};

const proc_func_info* processing_functions[] = {&update_imu_data_buffers_info,
													   &initialize_navigation_algorithm_info,
													   &strapdown_mechanisation_equations_info,
													   &time_up_data_info,
													   &ZUPT_detector_info,
													   &zupt_update_info,
													   &precision_gyro_bias_null_calibration_info,
													   &calibrate_accelerometers_info};


#endif /* PROCESSING_FUNCTIONS_H_ */