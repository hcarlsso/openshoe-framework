/*
 * processing_functions.c
 *
 * Created: 2011-12-14 10:59:45
 *  Author: jnil02
 */ 


#include "processing_functions.h"

// Externally declared processing functions
extern void update_imu_data_buffers(void);
extern void initialize_navigation_algorithm(void);
extern void strapdown_mechanisation_equations(void);
extern void time_up_data(void);
extern void ZUPT_detector(void);
extern void zupt_update(void);
extern void precision_gyro_bias_null_calibration(void);
extern void calibrate_accelerometers(void);

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

// Array containing the processing functions to run
proc_func_info* processing_functions_by_id[256];

void processing_functions_init(void){
	for(int i = 0;i<(sizeof(processing_functions)/sizeof(processing_functions[0])); i++){
		processing_functions_by_id[processing_functions[i]->id] = processing_functions[i];}
}