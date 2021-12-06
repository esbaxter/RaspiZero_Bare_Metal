/*Copyright 2021 Eric Baxter <ericwbaxter85@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining 
a copy of this software and associated documentation files (the "Software"), 
to deal in the Software without restriction, including without limitation 
the rights to use, copy, modify, merge, publish, distribute, sublicense, 
and/or sell copies of the Software, and to permit persons to whom the Software 
is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all 
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

File:  altitude_package.c

Implements an altitude tracking function using the Bosch BME 280 temperature, pressure
and humidity chip.

*/

#include "altitude_package.h"
#include "bme280.h"
#include "mpu6050.h"
#include "log.h"
#include "arm_timer.h"
#include <math.h>

#define BME280_MEASUREMENT_ERROR .002
#define BME280_INITIAL_ERROR_ESTIMATE 0.4
#define BME280_CONVERGENCE_LOOP_COUNT 200

#define MAGIC_EXPONENT				0.1902225603956629
#define CENTIGRADE_TO_KELVIN		273.15 
#define ATMOSPHERIC_LAPSE_RATE		0.000065  // This is in K/cm
#define STANDARD_MSL_HPASCALS		1013.25

//double fabs(double x);

typedef struct Kalman_Data {
	double measurement_error;
	double estimate_error;	
	double estimate;
} Kalman_Filter_Data;

static double base_pressure[BME280_NUMBER_SUPPORTED_DEVICES];
static Kalman_Filter_Data kalman_filter_data[BME280_NUMBER_SUPPORTED_DEVICES];

void update_estimate(double measure, Kalman_Filter_Data *filter_data)
{	
	double kalman_gain = filter_data->estimate_error/(filter_data->estimate_error + filter_data->measurement_error);
	filter_data->estimate = filter_data->estimate + (kalman_gain * (measure - filter_data->estimate));
	filter_data->estimate_error = (1.0 - kalman_gain)*filter_data->estimate_error;
	//printf("%f,%f,%f,%f\n\r", measure, filter_data->estimate, kalman_gain, filter_data->estimate_error);
	return;
}

Error_Returns get_filtered_readings()
{
	Error_Returns to_return = RPi_Success;
	do
	{
		for(BME280_id bme280_id = bme280_one; bme280_id <= bme280_two; bme280_id++)
		{
			int32_t bme280_offset = bme280_get_offset_from_id(bme280_id);
			double raw;
			to_return = bme280_get_current_pressure(bme280_id, &raw);
			if (to_return != RPi_Success)
			{
				log_string_plus("altitude_package: get_filtered_readings failed: ", to_return);
				break;
			}
			update_estimate(raw, &kalman_filter_data[bme280_offset]);
		}
		
		if (to_return != RPi_Success)
		{
			break;
		}	
	} while(0);
	return to_return;
}

Error_Returns reset_base_pressure()
{
	Error_Returns to_return = RPi_Success;
	do
	{
		for(BME280_id bme280_id = bme280_one; bme280_id <= bme280_two; bme280_id++)
		{			
			int32_t bme280_offset = bme280_get_offset_from_id(bme280_id);
			to_return = bme280_get_current_pressure(bme280_id, &kalman_filter_data[bme280_offset].estimate);
			if (to_return != RPi_Success)
			{
				log_string_plus("altitude_package: get_filtered_readings failed to get initialize pressure: ", to_return);
				break;
			}
			kalman_filter_data[bme280_offset].measurement_error = BME280_MEASUREMENT_ERROR;
			kalman_filter_data[bme280_offset].measurement_error = BME280_MEASUREMENT_ERROR;
			kalman_filter_data[bme280_offset].estimate_error = BME280_INITIAL_ERROR_ESTIMATE;
		}

		//Find a stable value for the at rest pressure
		for (unsigned int count = 0; count < BME280_CONVERGENCE_LOOP_COUNT; count++)
		{
			to_return = get_filtered_readings();
			if (to_return != RPi_Success)
			{
				log_string_plus("altitude_package: reset_base_pressure() failed to get filtered reading: ", to_return);
				break;
			}
			spin_wait_milliseconds(10);
		}
		base_pressure[0] = kalman_filter_data[0].estimate;
		base_pressure[1] = kalman_filter_data[1].estimate;
	} while(0);
	return to_return;
}

Error_Returns altitude_initialize()
{
	Error_Returns to_return = RPi_Success;

	do
	{
		//Initialize each BME 280
		for(BME280_id bme280_id = bme280_one; bme280_id <= bme280_two; bme280_id++)
		{
			to_return = bme280_init(bme280_id, bme280_altitude_mode);
			if (to_return != RPi_Success)
			{
				log_string_plus("altitude_package: bme280_one bme280_init failed: ", to_return);
				break;
			}
		}
		
		to_return = reset_base_pressure();	
		if (to_return != RPi_Success)
			{
				break;
			}
			
			
		printf("base pressures: %f,%f\n\r", kalman_filter_data[0].estimate, kalman_filter_data[1].estimate);
		if (to_return != RPi_Success)
		{
			break;
		}
			
		to_return = mpu6050_init();
		if (to_return != RPi_Success)
		{
			log_string_plus("altitude_package: mpu6050_init failed: ", to_return);
		}		
		
	} while(0);	
	return to_return;
}

Error_Returns altitude_reset()
{
	return reset_base_pressure();
}

Error_Returns altitude_get_delta(int32_t *delta_cm_ptr)
{
	Error_Returns to_return = RPi_Success;
	double altitude_average = 0.0;
	
	do
	{		
		double altitude[BME280_NUMBER_SUPPORTED_DEVICES];
		
		to_return = get_filtered_readings();
		//printf("%f,%f\n\r", kalman_filter_data[0].estimate, kalman_filter_data[1].estimate);
		if (to_return != RPi_Success)
		{
			log_string_plus("altitude_package: get_filtered_readings failed while in altitude_get_delta: ", to_return);
			break;
		}
		
		for(BME280_id bme280_id = bme280_one; bme280_id <= bme280_two; bme280_id++)
		{
			double current_temperature;
			int32_t bme280_offset = bme280_get_offset_from_id(bme280_id);
			to_return = bme280_get_current_temperature(bme280_id, &current_temperature);
			if (to_return != RPi_Success)
			{
				log_string_plus("altitude_package: bme280_get_current_temperature failed: ", to_return);
				break;
			}
			
			/* Calculation is based on the hypsometric formula found here:
			   https://keisan.casio.com/exec/system/1224585971
			*/
			altitude[bme280_offset] = (((pow(base_pressure[bme280_offset] / 
				kalman_filter_data[bme280_offset].estimate, MAGIC_EXPONENT) - 1) * 
				(current_temperature + CENTIGRADE_TO_KELVIN)) / ATMOSPHERIC_LAPSE_RATE);
			altitude_average += altitude[bme280_offset];
		}
		
		if (to_return != RPi_Success)
		{
			break;
		}
		
		altitude_average /= 2.0;
		printf("%f, %f, %f\n\r", altitude[0], altitude[1], altitude_average);
		*delta_cm_ptr = (int32_t)altitude_average;
	} while(0);
	
	return to_return;
}
