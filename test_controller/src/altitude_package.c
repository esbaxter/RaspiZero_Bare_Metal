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

Implements an altitude and position tracking function using the Bosch BME 280
temperature, pressure and humidity chip and the MPU 6050.

So the idea behind this was to use the BME 280 chip, grab a base pressure reading
and then at a given time interval grab an update pressure reading, compare the
two using a rearrangement of the barometric formula to get a change in altitude.

Unfortunately, the specs on the BME 280 are nice for gross altitude change
calculations, however, I was aiming for 1-2 cm of accuracy.  The best it appears I
can get from the chip is .12 hPa relative accuracy which equates to 1 meter.

So I implemented a Kalman filter (well, two of them actually) for the actual pressure
readings and the calculated altitude change.  It works somewhat but still not accurate
enough for what I was looking for.

*/

#include "altitude_package.h"
#include "bme280.h"
#include "mpu6050.h"
#include "log.h"
#include "arm_timer.h"
#include "aux_peripherals.h"
#include <math.h>

#define BME280_MEASUREMENT_ERROR 		1.0
#define BME280_INITIAL_ESTIMATE_ERROR 	1.0
#define BME280_INITIAL_KALMAN_GAIN		1.0
#define BME280_Q_FACTOR					0.01
#define BME280_CONVERGENCE_LOOP_COUNT 	10

#define MAGIC_EXPONENT				0.1902225603956629  //Basically 1/5.22
#define MAGIC_MULTIPLIER			44330

#define ALTITUDE_MEASUREMENT_ERROR		1.0
#define ALTITUDE_INITIAL_ESTIMATE_ERROR	1.0
#define ALTITUDE_INITIAL_KALMAN_GAIN	1.0
#define ALTITUDE_Q_FACTOR				0.01

#define STANDARD_MSL_HPASCALS		101325.0

#define ALT_PACKAGE_TICK_TIME 		10 //In milliseconds

typedef struct Kalman_Data {
	double measurement_error;
	double estimate_error;	
	double last_estimate;
	double estimate;
	double kalman_gain;
	double q_factor;
} Kalman_Filter_Data;

static double base_pressure[BME280_NUMBER_SUPPORTED_DEVICES];
static Kalman_Filter_Data kalman_filter_data[BME280_NUMBER_SUPPORTED_DEVICES];
static Kalman_Filter_Data current_altitude;

static Error_Returns altitude_state = RPi_Success;

static void reset_kalman_filter_pressure_data(int32_t bme280_offset)
{
	kalman_filter_data[bme280_offset].measurement_error = BME280_MEASUREMENT_ERROR;
	kalman_filter_data[bme280_offset].estimate_error = BME280_INITIAL_ESTIMATE_ERROR;
	kalman_filter_data[bme280_offset].last_estimate = STANDARD_MSL_HPASCALS;
	kalman_filter_data[bme280_offset].estimate = STANDARD_MSL_HPASCALS;
	kalman_filter_data[bme280_offset].kalman_gain = BME280_INITIAL_KALMAN_GAIN;
	kalman_filter_data[bme280_offset].q_factor = BME280_Q_FACTOR;
}

static void reset_altitude_filter_data()
{
	current_altitude.measurement_error = ALTITUDE_MEASUREMENT_ERROR;
	current_altitude.estimate_error = ALTITUDE_INITIAL_ESTIMATE_ERROR;
	current_altitude.last_estimate = 0; //We assume we haven't moved from the point the base pressure is set
	current_altitude.estimate = 0; //We assume we haven't moved from the point the base pressure is set
	current_altitude.kalman_gain = ALTITUDE_INITIAL_KALMAN_GAIN;
	current_altitude.q_factor = ALTITUDE_Q_FACTOR;
}

// update_estimate is based on information available at kalmanfilter.net

static void update_estimate(double measure, Kalman_Filter_Data *filter_data)
{	
	filter_data->kalman_gain = filter_data->estimate_error/(filter_data->estimate_error + filter_data->measurement_error);
	filter_data->estimate = filter_data->last_estimate + (filter_data->kalman_gain * (measure - filter_data->last_estimate));
	filter_data->estimate_error = (1.0 - filter_data->kalman_gain)*filter_data->estimate_error +
				fabs(filter_data->last_estimate - filter_data->estimate) * filter_data->q_factor;
	filter_data->last_estimate = filter_data->estimate;
	return;
}

//Update the Kalman filter for each enabled BME 280.

static Error_Returns get_filtered_readings()
{
	Error_Returns to_return = RPi_Success;
	do
	{
		for(uint32_t bme280_id = 0; bme280_id < BME280_NUMBER_SUPPORTED_DEVICES; bme280_id++)
		{
			double raw_pressure;
			to_return = bme280_get_current_pressure(bme280_id, &raw_pressure);
			if (to_return != RPi_Success)
			{
				log_string_plus("altitude_package: get_filtered_readings failed: ", to_return);
				break;
			}
			update_estimate(raw_pressure, &kalman_filter_data[bme280_id]);
		}

	} while(0);
	return to_return;
}

static double convert_pressure_to_altitude(double base_pressure, double current_pressure)
{
	//This equation is the barometric formula from the Bosch BMP180 datasheet.
	//This function returns the difference between the base pressure and the current pressure
	//in meters.
	return MAGIC_MULTIPLIER * (1-pow((current_pressure/base_pressure), MAGIC_EXPONENT));

}

/* This function assumes sole access to the BME280(s) */
static Error_Returns reset_base_pressure()
{
	Error_Returns to_return = RPi_Success;
	do
	{
		for (uint32_t offset = 0; offset < BME280_NUMBER_SUPPORTED_DEVICES; offset++)
			{
			reset_kalman_filter_pressure_data(offset);
			}

		//Find a stable value for the at rest pressure
		for (unsigned int count = 0; count < BME280_CONVERGENCE_LOOP_COUNT; count++)
		{
			spin_wait_milliseconds(ALT_PACKAGE_TICK_TIME);
			to_return = get_filtered_readings();
			if (to_return != RPi_Success)
			{
				log_string_plus("altitude_package: reset_base_pressure() failed to get filtered reading: ", to_return);
				break;
			}
		}

		for (uint32_t offset = 0; offset < BME280_NUMBER_SUPPORTED_DEVICES; offset++)
		{
			base_pressure[offset] = kalman_filter_data[offset].estimate;
			//reset_kalman_filter_pressure_data(offset);
		}
	} while(0);
	return to_return;
}

//Interrupt handling routine to get readings every ALT_PACKAGE_TICK_TIME milliseconds.
void altitude_tick_handler()
{
	if (altitude_state == RPi_Success)
	{
		altitude_state = get_filtered_readings();
	}
	else
	{
		log_interrupt_string_plus("altitude_tick_handler: state = ", (unsigned int)altitude_state);
	}
}

//Set up both the BME 280(s) and the MPU 6050 and initialize the tick timer to interrupt
//every ALT_PACKAGE_TICK_TIME milliseconds.
Error_Returns altitude_initialize()
{
	Error_Returns to_return = RPi_Success;

	do
	{
		//Initialize each BME 280
		for(uint32_t bme280_id = 0; bme280_id < BME280_NUMBER_SUPPORTED_DEVICES; bme280_id++)
		{
			to_return = bme280_init(bme280_id, bme280_kalman_filter_mode);
			if (to_return != RPi_Success)
			{
				log_string_plus("altitude_package: bme280_init failed: ", to_return);
				break;
			}
		}
		
		if (to_return != RPi_Success)
		{
			break;
		}

		//to_return = mpu6050_init();
		if (to_return != RPi_Success)
		{
			log_string_plus("altitude_package: mpu6050_init failed: ", to_return);
			break;
		}

		to_return = arm_timer_init();
		if (to_return != RPi_Success)
		{
			log_string_plus("altitude_package: arm_timer_init failed: ", to_return);
			break;
		}
		
		to_return = altitude_reset();

		altitude_state = RPi_Success;
	} while(0);
	return to_return;
}

//Resets the base pressure to the current stable pressure readings
Error_Returns altitude_reset()
{
	Error_Returns to_return = RPi_Success;
	log_string("Resetting base pressure");
	do
	{
		//Turn off the tick timer, accessing the I2C bus from
		//two different places (reset_base_pressure and altitude_tick_handler) causes
		//hangs.
		arm_timer_disable();

		to_return = reset_base_pressure();
		if (to_return != RPi_Success)
		{
			log_string_plus("altitude_package: reset_base_pressure failed: ", to_return);
			break;
		}

		reset_altitude_filter_data();

		to_return = arm_timer_enable(altitude_tick_handler, ALT_PACKAGE_TICK_TIME);
		if (to_return != RPi_Success)
		{
			log_string_plus("altitude_package: arm_timer_enable failed: ", to_return);
			break;
		}
	} while(0);

	return to_return;
}

//Returns the current difference between the base altitude that is obtained at start up
//or after a call to altitude_reset.
Error_Returns altitude_get_delta(double *delta_meters_ptr)
{
	Error_Returns to_return = RPi_Success;

	do
	{
		if (altitude_state != RPi_Success)
		{
			to_return = altitude_state;
			log_string_plus("altitude_get_delta: Bad altitude state ", altitude_state);
			break;
		}

		for (uint32_t bme280_id = 0; bme280_id < BME280_NUMBER_SUPPORTED_DEVICES; bme280_id++)
		{
			double altitude;

			altitude = convert_pressure_to_altitude(base_pressure[bme280_id],
					kalman_filter_data[bme280_id].estimate);
			update_estimate(altitude, &current_altitude);
			reset_kalman_filter_pressure_data(bme280_id);
		}

		*delta_meters_ptr= current_altitude.estimate;
	} while(0);
	
	return to_return;
}
