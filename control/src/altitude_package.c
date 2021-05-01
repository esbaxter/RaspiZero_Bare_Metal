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

#define MAGIC_CONSTANT				44307.69396
#define MAGIC_EXPONENT				0.190284
#define ATMOSPHERIC_LAPSE_RATE		0.65  // This is in K/cm

//static double base_temperature_factor; //In cm which is equal to the base temperature/atmospheric lapse rate
//static double base_pressure;
static uint32_t base_pressure = 0;

Error_Returns altitude_initialize()
{
	Error_Returns to_return = RPi_Success;
	int32_t current_temperature =0;
	//uint32_t current_pressure = 0;

	do
	{
		to_return = bme280_init(bme280_altitude_mode);
		if (to_return != RPi_Success)
		{
			log_string_plus("altitude_package: bme280_init failed: ", to_return);
			break;
		}

		to_return = bme280_get_current_temperature_pressure_int(&current_temperature, &base_pressure);
		if (to_return != RPi_Success)
		{
			log_string_plus("altitude_package: bme280_get_current_temperature_pressure_int failed: ", to_return);
			break;
		}
		//base_temperature_factor = ((double)(total_temperature/8) / 100.0) + 273.15; //Convert to Kelvin
		//base_temperature_factor /= ATMOSPHERIC_LAPSE_RATE;
		//base_pressure = ((double)(total_pressure/8)) / 256.0;  //Convert from Q24.8 format
		
		
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
	int32_t current_temperature;
	//uint32_t current_pressure;
	Error_Returns to_return = bme280_get_current_temperature_pressure_int(&current_temperature, &base_pressure);
	//base_temperature_factor = ((double)current_temperature / 100.0) + 273.15; //Convert to Kelvin
	//base_temperature_factor /= ATMOSPHERIC_LAPSE_RATE;
	//base_pressure = ((double)current_pressure) / 256.0;  //Convert from Q24.8 format
	return to_return;
}

Error_Returns altitude_get_delta(int32_t *delta_cm_ptr)
{
	Error_Returns to_return = RPi_Success;
	
	do
	{
		uint32_t current_pressure = 0;
		to_return = bme280_get_current_pressure_int(&current_pressure);
		//double pressure = (double)current_pressure /  256.0; //Convert from Q24.8 format
		
		if (to_return != RPi_Success)
		{
			log_string_plus("altitude_package: bme280_get_current_pressure_int failed: ", to_return);
			break;
		}
		
		/* Calculation is based on the simplified formula found here:
		   https://en.wikipedia.org/wiki/Vertical_pressure_variation
		*/
		//pressure = base_temperature_factor * (1 - pow(pressure / base_pressure, MAGIC_EXPONENT));
		//*delta_cm_ptr = (int32_t)pressure;
		*delta_cm_ptr = base_pressure - current_pressure;
	} while(0);
	
	return to_return;
}