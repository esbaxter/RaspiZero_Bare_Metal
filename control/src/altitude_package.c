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

#define MAGIC_EXPONENT				0.1902225603956629
#define CENTIGRADE_TO_KELVIN		273.15 
#define ATMOSPHERIC_LAPSE_RATE		0.000065  // This is in K/cm

static double base_pressure = 0;

Error_Returns altitude_initialize()
{
	Error_Returns to_return = RPi_Success;

	do
	{
		to_return = bme280_init(bme280_altitude_mode);
		if (to_return != RPi_Success)
		{
			log_string_plus("altitude_package: bme280_init failed: ", to_return);
			break;
		}

		to_return = bme280_get_current_pressure(&base_pressure);
		if (to_return != RPi_Success)
		{
			log_string_plus("altitude_package: bme280_get_current_temperature_pressure_int failed: ", to_return);
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
	return bme280_get_current_pressure(&base_pressure);
}

Error_Returns altitude_get_delta(int32_t *delta_cm_ptr)
{
	Error_Returns to_return = RPi_Success;
	
	do
	{
		double current_pressure;
		double current_temp;
		to_return = bme280_get_current_temperature_pressure(&current_temp, &current_pressure);
		
		if (to_return != RPi_Success)
		{
			log_string_plus("altitude_package: bme280_get_current_pressure_int failed: ", to_return);
			break;
		}
		
		/* Calculation is based on the hypsometric formula found here:
		   https://keisan.casio.com/exec/system/1224585971
		*/
		
		*delta_cm_ptr = (int32_t)(((pow(base_pressure / current_pressure, MAGIC_EXPONENT) - 1) * (current_temp + CENTIGRADE_TO_KELVIN)) / ATMOSPHERIC_LAPSE_RATE);
	} while(0);
	
	return to_return;
}