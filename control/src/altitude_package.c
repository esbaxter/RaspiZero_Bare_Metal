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
#include "log.h"
#include <math.h>

#define MAGIC_CONSTANT				44307.69396
#define MAGIC_EXPONENT				0.190284

static double base_pressure = 0.0;

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
	} while(0);	
	return to_return;
}

Error_Returns altitude_reset()
{
	return bme280_get_current_pressure(&base_pressure);
}

Error_Returns altitude_get_delta(double *delta_meter_ptr)
{
	Error_Returns to_return = RPi_Success;
	double current_pressure = 0.0;
	do
	{
		to_return = bme280_get_current_pressure(&current_pressure);
		if (to_return != RPi_Success)
		{
			log_string_plus("altitude_package: altitude_get_delta failed: ", to_return);
			break;
		}
		
		if (base_pressure <= current_pressure)
		{
			*delta_meter_ptr = 0.0;
		}
		else
		{
			/* Calculation is based on the simplified formula found here:
			   https://en.wikipedia.org/wiki/Vertical_pressure_variation
			*/
			*delta_meter_ptr = (1 - pow(current_pressure / base_pressure, MAGIC_EXPONENT)) * MAGIC_CONSTANT;
		}
	} while(0);
	
	return to_return;
}