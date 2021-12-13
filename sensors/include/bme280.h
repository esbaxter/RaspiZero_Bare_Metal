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

File:  bme280.h

Interface into the support software for the BME 280 humidity, temperature
and barometric pressure chip.

*/

#pragma once
#include "common.h"

#define BME280_NUMBER_SUPPORTED_DEVICES 2

typedef int BME280_S32_t;
typedef unsigned int BME280_U32_t;
typedef long long signed int BME280_S64_t;

typedef enum {
	bme280_temp_pressure_humidity,
	bme280_altitude_mode
} BME280_mode;

typedef enum {
	bme280_one = 0x76,
	bme280_two = 0x77
} BME280_id;

int32_t bme280_get_offset_from_id(BME280_id id);

Error_Returns bme280_init(BME280_id id, BME280_mode mode);

Error_Returns bme280_reset(BME280_id id);

Error_Returns bme280_print_compensated_values(BME280_id id);

Error_Returns bme280_get_current_pressure(BME280_id id, double *pressure_ptr);

Error_Returns bme280_get_current_temperature(BME280_id id, double *temperature_ptr);

Error_Returns bme280_get_current_temperature_pressure(BME280_id id, double *temperature_ptr, double *pressure_ptr);
