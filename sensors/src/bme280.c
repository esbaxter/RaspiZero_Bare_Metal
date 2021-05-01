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

File:  bme280.c

Implements support for the Bosch BME 280 temperature, pressure and humidity chip.

Note:  The conversion algorithms were taken directly from the BME 280 spec sheet.

*/

#include <stdio.h>
#include "spi.h"
#include "i2c.h"
#include "log.h"
#include "bme280.h"
#include "arm_timer.h"

#define I2C_SLAVE_ADDRESS 0x76

#define BME280_CHIP_ID 0x60
#define BME280_CHIP_RESET_WORD 0xB6

#define BME280_FIRST_TRIM_PARAMETER 0x88
#define BME280_SECOND_TRIM_PARAMETER 0xA1
#define BME280_THIRD_TRIM_PARAMETER 0xE1
#define BME280_CHIP_RPi_REGISTER 0xD0
#define BME280_CHIP_RESET_REGISTER 0xE0
#define BME280_CTRL_HUMIDITY_REGISTER 0xF2
#define BME280_STATUS_REGISTER 0xF2
#define BME280_CTRL_MEASURE_REGISTER 0xF4
#define BME280_CTRL_CONFIG_REGISTER 0xF5
#define BME280_FIRST_DATA_REGISTER 0xF7

#define BME280_CTRL_REGISTER_WRITE_SIZE 2
#define BME280_DATA_REGISTER_SIZE 0x8
#define BME280_TRIM_PARAMETER_BYTES 24

#define BME280_SLEEP_MODE 0
#define BME280_IIR_OFF_500MS_STANDBY 0x80
#define BME280_HUMIDITY_1X	0x01
#define BME280_PRESS_TEMP_1X  0x27
#define BME280_IIR_16_500MS_STANDBY 0x10
#define BME280_PRESS8X_TEMP_1X	0x33
#define BME280_PRESS16X_TEMP_2X 0x53

#define TIME_DELAY 900000

static unsigned short dig_T1 = 0;
static signed short dig_T2 = 0;
static signed short dig_T3 = 0;

static unsigned short dig_P1 = 0;
static signed short dig_P2 = 0;
static signed short dig_P3 = 0;
static signed short dig_P4 = 0;
static signed short dig_P5 = 0;
static signed short dig_P6 = 0;
static signed short dig_P7 = 0;
static signed short dig_P8 = 0;
static signed short dig_P9 = 0;

static unsigned char dig_H1  = 0;
static signed short dig_H2 = 0;
static unsigned char dig_H3 = 0;
static signed short dig_H4 = 0;
static signed short dig_H5 = 0;
static char dig_H6  = 0;

static BME280_S32_t t_fine;

static unsigned char bme280_ready = 0;

static Error_Returns bme280_write(unsigned char *buffer, unsigned int tx_bytes)
{
	Error_Returns to_return = RPi_Success;
#ifndef SPI_MODE
	to_return = i2c_write(I2C_SLAVE_ADDRESS, buffer, tx_bytes);
#else
	buffer[0] &= 0x7F;  //Most significant bit must be zero to specify write to BME280
	to_return = spi_write(spi_ce_zero, spi_cpol_low, spi_cpha_middle,
	  spi_cpol_low, buffer, tx_bytes);
#endif
	return to_return;
}

static Error_Returns bme280_read(unsigned char *buffer, unsigned int rx_bytes)
{
	Error_Returns to_return = RPi_Success;
#ifndef SPI_MODE
	to_return = i2c_write(I2C_SLAVE_ADDRESS, buffer, 1);
	if (to_return == RPi_Success)
	{
		to_return = i2c_read(I2C_SLAVE_ADDRESS, buffer, rx_bytes);
	}
#else
	to_return = spi_read(spi_ce_zero, spi_cpol_low, spi_cpha_middle,
	  spi_cpol_low, buffer, rx_bytes);
#endif
	return to_return;
}

static int32_t compensateTemperatureInt(int32_t adc_T)
{
    int32_t var1;
    int32_t var2;
    int32_t temperature;
    int32_t temperature_min = -4000;
    int32_t temperature_max = 8500;

    var1 = (int32_t)((adc_T / 8) - ((int32_t)dig_T1 * 2));
    var1 = (var1 * ((int32_t)dig_T2)) / 2048;
    var2 = (int32_t)((adc_T / 16) - ((int32_t)dig_T1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)dig_T3)) / 16384;
    t_fine = var1 + var2;
    temperature = (t_fine * 5 + 128) / 256;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }
    return temperature;
}

static uint32_t compensatePressureInt(int32_t adc_P)
{
   	int64_t var1;
	int64_t var2;
	int64_t pressure;
	
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
	var2 = var2 + (((int64_t)dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
	if (var1 < 0)
	{
		return 0;
	}
	
	pressure = 1048576 - adc_P;
	pressure = (((pressure << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)dig_P9) * (pressure >> 13) * (pressure >> 13)) >> 25;
	var2 = (((int64_t)dig_P8) * pressure) >> 19;
	pressure = ((pressure + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
	return (uint32_t) pressure;
}

static double compensateTemperature(int32_t adc_T)
{
  double v_x1_u32;
  double v_x2_u32;
  double temperature;
  
  v_x1_u32  = (((double)adc_T) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
  v_x2_u32  = ((((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0) * (((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
  t_fine = (BME280_S32_t)(v_x1_u32 + v_x2_u32);
  temperature  = (v_x1_u32 + v_x2_u32) / 5120.0;
  return temperature;
}

static double compensatePressure(int32_t adc_P)
{
  double v_x1_u32;
  double v_x2_u32;
  double pressure;
  
  v_x1_u32 = ((double)t_fine / 2.0) - 64000.0;
  v_x2_u32 = v_x1_u32 * v_x1_u32 * ((double)dig_P6) / 32768.0;
  v_x2_u32 = v_x2_u32 + v_x1_u32 * ((double)dig_P5) * 2.0;
  v_x2_u32 = (v_x2_u32 / 4.0) + (((double)dig_P4) * 65536.0);
  v_x1_u32 = (((double)dig_P3) * v_x1_u32 * v_x1_u32 / 524288.0 + ((double)dig_P2) * v_x1_u32) / 524288.0;
  v_x1_u32 = (1.0 + v_x1_u32 / 32768.0) * ((double)dig_P1);
  pressure = 1048576.0 - (double)adc_P;
  // Avoid exception caused by division by zero.
  if (v_x1_u32 != 0) pressure = (pressure - (v_x2_u32 / 4096.0)) * 6250.0 / v_x1_u32;
  else return 0;
  v_x1_u32 = ((double)dig_P9) * pressure * pressure / 2147483648.0;
  v_x2_u32 = pressure * ((double)dig_P8) / 32768.0;
  pressure = pressure + (v_x1_u32 + v_x2_u32 + ((double)dig_P7)) / 16.0;
  
  return pressure;
}

static double compensateHumidity(int32_t adc_H)
{
  double var_h;
  
  var_h = (((double)t_fine) - 76800.0);
  if (var_h != 0)
  {
    var_h = (adc_H - (((double)dig_H4) * 64.0 + ((double)dig_H5) / 16384.0 * var_h)) * 
      (((double)dig_H2) / 65536.0 * (1.0 + ((double) dig_H6) / 67108864.0 * 
      var_h * (1.0 + ((double)dig_H3) / 67108864.0 * var_h)));
  }
  else return 0;
  var_h = var_h * (1.0 - ((double)dig_H1)*var_h / 524288.0);
  if (var_h > 100.0) var_h = 100.0;
  else if (var_h < 0.0) var_h = 0.0;
  return var_h;
}

static Error_Returns bme280_read_data(BME280_S32_t *adc_T_ptr, BME280_S32_t *adc_P_ptr, BME280_S32_t *adc_H_ptr)
{
	Error_Returns to_return = RPi_NotInitialized;
	unsigned int data_xlsb = 0;
    unsigned int data_lsb = 0;
    unsigned int data_msb = 0;
	
	unsigned char buffer[BME280_DATA_REGISTER_SIZE];
	unsigned int index = 0;
	unsigned int status_attempts = 0;
	
	if (bme280_ready)
	{
		do
		{
			do
			{
				buffer[0] = BME280_STATUS_REGISTER;
				to_return = bme280_read(buffer, 1);
				++status_attempts;
				if (buffer[0] & 0x08)
					spin_wait(TIME_DELAY);
			} while(((status_attempts < 5) && (buffer[0] & 0x08)) && (to_return == RPi_Success));
			
			if ((status_attempts == 5) || (to_return != RPi_Success))
			{
				log_string("Error:  BME280, too many status register reads showed measuring");
				break;
			}
			
			for(index = 0;index < BME280_DATA_REGISTER_SIZE; index++) buffer[index] = 0;
			
			buffer[0] = BME280_FIRST_DATA_REGISTER;
			to_return = bme280_read(buffer, BME280_DATA_REGISTER_SIZE);
			if (to_return != RPi_Success) break;  //No need to continue, just return the error

		   /* Store the parsed register values for pressure data */
			data_msb = (unsigned int)buffer[0] << 12;
			data_lsb = (unsigned int)buffer[1] << 4;
			data_xlsb = (unsigned int)buffer[2] >> 4;
			*adc_P_ptr = data_msb | data_lsb | data_xlsb;

			/* Store the parsed register values for temperature data */
			data_msb = (unsigned int)buffer[3] << 12;
			data_lsb = (unsigned int)buffer[4] << 4;
			data_xlsb = (unsigned int)buffer[5] >> 4;
			*adc_T_ptr = data_msb | data_lsb | data_xlsb;
			
			/* Store the parsed register values for humidity data */
			data_msb = (unsigned int)buffer[6] << 8;
			data_lsb = (unsigned int)buffer[7];
			*adc_H_ptr = data_msb | data_lsb;
		} while(0);
	}
	
	return to_return;
}

Error_Returns bme280_init(BME280_mode mode)
{	
	Error_Returns to_return = RPi_Success;
	unsigned char buffer[BME280_TRIM_PARAMETER_BYTES];
	unsigned int index = 0;
	
	do
	{	
#ifndef SPI_MODE
		i2c_init();
#else
		spi_init();
#endif
		
		for(index = 0; index < BME280_TRIM_PARAMETER_BYTES; index++) buffer[index] = 0;
		
		buffer[0] = BME280_CHIP_RPi_REGISTER;
		to_return = bme280_read(buffer, 1);
		if (to_return != RPi_Success) break;  //No need to continue just return the failure
		if (buffer[0] != BME280_CHIP_ID)
		{
			log_string_plus("bme280_init():  Error chip ID read was ", buffer[0]);
		}
		
		for(index = 0; index < BME280_TRIM_PARAMETER_BYTES; index++) buffer[index] = 0;
		buffer[0] = BME280_FIRST_TRIM_PARAMETER;
		to_return = bme280_read(buffer, BME280_TRIM_PARAMETER_BYTES);
		if (to_return != RPi_Success) break;  //No need to continue just return the failure
		
		index = 0;

		dig_T1 = buffer[index++];
		dig_T1 |= buffer[index++]<<8;
		dig_T2 = buffer[index++];
		dig_T2 |= buffer[index++]<<8;
		dig_T3 = buffer[index++];
		dig_T3 |= buffer[index++]<<8;

		dig_P1 = buffer[index++];
		dig_P1 |= buffer[index++]<<8;
		dig_P2 = buffer[index++];
		dig_P2 |= buffer[index++]<<8;
		dig_P3 = buffer[index++];
		dig_P3 |= buffer[index++]<<8;
		dig_P4 = buffer[index++];
		dig_P4 |= buffer[index++]<<8;
		dig_P5 = buffer[index++];
		dig_P5 |= buffer[index++]<<8;
		dig_P6 = buffer[index++];
		dig_P6 |= buffer[index++]<<8;
		dig_P7 = buffer[index++];
		dig_P7 |= buffer[index++]<<8;
		dig_P8 = buffer[index++];
		dig_P8 |= buffer[index++]<<8;
		dig_P9 = buffer[index++];
		dig_P9 |= buffer[index++]<<8;
		
		for(index = 0; index < BME280_TRIM_PARAMETER_BYTES; index++) buffer[index] = 0;
		buffer[0] = BME280_SECOND_TRIM_PARAMETER;
		to_return = bme280_read(buffer, 1);
		if (to_return != RPi_Success) break;  //No need to continue just return the failure
		
		dig_H1 = buffer[0] & 0xFF;
		
		for(index = 0; index < BME280_TRIM_PARAMETER_BYTES; index++) buffer[index] = 0;
		buffer[0] = BME280_THIRD_TRIM_PARAMETER;
		to_return = bme280_read(buffer, 7);
		if (to_return != RPi_Success) break;  //No need to continue just return the failure
		
		index = 0;
		
		dig_H2 = buffer[index++] & 0xFF;
		dig_H2|= buffer[index++]<<8;
		dig_H3 = buffer[index++];
		
		dig_H4 = buffer[index++] <<4;
		dig_H4 |= buffer[index] & 0x0F;
		dig_H5 = (buffer[index++] >> 4) & 0x0F;
		dig_H5 |= buffer[index++]<<4;
		dig_H6 = buffer[index++] & 0xFF;
		
		switch(mode)
		{
			case bme280_temp_pressure_humidity:
			{
				buffer[0] = BME280_CTRL_MEASURE_REGISTER;
				buffer[1] = BME280_SLEEP_MODE; //Need to set up config in sleep mode
				to_return = bme280_write(buffer, BME280_CTRL_REGISTER_WRITE_SIZE);
				if (to_return != RPi_Success) break; //Don't continue just return
				
				buffer[0] = BME280_CTRL_CONFIG_REGISTER;  
				buffer[1] = BME280_IIR_OFF_500MS_STANDBY; //4 wire SPI, IIR filter off, 500 ms standby time
				to_return = bme280_write(buffer, BME280_CTRL_REGISTER_WRITE_SIZE);
				if (to_return != RPi_Success) break; //Don't continue just return
				
				buffer[0] = BME280_CTRL_HUMIDITY_REGISTER;
				buffer[1] = BME280_HUMIDITY_1X;  //Humidity oversampling x1;
				to_return = bme280_write(buffer, BME280_CTRL_REGISTER_WRITE_SIZE);
				if (to_return != RPi_Success) break; //Don't continue just return
				
				buffer[0] = BME280_CTRL_MEASURE_REGISTER;
				buffer[1] = BME280_PRESS_TEMP_1X; //Pressure and temp oversampling x1, normal mode
				to_return = bme280_write(buffer, BME280_CTRL_REGISTER_WRITE_SIZE);
				if (to_return != RPi_Success) break; //Don't continue just return
				 
				bme280_ready = 1;
				break;
			}
			case bme280_altitude_mode:
			{
				buffer[0] = BME280_CTRL_MEASURE_REGISTER;
				buffer[1] = BME280_SLEEP_MODE; //Need to set up config in sleep mode
				to_return = bme280_write(buffer, BME280_CTRL_REGISTER_WRITE_SIZE);
				if (to_return != RPi_Success) break; //Don't continue just return
				
				buffer[0] = BME280_CTRL_CONFIG_REGISTER;  
				buffer[1] = BME280_IIR_16_500MS_STANDBY; //4 wire SPI, IIR 16, .5 ms standby time
				to_return = bme280_write(buffer, BME280_CTRL_REGISTER_WRITE_SIZE);
				if (to_return != RPi_Success) break; //Don't continue just return
				
				
				buffer[0] = BME280_CTRL_HUMIDITY_REGISTER;
				buffer[1] = BME280_HUMIDITY_1X;  //Humidity oversampling x1;
				to_return = bme280_write(buffer, BME280_CTRL_REGISTER_WRITE_SIZE);
				if (to_return != RPi_Success) break; //Don't continue just return

				buffer[0] = BME280_CTRL_MEASURE_REGISTER;
				//Pressure oversample x16, temp oversample x2, normal mode
				buffer[1] = BME280_PRESS16X_TEMP_2X;
				to_return = bme280_write(buffer, BME280_CTRL_REGISTER_WRITE_SIZE);
				if (to_return != RPi_Success) break; //Don't continue just return			
				
				bme280_ready = 1;
				break;
			}
		}
		spin_wait(TIME_DELAY); 				 
	} while(0);
	
	return to_return;
}

Error_Returns bme280_reset()
{
	Error_Returns to_return = RPi_NotInitialized;
	unsigned char buffer[BME280_CTRL_REGISTER_WRITE_SIZE];
	buffer[0] = BME280_CHIP_RESET_REGISTER;
	buffer[1] = BME280_CHIP_RESET_WORD;
	if (bme280_ready)
	{
		to_return = bme280_write(buffer, BME280_CTRL_REGISTER_WRITE_SIZE);
		spin_wait(TIME_DELAY);  //Delay to allow reset
	}
	return to_return;
}

Error_Returns bme280_print_compensated_values()
{
	Error_Returns to_return = RPi_Success;
	BME280_S32_t adc_P = 0;
	BME280_S32_t adc_T = 0;
	BME280_S32_t adc_H = 0;
		
	do
	{	
		to_return = bme280_read_data(&adc_T, &adc_P, &adc_H);
		if (to_return != RPi_Success) break;  //No need to continue, just return the error

		printf("Temperature %f\n\r", compensateTemperature(adc_T));	
		printf("Pressure %f\n\r", compensatePressure(adc_P));	
		printf("Humidity %f\n\r", compensateHumidity(adc_H));
	} while(0);
	
	return to_return;
}

Error_Returns bme280_get_current_temperature_pressure(double *temperature_ptr, double *pressure_ptr)
{
	Error_Returns to_return = RPi_Success;
	BME280_S32_t adc_P = 0;
	BME280_S32_t adc_T = 0;
	BME280_S32_t adc_H = 0;
	
	do
	{
		to_return = bme280_read_data(&adc_T, &adc_P, &adc_H);
		if (to_return != RPi_Success) break;  //No need to continue, just return the error
		*temperature_ptr = compensateTemperature(adc_T);	
		*pressure_ptr = compensatePressure(adc_P);		
	}  while(0);
	return to_return;
}


Error_Returns bme280_get_current_temperature_pressure_int(int32_t *temperature_ptr, uint32_t *pressure_ptr)
{
	Error_Returns to_return = RPi_Success;
	BME280_S32_t adc_P = 0;
	BME280_S32_t adc_T = 0;
	BME280_S32_t adc_H = 0;
	
	do
	{
		to_return = bme280_read_data(&adc_T, &adc_P, &adc_H);
		if (to_return != RPi_Success) break;  //No need to continue, just return the error
		*temperature_ptr = compensateTemperatureInt(adc_T);	
		*pressure_ptr = compensatePressureInt(adc_P);		
	}  while(0);
	return to_return;
}


Error_Returns bme280_get_current_pressure(double *pressure_ptr)
{
	Error_Returns to_return = RPi_Success;
	BME280_S32_t adc_P = 0;
	BME280_S32_t adc_T = 0;
	BME280_S32_t adc_H = 0;
	
	do
	{
		to_return = bme280_read_data(&adc_T, &adc_P, &adc_H);
		if (to_return != RPi_Success) break;  //No need to continue, just return the error
		compensateTemperature(adc_T);	
		*pressure_ptr = compensatePressure(adc_P);		
	}  while(0);
	return to_return;
}

Error_Returns bme280_get_current_pressure_int(uint32_t *pressure_ptr)
{
	Error_Returns to_return = RPi_Success;
	BME280_S32_t adc_P = 0;
	BME280_S32_t adc_T = 0;
	BME280_S32_t adc_H = 0;
	
	do
	{
		to_return = bme280_read_data(&adc_T, &adc_P, &adc_H);
		if (to_return != RPi_Success) break;  //No need to continue, just return the error
		compensateTemperatureInt(adc_T);	
		*pressure_ptr = compensatePressureInt(adc_P);		
	}  while(0);
	return to_return;
}