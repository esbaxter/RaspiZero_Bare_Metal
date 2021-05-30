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

File:  mpu6050.h

Interface into the support software for the InvenSense MPU 6050 motion processing chip.

*/

#pragma once
#include "common.h"

//TODO:  REMOVE!  This is here so the client knows how large this is
#define DMP_PACKET_SIZE 42

typedef struct {
	int32_t quat_w;
	int32_t quat_x;
	int32_t quat_y;
	int32_t quat_z;
	//Currently I don't see any reason to keep the accelerometer and gyro data around
/*
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	*/
} MPU6050_Accel_Gyro_Values;

Error_Returns mpu6050_init();

Error_Returns mpu6050_reset();

Error_Returns mpu6050_retrieve_values(MPU6050_Accel_Gyro_Values *values);
