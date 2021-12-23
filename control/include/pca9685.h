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

File:  pca9685.h

Interface into the support software for the NXP PWM chip being used as a servo
controller.

*/

#pragma once
#include "common.h"

typedef enum {
	PCA_9685_Internal_Clock,
	PCA_9685_External_Clock
}  PCA9685_Clock_Source;

//Note:  clk_frequency is ignored if PCA_9685_Internal_Clock is
//selected
Error_Returns pca9685_init(uint32_t i2c_id, PCA9685_Clock_Source clk_src,
		uint32_t input_clk_frequency, uint32_t output_frequency, uint32_t *pca9685_idx);

Error_Returns pca9685_register_servo(uint32_t pca9685_idx, uint32_t servo_channel,
		uint32_t *servo_idx);

//Move a given servo by setting up the PWM channel to the amount of time on
Error_Returns pca9685_move_servo(uint32_t pca9685_idx, uint32_t servo_idx,
		uint32_t active_pulse_width);
