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

File:  spi.h

Provides SPI support using the Broadcom 2835.

*/

#pragma once
#include "common.h"

typedef enum {
	spi_ce_zero,
	spi_ce_one
} SPI_CE;

typedef enum {
	spi_cpol_low,
	spi_cpol_high
} SPI_POL;

typedef enum {
	spi_cpha_middle,
	spi_cpha_begin
} SPI_CPHA;

void spi_dump_registers(void);

Error_Returns spi_init(void);

Error_Returns spi_read(SPI_CE chip_enable, SPI_POL clock_polarity, SPI_CPHA clock_phase,
   SPI_POL chip_select_polarity, unsigned char *command_buffer, uint32_t command_bytes);
   
Error_Returns spi_write(SPI_CE chip_enable, SPI_POL clock_polarity, SPI_CPHA clock_phase,
   SPI_POL chip_select_polarity, unsigned char *command_buffer, uint32_t command_bytes);
  