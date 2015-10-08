/*
 * Copyright (c) 2004 CNRS/LAAS
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef JR3_H
#define JR3_H

struct jr3_mem_str {
  void *start;
  void *startL;			/* used to download dsp program */
  unsigned long size;
};

extern struct jr3_mem_str jr3_mem;

struct froce_array_str
{
  int fx;
  int fy;
  int fz;
  int mx;
  int my;
  int mz;
  int v1;
  int v2;
};

#define JR3_MODULE_NAME "JR3_module"
#define PCI_VENDOR_ID_JR3 0x1762
#define PCI_DEVICE_ID_JR3 0x1111


#define JR3_MAJOR 200
#define JR3_MINOR 0

#define Jr3ResetAddr	0x18000

#define JR3DMADDRMASK   0x6000

/*
  address of register in the DSP memory
  see chapter "Data Locations and Definitions", 
  page 7 of "JR3 Software and Installation Manual"
*/

#define JR3_COPYRIGHT 0x40
#define JR3_COPYRIGHT_SIZE 0x18+1 /* +1 for null character */

#define JR3_FULL_SCALE 0x80
#define JR3_OFFSETS 0x88

#define JR3_EEPROM_VER_NO 0xf4
#define JR3_SOFTWARE_VER_NO 0xf5

#define JR3_SOFTWARE_DAY 0xf6
#define JR3_SOFTWARE_YEAR 0xf7

#define JR3_SERIAL_NO 0xf8
#define JR3_MODEL_NO 0xf9

#define JR3_CAL_DAY 0xfa
#define JR3_CAL_YEAR 0xfb

#define JR3_FILTER0 0x90
#define JR3_FILTER1 0x98
#define JR3_FILTER2 0xa0
#define JR3_FILTER3 0xa8
#define JR3_FILTER4 0xb0
#define JR3_FILTER5 0xb8
#define JR3_FILTER6 0xc0

#define JR3_COUNT1 0xe8
#define JR3_COUNT2 0xe9
#define JR3_COUNT3 0xea
#define JR3_COUNT4 0xeb
#define JR3_COUNT5 0xec
#define JR3_COUNT6 0xed

#define JR3_ERROR_COUNT 0xee

#define JR3_UNITS 0xfc

#define JR3_ADC_BITS_NUMBER 0xfd
#define JR3_THICKNESS 0xff

#define JR3_WARNING_BITS 0xf0
#define JR3_ERROR_BITS   0xf1
#endif
