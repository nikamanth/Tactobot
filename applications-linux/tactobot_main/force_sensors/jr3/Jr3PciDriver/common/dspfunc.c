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

#include "jr3.h"
#include "dspfunc.h"
#include "pcidsp.h"

void jr3_reset_dsp(void)
{
  WriteRawJr3l(Jr3ResetAddr, 0);
}

/*
  return a non-null terminated string containning copyright notice
*/
void jr3_get_dsp_copyright(char *c)
{
  union char_short_u
  {
    unsigned short s;
    unsigned char c[2];
  };

  union char_short_u cs;
  unsigned long addr;

  for (addr = (JR3_COPYRIGHT); 
       addr < (JR3_COPYRIGHT+JR3_COPYRIGHT_SIZE); 
       addr++)
    {
      cs.s = ReadJr3Dw(addr);
      *c = cs.c[1];
      c++;
      /* printk("%d, %d %d\n", cs.s, cs.c[0], cs.c[1]); */
    }

  *c = '\0';
}

unsigned short int jr3_get_software_ver(void)
{
  return(ReadJr3Dw(JR3_SOFTWARE_VER_NO));
}

unsigned short int jr3_get_eeprom_ver(void)
{
  return(ReadJr3Dw(JR3_EEPROM_VER_NO));
}

unsigned short int jr3_get_software_day(void)
{
  return(ReadJr3Dw(JR3_SOFTWARE_DAY));
}

unsigned short int jr3_get_software_year(void)
{
  return(ReadJr3Dw(JR3_SOFTWARE_YEAR));
}

unsigned short int jr3_get_serial_no(void)
{
  return(ReadJr3Dw(JR3_SERIAL_NO));
}

unsigned short int jr3_get_model_no(void)
{
  return(ReadJr3Dw(JR3_MODEL_NO));
}

unsigned short int jr3_get_cal_day(void)
{
  return(ReadJr3Dw(JR3_CAL_DAY));
}

unsigned short int jr3_get_cal_year(void)
{
  return(ReadJr3Dw(JR3_CAL_YEAR));
}

unsigned short int jr3_get_count(unsigned long int addr)
{
  /* verification */
  switch (addr)
    {
    case JR3_COUNT1:
      break;
    case JR3_COUNT2:
      break;
    case JR3_COUNT3:
      break;
    case JR3_COUNT4:
      break;
    case JR3_COUNT5:
      break;
    case JR3_COUNT6:
      break;
    default:
      /* how to rturn an error ? */
      break;
    }

  return (ReadJr3Dw(addr));
}

unsigned short int jr3_get_error_count(void)
{
  return (ReadJr3Dw(JR3_ERROR_COUNT));
}

unsigned short int jr3_get_units(void)
{
  return (ReadJr3Dw(JR3_UNITS));
}
