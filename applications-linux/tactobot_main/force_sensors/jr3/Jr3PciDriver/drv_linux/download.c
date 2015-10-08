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
#include "pcidsp.h"
#include "jr3pci3.idm"

/* #define DEBUG_DOWNLOAD */

#ifdef DEBUG_DOWNLOAD
#endif

/*
  jr3_download
*/

int jr3_download(void)
{
  int count;
  int index = 0;

  /* The fist line is a line count */
  count = dsp[index++];

  /* Read in file while the count is no 0xffff */
  while (count != 0xffff)
    {
      int addr;

      /* After the count is the address */
      addr = dsp[index++];
#ifdef DEBUG_DOWNLOAD
      logMsg("addr: %4.4x cnt: %d\n", addr, count);
#endif
		
      /* loop count times and write the data to the dsp memory */
      while (count > 0)
	{
	  /* Check to see if this is data memory or program memory */
	  if (addr & 0x4000)
	    {
	      int data = 0;

	      /* Data memory is 16 bits and is on one line */
	      data = dsp[index++];
	      WriteRawJr3l(addr, data);
	      count--;
	      if (data!=ReadRawJr3Dw(addr))
		{
#ifdef DEBUG_DOWNLOAD
		  logMsg("data addr: %4.4x out: %4.4x in: %4.4x\n",
			 addr, data, ReadRawJr3Dw(addr));
#endif
		}
	    }
	  else
	    {
	      int data, data2;

	      /* Program memory is 24 bits and is on two line */
	      data = dsp[index++];
	      data2 = dsp[index++];
	      WriteJr3Pm2(addr, data, data2); 
	      count -= 2;
	      
	      /* Verify the write */
	      if((((data << 8) | (data2 & 0xff))) != ReadJr3Pm(addr))
		{
#ifdef DEBUG_DOWNLOAD
		  logMsg("pro addr: %4.4x out: %6.6x in: %6.6lx\n",
			 addr, (data << 8)|(data2 & 0xff), ReadJr3Pm(addr));
#endif
		}
	    }
	  addr++;
	} 
      count = dsp[index++];
    }
  return(0);
}


