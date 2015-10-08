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

/*
  macro to access dsp memory from PCI bus
*/

/*
  read a word (16 bits) from the pci card.
  addr: an address define in jr3.h
*/
extern unsigned short ReadJr3Dw(unsigned long int addr);

/*
  read a long word (32 bits) from the pci card.
  addr: an address define in jr3.h
*/
extern unsigned long int ReadJr3Dl(unsigned long int addr);


extern int pcidsp_init(void);

/*
  used to download dsp program and undocumented stuff... (reset)
  only used in linux driver.
*/
extern unsigned short ReadRawJr3Dw(unsigned long int addr);

extern void WriteRawJr3l(unsigned long int addr, unsigned long int data);

extern void WriteJr3Pm2(unsigned long int addr,
			unsigned short data,
			unsigned short data2);


extern unsigned long int ReadJr3Pm(unsigned long int);

extern void WriteJr3Dw(unsigned long, unsigned short);
