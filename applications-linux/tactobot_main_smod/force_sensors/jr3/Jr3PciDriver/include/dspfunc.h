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

extern void jr3_reset_dsp(void);

extern void jr3_get_dsp_copyright(char *);

extern unsigned short int jr3_get_software_ver(void);

extern unsigned short int jr3_get_eeprom_ver(void);

extern unsigned short int jr3_get_software_day(void);

extern unsigned short int jr3_get_software_year(void);

extern unsigned short int jr3_get_serial_no(void);
extern unsigned short int jr3_get_model_no(void);

extern unsigned short int jr3_get_cal_day(void);
extern unsigned short int jr3_get_cal_year(void);

unsigned short int jr3_get_count(unsigned long int addr);

extern unsigned short int jr3_get_error_count(void);

extern unsigned short int jr3_get_units(void);
