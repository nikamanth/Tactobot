/*	$LAAS$ */
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

#include "jr3_linux.h"

#include "jr3.h"
#include "pcidsp.h"

#include "dspfunc.h"

#define ToJr3PciAddr(addr) (unsigned long*)(((addr) << 2) + jr3_mem.start)
#define ToJr3PciAddrL(addr) (unsigned long*)(((addr) << 2) + jr3_mem.startL)

MODULE_AUTHOR ("LAAS/CNRS");
MODULE_DESCRIPTION ("jr3 - driver for force sensor");
MODULE_LICENSE ("BSD");

extern int jr3_download(void);

static void jr3_wait(void);

struct jr3_mem_str jr3_mem = {NULL, NULL};

static unsigned long int jr3_seek_pos;

loff_t jr3_seek(struct file *jr3_filp, loff_t jr3_off, int whence);
ssize_t jr3_read(struct file *, char *, size_t, loff_t *);
ssize_t jr3_write(struct file *, const char *, size_t, loff_t *);

int jr3_open(struct inode *, struct file *);
int jr3_release(struct inode *, struct file *);

static struct pci_device_id jr3_pci_tbl[] = {
  {
    PCI_VENDOR_ID_JR3, 
    PCI_DEVICE_ID_JR3,
    PCI_ANY_ID,
    PCI_ANY_ID,
    0,
    0,
    0},
  {0,0,0,0,0,0,0}
};

MODULE_DEVICE_TABLE(pci, jr3_pci_tbl);

static int __init jr3_probe(struct pci_dev *pci_dev,
			    const struct pci_device_id *pci_id);
static void jr3_remove(struct pci_dev *pci_dev);

static struct pci_driver jr3_pci_driver = {
  name:JR3_MODULE_NAME,
  id_table:jr3_pci_tbl,
  probe:jr3_probe,
  remove:jr3_remove,
};

struct file_operations jr3_fops = {
  llseek:  jr3_seek,
  read:    jr3_read,
  write:   jr3_write,
  open:    jr3_open,
  release: jr3_release,
};

loff_t jr3_seek(struct file *jr3_filp, loff_t jr3_off, int whence)
{
  /* make some verification */

  switch (whence)
    {
    case 0: /* SEEK_SET */
      jr3_seek_pos = (unsigned long int)jr3_off;
      break;
    case 1: /* SEEK_CUR */
      break;
    case 2: /* SEEK_END */
      break;
    default:
      return -EINVAL;
    }

  /* printk("JR3 seek\n"); */

  return((loff_t)jr3_seek_pos);
}

ssize_t jr3_read(struct file *jr3_filp, char *jr3_buf, 
		 size_t jr3_count, loff_t *jr3_offp)
{
  unsigned long int addr;
  short int data;
  short int nb_short;
  short int *buf_short;

  if(!access_ok(VERIFY_WRITE, jr3_buf, jr3_count))
    {
      return -1;
    }

  buf_short = (short int *)jr3_buf;
  
  /* how many short int ? */
  nb_short = jr3_count / 2;

  for(addr=0; addr < nb_short; addr++)
    {
      data = ReadJr3Dw(jr3_seek_pos + addr);
      put_user(data, buf_short);
      buf_short++;
    }

  return jr3_count;
}

ssize_t jr3_write(struct file *jr3_filp, const char *jr3_buf,
		  size_t jr3_count, loff_t *jr3_offp)
{
  unsigned long int addr;
  short int data;
  short int nb_short;
  short int *buf_short;

  if (!access_ok(VERIFY_READ, jr3_buf, jr3_count)) {
	  return -1;
  }

  buf_short = (short *)jr3_buf;

  /* how many shorts ? */
  nb_short = jr3_count / (sizeof(short));
  
  for (addr = 0; addr < nb_short; addr++) {
	  get_user(data, buf_short++);
	  WriteJr3Dw(jr3_seek_pos + addr, data);
  }
  return jr3_count;
}

int jr3_release(struct inode *jr3_inodp, struct file *jr3_filp)
{
	// MOD_DEC_USE_COUNT;

  /* printk("JR3 release\n"); */

  return(0);
}

int jr3_open(struct inode *jr3_inodp, struct file *jr3_filp)
{
	// MOD_INC_USE_COUNT;

  /* printk("JR3 open\n"); */

  return 0;
}


static int __init jr3_probe(struct pci_dev *pci_dev,
			    const struct pci_device_id *pci_id)
{
  /* in pci address space */
  unsigned long jr3_io_resource0_start;
  unsigned long jr3_io_resource0_end;
  

  if (pci_enable_device(pci_dev))
    return -EIO;

  jr3_io_resource0_start = pci_resource_start(pci_dev, 0);

  jr3_io_resource0_end = pci_resource_end(pci_dev, 0);

  jr3_mem.size = pci_resource_len(pci_dev, 0);

#if 0
  printk("jr3_mem (0x%lx:0x%lx), 0x%lx bytes\n",
	 jr3_io_resource0_start,
	 jr3_io_resource0_end,
	 jr3_mem.size);
#endif
  
  if (pci_request_region(pci_dev, 
	  0, JR3_MODULE_NAME) != 0) {
	  printk("Can't request pci region\n");
  }

  jr3_mem.start = ioremap(jr3_io_resource0_start, jr3_mem.size);
  if (jr3_mem.start == NULL)
    {
      printk("can't remap memory\n");
    }

  /* used to download dsp program */
  jr3_mem.startL = ioremap(jr3_io_resource0_start+0x40000, jr3_mem.size);
  if (jr3_mem.startL == NULL)
    {
      printk("can't remap memory\n");
    }

  return 0;
}

static void jr3_remove(struct pci_dev *pci_dev)
{
  pci_release_region(pci_dev, 0);  

  return;
}

static void jr3_wait(void)
{
  struct timeval tv, tv1;

  do_gettimeofday(&tv1);
  
  do
    {
      do_gettimeofday(&tv);
    } while (tv.tv_sec -  tv1.tv_sec < 3); // wait for 3 Secs.
}


int jr3_module_init(void)
{
  char copyright[JR3_COPYRIGHT_SIZE+1];

  pci_register_driver(&jr3_pci_driver);

  /* reset DSP */
  jr3_reset_dsp();
  jr3_wait();

  /* download dsp program */
  jr3_download();
  jr3_wait();  

  jr3_get_dsp_copyright(copyright);
  printk("JR3 dsp software ver. %d, release date %d/%d, %s\n", 
	 jr3_get_software_ver(),
	 jr3_get_software_day(),
	 jr3_get_software_year(),
	 copyright);

  printk("JR3 force sensor model %d, serial no. %d, EEPROM rev. %d\n",
	 jr3_get_model_no(),
	 jr3_get_serial_no(),
	 jr3_get_eeprom_ver());

  printk("JR3 calibration done %d/%d\n",
	 jr3_get_cal_day(),
	 jr3_get_cal_year());
  

  if(register_chrdev(JR3_MAJOR, JR3_MODULE_NAME, &jr3_fops) != 0)
    {
      printk("can't register jr3 device\n");
      return(-1);
    }

  printk("JR3 force sensor ready.\n");

  return 0;
}

void jr3_module_exit(void)
{

  pci_unregister_driver(&jr3_pci_driver);
  unregister_chrdev(JR3_MAJOR, JR3_MODULE_NAME);
  printk("JR3 module unloaded\n");
  return;
}

void WriteRawJr3l(unsigned long int addr, unsigned long int data)
{

  writel(data,ToJr3PciAddr((addr)));
}

void WriteJr3Dw(unsigned long addr, unsigned short data)
{
  writew(data, ToJr3PciAddr((addr|JR3DMADDRMASK)));
}

unsigned short ReadJr3Dw(unsigned long int addr)
{
  static unsigned short short_read;

  short_read = readw(ToJr3PciAddr((addr|JR3DMADDRMASK)));
  return short_read;
}


unsigned long int ReadJr3Dl(unsigned long int addr)
{
  static unsigned long int long_read;
  
  long_read = readl(ToJr3PciAddr((addr|JR3DMADDRMASK)));
  return long_read;
}


unsigned short ReadRawJr3Dw(unsigned long int addr)
{
  static unsigned short short_read;

  short_read = readw(ToJr3PciAddr(addr));
  return short_read;
}

extern void WriteJr3Pm2(unsigned long int addr,
			unsigned short data,
			unsigned short data2)
{
  writew(data,ToJr3PciAddr((addr)));
  writew(data2,ToJr3PciAddrL((addr)));
}

extern unsigned long int ReadJr3Pm(unsigned long int addr)
{
  static unsigned long int long_read;

  long_read = ((readw(ToJr3PciAddr((addr))) << 8) | 
	       ((readb(ToJr3PciAddrL((addr))))));
  
  return long_read;
}

module_init(jr3_module_init);
module_exit(jr3_module_exit);
