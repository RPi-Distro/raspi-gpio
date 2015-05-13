/*
 * Reads GPIO state and dumps to console
 * 
 * use gcc -o gpiostate gpiostate.c
 * 
 * */

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>

char *gpio_alt_names[54*6] =
{
"SDA0"      , "SA5"        , "PCLK"      , "AVEOUT_VCLK"   , "AVEIN_VCLK" , "-"         ,
"SCL0"      , "SA4"        , "DE"        , "AVEOUT_DSYNC"  , "AVEIN_DSYNC", "-"         ,
"SDA1"      , "SA3"        , "LCD_VSYNC" , "AVEOUT_VSYNC"  , "AVEIN_VSYNC", "-"         ,
"SCL1"      , "SA2"        , "LCD_HSYNC" , "AVEOUT_HSYNC"  , "AVEIN_HSYNC", "-"         ,
"GPCLK0"    , "SA1"        , "DPI_D0"    , "AVEOUT_VID0"   , "AVEIN_VID0" , "ARM_TDI"   ,
"GPCLK1"    , "SA0"        , "DPI_D1"    , "AVEOUT_VID1"   , "AVEIN_VID1" , "ARM_TDO"   ,
"GPCLK2"    , "SOE_N_SE"   , "DPI_D2"    , "AVEOUT_VID2"   , "AVEIN_VID2" , "ARM_RTCK"  ,
"SPI0_CE1_N", "SWE_N_SRW_N", "DPI_D3"    , "AVEOUT_VID3"   , "AVEIN_VID3" , "-"         ,
"SPI0_CE0_N", "SD0"        , "DPI_D4"    , "AVEOUT_VID4"   , "AVEIN_VID4" , "-"         ,
"SPI0_MISO" , "SD1"        , "DPI_D5"    , "AVEOUT_VID5"   , "AVEIN_VID5" , "-"         ,
"SPI0_MOSI" , "SD2"        , "DPI_D6"    , "AVEOUT_VID6"   , "AVEIN_VID6" , "-"         ,
"SPI0_SCLK" , "SD3"        , "DPI_D7"    , "AVEOUT_VID7"   , "AVEIN_VID7" , "-"         ,
"PWM0"      , "SD4"        , "DPI_D8"    , "AVEOUT_VID8"   , "AVEIN_VID8" , "ARM_TMS"   ,
"PWM1"      , "SD5"        , "DPI_D9"    , "AVEOUT_VID9"   , "AVEIN_VID9" , "ARM_TCK"   ,
"TXD0"      , "SD6"        , "DPI_D10"   , "AVEOUT_VID10"  , "AVEIN_VID10", "TXD1"      ,
"RXD0"      , "SD7"        , "DPI_D11"   , "AVEOUT_VID11"  , "AVEIN_VID11", "RXD1"      ,
"FL0"       , "SD8"        , "DPI_D12"   , "CTS0"          , "SPI1_CE2_N" , "CTS1"      ,
"FL1"       , "SD9"        , "DPI_D13"   , "RTS0"          , "SPI1_CE1_N" , "RTS1"      ,
"PCM_CLK"   , "SD10"       , "DPI_D14"   , "I2CSL_SDA_MOSI", "SPI1_CE0_N" , "PWM0"      ,
"PCM_FS"    , "SD11"       , "DPI_D15"   , "I2CSL_SCL_SCLK", "SPI1_MISO"  , "PWM1"      ,
"PCM_DIN"   , "SD12"       , "DPI_D16"   , "I2CSL_MISO"    , "SPI1_MOSI"  , "GPCLK0"    ,
"PCM_DOUT"  , "SD13"       , "DPI_D17"   , "I2CSL_CE_N"    , "SPI1_SCLK"  , "GPCLK1"    ,
"SD0_CLK"   , "SD14"       , "DPI_D18"   , "SD1_CLK"       , "ARM_TRST"   , "-"         ,
"SD0_CMD"   , "SD15"       , "DPI_D19"   , "SD1_CMD"       , "ARM_RTCK"   , "-"         ,
"SD0_DAT0"  , "SD16"       , "DPI_D20"   , "SD1_DAT0"      , "ARM_TDO"    , "-"         ,
"SD0_DAT1"  , "SD17"       , "DPI_D21"   , "SD1_DAT1"      , "ARM_TCK"    , "-"         ,
"SD0_DAT2"  , "TE0"        , "DPI_D22"   , "SD1_DAT2"      , "ARM_TDI"    , "-"         ,
"SD0_DAT3"  , "TE1"        , "DPI_D23"   , "SD1_DAT3"      , "ARM_TMS"    , "-"         ,
"SDA0"      , "SA5"        , "PCM_CLK"   , "FL0"           , "-"          , "-"         ,
"SCL0"      , "SA4"        , "PCM_FS"    , "FL1"           , "-"          , "-"         ,
"TE0"       , "SA3"        , "PCM_DIN"   , "CTS0"          , "-"          , "CTS1"      ,
"FL0"       , "SA2"        , "PCM_DOUT"  , "RTS0"          , "-"          , "RTS1"      ,
"GPCLK0"    , "SA1"        , "RING_OCLK" , "TXD0"          , "-"          , "TXD1"      ,
"FL1"       , "SA0"        , "TE1"       , "RXD0"          , "-"          , "RXD1"      ,
"GPCLK0"    , "SOE_N_SE"   , "TE2"       , "SD1_CLK"       , "-"          , "-"         ,
"SPI0_CE1_N", "SWE_N_SRW_N", "-"         , "SD1_CMD"       , "-"          , "-"         ,
"SPI0_CE0_N", "SD0"        , "TXD0"      , "SD1_DAT0"      , "-"          , "-"         ,
"SPI0_MISO" , "SD1"        , "RXD0"      , "SD1_DAT1"      , "-"          , "-"         ,
"SPI0_MOSI" , "SD2"        , "RTS0"      , "SD1_DAT2"      , "-"          , "-"         ,
"SPI0_SCLK" , "SD3"        , "CTS0"      , "SD1_DAT3"      , "-"          , "-"         ,
"PWM0"      , "SD4"        , "-"         , "SD1_DAT4"      , "SPI2_MISO"  , "TXD1"      ,
"PWM1"      , "SD5"        , "TE0"       , "SD1_DAT5"      , "SPI2_MOSI"  , "RXD1"      ,
"GPCLK1"    , "SD6"        , "TE1"       , "SD1_DAT6"      , "SPI2_SCLK"  , "RTS1"      ,
"GPCLK2"    , "SD7"        , "TE2"       , "SD1_DAT7"      , "SPI2_CE0_N" , "CTS1"      ,
"GPCLK1"    , "SDA0"       , "SDA1"      , "TE0"           , "SPI2_CE1_N" , "-"         ,
"PWM1"      , "SCL0"       , "SCL1"      , "TE1"           , "SPI2_CE2_N" , "-"         ,
"SDA0"      , "SDA1"       , "SPI0_CE0_N", "-"             , "-"          , "SPI2_CE1_N",
"SCL0"      , "SCL1"       , "SPI0_MISO" , "-"             , "-"          , "SPI2_CE0_N",
"SD0_CLK"   , "FL0"        , "SPI0_MOSI" , "SD1_CLK"       , "ARM_TRST"   , "SPI2_SCLK" ,
"SD0_CMD"   , "GPCLK0"     , "SPI0_SCLK" , "SD1_CMD"       , "ARM_RTCK"   , "SPI2_MOSI" ,
"SD0_DAT0"  , "GPCLK1"     , "PCM_CLK"   , "SD1_DAT0"      , "ARM_TDO"    , "-"         ,
"SD0_DAT1"  , "GPCLK2"     , "PCM_FS"    , "SD1_DAT1"      , "ARM_TCK"    , "-"         ,
"SD0_DAT2"  , "PWM0"       , "PCM_DIN"   , "SD1_DAT2"      , "ARM_TDI"    , "-"         ,
"SD0_DAT3"  , "PWM1"       , "PCM_DOUT"  , "SD1_DAT3"      , "ARM_TMS"    , "-"
};

#define BCM2708_PERI_BASE 0x20000000
#define BCM2709_PERI_BASE 0x3F000000

#define GPIO_BASE_OFFSET  0x00200000

#define BLOCK_SIZE  (4*1024)

#define PROC_TYPE_BCM2708 0
#define PROC_TYPE_BCM2709 1

/* Pointer to HW */
static volatile uint32_t *gpio_base ;

int get_cpu_type(void)
{
  FILE *fd;
  char line[256];
  int found = 0;

  if ((fd = fopen("/proc/cpuinfo", "r")) == NULL)
  {
    printf ("Can't open /proc/cpuinfo\n") ;
    return -1;
  }

  while (fgets(line, 120, fd) != NULL)
  {
    if (strncmp (line, "Hardware", 8) == 0)
    {
      found = 1;
      break;
    }
  }

  if (!found) {
    printf("Can't find hardware type!\n");
    return -1;
  }

  if (strstr(line, "BCM2709") != NULL)
    return PROC_TYPE_BCM2709;
  else if (strstr(line, "BCM2708") != NULL)
    return PROC_TYPE_BCM2708;
  else
  {
    printf("Unknown hardware type %s\n", line);
    return -1;
  }
}

int get_gpio_fsel(int gpio)
{
  /* GPIOFSEL0-GPIOFSEL5 with 10 sels per 32 bit reg, 
     3 bits per sel (so bits 0:29 used) */
  uint32_t reg = gpio / 10;
  uint32_t sel = gpio % 10;
  if(gpio < 0 || gpio > 53) return -1;
  /*printf("reg = %d, sel = %d ", reg, sel);*/
  return (int)((*(gpio_base+reg))>>(3*sel))&0x7;
}

int get_gpio_level(int gpio)
{
  if(gpio < 0 || gpio > 53) return -1;
  if(gpio < 32)
  {
    return ((*(gpio_base+13))>>gpio)&0x1;
  } else
  {
    gpio = gpio-32;
    return ((*(gpio_base+14))>>gpio)&0x1;
  }
}

int gpio_fsel_to_namestr(int gpio, int fsel, char *name)
{
  int altfn = 0;
  if(gpio < 0 || gpio > 53) return -1;
  switch (fsel)
  {
    case 0: return sprintf(name, "INPUT");
    case 1: return sprintf(name, "OUTPUT");
    case 2: altfn = 5; break;
    case 3: altfn = 4; break;
    case 4: altfn = 0; break;
    case 5: altfn = 1; break;
    case 6: altfn = 2; break;
    default:  /*case 7*/
      altfn = 3;
      break;
  }
  return sprintf(name, "%s", gpio_alt_names[gpio*6 + altfn]);
}

int main (int argc, char *argv[])
{

  int fd;
  int n;

  int cpu_type;

  int fsel;
  char name[512];
  int level;

  /* 2708 or 2709? */
  cpu_type = get_cpu_type();

  if(cpu_type==0)
    printf("Found a BCM2708\n");
  else if(cpu_type==1)
    printf("Found a BCM2709\n");
  else
    return 1;

  if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
  {
    printf("Unable to open /dev/mem: %s\n", strerror (errno)) ;
    return 1;
  }

  if(cpu_type==0)
    gpio_base = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE_OFFSET+BCM2708_PERI_BASE) ;
  else
    gpio_base = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE_OFFSET+BCM2709_PERI_BASE) ;

  if ((int32_t)gpio_base == -1)
  {
    printf("mmap (GPIO) failed: %s\n", strerror (errno)) ;
    return 1;
  }  

  for(n = 0; n < 54; n++)
  {
    fsel = get_gpio_fsel(n);
    gpio_fsel_to_namestr(n, fsel, name);
    level = get_gpio_level(n);
    printf("GPIO %02d: fsel=%d level=%d, func=%s\n", n, fsel, level, name);
  }

  return 0;

}
