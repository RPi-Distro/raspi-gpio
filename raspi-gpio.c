/*
  Reads GPIO state and dumps to console.
  Allows GPIO hacking to set and get GPIO state.
  Author: James Adams
*/

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

char *gpio_fsel_alts[8] =
{
  " ", " ", "5", "4", "0", "1", "2", "3"
};

/* 0 = none, 1 = down, 2 = up */
int gpio_default_pullstate[54] = 
{
  2,2,2,2,2,2,2,2,2, /*GPIO0-8 UP*/
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, /*GPIO9-27 DOWN*/
  0,0, /*GPIO28-29 NONE*/
  1,1,1,1, /*GPIO30-33 DOWN*/
  2,2,2, /*GPIO34-36 UP*/
  1,1,1,1,1,1,1, /*GPIO37-43 DOWN*/
  0,0, /*GPIO44-45 NONE*/
  2,2,2,2,2,2,2,2 /*GPIO46-53 UP*/
};

#define BCM2708_PERI_BASE 0x20000000
#define BCM2709_PERI_BASE 0x3F000000

#define GPIO_BASE_OFFSET  0x00200000

#define BLOCK_SIZE  (4*1024)

#define PROC_TYPE_BCM2708 0
#define PROC_TYPE_BCM2709 1

#define GPSET0    7
#define GPSET1    8
#define GPCLR0    10
#define GPCLR1    11
#define GPLEV0    13
#define GPLEV1    14
#define GPPUD     37
#define GPPUDCLK0 38
#define GPPUDCLK1 39


/* Pointer to HW */
static volatile uint32_t *gpio_base ;

void print_gpio_alts_info(int gpio)
{
  int alt;
  printf("%d", gpio);
  if(gpio_default_pullstate[gpio] == 0)
    printf(", NONE");
  else if(gpio_default_pullstate[gpio] == 1)
    printf(", DOWN");
  else
    printf(", UP");
  for(alt=0; alt < 6; alt++)
  {
    printf(", %s", gpio_alt_names[gpio*6+alt]);
  }
  printf("\n");
}

void print_gpio_alts_table(int gpio)
{
  int n;
  printf("GPIO, DEFAULT PULL, ALT0, ALT1, ALT2, ALT3, ALT4, ALT5\n");
  if(gpio < 0) {
    for(n = 0; n < 54; n++)
    {
      print_gpio_alts_info(n);
    }
  } else 
  {
    print_gpio_alts_info(gpio);
  }
}

void delay_us(uint32_t delay)
{
  struct timespec tv_req;
  struct timespec tv_rem;
  int i;
  uint32_t del_ms, del_us;
  del_ms = delay / 1000;
  del_us = delay % 1000;
  for(i=0; i<=del_ms; i++)
  {
    tv_req.tv_sec = 0;
    if(i==del_ms) tv_req.tv_nsec = del_us*1000;
    else          tv_req.tv_nsec = 1000000;
    tv_rem.tv_sec = 0;
    tv_rem.tv_nsec = 0;
    nanosleep(&tv_req, &tv_rem);
    if(tv_rem.tv_sec != 0 || tv_rem.tv_nsec != 0)
      printf("timer oops!\n");
  }
}

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

int set_gpio_fsel(int gpio, int fsel)
{
  static volatile uint32_t *tmp;
  uint32_t reg = gpio / 10;
  uint32_t sel = gpio % 10;
  uint32_t mask;
  if(gpio < 0 || gpio > 53) return -1;
  tmp = gpio_base+reg;
  mask = 0x7<<(3*sel);
  mask = ~mask;
  /*printf("reg = %d, sel = %d, mask=%08X\n", reg, sel, mask);*/
  tmp = gpio_base+reg;
  *tmp = *tmp & mask;
  *tmp = *tmp | ((fsel&0x7)<<(3*sel));
  return (int)((*tmp)>>(3*sel))&0x7;
}

int get_gpio_level(int gpio)
{
  if(gpio < 0 || gpio > 53) return -1;
  if(gpio < 32)
  {
    return ((*(gpio_base+GPLEV0))>>gpio)&0x1;
  } else
  {
    gpio = gpio-32;
    return ((*(gpio_base+GPLEV1))>>gpio)&0x1;
  }
}

int set_gpio_value(int gpio, int value)
{
  if(gpio < 0 || gpio > 53) return -1;
  if(value != 0)
  {
    if(gpio < 32) {
      *(gpio_base+GPSET0) = 0x1<<gpio;
    }
    else {
      gpio -= 32;
      *(gpio_base+GPSET1) = 0x1<<gpio;
    }
  } else
  {
    if(gpio < 32) {
      *(gpio_base+GPCLR0) = 0x1<<gpio;
    }
    else {
      gpio -= 32;
      *(gpio_base+GPCLR1) = 0x1<<gpio;
    }
  }
  return 0;
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

void print_help()
{
  char *name = "raspi-gpio"; /* in case we want to rename */
  printf("\n");
  printf("WARNING! %s set writes directly to the GPIO control registers\n", name);
  printf("ignoring whatever else may be using them (such as Linux drivers) -\n");
  printf("it is designed as a debug tool, only use it if you know what you\n");
  printf("are doing and at your own risk!\n");
  printf("\n");
  printf("The %s tool is designed to help hack / debug BCM283x GPIO.\n", name);
  printf("Running %s with the help argument prints this help.\n", name);
  printf("%s can get and print the state of a GPIO (or all GPIOs)\n", name);
  printf("and can be used to set the function, pulls and value of a GPIO.\n");
  printf("%s must be run as root.\n", name);
  printf("Use:\n");
  printf("  %s get [GPIO]\n", name);
  printf("OR\n");
  printf("  %s set <GPIO> [options]\n", name);
  printf("OR\n");
  printf("  %s funcs [GPIO]\n", name);
  printf("Note that omitting [GPIO] from %s get prints all GPIOs.\n", name);
  printf("%s funcs will dump all the possible GPIO alt funcions in CSV format\n", name);
  printf("or if [GPIO] is specified the alternate funcs just for that specific GPIO.\n");
  printf("Valid [options] for %s set are:\n", name);
  printf("  ip      set GPIO as input\n");
  printf("  op      set GPIO as output\n");
  printf("  a0-a5   set GPIO to alternate function alt0-alt5\n");
  printf("  pu      set GPIO in-pad pull up\n");
  printf("  pd      set GPIO pin-pad pull down\n");
  printf("  pn      set GPIO pull none (no pull)\n");
  printf("  dh      set GPIO to drive to high (1) level (only valid if set to be an output)\n");
  printf("  dl      set GPIO to drive low (0) level (only valid if set to be an output)\n");
  printf("Examples:\n");
  printf("  %s get              Prints state of all GPIOs one per line\n", name);
  printf("  %s get 20           Prints state of GPIO20\n", name);
  printf("  %s set 20 a5        Set GPIO20 to ALT5 function (GPCLK0)\n", name);
  printf("  %s set 20 pu        Enable GPIO20 ~50k in-pad pull up\n", name);
  printf("  %s set 20 pd        Enable GPIO20 ~50k in-pad pull down\n", name);
  printf("  %s set 20 op        Set GPIO20 to be an output\n", name);
  printf("  %s set 20 dl        Set GPIO20 to output low/zero (must already be set as an output)\n", name);
  printf("  %s set 20 ip pd     Set GPIO20 to input with pull down\n", name);
  printf("  %s set 35 a0 pu     Set GPIO35 to ALT0 function (SPI_CE1_N) with pull up\n", name);
  printf("  %s set 20 op pn dh  Set GPIO20 to ouput with no pull and drving high\n", name);
}

/*
 * type:
 *   0 = no pull
 *   1 = pull down
 *   2 = pull up
 */
int gpio_set_pull(int gpio, int type)
{
  if(gpio < 0 || gpio > 53) return -1;
  if(type < 0 || type > 2) return -1;

  if(gpio < 32) {
    *(gpio_base+GPPUD) = type;
    delay_us(10);
    *(gpio_base+GPPUDCLK0) = 0x1<<gpio;
    delay_us(10);
    *(gpio_base+GPPUD) = 0;
    delay_us(10);
    *(gpio_base+GPPUDCLK0) = 0;
    delay_us(10);
  } else {
    gpio -= 32;
    *(gpio_base+GPPUD) = type;
    delay_us(10);
    *(gpio_base+GPPUDCLK1) = 0x1<<gpio;
    delay_us(10);
    *(gpio_base+GPPUD) = 0;
    delay_us(10);
    *(gpio_base+GPPUDCLK1) = 0;
    delay_us(10);
  }
}

int main (int argc, char *argv[])
{

  int fd;
  int n;

  int ret;

  int cpu_type;

  int fsel;
  char name[512];
  int level;

  /* arg parsing */

  int set = 0;
  int get = 0;
  int funcs = 0;
  int pullup = 0;
  int pulldn = 0;
  int pullnone = 0;
  int fsparam = -1;
  int pinnum = -1;
  int drivehigh = 0;
  int drivelow = 0;

  if(argc < 2)
  {
    printf("No arguments given - try \"raspi-gpio help\"\n");
    return 0;
  }

  if(strcmp(argv[1], "help") == 0)
  {
    print_help();
    return 0;
  }

  /* argc 2 or greater, next arg must be set, get or help */
  get = strcmp(argv[1], "get") == 0;
  set = strcmp(argv[1], "set") == 0;
  funcs = strcmp(argv[1], "funcs") == 0;
  if(!set && !get && !funcs)
  {
    printf("Unknown argument \"%s\" try \"raspi-gpio help\"\n", argv[1]);
    return 1;
  }

  if((get || funcs) && (argc > 3))
  {
    printf("Too many arguments\n");
    return 1;
  }

  if(argc < 3 && set)
  {
    printf("Need GPIO number to set\n");
    return 1;
  }

  if(argc > 2) /* expect pin number next */
  {
    ret = sscanf(argv[2], "%d", &pinnum);
    if(ret != 1 || pinnum < 0 || pinnum > 53)
    {
      printf("Unknown GPIO \"%s\"\n", argv[2]);
      return 1;
    }
  }

  if(set && argc < 4)
  {
      printf("Nothing to set\n");
      return 0;
  }

  /* parse remainng args */
  for(n = 3; n < argc; n++) {
    if(strcmp(argv[n], "dh") == 0) {
      drivehigh = 1;
    } else
    if(strcmp(argv[n], "dl") == 0) {
      drivelow = 1;
    } else
    if(strcmp(argv[n], "ip") == 0) {
      fsparam = 0;
    } else
    if(strcmp(argv[n], "op") == 0) {
      fsparam = 1;
    } else
    if(strcmp(argv[n], "a0") == 0) {
      fsparam = 4;
    } else
    if(strcmp(argv[n], "a1") == 0) {
      fsparam = 5;
    } else
    if(strcmp(argv[n], "a2") == 0) {
      fsparam = 6;
    } else
    if(strcmp(argv[n], "a3") == 0) {
      fsparam = 7;
    } else
    if(strcmp(argv[n], "a4") == 0) {
      fsparam = 3;
    } else
    if(strcmp(argv[n], "a5") == 0) {
      fsparam = 2;
    } else
    if(strcmp(argv[n], "pu") == 0) {
      pullup = 1;
    } else
    if(strcmp(argv[n], "pd") == 0) {
      pulldn = 1;
    } else
    if(strcmp(argv[n], "pn") == 0) {
      pullnone = 1;
    } else
    {
      printf("Unknown argument \"%s\"\n", argv[n]);
      return 1;
    }
  }

#if 0
  printf("argc = %d\n", argc);
  printf("set = %d\n", set);
  printf("get = %d\n", get);
  printf("funcs = %d\n", funcs);
  printf("pullup = %d\n", pullup);
  printf("pulldown = %d\n", pulldn);
  printf("pullnone = %d\n", pullnone);
  printf("fsparam = %d\n", fsparam);
  printf("pinnum = %d\n", pinnum);
  printf("drivehigh = %d\n", drivehigh);
  printf("drivelow = %d\n", drivelow);
#endif

  /* end arg parsing */

  if(funcs) {
    print_gpio_alts_table(pinnum);
    return 0;
  }

  /* Check for /dev/gpiomem, else we need root access for /dev/mem */
  if ((fd = open ("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC) ) >= 0)
  {
    gpio_base = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0) ;
  }
  else
  {
    if (geteuid())
    {
      printf("Must be root\n");
      return 0;
    }

    /* 2708 or 2709? */
    cpu_type = get_cpu_type();

    if(cpu_type > 1 || cpu_type < 0)
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
  }

  if ((int32_t)gpio_base == -1)
  {
    printf("mmap (GPIO) failed: %s\n", strerror (errno)) ;
    return 1;
  }


  if(get) {
    if(pinnum < 0) {
      for(n = 0; n < 54; n++)
      {
        if(n==0) printf("BANK0 (GPIO 0 to 27):\n");
        if(n==28) printf("BANK1 (GPIO 28 to 45):\n");
        if(n==46) printf("BANK2 (GPIO 46 to 53):\n");
        fsel = get_gpio_fsel(n);
        gpio_fsel_to_namestr(n, fsel, name);
        level = get_gpio_level(n);
        printf("  GPIO %02d: level=%d fsel=%d alt=%s func=%s\n", n, level, fsel, gpio_fsel_alts[fsel], name);
      }
    } else {
      /* print for single pin */
      fsel = get_gpio_fsel(pinnum);
      gpio_fsel_to_namestr(pinnum, fsel, name);
      level = get_gpio_level(pinnum);
      if(fsel < 2)
        printf("GPIO %d: level=%d fsel=%d func=%s\n", pinnum, level, fsel, name);
      else
        printf("GPIO %d: level=%d fsel=%d alt=%s func=%s\n", pinnum, level, fsel, gpio_fsel_alts[fsel], name);
    }
  }

  if(set) {

    /* set function */
    if(fsparam >= 0) {
      set_gpio_fsel(pinnum, fsparam);
    }

    /* set output value (check pin is output first) */
    if(drivehigh || drivelow) {
      if(get_gpio_fsel(pinnum) == 1) {
        if(drivehigh)
          set_gpio_value(pinnum, 1);
        else
          set_gpio_value(pinnum, 0);
      } else {
        printf("Can't set pin value, not an output\n");
        return 1;
      }
    }

    /* set pulls */
    if(pullnone) {
      gpio_set_pull(pinnum, 0);
    } else
    if(pulldn) {
      gpio_set_pull(pinnum, 1);
    } else
    if(pullup) {
      gpio_set_pull(pinnum, 2);
    }

  }

  return 0;

}
