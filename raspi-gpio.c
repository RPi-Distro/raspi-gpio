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
#include <time.h>

#define DRIVE_UNSET -1
#define DRIVE_LOW    0
#define DRIVE_HIGH   1

#define PULL_UNSET  -1
#define PULL_NONE    0
#define PULL_DOWN    1
#define PULL_UP      2

#define FUNC_UNSET  -1
#define FUNC_IP      0
#define FUNC_OP      1
#define FUNC_A0      4
#define FUNC_A1      5
#define FUNC_A2      6
#define FUNC_A3      7
#define FUNC_A4      3
#define FUNC_A5      2

#define GPIO_MIN     0
#define GPIO_MAX     53

static const char *gpio_alt_names_2708[54*6] =
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

static const char *gpio_alt_names_2711[54*6] =
{
    "SDA0"      , "SA5"        , "PCLK"      , "SPI3_CE0_N"    , "TXD2"            , "SDA6"        ,
    "SCL0"      , "SA4"        , "DE"        , "SPI3_MISO"     , "RXD2"            , "SCL6"        ,
    "SDA1"      , "SA3"        , "LCD_VSYNC" , "SPI3_MOSI"     , "CTS2"            , "SDA3"        ,
    "SCL1"      , "SA2"        , "LCD_HSYNC" , "SPI3_SCLK"     , "RTS2"            , "SCL3"        ,
    "GPCLK0"    , "SA1"        , "DPI_D0"    , "SPI4_CE0_N"    , "TXD3"            , "SDA3"        ,
    "GPCLK1"    , "SA0"        , "DPI_D1"    , "SPI4_MISO"     , "RXD3"            , "SCL3"        ,
    "GPCLK2"    , "SOE_N_SE"   , "DPI_D2"    , "SPI4_MOSI"     , "CTS3"            , "SDA4"        ,
    "SPI0_CE1_N", "SWE_N_SRW_N", "DPI_D3"    , "SPI4_SCLK"     , "RTS3"            , "SCL4"        ,
    "SPI0_CE0_N", "SD0"        , "DPI_D4"    , "I2CSL_CE_N"    , "TXD4"            , "SDA4"        ,
    "SPI0_MISO" , "SD1"        , "DPI_D5"    , "I2CSL_SDI_MISO", "RXD4"            , "SCL4"        ,
    "SPI0_MOSI" , "SD2"        , "DPI_D6"    , "I2CSL_SDA_MOSI", "CTS4"            , "SDA5"        ,
    "SPI0_SCLK" , "SD3"        , "DPI_D7"    , "I2CSL_SCL_SCLK", "RTS4"            , "SCL5"        ,
    "PWM0_0"    , "SD4"        , "DPI_D8"    , "SPI5_CE0_N"    , "TXD5"            , "SDA5"        ,
    "PWM0_1"    , "SD5"        , "DPI_D9"    , "SPI5_MISO"     , "RXD5"            , "SCL5"        ,
    "TXD0"      , "SD6"        , "DPI_D10"   , "SPI5_MOSI"     , "CTS5"            , "TXD1"        ,
    "RXD0"      , "SD7"        , "DPI_D11"   , "SPI5_SCLK"     , "RTS5"            , "RXD1"        ,
    "-"         , "SD8"        , "DPI_D12"   , "CTS0"          , "SPI1_CE2_N"      , "CTS1"        ,
    "-"         , "SD9"        , "DPI_D13"   , "RTS0"          , "SPI1_CE1_N"      , "RTS1"        ,
    "PCM_CLK"   , "SD10"       , "DPI_D14"   , "SPI6_CE0_N"    , "SPI1_CE0_N"      , "PWM0_0"      ,
    "PCM_FS"    , "SD11"       , "DPI_D15"   , "SPI6_MISO"     , "SPI1_MISO"       , "PWM0_1"      ,
    "PCM_DIN"   , "SD12"       , "DPI_D16"   , "SPI6_MOSI"     , "SPI1_MOSI"       , "GPCLK0"      ,
    "PCM_DOUT"  , "SD13"       , "DPI_D17"   , "SPI6_SCLK"     , "SPI1_SCLK"       , "GPCLK1"      ,
    "SD0_CLK"   , "SD14"       , "DPI_D18"   , "SD1_CLK"       , "ARM_TRST"        , "SDA6"        ,
    "SD0_CMD"   , "SD15"       , "DPI_D19"   , "SD1_CMD"       , "ARM_RTCK"        , "SCL6"        ,
    "SD0_DAT0"  , "SD16"       , "DPI_D20"   , "SD1_DAT0"      , "ARM_TDO"         , "SPI3_CE1_N"  ,
    "SD0_DAT1"  , "SD17"       , "DPI_D21"   , "SD1_DAT1"      , "ARM_TCK"         , "SPI4_CE1_N"  ,
    "SD0_DAT2"  , "-"          , "DPI_D22"   , "SD1_DAT2"      , "ARM_TDI"         , "SPI5_CE1_N"  ,
    "SD0_DAT3"  , "-"          , "DPI_D23"   , "SD1_DAT3"      , "ARM_TMS"         , "SPI6_CE1_N"  ,
    "SDA0"      , "SA5"        , "PCM_CLK"   , "-"             , "MII_A_RX_ERR"    , "RGMII_MDIO"  ,
    "SCL0"      , "SA4"        , "PCM_FS"    , "-"             , "MII_A_TX_ERR"    , "RGMII_MDC"   ,
    "-"         , "SA3"        , "PCM_DIN"   , "CTS0"          , "MII_A_CRS"       , "CTS1"        ,
    "-"         , "SA2"        , "PCM_DOUT"  , "RTS0"          , "MII_A_COL"       , "RTS1"        ,
    "GPCLK0"    , "SA1"        , "-"         , "TXD0"          , "SD_CARD_PRES"    , "TXD1"        ,
    "-"         , "SA0"        , "-"         , "RXD0"          , "SD_CARD_WRPROT"  , "RXD1"        ,
    "GPCLK0"    , "SOE_N_SE"   , "-"         , "SD1_CLK"       , "SD_CARD_LED"     , "RGMII_IRQ"   ,
    "SPI0_CE1_N", "SWE_N_SRW_N", "-"         , "SD1_CMD"       , "RGMII_START_STOP", "-"           ,
    "SPI0_CE0_N", "SD0"        , "TXD0"      , "SD1_DAT0"      , "RGMII_RX_OK"     , "MII_A_RX_ERR",
    "SPI0_MISO" , "SD1"        , "RXD0"      , "SD1_DAT1"      , "RGMII_MDIO"      , "MII_A_TX_ERR",
    "SPI0_MOSI" , "SD2"        , "RTS0"      , "SD1_DAT2"      , "RGMII_MDC"       , "MII_A_CRS"   ,
    "SPI0_SCLK" , "SD3"        , "CTS0"      , "SD1_DAT3"      , "RGMII_IRQ"       , "MII_A_COL"   ,
    "PWM1_0"    , "SD4"        , "-"         , "SD1_DAT4"      , "SPI0_MISO"       , "TXD1"        ,
    "PWM1_1"    , "SD5"        , "-"         , "SD1_DAT5"      , "SPI0_MOSI"       , "RXD1"        ,
    "GPCLK1"    , "SD6"        , "-"         , "SD1_DAT6"      , "SPI0_SCLK"       , "RTS1"        ,
    "GPCLK2"    , "SD7"        , "-"         , "SD1_DAT7"      , "SPI0_CE0_N"      , "CTS1"        ,
    "GPCLK1"    , "SDA0"       , "SDA1"      , "-"             , "SPI0_CE1_N"      , "SD_CARD_VOLT",
    "PWM0_1"    , "SCL0"       , "SCL1"      , "-"             , "SPI0_CE2_N"      , "SD_CARD_PWR0",
    "SDA0"      , "SDA1"       , "SPI0_CE0_N", "-"             , "-"               , "SPI2_CE1_N"  ,
    "SCL0"      , "SCL1"       , "SPI0_MISO" , "-"             , "-"               , "SPI2_CE0_N"  ,
    "SD0_CLK"   , "-"          , "SPI0_MOSI" , "SD1_CLK"       , "ARM_TRST"        , "SPI2_SCLK"   ,
    "SD0_CMD"   , "GPCLK0"     , "SPI0_SCLK" , "SD1_CMD"       , "ARM_RTCK"        , "SPI2_MOSI"   ,
    "SD0_DAT0"  , "GPCLK1"     , "PCM_CLK"   , "SD1_DAT0"      , "ARM_TDO"         , "SPI2_MISO"   ,
    "SD0_DAT1"  , "GPCLK2"     , "PCM_FS"    , "SD1_DAT1"      , "ARM_TCK"         , "SD_CARD_LED" ,
    "SD0_DAT2"  , "PWM0_0"     , "PCM_DIN"   , "SD1_DAT2"      , "ARM_TDI"         , "-"           ,
    "SD0_DAT3"  , "PWM0_1"     , "PCM_DOUT"  , "SD1_DAT3"      , "ARM_TMS"         , "-"           ,
};

static const char *gpio_fsel_alts[8] =
{
    " ", " ", "5", "4", "0", "1", "2", "3"
};

static const char *gpio_pull_names[4] =
{
    "NONE", "DOWN", "UP", "?"
};

/* 0 = none, 1 = down, 2 = up */
static const int gpio_default_pullstate[54] =
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

#define GPIO_BASE_OFFSET  0x00200000

#define BLOCK_SIZE  (4*1024)

#define GPSET0    7
#define GPSET1    8
#define GPCLR0    10
#define GPCLR1    11
#define GPLEV0    13
#define GPLEV1    14
#define GPPUD     37
#define GPPUDCLK0 38
#define GPPUDCLK1 39

/* 2711 has a different mechanism for pin pull-up/down/enable  */
#define GPPUPPDN0                57        /* Pin pull-up/down for pins 15:0  */
#define GPPUPPDN1                58        /* Pin pull-up/down for pins 31:16 */
#define GPPUPPDN2                59        /* Pin pull-up/down for pins 47:32 */
#define GPPUPPDN3                60        /* Pin pull-up/down for pins 57:48 */


/* Pointer to HW */
static volatile uint32_t *gpio_base;
static int is_2711;
static const char **gpio_alt_names;

void print_gpio_alts_info(int gpio)
{
    int alt;
    printf("%d", gpio);
    if (gpio_default_pullstate[gpio] == 0)
        printf(", NONE");
    else if (gpio_default_pullstate[gpio] == 1)
        printf(", DOWN");
    else
        printf(", UP");
    for (alt=0; alt < 6; alt++)
    {
        printf(", %s", gpio_alt_names[gpio*6+alt]);
    }
    printf("\n");
}

void delay_us(uint32_t delay)
{
    struct timespec tv_req;
    struct timespec tv_rem;
    int i;
    uint32_t del_ms, del_us;
    del_ms = delay / 1000;
    del_us = delay % 1000;
    for (i=0; i<=del_ms; i++)
    {
        tv_req.tv_sec = 0;
        if (i==del_ms) tv_req.tv_nsec = del_us*1000;
        else          tv_req.tv_nsec = 1000000;
        tv_rem.tv_sec = 0;
        tv_rem.tv_nsec = 0;
        nanosleep(&tv_req, &tv_rem);
        if (tv_rem.tv_sec != 0 || tv_rem.tv_nsec != 0)
            printf("timer oops!\n");
    }
}

uint32_t get_hwbase(void)
{
    const char *ranges_file = "/proc/device-tree/soc/ranges";
    uint8_t ranges[12];
    FILE *fd;
    uint32_t ret = 0;

    memset(ranges, 0, sizeof(ranges));

    if ((fd = fopen(ranges_file, "rb")) == NULL)
    {
        printf("Can't open '%s'\n", ranges_file);
    }
    else if (fread(ranges, 1, sizeof(ranges), fd) >= 8)
    {
        ret = (ranges[4] << 24) |
              (ranges[5] << 16) |
              (ranges[6] << 8) |
              (ranges[7] << 0);
        if (!ret)
            ret = (ranges[8] << 24) |
                  (ranges[9] << 16) |
                  (ranges[10] << 8) |
                  (ranges[11] << 0);
        if ((ranges[0] != 0x7e) ||
                (ranges[1] != 0x00) ||
                (ranges[2] != 0x00) ||
                (ranges[3] != 0x00) ||
                ((ret != 0x20000000) && (ret != 0x3f000000) && (ret != 0xfe000000)))
        {
            printf("Unexpected ranges data (%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x)\n",
                   ranges[0], ranges[1], ranges[2], ranges[3],
                   ranges[4], ranges[5], ranges[6], ranges[7],
                   ranges[8], ranges[9], ranges[10], ranges[11]);
            ret = 0;
        }
    }
    else
    {
        printf("Ranges data too short\n");
    }

    fclose(fd);

    return ret;
}

int get_gpio_fsel(int gpio)
{
    /* GPIOFSEL0-GPIOFSEL5 with 10 sels per 32 bit reg,
       3 bits per sel (so bits 0:29 used) */
    uint32_t reg = gpio / 10;
    uint32_t sel = gpio % 10;
    if (gpio < GPIO_MIN || gpio > GPIO_MAX) return -1;
    /*printf("reg = %d, sel = %d ", reg, sel);*/
    return (int)((*(gpio_base+reg))>>(3*sel))&0x7;
}

int set_gpio_fsel(int gpio, int fsel)
{
    static volatile uint32_t *tmp;
    uint32_t reg = gpio / 10;
    uint32_t sel = gpio % 10;
    uint32_t mask;
    if (gpio < GPIO_MIN || gpio > GPIO_MAX) return -1;
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
    if (gpio < GPIO_MIN || gpio > GPIO_MAX) return -1;
    if (gpio < 32)
    {
        return ((*(gpio_base+GPLEV0))>>gpio)&0x1;
    }
    else
    {
        gpio = gpio-32;
        return ((*(gpio_base+GPLEV1))>>gpio)&0x1;
    }
}

int set_gpio_value(int gpio, int value)
{
    if (gpio < GPIO_MIN || gpio > GPIO_MAX) return -1;
    if (value != 0)
    {
        if (gpio < 32)
        {
            *(gpio_base+GPSET0) = 0x1<<gpio;
        }
        else
        {
            gpio -= 32;
            *(gpio_base+GPSET1) = 0x1<<gpio;
        }
    }
    else
    {
        if (gpio < 32)
        {
            *(gpio_base+GPCLR0) = 0x1<<gpio;
        }
        else
        {
            gpio -= 32;
            *(gpio_base+GPCLR1) = 0x1<<gpio;
        }
    }
    return 0;
}

int gpio_fsel_to_namestr(int gpio, int fsel, char *name)
{
    int altfn = 0;
    if (gpio < GPIO_MIN || gpio > GPIO_MAX) return -1;
    switch (fsel)
    {
    case 0:
        return sprintf(name, "INPUT");
    case 1:
        return sprintf(name, "OUTPUT");
    case 2:
        altfn = 5;
        break;
    case 3:
        altfn = 4;
        break;
    case 4:
        altfn = 0;
        break;
    case 5:
        altfn = 1;
        break;
    case 6:
        altfn = 2;
        break;
    default:  /*case 7*/
        altfn = 3;
        break;
    }
    return sprintf(name, "%s", gpio_alt_names[gpio*6 + altfn]);
}

void print_raw_gpio_regs(void)
{
    int i;
    int end = is_2711 ? GPPUPPDN3 : GPPUDCLK1;

    for (i = 0; i <= end; i++)
    {
        /* Skip over non-GPIO registers on Pi4 models */
        if (i == (GPPUDCLK1 + 1))
        {
            i = GPPUPPDN0;
            printf("%02x:%*s", i * 4, (i & 3) * 9, "");
        }

        uint32_t val = *(gpio_base + i);
        if ((i & 3) == 0)
            printf("%02x:", i * 4);
        printf(" %08x", val);
        if ((i & 3) == 3)
            printf("\n");
    }
    if (i & 3)
        printf("\n");
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
    printf("OR\n");
    printf("  %s raw\n", name);
    printf("\n");
    printf("GPIO is a comma-separated list of pin numbers or ranges (without spaces),\n");
    printf("e.g. 4 or 18-21 or 7,9-11\n");
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
    printf("  %s get 20,21        Prints state of GPIO20 and GPIO21\n", name);
    printf("  %s set 20 a5        Set GPIO20 to ALT5 function (GPCLK0)\n", name);
    printf("  %s set 20 pu        Enable GPIO20 ~50k in-pad pull up\n", name);
    printf("  %s set 20 pd        Enable GPIO20 ~50k in-pad pull down\n", name);
    printf("  %s set 20 op        Set GPIO20 to be an output\n", name);
    printf("  %s set 20 dl        Set GPIO20 to output low/zero (must already be set as an output)\n", name);
    printf("  %s set 20 ip pd     Set GPIO20 to input with pull down\n", name);
    printf("  %s set 35 a0 pu     Set GPIO35 to ALT0 function (SPI_CE1_N) with pull up\n", name);
    printf("  %s set 20 op pn dh  Set GPIO20 to ouput with no pull and driving high\n", name);
}

/*
 * type:
 *   0 = no pull
 *   1 = pull down
 *   2 = pull up
 */
int gpio_set_pull(int gpio, int type)
{
    if (gpio < GPIO_MIN || gpio > GPIO_MAX) return -1;
    if (type < 0 || type > 2) return -1;

    if (is_2711)
    {
        int pullreg = GPPUPPDN0 + (gpio>>4);
        int pullshift = (gpio & 0xf) << 1;
        unsigned int pullbits;
        unsigned int pull;

        switch (type)
        {
        case PULL_NONE:
            pull = 0;
            break;
        case PULL_UP:
            pull = 1;
            break;
        case PULL_DOWN:
            pull = 2;
            break;
        default:
            return 1; /* An illegal value */
        }

        pullbits = *(gpio_base + pullreg);
        pullbits &= ~(3 << pullshift);
        pullbits |= (pull << pullshift);
        *(gpio_base + pullreg) = pullbits;
    }
    else
    {
        int clkreg = GPPUDCLK0 + (gpio>>5);
        int clkbit = 1 << (gpio & 0x1f);

        *(gpio_base + GPPUD) = type;
        delay_us(10);
        *(gpio_base + clkreg) = clkbit;
        delay_us(10);
        *(gpio_base + GPPUD) = 0;
        delay_us(10);
        *(gpio_base + clkreg) = 0;
        delay_us(10);
    }

    return 0;
}

int get_gpio_pull(int pinnum)
{
    int pull = PULL_UNSET;
    if (is_2711)
    {
        int pull_bits = (*(gpio_base + GPPUPPDN0 + (pinnum >> 4)) >> ((pinnum & 0xf)<<1)) & 0x3;
        switch (pull_bits)
        {
        case 0:
            pull = PULL_NONE;
            break;
        case 1:
            pull = PULL_UP;
            break;
        case 2:
            pull = PULL_DOWN;
            break;
        default:
            pull = PULL_UNSET;
            break; /* An illegal value */
        }
    }
    return pull;
}

int gpio_get(int pinnum)
{
    char name[512];
    char pullstr[12];
    int level;
    int fsel;
    int pull;
    int n;

    fsel = get_gpio_fsel(pinnum);
    gpio_fsel_to_namestr(pinnum, fsel, name);
    level = get_gpio_level(pinnum);
    pullstr[0] = '\0';
    pull = get_gpio_pull(pinnum);
    if (pull != PULL_UNSET)
        sprintf(pullstr, " pull=%s", gpio_pull_names[pull & 3]);
    if (fsel < 2)
        printf("GPIO %d: level=%d fsel=%d func=%s%s\n",
               pinnum, level, fsel, name, pullstr);
    else
        printf("GPIO %d: level=%d fsel=%d alt=%s func=%s%s\n",
               pinnum, level, fsel, gpio_fsel_alts[fsel], name, pullstr);
    return 0;
}

int gpio_set(int pinnum, int fsparam, int drive, int pull)
{
    /* set function */
    if (fsparam != FUNC_UNSET)
        set_gpio_fsel(pinnum, fsparam);

    /* set output value (check pin is output first) */
    if (drive != DRIVE_UNSET)
    {
        if (get_gpio_fsel(pinnum) == 1)
        {
            set_gpio_value(pinnum, drive);
        }
        else
        {
            printf("Can't set pin value, not an output\n");
            return 1;
        }
    }

    /* set pulls */
    if (pull != PULL_UNSET)
        return gpio_set_pull(pinnum, pull);

    return 0;
}

int main(int argc, char *argv[])
{
    uint32_t hwbase;
    int fd;
    int ret;
    int n;

    /* arg parsing */

    int set = 0;
    int get = 0;
    int funcs = 0;
    int raw = 0;
    int pull = PULL_UNSET;
    int fsparam = FUNC_UNSET;
    int drive = DRIVE_UNSET;
    uint32_t gpiomask[2] = { 0, 0 }; /* Enough for 0-53 */
    int all_pins = 0;

    if (argc < 2)
    {
        printf("No arguments given - try \"raspi-gpio help\"\n");
        return 0;
    }

    if (strcmp(argv[1], "help") == 0)
    {
        print_help();
        return 0;
    }

    /* argc 2 or greater, next arg must be set, get or help */
    get = strcmp(argv[1], "get") == 0;
    set = strcmp(argv[1], "set") == 0;
    funcs = strcmp(argv[1], "funcs") == 0;
    raw = strcmp(argv[1], "raw") == 0;
    if (!set && !get && !funcs && !raw)
    {
        printf("Unknown argument \"%s\" try \"raspi-gpio help\"\n", argv[1]);
        return 1;
    }

    if ((get || funcs) && (argc > 3))
    {
        printf("Too many arguments\n");
        return 1;
    }

    if (argc < 3 && set)
    {
        printf("Need GPIO number to set\n");
        return 1;
    }

    if (argc > 2) /* expect pin number(s) next */
    {
        char *p = argv[2];
        while (p)
        {
            int pin, pin2, len;
            ret = sscanf(p, "%d%n", &pin, &len);
            if (ret != 1 || pin < GPIO_MIN || pin > GPIO_MAX)
                break;
            p += len;

            if (*p == '-')
            {
                p++;
                ret = sscanf(p, "%d%n", &pin2, &len);
                if (ret != 1 || pin2 < GPIO_MIN || pin2 > GPIO_MAX)
                    break;
                if (pin2 < pin)
                {
                    int tmp = pin2;
                    pin2 = pin;
                    pin = tmp;
                }
                p += len;
            }
            else
            {
                pin2 = pin;
            }
            while (pin <= pin2)
            {
                gpiomask[pin/32] |= (1 << (pin % 32));
                pin++;
            }
            if (*p == '\0')
            {
                p = NULL;
            }
            else
            {
                if (*p != ',')
                    break;
                p++;
            }
        }
        if (p)
        {
            printf("Unknown GPIO \"%s\"\n", p);
            return 1;
        }
    }

    if (set && argc < 4)
    {
        printf("Nothing to set\n");
        return 0;
    }

    /* parse remaining args */
    for (n = 3; n < argc; n++)
    {
        if (strcmp(argv[n], "dh") == 0)
            drive = DRIVE_HIGH;
        else if (strcmp(argv[n], "dl") == 0)
            drive = DRIVE_LOW;
        else if (strcmp(argv[n], "ip") == 0)
            fsparam = FUNC_IP;
        else if (strcmp(argv[n], "op") == 0)
            fsparam = FUNC_OP;
        else if (strcmp(argv[n], "a0") == 0)
            fsparam = FUNC_A0;
        else if (strcmp(argv[n], "a1") == 0)
            fsparam = FUNC_A1;
        else if (strcmp(argv[n], "a2") == 0)
            fsparam = FUNC_A2;
        else if (strcmp(argv[n], "a3") == 0)
            fsparam = FUNC_A3;
        else if (strcmp(argv[n], "a4") == 0)
            fsparam = FUNC_A4;
        else if (strcmp(argv[n], "a5") == 0)
            fsparam = FUNC_A5;
        else if (strcmp(argv[n], "pu") == 0)
            pull = PULL_UP;
        else if (strcmp(argv[n], "pd") == 0)
            pull = PULL_DOWN;
        else if (strcmp(argv[n], "pn") == 0)
            pull = PULL_NONE;
        else
        {
            printf("Unknown argument \"%s\"\n", argv[n]);
            return 1;
        }
    }

    /* end arg parsing */

    all_pins = !(gpiomask[0] | gpiomask[1]);

    hwbase = get_hwbase();

    if (funcs)
    {
        int pin;

        /* Make an educated guess that doesn't need root privilege */
        is_2711 = (hwbase == 0xfe000000);
        gpio_alt_names = is_2711 ? gpio_alt_names_2711 : gpio_alt_names_2708;
        printf("GPIO, DEFAULT PULL, ALT0, ALT1, ALT2, ALT3, ALT4, ALT5\n");
        for (pin = GPIO_MIN; pin <= GPIO_MAX; pin++)
        {
            if (all_pins || gpiomask[pin / 32] & (1 << (pin % 32)))
                print_gpio_alts_info(pin);
        }
        return 0;
    }

    /* Check for /dev/gpiomem, else we need root access for /dev/mem */
    if ((fd = open ("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC) ) >= 0)
    {
        gpio_base = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    }
    else
    {
        if (geteuid())
        {
            printf("Must be root\n");
            return 0;
        }

        if (!hwbase)
            return 1;

        if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
        {
            printf("Unable to open /dev/mem: %s\n", strerror (errno));
            return 1;
        }

        gpio_base = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE_OFFSET+hwbase);
    }

    if (gpio_base == (uint32_t *)-1)
    {
        printf("mmap (GPIO) failed: %s\n", strerror (errno));
        return 1;
    }

    is_2711 = (*(gpio_base+GPPUPPDN3) != 0x6770696f);
    gpio_alt_names = is_2711 ? gpio_alt_names_2711 : gpio_alt_names_2708;

    if (set || get)
    {
        int pin;
        for (pin = GPIO_MIN; pin <= GPIO_MAX; pin++)
        {
            if (all_pins)
            {
                if (pin==0) printf("BANK0 (GPIO 0 to 27):\n");
                if (pin==28) printf("BANK1 (GPIO 28 to 45):\n");
                if (pin==46) printf("BANK2 (GPIO 46 to 53):\n");
            }
            else if (!(gpiomask[pin / 32] & (1 << (pin % 32))))
            {
                continue;
            }
            if (get)
            {
                if (gpio_get(pin))
                    return 1;
            }
            else
            {
                if (gpio_set(pin, fsparam, drive, pull))
                    return 1;
            }
        }
    }

    if (raw)
        print_raw_gpio_regs();

    return 0;
}
