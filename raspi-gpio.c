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

#define GPIO_BASE_OFFSET 0x00200000

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
#define FUNC_ALT(x)  (2 + (x))
#define FUNC_A0      FUNC_ALT(0)
#define FUNC_A1      FUNC_ALT(1)
#define FUNC_A2      FUNC_ALT(2)
#define FUNC_A3      FUNC_ALT(3)
#define FUNC_A4      FUNC_ALT(4)
#define FUNC_A5      FUNC_ALT(5)

/* 2835 register offsets */
#define GPFSEL0      0
#define GPFSEL1      1
#define GPFSEL2      2
#define GPFSEL3      3
#define GPFSEL4      4
#define GPFSEL5      5
#define GPSET0       7
#define GPSET1       8
#define GPCLR0       10
#define GPCLR1       11
#define GPLEV0       13
#define GPLEV1       14
#define GPPUD        37
#define GPPUDCLK0    38
#define GPPUDCLK1    39

/* 2711 has a different mechanism for pin pull-up/down/enable  */
#define GPPUPPDN0    57        /* Pin pull-up/down for pins 15:0  */
#define GPPUPPDN1    58        /* Pin pull-up/down for pins 31:16 */
#define GPPUPPDN2    59        /* Pin pull-up/down for pins 47:32 */
#define GPPUPPDN3    60        /* Pin pull-up/down for pins 57:48 */

struct gpio_chip
{
    const char *name;
    uint32_t reg_base;
    uint32_t reg_size;
    unsigned int gpio_count;
    unsigned int fsel_count;
    const char *info_header;
    const char **alt_names;
    const int *default_pulls;

    int (*get_level)(struct gpio_chip *chip, unsigned int gpio);
    int (*get_fsel)(struct gpio_chip *chip, unsigned int gpio);
    int (*get_pull)(struct gpio_chip *chip, unsigned int gpio);
    int (*set_level)(struct gpio_chip *chip, unsigned int gpio, int level);
    int (*set_fsel)(struct gpio_chip *chip, unsigned int gpio, int fsel);
    int (*set_pull)(struct gpio_chip *chip, unsigned int gpio, int pull);
    int (*next_reg)(int reg);

    volatile uint32_t *base;
};

static int bcm2835_get_level(struct gpio_chip *chip, unsigned int gpio);
static int bcm2835_get_fsel(struct gpio_chip *chip, unsigned int gpio);
static int bcm2835_get_pull(struct gpio_chip *chip, unsigned int gpio);
static int bcm2835_set_level(struct gpio_chip *chip, unsigned int gpio, int level);
static int bcm2835_set_fsel(struct gpio_chip *chip, unsigned int gpio, int fsel);
static int bcm2835_set_pull(struct gpio_chip *chip, unsigned int gpio, int pull);
static int bcm2835_next_reg(int reg);

static int bcm2711_get_pull(struct gpio_chip *chip, unsigned int gpio);
static int bcm2711_set_pull(struct gpio_chip *chip, unsigned int gpio, int pull);
static int bcm2711_next_reg(int reg);

static const char *gpio_alt_names_2835[54*6] =
{
    "SDA0"      , "SA5"        , "PCLK"      , "AVEOUT_VCLK"   , "AVEIN_VCLK" , 0           ,
    "SCL0"      , "SA4"        , "DE"        , "AVEOUT_DSYNC"  , "AVEIN_DSYNC", 0           ,
    "SDA1"      , "SA3"        , "LCD_VSYNC" , "AVEOUT_VSYNC"  , "AVEIN_VSYNC", 0           ,
    "SCL1"      , "SA2"        , "LCD_HSYNC" , "AVEOUT_HSYNC"  , "AVEIN_HSYNC", 0           ,
    "GPCLK0"    , "SA1"        , "DPI_D0"    , "AVEOUT_VID0"   , "AVEIN_VID0" , "ARM_TDI"   ,
    "GPCLK1"    , "SA0"        , "DPI_D1"    , "AVEOUT_VID1"   , "AVEIN_VID1" , "ARM_TDO"   ,
    "GPCLK2"    , "SOE_N_SE"   , "DPI_D2"    , "AVEOUT_VID2"   , "AVEIN_VID2" , "ARM_RTCK"  ,
    "SPI0_CE1_N", "SWE_N_SRW_N", "DPI_D3"    , "AVEOUT_VID3"   , "AVEIN_VID3" , 0           ,
    "SPI0_CE0_N", "SD0"        , "DPI_D4"    , "AVEOUT_VID4"   , "AVEIN_VID4" , 0           ,
    "SPI0_MISO" , "SD1"        , "DPI_D5"    , "AVEOUT_VID5"   , "AVEIN_VID5" , 0           ,
    "SPI0_MOSI" , "SD2"        , "DPI_D6"    , "AVEOUT_VID6"   , "AVEIN_VID6" , 0           ,
    "SPI0_SCLK" , "SD3"        , "DPI_D7"    , "AVEOUT_VID7"   , "AVEIN_VID7" , 0           ,
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
    "SD0_CLK"   , "SD14"       , "DPI_D18"   , "SD1_CLK"       , "ARM_TRST"   , 0           ,
    "SD0_CMD"   , "SD15"       , "DPI_D19"   , "SD1_CMD"       , "ARM_RTCK"   , 0           ,
    "SD0_DAT0"  , "SD16"       , "DPI_D20"   , "SD1_DAT0"      , "ARM_TDO"    , 0           ,
    "SD0_DAT1"  , "SD17"       , "DPI_D21"   , "SD1_DAT1"      , "ARM_TCK"    , 0           ,
    "SD0_DAT2"  , "TE0"        , "DPI_D22"   , "SD1_DAT2"      , "ARM_TDI"    , 0           ,
    "SD0_DAT3"  , "TE1"        , "DPI_D23"   , "SD1_DAT3"      , "ARM_TMS"    , 0           ,
    "SDA0"      , "SA5"        , "PCM_CLK"   , "FL0"           , 0            , 0           ,
    "SCL0"      , "SA4"        , "PCM_FS"    , "FL1"           , 0            , 0           ,
    "TE0"       , "SA3"        , "PCM_DIN"   , "CTS0"          , 0            , "CTS1"      ,
    "FL0"       , "SA2"        , "PCM_DOUT"  , "RTS0"          , 0            , "RTS1"      ,
    "GPCLK0"    , "SA1"        , "RING_OCLK" , "TXD0"          , 0            , "TXD1"      ,
    "FL1"       , "SA0"        , "TE1"       , "RXD0"          , 0            , "RXD1"      ,
    "GPCLK0"    , "SOE_N_SE"   , "TE2"       , "SD1_CLK"       , 0            , 0           ,
    "SPI0_CE1_N", "SWE_N_SRW_N", 0           , "SD1_CMD"       , 0            , 0           ,
    "SPI0_CE0_N", "SD0"        , "TXD0"      , "SD1_DAT0"      , 0            , 0           ,
    "SPI0_MISO" , "SD1"        , "RXD0"      , "SD1_DAT1"      , 0            , 0           ,
    "SPI0_MOSI" , "SD2"        , "RTS0"      , "SD1_DAT2"      , 0            , 0           ,
    "SPI0_SCLK" , "SD3"        , "CTS0"      , "SD1_DAT3"      , 0            , 0           ,
    "PWM0"      , "SD4"        , 0           , "SD1_DAT4"      , "SPI2_MISO"  , "TXD1"      ,
    "PWM1"      , "SD5"        , "TE0"       , "SD1_DAT5"      , "SPI2_MOSI"  , "RXD1"      ,
    "GPCLK1"    , "SD6"        , "TE1"       , "SD1_DAT6"      , "SPI2_SCLK"  , "RTS1"      ,
    "GPCLK2"    , "SD7"        , "TE2"       , "SD1_DAT7"      , "SPI2_CE0_N" , "CTS1"      ,
    "GPCLK1"    , "SDA0"       , "SDA1"      , "TE0"           , "SPI2_CE1_N" , 0           ,
    "PWM1"      , "SCL0"       , "SCL1"      , "TE1"           , "SPI2_CE2_N" , 0           ,
    "SDA0"      , "SDA1"       , "SPI0_CE0_N", 0               , 0            , "SPI2_CE1_N",
    "SCL0"      , "SCL1"       , "SPI0_MISO" , 0               , 0            , "SPI2_CE0_N",
    "SD0_CLK"   , "FL0"        , "SPI0_MOSI" , "SD1_CLK"       , "ARM_TRST"   , "SPI2_SCLK" ,
    "SD0_CMD"   , "GPCLK0"     , "SPI0_SCLK" , "SD1_CMD"       , "ARM_RTCK"   , "SPI2_MOSI" ,
    "SD0_DAT0"  , "GPCLK1"     , "PCM_CLK"   , "SD1_DAT0"      , "ARM_TDO"    , 0           ,
    "SD0_DAT1"  , "GPCLK2"     , "PCM_FS"    , "SD1_DAT1"      , "ARM_TCK"    , 0           ,
    "SD0_DAT2"  , "PWM0"       , "PCM_DIN"   , "SD1_DAT2"      , "ARM_TDI"    , 0           ,
    "SD0_DAT3"  , "PWM1"       , "PCM_DOUT"  , "SD1_DAT3"      , "ARM_TMS"    , 0
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
    0           , "SD8"        , "DPI_D12"   , "CTS0"          , "SPI1_CE2_N"      , "CTS1"        ,
    0           , "SD9"        , "DPI_D13"   , "RTS0"          , "SPI1_CE1_N"      , "RTS1"        ,
    "PCM_CLK"   , "SD10"       , "DPI_D14"   , "SPI6_CE0_N"    , "SPI1_CE0_N"      , "PWM0_0"      ,
    "PCM_FS"    , "SD11"       , "DPI_D15"   , "SPI6_MISO"     , "SPI1_MISO"       , "PWM0_1"      ,
    "PCM_DIN"   , "SD12"       , "DPI_D16"   , "SPI6_MOSI"     , "SPI1_MOSI"       , "GPCLK0"      ,
    "PCM_DOUT"  , "SD13"       , "DPI_D17"   , "SPI6_SCLK"     , "SPI1_SCLK"       , "GPCLK1"      ,
    "SD0_CLK"   , "SD14"       , "DPI_D18"   , "SD1_CLK"       , "ARM_TRST"        , "SDA6"        ,
    "SD0_CMD"   , "SD15"       , "DPI_D19"   , "SD1_CMD"       , "ARM_RTCK"        , "SCL6"        ,
    "SD0_DAT0"  , "SD16"       , "DPI_D20"   , "SD1_DAT0"      , "ARM_TDO"         , "SPI3_CE1_N"  ,
    "SD0_DAT1"  , "SD17"       , "DPI_D21"   , "SD1_DAT1"      , "ARM_TCK"         , "SPI4_CE1_N"  ,
    "SD0_DAT2"  , 0            , "DPI_D22"   , "SD1_DAT2"      , "ARM_TDI"         , "SPI5_CE1_N"  ,
    "SD0_DAT3"  , 0            , "DPI_D23"   , "SD1_DAT3"      , "ARM_TMS"         , "SPI6_CE1_N"  ,
    "SDA0"      , "SA5"        , "PCM_CLK"   , 0               , "MII_A_RX_ERR"    , "RGMII_MDIO"  ,
    "SCL0"      , "SA4"        , "PCM_FS"    , 0               , "MII_A_TX_ERR"    , "RGMII_MDC"   ,
    0           , "SA3"        , "PCM_DIN"   , "CTS0"          , "MII_A_CRS"       , "CTS1"        ,
    0           , "SA2"        , "PCM_DOUT"  , "RTS0"          , "MII_A_COL"       , "RTS1"        ,
    "GPCLK0"    , "SA1"        , 0           , "TXD0"          , "SD_CARD_PRES"    , "TXD1"        ,
    0           , "SA0"        , 0           , "RXD0"          , "SD_CARD_WRPROT"  , "RXD1"        ,
    "GPCLK0"    , "SOE_N_SE"   , 0           , "SD1_CLK"       , "SD_CARD_LED"     , "RGMII_IRQ"   ,
    "SPI0_CE1_N", "SWE_N_SRW_N", 0           , "SD1_CMD"       , "RGMII_START_STOP", 0             ,
    "SPI0_CE0_N", "SD0"        , "TXD0"      , "SD1_DAT0"      , "RGMII_RX_OK"     , "MII_A_RX_ERR",
    "SPI0_MISO" , "SD1"        , "RXD0"      , "SD1_DAT1"      , "RGMII_MDIO"      , "MII_A_TX_ERR",
    "SPI0_MOSI" , "SD2"        , "RTS0"      , "SD1_DAT2"      , "RGMII_MDC"       , "MII_A_CRS"   ,
    "SPI0_SCLK" , "SD3"        , "CTS0"      , "SD1_DAT3"      , "RGMII_IRQ"       , "MII_A_COL"   ,
    "PWM1_0"    , "SD4"        , 0           , "SD1_DAT4"      , "SPI0_MISO"       , "TXD1"        ,
    "PWM1_1"    , "SD5"        , 0           , "SD1_DAT5"      , "SPI0_MOSI"       , "RXD1"        ,
    "GPCLK1"    , "SD6"        , 0           , "SD1_DAT6"      , "SPI0_SCLK"       , "RTS1"        ,
    "GPCLK2"    , "SD7"        , 0           , "SD1_DAT7"      , "SPI0_CE0_N"      , "CTS1"        ,
    "GPCLK1"    , "SDA0"       , "SDA1"      , 0               , "SPI0_CE1_N"      , "SD_CARD_VOLT",
    "PWM0_1"    , "SCL0"       , "SCL1"      , 0               , "SPI0_CE2_N"      , "SD_CARD_PWR0",
    "SDA0"      , "SDA1"       , "SPI0_CE0_N", 0               , 0                 , "SPI2_CE1_N"  ,
    "SCL0"      , "SCL1"       , "SPI0_MISO" , 0               , 0                 , "SPI2_CE0_N"  ,
    "SD0_CLK"   , 0            , "SPI0_MOSI" , "SD1_CLK"       , "ARM_TRST"        , "SPI2_SCLK"   ,
    "SD0_CMD"   , "GPCLK0"     , "SPI0_SCLK" , "SD1_CMD"       , "ARM_RTCK"        , "SPI2_MOSI"   ,
    "SD0_DAT0"  , "GPCLK1"     , "PCM_CLK"   , "SD1_DAT0"      , "ARM_TDO"         , "SPI2_MISO"   ,
    "SD0_DAT1"  , "GPCLK2"     , "PCM_FS"    , "SD1_DAT1"      , "ARM_TCK"         , "SD_CARD_LED" ,
    "SD0_DAT2"  , "PWM0_0"     , "PCM_DIN"   , "SD1_DAT2"      , "ARM_TDI"         , 0             ,
    "SD0_DAT3"  , "PWM0_1"     , "PCM_DOUT"  , "SD1_DAT3"      , "ARM_TMS"         , 0             ,
};

static const char *gpio_pull_names[4] =
{
    "NONE", "DOWN", "UP", "?"
};

/* 0 = none, 1 = down, 2 = up */
static const int gpio_default_pullstate_2835[54] =
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

struct gpio_chip gpio_chip_2835 =
{
    "bcm2835",
    0x00200000,
    0x1000,
    54,
    6,
    "GPIO, DEFAULT PULL, ALT0, ALT1, ALT2, ALT3, ALT4, ALT5",
    gpio_alt_names_2835,
    gpio_default_pullstate_2835,
    bcm2835_get_level,
    bcm2835_get_fsel,
    bcm2835_get_pull,
    bcm2835_set_level,
    bcm2835_set_fsel,
    bcm2835_set_pull,
    bcm2835_next_reg,
};

struct gpio_chip gpio_chip_2711 =
{
    "bcm2711",
    0x00200000,
    0x1000,
    54,
    6,
    "GPIO, DEFAULT PULL, ALT0, ALT1, ALT2, ALT3, ALT4, ALT5",
    gpio_alt_names_2711,
    gpio_default_pullstate_2835,
    bcm2835_get_level,
    bcm2835_get_fsel,
    bcm2711_get_pull,
    bcm2835_set_level,
    bcm2835_set_fsel,
    bcm2711_set_pull,
    bcm2711_next_reg,
};

struct gpio_chip *chip;

void print_gpio_alts_info(struct gpio_chip *chip, int gpio)
{
    int alt;
    printf("%d, %s", gpio, gpio_pull_names[chip->default_pulls[gpio]]);
    for (alt = 0; alt < 6; alt++)
    {
        const char *name = chip->alt_names[gpio * chip->fsel_count + alt];
        printf(", %s", name ? name : "-");
    }
    printf("\n");
}

struct gpio_chip *get_gpio_chip(void)
{
    struct gpio_chip *chip;
    const char *revision_file = "/proc/device-tree/system/linux,revision";
    uint8_t revision[4] = { 0 };
    uint32_t cpu = 0;
    FILE *fd;

    if ((fd = fopen(revision_file, "rb")) == NULL)
    {
        printf("Can't open '%s'\n", revision_file);
    }
    else
    {
        if (fread(revision, 1, sizeof(revision), fd) == 4)
            cpu = (revision[2] >> 4) & 0xf;
        else
            printf("Revision data too short\n");

        fclose(fd);
    }

    switch (cpu)
    {
    case 0: /* BCM2835 */
        chip = &gpio_chip_2835;
        chip->reg_base = 0x20000000 + GPIO_BASE_OFFSET;
        break;
    case 1: /* BCM2836 */
    case 2: /* BCM2837 */
        chip = &gpio_chip_2835;
        chip->reg_base = 0x3f000000 + GPIO_BASE_OFFSET;
        break;
    case 3: /* BCM2711 */
        chip = &gpio_chip_2711;
        chip->reg_base = 0xfe000000 + GPIO_BASE_OFFSET;
        break;
    default:
        printf("Unrecognised revision code\n");
        exit(1);
    }

    return chip;
}

const char *gpio_fsel_to_namestr(unsigned int gpio, int fsel)
{
    const char *name = NULL;
    static char alt_str[16];

    if (gpio >= chip->gpio_count)
        return NULL;

    if (fsel == FUNC_IP)
        return "INPUT";
    else if (fsel == FUNC_OP)
        return "OUTPUT";

    fsel -= FUNC_A0;
    if (fsel >= 0 && fsel < chip->fsel_count)
        name = chip->alt_names[gpio * chip->fsel_count + fsel];
    if (!name)
    {
        sprintf(alt_str, "alt%d", fsel);
        name = alt_str;
    }

    return name;
}

void print_raw_gpio_regs(struct gpio_chip *chip)
{
    int i = -1;

    while (1)
    {
        int new_i = chip->next_reg(i++);

        if (new_i < 0)
            break;
        if (new_i != i)
        {
            /* Change rows if needed */
            if ((i & ~3) != (new_i & ~3)) /* Not on the same row */
            {
                if (i & 3)                /* This row has been started */
                    printf("\n");
                i = new_i & ~3;
            }
        }
        if ((i & 3) == 0)
            printf("%02x:", i * 4);
        if (new_i != i)
        {
            /* Insert padding if needed */
            printf("%*s", (new_i - i) * 9, "");
            i = new_i;
        }

        printf(" %08x", chip->base[i]);

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
    printf("  %s [<n>] get [GPIO]\n", name);
    printf("OR\n");
    printf("  %s [<n>] set <GPIO> [options]\n", name);
    printf("OR\n");
    printf("  %s [<n>] funcs [GPIO]\n", name);
    printf("OR\n");
    printf("  %s [<n>] raw\n", name);
    printf("\n");
    printf("<n> is an option GPIO chip index (default 0)\n");
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
    printf("  pd      set GPIO in-pad pull down\n");
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

int gpio_get(unsigned int gpio)
{
    const char *name;
    int level;
    int fsel;
    int pull;

    fsel = chip->get_fsel(chip, gpio);
    name = gpio_fsel_to_namestr(gpio, fsel);
    level = chip->get_level(chip, gpio);

    printf("GPIO %d: level=%d", gpio, level);

    if (fsel >= FUNC_A0)
        printf(" alt=%d", fsel - FUNC_A0);
    printf(" func=%s", name);

    pull = chip->get_pull(chip, gpio);
    if (pull != PULL_UNSET)
        printf(" pull=%s", gpio_pull_names[pull & 3]);
    printf("\n");
    return 0;
}

int gpio_set(unsigned int gpio, int fsparam, int drive, int pull)
{
    if (fsparam != FUNC_UNSET)
        chip->set_fsel(chip, gpio, fsparam);

    if (drive != DRIVE_UNSET)
    {
        if (chip->get_fsel(chip, gpio) == FUNC_OP)
        {
            chip->set_level(chip, gpio, drive);
        }
        else
        {
            printf("Can't set pin value, not an output\n");
            return 1;
        }
    }

    if (pull != PULL_UNSET)
        return chip->set_pull(chip, gpio, pull);

    return 0;
}

int main(int argc, char *argv[])
{
    int fd;
    int ret;

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

    const char *cmd;

    argv++;
    argc--;

    if (!argc)
    {
        printf("No arguments given - try \"raspi-gpio help\"\n");
        return 1;
    }

    cmd = *(argv++);
    argc--;

    if (strcmp(cmd, "help") == 0)
    {
        print_help();
        return 0;
    }

    chip = get_gpio_chip();

    /* argc 2 or greater, next arg must be set, get or help */
    get = strcmp(cmd, "get") == 0;
    set = strcmp(cmd, "set") == 0;
    funcs = strcmp(cmd, "funcs") == 0;
    raw = strcmp(cmd, "raw") == 0;
    if (!set && !get && !funcs && !raw)
    {
        printf("Unknown argument \"%s\" try \"raspi-gpio help\"\n", cmd);
        return 1;
    }

    if ((get || funcs) && (argc > 1))
    {
        printf("Too many arguments\n");
        return 1;
    }

    if (!argc && set)
    {
        printf("Need GPIO number to set\n");
        return 1;
    }

    if (argc) /* expect pin number(s) next */
    {
        char *p = *(argv++);
        argc--;

        while (p)
        {
            int pin, pin2, len;
            ret = sscanf(p, "%d%n", &pin, &len);
            if (ret != 1 || pin >= chip->gpio_count)
                break;
            p += len;

            if (*p == '-')
            {
                p++;
                ret = sscanf(p, "%d%n", &pin2, &len);
                if (ret != 1 || pin2 >= chip->gpio_count)
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

    if (set && !argc)
    {
        printf("Nothing to set\n");
        return 1;
    }

    /* parse remaining args */
    while (argc)
    {
        const char *arg = *(argv++);
        argc--;

        if (strcmp(arg, "dh") == 0)
            drive = DRIVE_HIGH;
        else if (strcmp(arg, "dl") == 0)
            drive = DRIVE_LOW;
        else if (strcmp(arg, "ip") == 0)
            fsparam = FUNC_IP;
        else if (strcmp(arg, "op") == 0)
            fsparam = FUNC_OP;
        else if (strcmp(arg, "a0") == 0)
            fsparam = FUNC_A0;
        else if (strcmp(arg, "a1") == 0)
            fsparam = FUNC_A1;
        else if (strcmp(arg, "a2") == 0)
            fsparam = FUNC_A2;
        else if (strcmp(arg, "a3") == 0)
            fsparam = FUNC_A3;
        else if (strcmp(arg, "a4") == 0)
            fsparam = FUNC_A4;
        else if (strcmp(arg, "a5") == 0)
            fsparam = FUNC_A5;
        else if (strcmp(arg, "pu") == 0)
            pull = PULL_UP;
        else if (strcmp(arg, "pd") == 0)
            pull = PULL_DOWN;
        else if (strcmp(arg, "pn") == 0)
            pull = PULL_NONE;
        else
        {
            printf("Unknown argument \"%s\"\n", arg);
            return 1;
        }
    }

    if (fsparam >= FUNC_A0 && (fsparam - FUNC_A0) >= chip->fsel_count)
    {
        printf("Alt function a%d out of range\n", fsparam - FUNC_A0);
        return 1;
    }
    /* end arg parsing */

    all_pins = !(gpiomask[0] | gpiomask[1]);

    if (funcs)
    {
        int pin;

        printf("GPIO, DEFAULT PULL, ALT0, ALT1, ALT2, ALT3, ALT4, ALT5\n");
        for (pin = 0; pin < chip->gpio_count; pin++)
        {
            if (all_pins || gpiomask[pin / 32] & (1 << (pin % 32)))
                print_gpio_alts_info(chip, pin);
        }
        return 0;
    }

    /* Check for /dev/gpiomem, else we need root access for /dev/mem */
    if ((fd = open ("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC) ) >= 0)
    {
        chip->base = (uint32_t *)mmap(0, chip->reg_size,
                                      PROT_READ|PROT_WRITE, MAP_SHARED,
                                      fd, 0);
    }
    else
    {
        if (geteuid())
        {
            printf("Must be root\n");
            return 1;
        }

        if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
        {
            printf("Unable to open /dev/mem: %s\n", strerror (errno));
            return 1;
        }

        chip->base = (uint32_t *)mmap(0, chip->reg_size,
                                      PROT_READ|PROT_WRITE, MAP_SHARED,
                                      fd, chip->reg_base);
    }

    if (chip->base == (uint32_t *)-1)
    {
        printf("mmap (GPIO) failed: %s\n", strerror (errno));
        return 1;
    }

    if (set || get)
    {
        int pin;
        for (pin = 0; pin < chip->gpio_count; pin++)
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
        print_raw_gpio_regs(chip);

    return 0;
}

static int bcm2835_get_level(struct gpio_chip *chip, unsigned int gpio)
{
    if (gpio >= chip->gpio_count)
        return -1;

    return (chip->base[GPLEV0 + (gpio / 32)] >> (gpio % 32)) & 1;
}

static int bcm2835_get_fsel(struct gpio_chip *chip, unsigned int gpio)
{
    /* GPFSEL0-5 with 10 sels per reg, 3 bits per sel (so bits 0:29 used) */
    uint32_t reg = GPFSEL0 + (gpio / 10);
    uint32_t lsb = (gpio % 10) * 3;

    if (gpio < chip->gpio_count)
    {
        switch ((chip->base[reg] >> lsb) & 7)
        {
        case 0: return FUNC_IP;
        case 1: return FUNC_OP;
        case 2: return FUNC_A5;
        case 3: return FUNC_A4;
        case 4: return FUNC_A0;
        case 5: return FUNC_A1;
        case 6: return FUNC_A2;
        case 7: return FUNC_A3;
        }
    }

    return -1;
}

static int bcm2835_get_pull(struct gpio_chip *chip, unsigned int gpio)
{
    /* This is a write-only mechanism */
    return PULL_UNSET;
}

static int bcm2835_set_level(struct gpio_chip *chip, unsigned int gpio, int level)
{
    if (gpio >= chip->gpio_count)
        return -1;

    chip->base[(level ? GPSET0 : GPCLR0) + (gpio / 32)] = (1 << (gpio % 32));

    return 0;
}

static int bcm2835_set_fsel(struct gpio_chip *chip, unsigned int gpio, int fsel)
{
    /* GPFSEL0-5 with 10 sels per reg, 3 bits per sel (so bits 0:29 used) */
    uint32_t reg = GPFSEL0 + (gpio / 10);
    uint32_t lsb = (gpio % 10) * 3;

    switch (fsel)
    {
    case FUNC_IP: fsel = 0; break;
    case FUNC_OP: fsel = 1; break;
    case FUNC_A0: fsel = 4; break;
    case FUNC_A1: fsel = 5; break;
    case FUNC_A2: fsel = 6; break;
    case FUNC_A3: fsel = 7; break;
    case FUNC_A4: fsel = 3; break;
    case FUNC_A5: fsel = 2; break;
    default:
        return -1;
    }

    if (gpio < chip->gpio_count)
    {
        chip->base[reg] = (chip->base[reg] & ~(0x7 << lsb)) | (fsel << lsb);
        return 0;
    }

    return -1;
}

static int bcm2835_set_pull(struct gpio_chip *chip, unsigned int gpio, int pull)
{
    int clkreg = GPPUDCLK0 + (gpio / 32);
    int clkbit = 1 << (gpio % 32);

    if (gpio >= chip->gpio_count)
        return -1;

    if (pull < 0 || pull > 2) return -1;

    chip->base[GPPUD] = pull;
    usleep(10);
    chip->base[clkreg] = clkbit;
    usleep(10);
    chip->base[GPPUD] = 0;
    usleep(10);
    chip->base[clkreg] = 0;
    usleep(10);

    return 0;
}

static int bcm2835_next_reg(int reg)
{
    if (reg < 0)
        return 0;
    else if (reg == GPPUDCLK1)
        return -1;
    return reg + 1;
}

static int bcm2711_get_pull(struct gpio_chip *chip, unsigned int gpio)
{
    int reg = GPPUPPDN0 + (gpio / 16);
    int lsb = (gpio % 16) * 2;

    if (gpio >= chip->gpio_count)
        return -1;

    switch ((chip->base[reg] >> lsb) & 3)
    {
    case 0: return PULL_NONE;
    case 1: return PULL_UP;
    case 2: return PULL_DOWN;
    }
    return -1;
}

static int bcm2711_set_pull(struct gpio_chip *chip, unsigned int gpio, int pull)
{
    int reg = GPPUPPDN0 + (gpio / 16);
    int lsb = (gpio % 16) * 2;

    if (gpio >= chip->gpio_count)
        return -1;

    switch (pull)
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
        return -1;
    }

    chip->base[reg] = (chip->base[reg] & ~(3 << lsb)) | (pull << lsb);

    return 0;
}

static int bcm2711_next_reg(int reg)
{
    /* Skip over non-GPIO registers */
    if (reg < 0)
        return 0;
    else if (reg == GPPUPPDN3)
        return -1;
    else if (reg == GPPUDCLK1)
        return GPPUPPDN0;
    return reg + 1;
}
