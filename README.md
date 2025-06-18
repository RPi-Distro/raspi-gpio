# raspi-gpio

**This tool is no longer maintained or supported. It has been replaced by the newer [pinctrl](https://github.com/raspberrypi/utils/tree/master/pinctrl) tool.** 

----

Tool to help debug / hack at the BCM283x GPIO. 

You can dump the state of a GPIO or (all GPIOs).

You can change a GPIO mode and pulls (and level if set as an output).

Beware this tool writes directly to the BCM283x GPIO registers, ignoring anything else that may be using them (like Linux drivers).

## Build from source

This tool is pre-installed in Raspberry Pi OS, but to build from source, you should be able to build and install raspi-gpio with:

```bash
sudo apt install automake
./configure
make
sudo make install
```

## Usage

```
Use:
  raspi-gpio get [GPIO]
OR
  raspi-gpio set <GPIO> [options]
OR
  raspi-gpio funcs [GPIO]
OR
  raspi-gpio raw

GPIO is a comma-separated list of pin numbers or ranges (without spaces),
e.g. 4 or 18-21 or 7,9-11
Note that omitting [GPIO] from raspi-gpio get prints all GPIOs.
raspi-gpio funcs will dump all the possible GPIO alt funcions in CSV format
or if [GPIO] is specified the alternate funcs just for that specific GPIO.

Valid [options] for raspi-gpio set are:
  ip      set GPIO as input
  op      set GPIO as output
  a0-a5   set GPIO to alternate function alt0-alt5
  pu      set GPIO in-pad pull up
  pd      set GPIO pin-pad pull down
  pn      set GPIO pull none (no pull)
  dh      set GPIO to drive to high (1) level (only valid if set to be an output)
  dl      set GPIO to drive low (0) level (only valid if set to be an output)
  
Examples:
  raspi-gpio get              Prints state of all GPIOs one per line
  raspi-gpio get 20           Prints state of GPIO20
  raspi-gpio get 20,21        Prints state of GPIO20 and GPIO21
  raspi-gpio set 20 a5        Set GPIO20 to ALT5 function (GPCLK0)
  raspi-gpio set 20 pu        Enable GPIO20 ~50k in-pad pull up
  raspi-gpio set 20 pd        Enable GPIO20 ~50k in-pad pull down
  raspi-gpio set 20 op        Set GPIO20 to be an output
  raspi-gpio set 20 dl        Set GPIO20 to output low/zero (must already be set as an output)
  raspi-gpio set 20 ip pd     Set GPIO20 to input with pull down
  raspi-gpio set 35 a0 pu     Set GPIO35 to ALT0 function (SPI_CE1_N) with pull up
  raspi-gpio set 20 op pn dh  Set GPIO20 to ouput with no pull and driving high
```
