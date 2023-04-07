# zephyr-nfc08a1
Zephyr app to drive the X-NUCLEO-NFC08A1

### Do not clone directly from git!

This is a Zephyr-based firmware repo.
Read the getting-started guide here: <https://docs.zephyrproject.org/latest/getting_started/index.html>

This repo is using **Workflow 4: Application as the manifest repository**,
as described in <https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.6.0/nrf/dm_adding_code.html#user-workflows>


## How to use

This repo is NOT to be cloned directy from git!
Use it with Zephyr's `west` metatool, e.g.:
```
mkdir vouch-zephyr-nfc08a1
cd vouch-zephyr-nfc08a1
west init -m git@github.com:vouchio/zephyr-nfc08a1.git
west update
```

For more information about `west`, see <https://docs.zephyrproject.org/1.14.0/guides/west/repo-tool.html>

## Configuring for I2C

By default, the hardware and software are configured for SPI. The following changes are required for I2C.

### Hardware

Reconfigure the X-NUCLEO-NFC08A1 for I2C by

0. Soldering pin headers onto the SCL and SDA connections (ST2 and ST4) and add jumpers.
0. Cut traces related to SCLK and MISO (ST5 and ST6 dogbones), and optionally add pinheaders so they can be-rejumpered for SPI mode.
0. Soldering 1k65 pullup resistors R116 and R117
0. Move resistor R205 to R204 so that I2C_ENAB is pulled low.


### Software

0. Change `prj.conf` to specify `CONFIG_I2C=y` and `CONFIG_RFAL_USE_I2C=y`
0. Revise the board overlay (in the `boards` directory) to uncomment the I2C devicetree configuration and comment the SPI configuration.

## Building an application

* `cd` to the application directory.\
 E.g.:
 ```
 cd /path/to/vouch-zephyr-nfc08a1/vouch/apps/nfc08a1
 ```

* Build using west. The first time, the board needs to specified:
 ```
 west build --board=nrf5340dk_nrf5340_cpuapp -p always
 ```
 or
 ```
 west build --board=nrf52840dk_nrf52840 -p always
 ```
 or
 ```
 west build --board=nucleo_l053r8 -p always -- -DOVERLAY_CONFIG=nucleo_l053r8.conf
 ```

 Next time, you can just invoke the build command:
 ```
 west build
 ```

* Flash or debug your application:
 ```
 west flash
 # or
 west debug
 ```
