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

## Building an application

* `cd` to the application directory.\
 E.g.:
 ```
 cd /path/to/vouch-zephyr/vouch/apps/dk-test-app`
 ```

* Build using west. The first time, the board needs to specified:\
 ```
 west build --board=qemu_cortex_m3 -p always
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
