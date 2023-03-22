#include <zephyr/kernel.h>

#include "rfal_nfc.h"

int main()
{
    rfalNfcInitialize();

    while (1) {
        printk("main loop\n");
        k_sleep(K_MSEC(5000));

    }
}
