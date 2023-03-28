#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "rfal_nfc.h"

LOG_MODULE_DECLARE(ST25R);

int main()
{

    ReturnCode rc = rfalNfcInitialize();
    if (rc != ERR_NONE) {
        LOG_ERR("RFAL init failed");
    }

    while (1) {
        LOG_INF("main loop");
        k_sleep(K_MSEC(5000));

    }
}
