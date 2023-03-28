#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "demo_polling.h"

LOG_MODULE_DECLARE(ST25R);

int main()
{

    bool rv = demoIni();
    if (!rv) {
        LOG_ERR("failed to initialize");
    } else {
        while (1) {
            LOG_INF("main loop");
            demoCycle();
        }
    }

    return 0;
}
