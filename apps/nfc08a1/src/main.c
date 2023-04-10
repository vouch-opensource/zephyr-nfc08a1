#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "demo.h"

LOG_MODULE_DECLARE(ST25R);

int main()
{

    bool rv = demoIni();
    if (!rv) {
        LOG_ERR("failed to initialize");
    } else {
        while (1) {
            demoCycle();
        }
    }

    return 0;
}
