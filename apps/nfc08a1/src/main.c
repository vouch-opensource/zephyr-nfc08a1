#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(ST25R);

int main()
{
    while (1) {
        LOG_INF("main loop");
        k_sleep(K_MSEC(5000));

    }
}
