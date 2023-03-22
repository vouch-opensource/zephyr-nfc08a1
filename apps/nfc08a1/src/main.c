#include <zephyr/kernel.h>

int main()
{
    while (1) {
        printk("main loop\n");
        k_sleep(K_MSEC(5000));

    }
}
