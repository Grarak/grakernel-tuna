#include <linux/gpio.h>

struct gps_elton_platform_data_s {
	unsigned gpio_reset;
	unsigned gpio_on_off;
	unsigned gpio_awake;
	unsigned functional;
};

