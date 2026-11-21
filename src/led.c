#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "led.h"

LOG_MODULE_REGISTER(led, CONFIG_CANOPENNODE_LOG_LEVEL);

static struct gpio_dt_spec led_green_gpio =
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(green_led), gpios, {0});
static struct gpio_dt_spec led_red_gpio =
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(red_led), gpios, {0});

static void led_callback(bool value, void *arg)
{
	struct gpio_dt_spec *gpio = arg;

	if (gpio != NULL && gpio->port != NULL) {
		gpio_pin_set_dt(gpio, value);
	}
}

int led_init(void)
{
	struct gpio_dt_spec *leds[] = {&led_green_gpio, &led_red_gpio};

	for (size_t i = 0; i < ARRAY_SIZE(leds); i++) {
		struct gpio_dt_spec *gpio = leds[i];

		if (gpio->port == NULL) {
			continue;
		}
		if (!gpio_is_ready_dt(gpio)) {
			LOG_ERR("LED GPIO device not ready");
			gpio->port = NULL;
			continue;
		}
		int ret = gpio_pin_configure_dt(gpio, GPIO_OUTPUT_INACTIVE);

		if (ret != 0) {
			LOG_ERR("Failed to configure LED GPIO: %d", ret);
			gpio->port = NULL;
		}
	}
	return 0;
}

void led_configure(CO_NMT_t *nmt)
{
	canopen_leds_init(nmt, led_callback, &led_green_gpio,
			 led_callback, &led_red_gpio);
}
