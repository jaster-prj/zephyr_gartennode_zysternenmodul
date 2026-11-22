#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <canopennode.h>

#include "sensor.h"
#include "pressure_measurement.h"
#include "ultrasonic_measurement.h"
#include "temperature_measurement.h"

LOG_MODULE_REGISTER(sensor, CONFIG_CANOPENNODE_LOG_LEVEL);

#define SENSOR_STACK_SIZE 1024
#define SENSOR_THREAD_PRIORITY K_LOWEST_APPLICATION_THREAD_PRIO

enum sensor_index {
	PRESSURE,
	ULTRASONIC,
	TEMPERATURE,
	SENSOR_COUNT,
};

struct sensor_control {
	struct gpio_dt_spec power;
	int (*init)(void);
	int (*read)(void);
	atomic_t enabled;
	struct k_mutex lock;
	bool ready;
};

static bool locks_initialized;

static struct sensor_control sensors[] = {
	[PRESSURE] = {
		.power = GPIO_DT_SPEC_GET(DT_NODELABEL(pressure_sw), gpios),
		.init = pressure_measurement_init,
		.read = pressure_measurement_read,
	},
	[ULTRASONIC] = {
		.power = GPIO_DT_SPEC_GET(DT_NODELABEL(ultrasonic_sw), gpios),
		.init = ultrasonic_measurement_init,
		.read = ultrasonic_measurement_read,
	},
	[TEMPERATURE] = {
		.power = GPIO_DT_SPEC_GET(DT_NODELABEL(temperature_sw), gpios),
		.init = temperature_measurement_init,
		.read = temperature_measurement_read,
	},
};

K_THREAD_STACK_ARRAY_DEFINE(sensor_stacks, SENSOR_COUNT, SENSOR_STACK_SIZE);
static struct k_thread sensor_threads[SENSOR_COUNT];
static bool threads_started;

static void sensor_thread(void *arg, void *unused1, void *unused2)
{
	struct sensor_control *sensor = arg;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	while (true) {
		uint32_t period = 100U;

		k_mutex_lock(&sensor->lock, K_FOREVER);
		if (atomic_get(&sensor->enabled)) {
			(void)sensor->read();
			if (sensor == &sensors[PRESSURE]) {
				period = OD_pressurePeriode;
			} else if (sensor == &sensors[ULTRASONIC]) {
				period = OD_ultrasonicPeriode;
			} else {
				period = OD_temperaturePeriode;
			}
		}
		k_mutex_unlock(&sensor->lock);
		k_sleep(K_MSEC(MAX(period, 1U)));
	}
}
int sensor_init(void)
{
	int result = 0;

	if (!locks_initialized) {
		for (size_t i = 0; i < ARRAY_SIZE(sensors); i++) {
			k_mutex_init(&sensors[i].lock);
		}
		locks_initialized = true;
	}
	for (size_t i = 0; i < ARRAY_SIZE(sensors); i++) {
		struct sensor_control *sensor = &sensors[i];
		int ret;

		if (!gpio_is_ready_dt(&sensor->power)) {
			result = -ENODEV;
			continue;
		}
		ret = gpio_pin_configure_dt(&sensor->power, GPIO_OUTPUT_INACTIVE);
		if (ret == 0) {
			ret = sensor->init();
		}
		sensor->ready = (ret == 0);
		if (ret != 0) {
			LOG_ERR("Sensor %u initialization failed: %d", (unsigned int)i, ret);
			result = ret;
		}
	}
	if (!threads_started) {
		for (size_t i = 0; i < ARRAY_SIZE(sensors); i++) {
			k_thread_create(&sensor_threads[i], sensor_stacks[i],
					K_THREAD_STACK_SIZEOF(sensor_stacks[i]),
					sensor_thread, &sensors[i], NULL, NULL,
					SENSOR_THREAD_PRIORITY, 0, K_NO_WAIT);
		}
		threads_started = true;
	}
	return result;
}

static int sensor_set_enabled(enum sensor_index index, uint8_t value)
{
	struct sensor_control *sensor = &sensors[index];
	int ret;

	if (!sensor->ready) {
		return -ENODEV;
	}
	ret = gpio_pin_set_dt(&sensor->power, value != 0U);
	if (ret == 0) {
		atomic_set(&sensor->enabled, value != 0U);
	}
	return ret;
}

int pressure_enable(uint8_t value)
{
	return sensor_set_enabled(PRESSURE, value);
}

int ultrasonic_enable(uint8_t value)
{
	return sensor_set_enabled(ULTRASONIC, value);
}

int temperature_enable(uint8_t value)
{
	return sensor_set_enabled(TEMPERATURE, value);
}

void sensor_stop(void)
{
	/* Drain in-flight reads before CANopen reinitializes the object dictionary. */
	for (size_t i = 0; i < ARRAY_SIZE(sensors); i++) {
		k_mutex_lock(&sensors[i].lock, K_FOREVER);
		atomic_clear(&sensors[i].enabled);
		k_mutex_unlock(&sensors[i].lock);
	}
}

void sensor_process(void)
{
	/* Sensor reads run on dedicated workers and must not block CANopen. */
}
