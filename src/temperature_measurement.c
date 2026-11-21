#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/w1_sensor.h>
#include <zephyr/logging/log.h>
#include <canopennode.h>

#include "temperature_measurement.h"

LOG_MODULE_REGISTER(temperature_measurement, CONFIG_CANOPENNODE_LOG_LEVEL);

static const struct device *const sensors[] = {
	DEVICE_DT_GET(DT_NODELABEL(temp_0)),
	DEVICE_DT_GET(DT_NODELABEL(temp_1)),
};

#if !defined(CONFIG_BOARD_NATIVE_SIM)
static const uint64_t rom_addresses[] = {0x281e1a4398230066, 0x2872f7e062200123};
#endif

int temperature_measurement_init(void)
{
	int ret = 0;

	for (size_t i = 0; i < ARRAY_SIZE(sensors); i++) {
		if (!device_is_ready(sensors[i])) {
			LOG_ERR("Temperature sensor %u not ready", (unsigned int)i);
			ret = -ENODEV;
		}
	}
	return ret;
}

int temperature_measurement_read(void)
{
	int result = 0;

	for (size_t i = 0; i < ARRAY_SIZE(sensors); i++) {
		struct sensor_value temperature;
		int ret;

		if (!device_is_ready(sensors[i])) {
			result = -ENODEV;
			continue;
		}
#if !defined(CONFIG_BOARD_NATIVE_SIM)
		struct w1_rom rom;
		struct sensor_value address;

		w1_uint64_to_rom(rom_addresses[i], &rom);
		w1_rom_to_sensor_value(&rom, &address);
		(void)sensor_attr_set(sensors[i], SENSOR_CHAN_ALL, SENSOR_ATTR_W1_ROM,
				      &address);
#endif
		ret = sensor_sample_fetch(sensors[i]);
		if (ret == 0) {
			ret = sensor_channel_get(sensors[i], SENSOR_CHAN_AMBIENT_TEMP,
						 &temperature);
		}
		if (ret != 0) {
			LOG_ERR("Could not read temperature sensor %u: %d",
				(unsigned int)i, ret);
			result = ret;
			continue;
		}
		CO_LOCK_OD();
		OD_temperatureValue[i] =
			(int16_t)(temperature.val1 * 1000 + temperature.val2 / 1000);
		CO_UNLOCK_OD();
	}
	return result;
}
