#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/w1_sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <canopennode.h>

#include "temperature_measurement.h"

LOG_MODULE_REGISTER(temperature_measurement, CONFIG_CANOPENNODE_LOG_LEVEL);

static const struct device *const sensors[] = {
	DEVICE_DT_GET(DT_NODELABEL(temp_0)),
	DEVICE_DT_GET(DT_NODELABEL(temp_1)),
};

#define TEMPERATURE_SENSOR_COUNT ARRAY_SIZE(sensors)
#define W1_SEARCH_ATTEMPTS 3
static uint64_t assigned_rom_ids[TEMPERATURE_SENSOR_COUNT];

struct temperature_discovery {
	uint64_t rom_addresses[TEMPERATURE_SENSOR_COUNT];
	size_t count;
};

static void temperature_search_callback(struct w1_rom rom, void *user_data)
{
	struct temperature_discovery *discovery = user_data;
	uint64_t rom_id = w1_rom_to_uint64(&rom);

	if (rom.family != 0x28) {
		LOG_DBG("Ignoring 1-Wire device with family code 0x%02x", rom.family);
		return;
	}
	if (discovery->count >= ARRAY_SIZE(discovery->rom_addresses)) {
		LOG_WRN("More DS18B20 sensors found than supported; ignoring 0x%016llx",
			(unsigned long long)rom_id);
		return;
	}

	discovery->rom_addresses[discovery->count++] = rom_id;
	LOG_INF("Found DS18B20 sensor %u: 0x%016llx",
		(unsigned int)discovery->count, (unsigned long long)rom_id);
}

static int temperature_set_rom(size_t slot, uint64_t rom_id)
{
	struct w1_rom rom;
	struct sensor_value address;

	w1_uint64_to_rom(rom_id, &rom);
	w1_rom_to_sensor_value(&rom, &address);
	int ret = sensor_attr_set(sensors[slot], SENSOR_CHAN_ALL,
				  SENSOR_ATTR_W1_ROM, &address);

	if (ret == 0) {
		assigned_rom_ids[slot] = rom_id;
	}
	return ret;
}

static int temperature_assign_rom(size_t slot, uint64_t rom_id)
{
	int ret = temperature_set_rom(slot, rom_id);

	if (ret != 0) {
		return ret;
	}
	for (int attempt = 0; attempt < W1_SEARCH_ATTEMPTS; attempt++) {
		struct sensor_value temperature;

		ret = sensor_sample_fetch(sensors[slot]);
		if (ret == 0) {
			ret = sensor_channel_get(sensors[slot],
						 SENSOR_CHAN_AMBIENT_TEMP,
						 &temperature);
		}
		if (ret == 0) {
			return 0;
		}
		if (attempt + 1 < W1_SEARCH_ATTEMPTS) {
			k_sleep(K_MSEC(100));
		}
	}
	return ret;
}

static bool temperature_id_is_assigned(const uint64_t *ids, uint64_t rom_id)
{
	for (size_t i = 0; i < TEMPERATURE_SENSOR_COUNT; i++) {
		if (ids[i] == rom_id) {
			return true;
		}
	}
	return false;
}

static int temperature_discover_sensors(void)
{
	const struct device *w1_dev = DEVICE_DT_GET(DT_NODELABEL(w1_0));
	struct temperature_discovery discovery = {0};
	uint64_t configured_ids[TEMPERATURE_SENSOR_COUNT];
	uint64_t active_ids[TEMPERATURE_SENSOR_COUNT] = {0};

	if (!device_is_ready(w1_dev)) {
		LOG_ERR("1-Wire bus is not ready");
		return -ENODEV;
	}

	CO_LOCK_OD();
	for (size_t i = 0; i < TEMPERATURE_SENSOR_COUNT; i++) {
		configured_ids[i] = OD_temperatureDS18B20ID[i];
	}
	CO_UNLOCK_OD();

	/* Reuse responding configured sensors in their existing water/air slots. */
	for (size_t i = 0; i < TEMPERATURE_SENSOR_COUNT; i++) {
		if (configured_ids[i] == 0U ||
		    temperature_id_is_assigned(active_ids, configured_ids[i])) {
			continue;
		}
		if (temperature_assign_rom(i, configured_ids[i]) == 0) {
			active_ids[i] = configured_ids[i];
			LOG_INF("Using configured DS18B20 in slot %u: 0x%016llx",
				(unsigned int)i,
				(unsigned long long)configured_ids[i]);
		} else {
			LOG_WRN("Configured DS18B20 in slot %u did not respond after %d tries; clearing ID",
				(unsigned int)i, W1_SEARCH_ATTEMPTS);
		}
	}

	if (active_ids[0] != 0U && active_ids[1] != 0U) {
		LOG_INF("Both configured DS18B20 sensors responded; skipping discovery");
		return 0;
	}

	LOG_INF("Searching for unassigned DS18B20 sensors");
	(void)w1_search_rom(w1_dev, temperature_search_callback, &discovery);
	for (size_t n = 0; n < discovery.count; n++) {
		uint64_t rom_id = discovery.rom_addresses[n];
		size_t slot;

		if (temperature_id_is_assigned(active_ids, rom_id)) {
			continue;
		}
		for (slot = 0; slot < TEMPERATURE_SENSOR_COUNT; slot++) {
			if (active_ids[slot] == 0U) {
				break;
			}
		}
		if (slot == TEMPERATURE_SENSOR_COUNT) {
			break;
		}
		if (temperature_assign_rom(slot, rom_id) == 0) {
			active_ids[slot] = rom_id;
			LOG_INF("Assigned new DS18B20 to slot %u: 0x%016llx",
				(unsigned int)slot, (unsigned long long)rom_id);
		} else {
			LOG_WRN("Discovered DS18B20 did not respond in slot %u: 0x%016llx",
				(unsigned int)slot, (unsigned long long)rom_id);
		}
	}

	CO_LOCK_OD();
	for (size_t i = 0; i < TEMPERATURE_SENSOR_COUNT; i++) {
		OD_temperatureDS18B20ID[i] = active_ids[i];
	}
	CO_UNLOCK_OD();
	return (active_ids[0] != 0U || active_ids[1] != 0U) ? 0 : -ENODEV;
}

int temperature_measurement_init(void)
{
	int ret = 0;

	for (size_t i = 0; i < ARRAY_SIZE(sensors); i++) {
		if (!device_is_ready(sensors[i])) {
			LOG_ERR("Temperature sensor %u not ready", (unsigned int)i);
			ret = -ENODEV;
		}
	}
	if (ret == 0) {
		ret = temperature_discover_sensors();
	}
	return ret;
}

int temperature_measurement_read(void)
{
	int result = 0;

	for (size_t i = 0; i < ARRAY_SIZE(sensors); i++) {
		struct sensor_value temperature;
		uint64_t configured_id;
		int ret;

		if (!device_is_ready(sensors[i])) {
			result = -ENODEV;
			continue;
		}
		CO_LOCK_OD();
		configured_id = OD_temperatureDS18B20ID[i];
		CO_UNLOCK_OD();
		if (configured_id != 0U && configured_id != assigned_rom_ids[i] &&
		    temperature_set_rom(i, configured_id) != 0) {
			result = -EINVAL;
			continue;
		}
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
		float temperature_c =
			(float)temperature.val1 + (float)temperature.val2 / 1000000.0f;
		CO_LOCK_OD();
		OD_temperatureRaw[i] =
			(int16_t)(temperature.val1 * 1000 + temperature.val2 / 1000);
		OD_temperatureMeasurement[i] = temperature_c;
		CO_UNLOCK_OD();
	}
	return result;
}
