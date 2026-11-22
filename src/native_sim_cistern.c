#include "native_sim_cistern.h"

#if defined(CONFIG_BOARD_NATIVE_SIM)
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/adc/adc_emul.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/w1.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <canopennode.h>
#include "native_sim_ds18b20.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

LOG_MODULE_REGISTER(native_sim_cistern, LOG_LEVEL_INF);

#define INPUT_PORT 46005
#define INPUT_PACKET_SIZE 96
#define ADC_MAX_RAW 4095

static const struct adc_dt_spec pressure_adc =
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
struct cistern_inputs {
	double level_mm;
	uint32_t pressure_adc_raw;
	double ultrasonic_height;
	double water_temperature;
	double air_temperature;
	uint64_t water_temperature_rom_id;
	uint64_t air_temperature_rom_id;
};

static struct cistern_inputs inputs = {
	.level_mm = 800.0,
	.pressure_adc_raw = 1065,
	.ultrasonic_height = 2600.0,
	.water_temperature = 18.0,
	.air_temperature = 20.0,
	.water_temperature_rom_id = UINT64_C(0x2801000000000029),
	.air_temperature_rom_id = UINT64_C(0x2802000000000070),
};
static int input_fd = -1;
K_MUTEX_DEFINE(inputs_mutex);

struct simulated_sensor_config {
	enum sensor_channel channel;
};

struct simulated_sensor_data {
	struct sensor_value value;
};

static int32_t simulated_distance_mm;
K_MUTEX_DEFINE(simulated_values_mutex);

static int simulated_sensor_fetch(const struct device *dev, enum sensor_channel channel)
{
	const struct simulated_sensor_config *cfg = dev->config;
	struct simulated_sensor_data *data = dev->data;
	int64_t micro;

	if (channel != SENSOR_CHAN_ALL && channel != cfg->channel) {
		return -ENOTSUP;
	}
	k_mutex_lock(&simulated_values_mutex, K_FOREVER);
	micro = (int64_t)simulated_distance_mm * 1000;
	k_mutex_unlock(&simulated_values_mutex);
	return sensor_value_from_micro(&data->value, micro);
}

static int simulated_sensor_channel_get(const struct device *dev,
					enum sensor_channel channel, struct sensor_value *value)
{
	const struct simulated_sensor_config *cfg = dev->config;
	struct simulated_sensor_data *data = dev->data;

	if (channel != cfg->channel) {
		return -ENOTSUP;
	}
	*value = data->value;
	return 0;
}

static DEVICE_API(sensor, simulated_sensor_api) = {
	.sample_fetch = simulated_sensor_fetch,
	.channel_get = simulated_sensor_channel_get,
};

#define DEFINE_SIMULATED_SENSOR(name, node_id, channel_id) \
	static struct simulated_sensor_data data_##name; \
	static const struct simulated_sensor_config cfg_##name = { \
		.channel = channel_id \
	}; \
	SENSOR_DEVICE_DT_DEFINE(node_id, NULL, NULL, &data_##name, &cfg_##name, \
				POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &simulated_sensor_api)

DEFINE_SIMULATED_SENSOR(ultrasonic, DT_NODELABEL(us0), SENSOR_CHAN_DISTANCE);

static void update_emulators(const struct cistern_inputs *state)
{
	double level = CLAMP(state->level_mm, 0.0, 2500.0);
	uint32_t distance_mm = (uint32_t)MAX(0.0, state->ultrasonic_height - level);
	(void)adc_emul_const_raw_value_set(pressure_adc.dev, pressure_adc.channel_id,
					   state->pressure_adc_raw);
	k_mutex_lock(&simulated_values_mutex, K_FOREVER);
	simulated_distance_mm = (int32_t)distance_mm;
	k_mutex_unlock(&simulated_values_mutex);
	native_sim_ds18b20_set_temperature(0, state->water_temperature);
	native_sim_ds18b20_set_temperature(1, state->air_temperature);
}

static void set_rom_input(const char *key, uint64_t rom_id)
{
	struct cistern_inputs snapshot;
	size_t index;

	k_mutex_lock(&inputs_mutex, K_FOREVER);
	if (strcmp(key, "water_temperature_rom_id") == 0) {
		inputs.water_temperature_rom_id = rom_id;
		index = 0;
	} else if (strcmp(key, "air_temperature_rom_id") == 0) {
		inputs.air_temperature_rom_id = rom_id;
		index = 1;
	} else {
		k_mutex_unlock(&inputs_mutex);
		return;
	}
	snapshot = inputs;
	k_mutex_unlock(&inputs_mutex);

	if (native_sim_ds18b20_set_roms(snapshot.water_temperature_rom_id,
					snapshot.air_temperature_rom_id) == 0) {
		LOG_INF("Changed emulated DS18B20 hardware ROM in slot %u to 0x%016llx",
			(unsigned int)index, (unsigned long long)rom_id);
	}
}

static bool valid_ds18b20_rom_id(uint64_t rom_id)
{
	struct w1_rom rom;

	w1_uint64_to_rom(rom_id, &rom);
	return rom.family == 0x28 &&
		rom.crc == w1_crc8((const uint8_t *)&rom, 7);
}

static void set_input(const char *key, double value)
{
	struct cistern_inputs snapshot;

	if (!isfinite(value)) {
		return;
	}
	k_mutex_lock(&inputs_mutex, K_FOREVER);
	if (strcmp(key, "level_mm") == 0) {
		inputs.level_mm = value;
	} else if (strcmp(key, "pressure_adc_raw") == 0) {
		inputs.pressure_adc_raw = (uint32_t)CLAMP(lround(value), 0, ADC_MAX_RAW);
	} else if (strcmp(key, "ultrasonic_height") == 0) {
		inputs.ultrasonic_height = value;
	} else if (strcmp(key, "water_temperature") == 0) {
		inputs.water_temperature = value;
	} else if (strcmp(key, "air_temperature") == 0) {
		inputs.air_temperature = value;
	} else {
		k_mutex_unlock(&inputs_mutex);
		return;
	}
	snapshot = inputs;
	k_mutex_unlock(&inputs_mutex);
	update_emulators(&snapshot);
}

void native_sim_cistern_process(void)
{
	char packet[INPUT_PACKET_SIZE];
	for (;;) {
		ssize_t length = recvfrom(input_fd, packet, sizeof(packet) - 1,
					  MSG_DONTWAIT, NULL, NULL);

		if (length < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
			return;
		}
		if (length <= 0) {
			return;
		}
		packet[length] = '\0';
		char *separator = strchr(packet, '=');
		if (separator == NULL) {
			continue;
		}
		*separator++ = '\0';
		if (strcmp(packet, "water_temperature_rom_id") == 0 ||
		    strcmp(packet, "air_temperature_rom_id") == 0) {
			char *rom_end = NULL;
			unsigned long long rom_id;

			if (strlen(separator) != 16) {
				continue;
			}
			rom_id = strtoull(separator, &rom_end, 16);
			if (rom_end != separator && *rom_end == '\0' &&
			    valid_ds18b20_rom_id((uint64_t)rom_id)) {
				set_rom_input(packet, (uint64_t)rom_id);
			}
			continue;
		}
		char *end = NULL;
		double value = strtod(separator, &end);
		if (end != separator && (*end == '\0' || *end == '\n')) {
			set_input(packet, value);
			LOG_INF("Applied virtual sensor input %s", packet);
		}
	}
}

void native_sim_cistern_start(void)
{
	if (!adc_is_ready_dt(&pressure_adc)) {
		LOG_ERR("Pressure ADC emulator is not ready");
		return;
	}
	if (native_sim_ds18b20_init() != 0) {
		LOG_ERR("Could not initialize virtual DS18B20 W1 devices");
		return;
	}
	update_emulators(&inputs);
	input_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (input_fd < 0) {
		LOG_ERR("Could not create virtual sensor input socket: %d", errno);
		return;
	}
	struct sockaddr_in address = {
		.sin_family = AF_INET,
		.sin_port = htons(INPUT_PORT),
		.sin_addr.s_addr = htonl(INADDR_ANY),
	};
	if (bind(input_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
		LOG_ERR("Could not bind virtual sensor input socket: %d", errno);
		close(input_fd);
		input_fd = -1;
		return;
	}
	LOG_INF("Virtual sensor input listening on UDP %d", INPUT_PORT);
}
#endif
