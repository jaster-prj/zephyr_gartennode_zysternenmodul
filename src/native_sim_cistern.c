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
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

LOG_MODULE_REGISTER(native_sim_cistern, LOG_LEVEL_INF);

#define INPUT_PORT 46005
#define INPUT_PACKET_SIZE 96
#define ADC_VREF_MV 3300
#define ADC_MAX_RAW 4095

static const struct adc_dt_spec pressure_adc =
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
struct cistern_inputs {
	double level_mm;
	double pressure_sensor_height;
	double pressure_zero_voltage;
	double pressure_span_voltage;
	double ultrasonic_height;
	double water_temperature;
	double air_temperature;
};

static struct cistern_inputs inputs = {
	.level_mm = 800.0,
	.pressure_sensor_height = 20.0,
	.pressure_zero_voltage = 0.0,
	.pressure_span_voltage = 3.3,
	.ultrasonic_height = 2600.0,
	.water_temperature = 18.0,
	.air_temperature = 20.0,
};
static int input_fd = -1;
K_MUTEX_DEFINE(inputs_mutex);

struct simulated_sensor_config {
	enum sensor_channel channel;
	uint8_t input_index;
};

struct simulated_sensor_data {
	struct sensor_value value;
};

static int32_t simulated_values[3];
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
	if (cfg->input_index == 0) {
		micro = (int64_t)simulated_values[0] * 1000;
	} else {
		micro = (int64_t)simulated_values[cfg->input_index] * 1000;
	}
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

#define DEFINE_SIMULATED_SENSOR(name, node_id, channel_id, index) \
	static struct simulated_sensor_data data_##name; \
	static const struct simulated_sensor_config cfg_##name = { \
		.channel = channel_id, .input_index = index \
	}; \
	SENSOR_DEVICE_DT_DEFINE(node_id, NULL, NULL, &data_##name, &cfg_##name, \
				POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &simulated_sensor_api)

DEFINE_SIMULATED_SENSOR(ultrasonic, DT_NODELABEL(us0), SENSOR_CHAN_DISTANCE, 0);
DEFINE_SIMULATED_SENSOR(water_temp, DT_NODELABEL(temp_0), SENSOR_CHAN_AMBIENT_TEMP, 1);
DEFINE_SIMULATED_SENSOR(air_temp, DT_NODELABEL(temp_1), SENSOR_CHAN_AMBIENT_TEMP, 2);

static void update_emulators(const struct cistern_inputs *state)
{
	double level = CLAMP(state->level_mm, 0.0, 2500.0);
	double mount = CLAMP(state->pressure_sensor_height, 0.0, 2500.0);
	double zero = CLAMP(state->pressure_zero_voltage, 0.0, 3.3);
	double span = CLAMP(state->pressure_span_voltage, 0.0, 3.3);
	double fraction = CLAMP((level - mount) / 3000.0, 0.0, 1.0);
	uint32_t voltage_mv = (uint32_t)lround((zero + fraction * (span - zero)) * 1000.0);
	uint32_t adc_raw = (voltage_mv * ADC_MAX_RAW + ADC_VREF_MV / 2) / ADC_VREF_MV;
	uint32_t distance_mm = (uint32_t)MAX(0.0, state->ultrasonic_height - level);
	int32_t water_temp_milli = (int32_t)lround(state->water_temperature * 1000.0);
	int32_t air_temp_milli = (int32_t)lround(state->air_temperature * 1000.0);

	(void)adc_emul_const_value_set(pressure_adc.dev, pressure_adc.channel_id, adc_raw);
	k_mutex_lock(&simulated_values_mutex, K_FOREVER);
	simulated_values[0] = (int32_t)distance_mm;
	simulated_values[1] = water_temp_milli;
	simulated_values[2] = air_temp_milli;
	k_mutex_unlock(&simulated_values_mutex);
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
	} else if (strcmp(key, "pressure_sensor_height") == 0) {
		inputs.pressure_sensor_height = value;
	} else if (strcmp(key, "pressure_zero_voltage") == 0) {
		inputs.pressure_zero_voltage = value;
	} else if (strcmp(key, "pressure_span_voltage") == 0) {
		inputs.pressure_span_voltage = value;
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
