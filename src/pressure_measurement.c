#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <canopennode.h>

#include "pressure_measurement.h"

LOG_MODULE_REGISTER(pressure_measurement, CONFIG_CANOPENNODE_LOG_LEVEL);

#define DT_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)
};

int pressure_measurement_init(void)
{
	if (!adc_is_ready_dt(&adc_channels[0])) {
		LOG_ERR("Pressure ADC controller not ready");
		return -ENODEV;
	}
	return adc_channel_setup_dt(&adc_channels[0]);
}

int pressure_measurement_read(void)
{
	uint16_t raw;
	struct adc_sequence sequence = {
		.buffer = &raw,
		.buffer_size = sizeof(raw),
	};
	int ret = adc_sequence_init_dt(&adc_channels[0], &sequence);

	if (ret == 0) {
		ret = adc_read_dt(&adc_channels[0], &sequence);
	}
	if (ret < 0) {
		LOG_ERR("Could not read pressure ADC: %d", ret);
		return ret;
	}

	CO_LOCK_OD();
	OD_pressureMeasurement = raw;
	if (OD_pressureDistanceMax != 0U) {
		OD_pressureDistance = (uint32_t)(OD_pressureDistanceMax / 4096U * raw);
	}
	CO_UNLOCK_OD();
	return 0;
}
