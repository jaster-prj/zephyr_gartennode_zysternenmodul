#include <stdint.h>

#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <canopennode.h>

#include "pressure_measurement.h"

LOG_MODULE_REGISTER(pressure_measurement, CONFIG_CANOPENNODE_LOG_LEVEL);

#define DT_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx),
#define ADC_REFERENCE_MV 3300U
#define ADC_COUNTS_PER_FULL_SCALE 4096U

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
	if (raw == 0U || OD_pressureSpanMv <= OD_pressureZeroMv ||
	    OD_pressureMeasurementRangeMm == 0U) {
		OD_pressureDistanceMm = UINT32_MAX;
	} else {
		uint32_t voltage_mv = (uint32_t)raw * ADC_REFERENCE_MV /
				      ADC_COUNTS_PER_FULL_SCALE;
		uint32_t zero_mv = OD_pressureZeroMv;
		uint32_t span_mv = OD_pressureSpanMv;
		uint32_t measured_mv = voltage_mv > zero_mv ? voltage_mv - zero_mv : 0U;
		uint32_t calibration_span_mv = span_mv - zero_mv;
		if (measured_mv > calibration_span_mv) {
			measured_mv = calibration_span_mv;
		}
		uint64_t pressure_depth_mm = (uint64_t)measured_mv *
					     OD_pressureMeasurementRangeMm /
					     calibration_span_mv;
		uint64_t water_level_mm = (uint64_t)OD_pressureOffsetMm +
					  pressure_depth_mm;

		OD_pressureDistanceMm = (uint32_t)MIN(water_level_mm, UINT32_MAX);
	}
	CO_UNLOCK_OD();
	return 0;
}
