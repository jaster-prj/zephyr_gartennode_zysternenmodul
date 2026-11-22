#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <canopennode.h>

#include "ultrasonic_measurement.h"

LOG_MODULE_REGISTER(ultrasonic_measurement, CONFIG_CANOPENNODE_LOG_LEVEL);

static const struct device *const ultrasonic = DEVICE_DT_GET(DT_NODELABEL(us0));

int ultrasonic_measurement_init(void)
{
	return device_is_ready(ultrasonic) ? 0 : -ENODEV;
}

int ultrasonic_measurement_read(void)
{
	struct sensor_value distance;
	int ret = sensor_sample_fetch_chan(ultrasonic, SENSOR_CHAN_ALL);

	if (ret == 0) {
		ret = sensor_channel_get(ultrasonic, SENSOR_CHAN_DISTANCE, &distance);
	}
	if (ret != 0) {
		LOG_ERR("Could not read ultrasonic sensor: %d", ret);
		return ret;
	}

	CO_LOCK_OD();
	int64_t measured_mm = (int64_t)distance.val1 * 1000 + distance.val2 / 1000;
	OD_ultrasonicMeasurement = measured_mm > 0 ? (uint32_t)measured_mm : 0U;
	OD_ultrasonicDistanceMm = OD_ultrasonicOffsetMm > OD_ultrasonicMeasurement
					  ? OD_ultrasonicOffsetMm - OD_ultrasonicMeasurement
					  : 0U;
	CO_UNLOCK_OD();
	return 0;
}
