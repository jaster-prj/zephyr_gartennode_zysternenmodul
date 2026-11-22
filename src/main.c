/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(CONFIG_BOARD_NATIVE_SIM)
#define _POSIX_C_SOURCE 200809L
#include <time.h>
#endif

#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/settings/settings.h>
#include <zephyr/storage/flash_map.h>
#include <canopennode.h>
#include <string.h>

#include "led.h"
#include "sensor.h"
#include "update_control.h"
#include "update_diagnostics.h"
#if defined(CONFIG_BOARD_NATIVE_SIM)
#include "native_sim_cistern.h"
#endif

#define LOG_LEVEL CONFIG_CANOPENNODE_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

#define CAN_INTERFACE DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus))
#define CAN_BITRATE (DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bitrate, \
					  DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bus_speed, \
						     CONFIG_CAN_DEFAULT_BITRATE)) / 1000)

#if defined(CONFIG_BOARD_NATIVE_SIM)
K_SEM_DEFINE(native_sim_can_rx_wakeup, 0, 1);

static int64_t native_sim_monotonic_ms(void)
{
	struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);
	return (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

static void native_sim_can_bus_activity(const struct device *dev,
					struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(frame);
	ARG_UNUSED(user_data);
	k_sem_give(&native_sim_can_rx_wakeup);
}
#endif

static void initialize(void)
{
	if (led_init() != 0) {
		LOG_ERR("LED initialization failed");
	}
	if (sensor_init() != 0) {
		LOG_ERR("Sensor initialization failed");
	}
}

static CO_SDO_abortCode_t set_sensor_enable(CO_ODF_arg_t *arg,
					   int (*enable)(uint8_t))
{
	if (arg->reading) {
		return CO_SDO_AB_NONE;
	}
	if (arg->data == NULL) {
		return CO_SDO_AB_GENERAL;
	}
	return enable(*arg->data) == 0 ? CO_SDO_AB_NONE : CO_SDO_AB_GENERAL;
}

static CO_SDO_abortCode_t odf_2102(CO_ODF_arg_t *arg)
{
	return set_sensor_enable(arg, pressure_enable);
}

static CO_SDO_abortCode_t odf_2110(CO_ODF_arg_t *arg)
{
	return set_sensor_enable(arg, ultrasonic_enable);
}

static CO_SDO_abortCode_t odf_2115(CO_ODF_arg_t *arg)
{
	return set_sensor_enable(arg, temperature_enable);
}

/**
 * @brief Initialize CANopen and perform non-real-time application processing.
 */
int main(void)
{
	CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
	CO_ReturnError_t err;
	struct canopen_context can;
	uint16_t timeout;
	uint32_t elapsed;
	int64_t timestamp;
#ifdef CONFIG_CANOPENNODE_STORAGE
	int ret;
#endif

	can.dev = CAN_INTERFACE;
	if (!device_is_ready(can.dev)) {
		LOG_ERR("CAN interface not ready");
		return 0;
	}
#if defined(CONFIG_BOARD_NATIVE_SIM)
	native_sim_cistern_start();
#endif

#ifdef CONFIG_CANOPENNODE_STORAGE
	LOG_INF("Initializing settings subsystem");
	ret = settings_subsys_init();
	if (ret) {
		const struct flash_area *storage = NULL;
		int error;

		LOG_ERR("failed to initialize settings subsystem (err = %d)", ret);
		error = flash_area_open(PARTITION_ID(storage_partition), &storage);
		if (error < 0) {
			return error;
		}
		error = flash_area_erase(storage, 0, PARTITION_SIZE(storage_partition));
		flash_area_close(storage);
		if (error < 0) {
			return error;
		}
		sys_reboot(SYS_REBOOT_COLD);
	}
	ret = settings_load();
	if (ret) {
		LOG_ERR("failed to load settings (err = %d)", ret);
		return 0;
	}
#endif

	OD_powerOnCounter++;

	while (reset != CO_RESET_APP) {
		elapsed = 0U;

		LOG_INF("CANopen stack initialize");
		err = CO_init(&can, CONFIG_CANOPEN_NODE_ID, CAN_BITRATE);
		if (err != CO_ERROR_NO) {
			LOG_ERR("CO_init failed (err = %d)", err);
			return 0;
		}
		LOG_INF("CANopen stack initialized");
#ifdef CONFIG_CANOPENNODE_STORAGE
		canopen_storage_attach(CO->SDO[0], CO->em);
#endif

		initialize();
		led_configure(CO->NMT);
		CO_OD_configure(CO->SDO[0], OD_2102_pressureEnable,
				odf_2102, NULL, 0U, 0U);
		CO_OD_configure(CO->SDO[0], OD_2110_ultrasonicEnable,
				odf_2110, NULL, 0U, 0U);
		CO_OD_configure(CO->SDO[0], OD_2115_temperatureEnable,
				odf_2115, NULL, 0U, 0U);

		if (IS_ENABLED(CONFIG_CANOPENNODE_PROGRAM_DOWNLOAD)) {
			canopen_program_download_attach(CO->NMT, CO->SDO[0], CO->em);
			cistern_update_control_attach(CO->SDO[0]);
#if defined(CONFIG_CISTERN_UPDATE_DIAGNOSTICS)
			cistern_update_diagnostics_attach(CO->SDO[0]);
#endif
		}

#if defined(CONFIG_BOARD_NATIVE_SIM)
		/* Exercise pressure acquisition and TPDOs with the ADC emulator. */
		OD_pressureEnable = 1U;
#endif
		pressure_enable(OD_pressureEnable);
		ultrasonic_enable(OD_ultrasonicEnable);
		temperature_enable(OD_temperatureEnable);
		LOG_INF("Sensor acquisition tasks enabled (pressure=%u, ultrasonic=%u, temperature=%u)",
			OD_pressureEnable, OD_ultrasonicEnable, OD_temperatureEnable);

#if defined(CONFIG_BOARD_NATIVE_SIM)
		const struct can_filter native_sim_wakeup_filter = {
			.id = 0U,
			.mask = 0U,
		};
		int native_sim_wakeup_filter_id =
			can_add_rx_filter(can.dev, native_sim_can_bus_activity, NULL,
					  &native_sim_wakeup_filter);

		if (native_sim_wakeup_filter_id < 0) {
			LOG_ERR("Failed to add native-sim CAN activity filter: %d",
				native_sim_wakeup_filter_id);
		}
#endif
		CO_CANsetNormalMode(CO->CANmodule[0]);
		LOG_INF("CAN interface entered normal mode");
#ifdef CONFIG_CANOPENNODE_STORAGE
		struct sCO_OD_EEPROM eeprom_old = CO_OD_EEPROM;
#endif

		while (true) {
			timeout = 1U;
#if defined(CONFIG_BOARD_NATIVE_SIM)
			timestamp = native_sim_monotonic_ms();
#else
			timestamp = k_uptime_get();
#endif
			reset = CO_process(CO, (uint16_t)elapsed, &timeout);
			if (reset != CO_RESET_NOT) {
				break;
			}
#if defined(CONFIG_BOARD_NATIVE_SIM)
			/*
			 * Process asynchronous PDOs here: native_sim does not reliably
			 * run the CANopenNode timer-driven sync worker.
			 */
			native_sim_cistern_process();
#endif
			sensor_process();
#if defined(CONFIG_BOARD_NATIVE_SIM)
			CO_LOCK_OD();
			CO_process_RPDO(CO, false);
			CO_process_TPDO(CO, false, elapsed * 1000U);
			CO_UNLOCK_OD();
#endif

			if (timeout > 0) {
#ifdef CONFIG_CANOPENNODE_STORAGE
				ret = memcmp(&eeprom_old, &CO_OD_EEPROM, sizeof(CO_OD_EEPROM));
				if (ret != 0) {
					LOG_DBG("EEPROM changed, saving to flash");
					memcpy(&eeprom_old, &CO_OD_EEPROM, sizeof(CO_OD_EEPROM));
					ret = canopen_storage_save(CANOPEN_STORAGE_EEPROM);
					if (ret) {
						LOG_ERR("failed to save EEPROM");
					}
				}
#endif
				/* Sleep for the requested timeout and measure elapsed time. */
#if defined(CONFIG_BOARD_NATIVE_SIM)
				/* Keep stack time advancing despite coarse native clock ticks. */
				uint32_t wait_ms = MAX(timeout, 10U);

				(void)k_sem_take(&native_sim_can_rx_wakeup, K_MSEC(wait_ms));
				elapsed = (uint32_t)(native_sim_monotonic_ms() - timestamp);
				if (elapsed == 0U) {
					elapsed = 1U;
				}
#else
				k_sleep(K_MSEC(timeout));
				elapsed = (uint32_t)k_uptime_delta(&timestamp);
#endif
			} else {
				elapsed = 0U;
#if defined(CONFIG_BOARD_NATIVE_SIM)
				/* Let native UDP-CAN RX and virtual sensor tasks run. */
				k_sleep(K_MSEC(1));
				elapsed = (uint32_t)(native_sim_monotonic_ms() - timestamp);
				if (elapsed == 0U) {
					elapsed = 1U;
				}
#endif
			}
		}

		sensor_stop();
#if defined(CONFIG_BOARD_NATIVE_SIM)
		if (native_sim_wakeup_filter_id >= 0) {
			can_remove_rx_filter(can.dev, native_sim_wakeup_filter_id);
		}
#endif
		if (reset == CO_RESET_COMM) {
			LOG_INF("Resetting communication");
		}
	}

	LOG_INF("Resetting device");
	CO_delete(&can);
	sys_reboot(SYS_REBOOT_COLD);
}
