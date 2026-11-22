#include "native_sim_ds18b20.h"

#include <errno.h>

#include <zephyr/sys/util.h>

#if defined(CONFIG_BOARD_NATIVE_SIM)
#include <math.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/serial/uart_emul.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/w1.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(native_sim_ds18b20, LOG_LEVEL_INF);

#define SENSOR_COUNT 2
#define ROM_BYTES 8
#define SCRATCHPAD_BYTES 9
#define ROM_SEARCH 0xF0
#define MATCH_ROM 0x55
#define SKIP_ROM 0xCC
#define READ_ROM 0x33
#define CONVERT_T 0x44
#define READ_SCRATCHPAD 0xBE
#define WRITE_SCRATCHPAD 0x4E
#define READ_POWER 0xB4

enum bus_phase {
	PHASE_ROM_COMMAND,
	PHASE_SEARCH,
	PHASE_MATCH_ROM,
	PHASE_FUNCTION_COMMAND,
	PHASE_WRITE_SCRATCHPAD,
	PHASE_READ_ROM,
	PHASE_READ_SCRATCHPAD,
	PHASE_READ_POWER,
};

struct bus_state {
	enum bus_phase phase;
	uint8_t input_byte;
	uint8_t input_bit;
	uint8_t match_rom[ROM_BYTES];
	uint8_t match_index;
	uint8_t search_bit;
	uint8_t search_slot;
	uint8_t read_index;
	uint8_t read_bit;
	uint8_t active_mask;
	uint8_t selected_mask;
	uint8_t scratchpad[SENSOR_COUNT][SCRATCHPAD_BYTES];
	uint8_t rom[SENSOR_COUNT][ROM_BYTES];
};

static struct bus_state bus = {
	.rom = {
		{ 0x28, 0x01, 0, 0, 0, 0, 0, 0x29 },
		{ 0x28, 0x02, 0, 0, 0, 0, 0, 0x70 },
	},
};
static atomic_t temperatures_q4[SENSOR_COUNT] = {
	ATOMIC_INIT(18 * 16),
	ATOMIC_INIT(20 * 16),
};
static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart_emul));
static const struct device *const w1_dev = DEVICE_DT_GET(DT_NODELABEL(w1_0));
static K_MUTEX_DEFINE(model_mutex);

static void model_reset(void)
{
	bus.phase = PHASE_ROM_COMMAND;
	bus.input_byte = 0;
	bus.input_bit = 0;
	bus.match_index = 0;
	bus.search_bit = 0;
	bus.search_slot = 0;
	bus.read_index = 0;
	bus.read_bit = 0;
	bus.active_mask = BIT(0) | BIT(1);
	bus.selected_mask = BIT(0) | BIT(1);
}

static void prepare_scratchpad(size_t index)
{
	int16_t raw = (int16_t)atomic_get(&temperatures_q4[index]);
	uint8_t *sp = bus.scratchpad[index];

	sp[0] = (uint8_t)raw;
	sp[1] = (uint8_t)((uint16_t)raw >> 8);
	sp[2] = 75;
	sp[3] = 70;
	sp[4] = 0x7F;
	sp[5] = 0xFF;
	sp[6] = 0x0C;
	sp[7] = 0x10;
	sp[8] = w1_crc8(sp, 8);
}

static void received_byte(uint8_t byte)
{
	switch (bus.phase) {
	case PHASE_ROM_COMMAND:
		switch (byte) {
		case ROM_SEARCH:
			bus.phase = PHASE_SEARCH;
			bus.active_mask = BIT(0) | BIT(1);
			bus.search_bit = 0;
			bus.search_slot = 0;
			break;
		case MATCH_ROM:
			bus.phase = PHASE_MATCH_ROM;
			bus.match_index = 0;
			memset(bus.match_rom, 0, sizeof(bus.match_rom));
			break;
		case SKIP_ROM:
			bus.selected_mask = BIT(0) | BIT(1);
			bus.phase = PHASE_FUNCTION_COMMAND;
			break;
		case READ_ROM:
			bus.phase = PHASE_READ_ROM;
			bus.selected_mask = bus.active_mask;
			bus.read_index = 0;
			bus.read_bit = 0;
			break;
		default:
			bus.selected_mask = 0;
			break;
		}
		break;
	case PHASE_MATCH_ROM:
		bus.match_rom[bus.match_index++] = byte;
		if (bus.match_index == ROM_BYTES) {
			bus.selected_mask = 0;
			for (size_t i = 0; i < SENSOR_COUNT; i++) {
				if (memcmp(bus.match_rom, bus.rom[i], ROM_BYTES) == 0) {
					bus.selected_mask |= BIT(i);
				}
			}
			bus.phase = bus.selected_mask ? PHASE_FUNCTION_COMMAND : PHASE_ROM_COMMAND;
		}
		break;
	case PHASE_FUNCTION_COMMAND:
		switch (byte) {
		case CONVERT_T:
			/* Virtual conversion completes immediately. */
			bus.phase = PHASE_ROM_COMMAND;
			break;
		case READ_SCRATCHPAD:
			for (size_t i = 0; i < SENSOR_COUNT; i++) {
				if (bus.selected_mask & BIT(i)) {
					prepare_scratchpad(i);
				}
			}
			bus.phase = PHASE_READ_SCRATCHPAD;
			bus.read_index = 0;
			bus.read_bit = 0;
			break;
		case WRITE_SCRATCHPAD:
			bus.phase = PHASE_WRITE_SCRATCHPAD;
			bus.match_index = 0;
			break;
		case READ_POWER:
			bus.phase = PHASE_READ_POWER;
			bus.read_index = 0;
			bus.read_bit = 0;
			break;
		default:
			bus.phase = PHASE_ROM_COMMAND;
			break;
		}
		break;
	case PHASE_WRITE_SCRATCHPAD:
		for (size_t i = 0; i < SENSOR_COUNT; i++) {
			if (bus.selected_mask & BIT(i)) {
				bus.scratchpad[i][2 + bus.match_index] = byte;
			}
		}
		if (++bus.match_index == 3) {
			bus.phase = PHASE_FUNCTION_COMMAND;
			bus.match_index = 0;
		}
		break;
	default:
		break;
	}
}

static uint8_t search_bit_value(bool complement)
{
	bool wired_and = true;
	bool participating = false;

	for (size_t i = 0; i < SENSOR_COUNT; i++) {
		if (!(bus.active_mask & BIT(i))) {
			continue;
		}
		participating = true;
		bool bit = (bus.rom[i][bus.search_bit / 8] >>
			    (bus.search_bit % 8)) & 1;
		wired_and &= complement ? !bit : bit;
	}
	return participating && wired_and ? 0xFF : 0x00;
}

static uint8_t read_slot(void)
{
	uint8_t result = 0xFF;

	if (bus.phase == PHASE_SEARCH) {
		result = search_bit_value(bus.search_slot == 1);
		bus.search_slot++;
		return result;
	}

	if (bus.phase == PHASE_READ_POWER) {
		if (++bus.read_bit == 8) {
			bus.read_bit = 0;
			bus.phase = PHASE_FUNCTION_COMMAND;
		}
		return 0xFF; /* Both simulated sensors use external power. */
	}

	for (size_t i = 0; i < SENSOR_COUNT; i++) {
		uint8_t value;

		if (!(bus.selected_mask & BIT(i))) {
			continue;
		}
		value = bus.phase == PHASE_READ_ROM
			? bus.rom[i][bus.read_index]
			: bus.scratchpad[i][bus.read_index];
		result &= ((value >> bus.read_bit) & 1) ? 0xFF : 0x00;
	}
	if (++bus.read_bit == 8) {
		bus.read_bit = 0;
		bus.read_index++;
		if ((bus.phase == PHASE_READ_ROM && bus.read_index == ROM_BYTES) ||
		    (bus.phase == PHASE_READ_SCRATCHPAD &&
		     bus.read_index == SCRATCHPAD_BYTES)) {
			bus.phase = bus.phase == PHASE_READ_ROM
				? PHASE_ROM_COMMAND : PHASE_FUNCTION_COMMAND;
		}
	}
	return result;
}

static uint8_t exchange_slot(uint8_t tx, uint32_t baudrate)
{
	bool released = tx == 0xFF;

	if (baudrate == 9600 && tx == 0xF0) {
		model_reset();
		return 0xE0;
	}
	if (bus.phase == PHASE_SEARCH) {
		if (bus.search_slot == 2) {
			for (size_t i = 0; i < SENSOR_COUNT; i++) {
				bool bit;

				if (!(bus.active_mask & BIT(i))) {
					continue;
				}
				bit = (bus.rom[i][bus.search_bit / 8] >>
				       (bus.search_bit % 8)) & 1;
				if (bit != released) {
					bus.active_mask &= (uint8_t)~BIT(i);
				}
			}
			bus.search_slot = 0;
			if (++bus.search_bit == 64) {
				bus.selected_mask = bus.active_mask;
				bus.phase = bus.active_mask
					? PHASE_FUNCTION_COMMAND : PHASE_ROM_COMMAND;
			}
			return released ? 0xFF : 0x00;
		}
		return read_slot();
	}
	if (bus.phase == PHASE_READ_ROM || bus.phase == PHASE_READ_SCRATCHPAD ||
	    bus.phase == PHASE_READ_POWER) {
		return read_slot();
	}
	if (released) {
		bus.input_byte |= BIT(bus.input_bit);
	}
	if (++bus.input_bit == 8) {
		uint8_t byte = bus.input_byte;

		bus.input_byte = 0;
		bus.input_bit = 0;
		received_byte(byte);
	}
	return 0xFF;
}

static void uart_tx_ready(const struct device *dev, size_t size, void *user_data)
{
	uint8_t tx[128];
	uint8_t rx[128];
	struct uart_config config;
	uint32_t count;

	ARG_UNUSED(user_data);
	if (uart_config_get(dev, &config) != 0) {
		return;
	}
	count = uart_emul_get_tx_data(dev, tx, MIN(size, sizeof(tx)));
	k_mutex_lock(&model_mutex, K_FOREVER);
	for (uint32_t i = 0; i < count; i++) {
		rx[i] = exchange_slot(tx[i], config.baudrate);
	}
	k_mutex_unlock(&model_mutex);
	(void)uart_emul_put_rx_data(dev, rx, count);
}

static void count_ds18b20(struct w1_rom rom, void *user_data)
{
	size_t *count = user_data;

	if (rom.family == 0x28 && rom.crc == w1_crc8((const uint8_t *)&rom, 7)) {
		(*count)++;
	}
}

int native_sim_ds18b20_init(void)
{
	size_t found_count = 0;

	if (!device_is_ready(uart_dev) || !device_is_ready(w1_dev)) {
		return -ENODEV;
	}
	model_reset();
	uart_emul_callback_tx_data_ready_set(uart_dev, uart_tx_ready, NULL);
	int found = w1_search_rom(w1_dev, count_ds18b20, &found_count);

	if (found != SENSOR_COUNT || found_count != SENSOR_COUNT) {
		LOG_ERR("Virtual W1 search returned %d devices (%u valid DS18B20s)",
			found, (unsigned int)found_count);
		return -EIO;
	}
	for (size_t i = 0; i < SENSOR_COUNT; i++) {
		struct w1_slave_config peripheral = {0};
		uint8_t scratchpad[SCRATCHPAD_BYTES];
		int16_t expected = (int16_t)atomic_get(&temperatures_q4[i]);

		memcpy(&peripheral.rom, bus.rom[i], ROM_BYTES);
		if (w1_reset_select(w1_dev, &peripheral) != 0 ||
		    w1_write_byte(w1_dev, WRITE_SCRATCHPAD) != 0 ||
		    w1_write_byte(w1_dev, 75) != 0 ||
		    w1_write_byte(w1_dev, 70) != 0 ||
		    w1_write_byte(w1_dev, 0x7F) != 0 ||
		    w1_reset_select(w1_dev, &peripheral) != 0 ||
		    w1_write_byte(w1_dev, CONVERT_T) != 0 ||
		    w1_reset_select(w1_dev, &peripheral) != 0 ||
		    w1_write_byte(w1_dev, READ_SCRATCHPAD) != 0 ||
		    w1_read_block(w1_dev, scratchpad, sizeof(scratchpad)) != 0 ||
		    scratchpad[8] != w1_crc8(scratchpad, 8) ||
		    (int16_t)((uint16_t)scratchpad[1] << 8 | scratchpad[0]) != expected) {
			LOG_ERR("Virtual DS18B20 %u scratchpad/protocol validation failed",
				(unsigned int)i);
			return -EIO;
		}
	}
	LOG_INF("Two virtual DS18B20 devices respond on W1");
	return 0;
}

int native_sim_ds18b20_set_roms(uint64_t water_rom, uint64_t air_rom)
{
	struct w1_rom roms[SENSOR_COUNT];
	uint64_t values[SENSOR_COUNT] = { water_rom, air_rom };

	for (size_t i = 0; i < SENSOR_COUNT; i++) {
		w1_uint64_to_rom(values[i], &roms[i]);
		if (roms[i].family != 0x28 ||
		    roms[i].crc != w1_crc8((const uint8_t *)&roms[i], 7)) {
			return -EINVAL;
		}
	}
	if (water_rom == air_rom || w1_lock_bus(w1_dev) != 0) {
		return -EINVAL;
	}
	k_mutex_lock(&model_mutex, K_FOREVER);
	memcpy(bus.rom[0], &roms[0], ROM_BYTES);
	memcpy(bus.rom[1], &roms[1], ROM_BYTES);
	model_reset();
	k_mutex_unlock(&model_mutex);
	(void)w1_unlock_bus(w1_dev);
	return 0;
}

void native_sim_ds18b20_set_temperature(size_t index, double temperature_c)
{
	if (index >= SENSOR_COUNT || !isfinite(temperature_c)) {
		return;
	}
	temperature_c = CLAMP(temperature_c, -55.0, 125.0);
	atomic_set(&temperatures_q4[index], (atomic_val_t)lround(temperature_c * 16.0));
}

#else
int native_sim_ds18b20_init(void)
{
	return -ENOTSUP;
}

void native_sim_ds18b20_set_temperature(size_t index, double temperature_c)
{
	ARG_UNUSED(index);
	ARG_UNUSED(temperature_c);
}

int native_sim_ds18b20_set_roms(uint64_t water_rom, uint64_t air_rom)
{
	ARG_UNUSED(water_rom);
	ARG_UNUSED(air_rom);
	return -ENOTSUP;
}

#endif
