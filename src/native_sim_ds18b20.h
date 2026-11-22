#ifndef CISTERN_NATIVE_SIM_DS18B20_H_
#define CISTERN_NATIVE_SIM_DS18B20_H_

#include <stddef.h>
#include <stdint.h>

int native_sim_ds18b20_init(void);
void native_sim_ds18b20_set_temperature(size_t index, double temperature_c);
int native_sim_ds18b20_set_roms(uint64_t water_rom, uint64_t air_rom);

#endif
