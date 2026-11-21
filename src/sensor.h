#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

int sensor_init(void);
int pressure_enable(uint8_t value);
int ultrasonic_enable(uint8_t value);
int temperature_enable(uint8_t value);
void sensor_process(void);
void sensor_stop(void);

#endif /* SENSOR_H */
