#ifndef OSSM_SOFTWARE_CURRENT_SENSOR_H
#define OSSM_SOFTWARE_CURRENT_SENSOR_H

#include <stdint.h>

// INA219 current sensor on the shared display I2C bus.
// Returns motor current in milliamps (absolute value).

void initCurrentSensor();

// Read the bus current averaged over `samples` reads (1ms delay between
// samples). Returns mA. Returns 0 if the sensor failed to initialize.
float getCurrentMilliAmps(uint16_t samples);

#endif  // OSSM_SOFTWARE_CURRENT_SENSOR_H
