#ifndef SENSOR_FILTER_H
#define SENSOR_FILTER_H

#include <stdint.h>

#include "system_state.h"

#define SENSOR_FILTER_INSTANCE_COUNT 2U

void sensor_filter_reset_all(void);

/* sensor_index: 0 = cam bien 1, 1 = cam bien 2 */
void sensor_filter_commit(uint8_t sensor_index, distance_data_t *d);

#endif
