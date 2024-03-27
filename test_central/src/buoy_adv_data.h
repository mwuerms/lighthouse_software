/**
 * Martin Egli
 * 2024-03-27
 * buoy data advertising data module
 */
#ifndef _BUOY_ADV_DATA_H_
#define _BUOY_ADV_DATA_H_

#include <stdint.h>

// __packed__ ?
#define BUOY_ADV_DATA_VERSION_01_LENGTH (9)
#define BUOY_ADV_DATA_VERSION_01 (0x01)
struct buoy_adv_data_version_01_struct {
	uint8_t length;
	uint8_t version;
	uint16_t serial_number;
	uint16_t interrupt_counter;
	uint16_t input_states;
	uint16_t vbat_mV;
};

#endif // _BUOY_ADV_DATA_H_
