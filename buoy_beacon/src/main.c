/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include "buoy_adv_data.h"

// - compile advertising data for buoy beacon
// #define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME "buoy_beacon"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

// static array, little endian
static struct buoy_adv_data_version_01_struct buoy_adv_data = {
	.length = BUOY_ADV_DATA_VERSION_01_LENGTH,
	.version = BUOY_ADV_DATA_VERSION_01,
	.serial_number = 0xABCD,
	.interrupt_counter = 1234,
	.input_states = 0x0001,
	.vbat_mV = 3300,
};

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, (uint8_t *)&buoy_adv_data, sizeof(buoy_adv_data)),
};

static void bt_ready(int err)
{
	char addr_s[BT_ADDR_LE_STR_LEN];
	bt_addr_le_t addr = {0};
	size_t count = 1;

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	/* Start advertising */
#if 0
	err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}
#endif
	// start advertising without scan response
	err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}


	/* For connectable advertising you would use
	 * bt_le_oob_get_local().  For non-connectable non-identity
	 * advertising an non-resolvable private address is used;
	 * there is no API to retrieve that.
	 */

	bt_id_get(&addr, &count);
	bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));

	printk("BUOY Beacon started, advertising as %s\n", addr_s);
}

int main(void)
{
	int err;

	printk("Starting RARARA Beacon Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
	return 0;
}
