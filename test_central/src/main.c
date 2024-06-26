/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

#include "usb_uart.h"
#include "timestamp.h"
#include "buoy_adv_data.h"

static void start_scan(void);

static struct bt_conn *default_conn;

// Adv_data: 0E:25:70:67:35:05 (random), rssi: -37, adv_data (hex, 25):0C0962756F795F626561636F6E0BFF0901CDABD2040100E40C
/*
0C (0C-1 = 0B)
09
62756F795F626561636F6E

0BFF0901CDABD2040100E40C
*/
#define BUOY_BEACON_DEVICE_NAME "buoy_beacon"
static const char buoy_bt_data_name_complete[] = BUOY_BEACON_DEVICE_NAME;
static int16_t filter_buoy_adv_data(struct net_buf_simple *ad) {
	uint8_t *adv_data = ad->data;
	uint8_t len, type, n, filter_res;
	// sanity checks
	if (ad == NULL) {
		// error
		return 0;
	}
	if (ad->len < strlen(buoy_bt_data_name_complete)) {
		// error, too short
		return 0;
	}
	// looking for BT_DATA_NAME_COMPLETE, buoy_bt_data_name_complete
	filter_res = 0;
	len = adv_data[0];
	type = adv_data[1];
	if (len == 0x0C) {
		if (type == BT_DATA_NAME_COMPLETE) {
			for(n = 0; n < (len-1); n++) {
				if(adv_data[2+n] == buoy_bt_data_name_complete[n]) {
					filter_res = 1;
				}
				else {
					filter_res = 0;
					break;
				}
			}
		}
	}
	if (filter_res == 0) {
		// error, buoy_bt_data_name_complete not found
		return 0;
	}
	return 1; // OK
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	int err;

	if (default_conn) {
		return;
	}

	/* We're only interested in connectable events * /
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}*/

	if(filter_buoy_adv_data(ad) == 0) {
		// not my data, continue looking
		return;
	}

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	printk("Device found: %s (RSSI %d)\n", addr_str, rssi);

	//usb_uart_print_adv_data("Adv_data: ", addr_str, rssi, ad->data, ad->len);
	uint32_t now_s, now_ms;
	timestamp_get_counter(&now_s, &now_ms);
	usb_uart_print_adv_data_timestamp(now_s, now_ms, "Adv_data: ", addr_str, rssi, ad->data, ad->len);
}

static void start_scan(void)
{
	int err;

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	//err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s (%u)\n", addr, err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	if (conn != default_conn) {
		return;
	}

	printk("Connected: %s\n", addr);

	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;

	start_scan();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

int main(void)
{
	int err;
	
	usb_uart_start();
	usb_uart_print_message("this is test_central, looking for buoy adv data, filter and print\n");
	//usb_uart_print_message(model);
	usb_uart_print_message("\n\n");

	timestamp_start_counter();
	//usb_uart_print_data_types();

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	start_scan();
	return 0;
}
