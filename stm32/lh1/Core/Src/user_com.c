/*
 * user_com.c
 *
 *  Created on: Sep 24, 2025
 *      Author: martin
 */

#include <string.h>
#include <usbd_cdc_if.h>

#include "version.h"
#include "user_com.h"



void user_com_parse(uint8_t *buf, uint32_t len) {
	if(buf[0] == '?') {
		char msg[] = "Dr. Lighthouse\n";
		CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
	}
}
