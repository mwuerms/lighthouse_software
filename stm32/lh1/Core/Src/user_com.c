/*
 * user_com.c
 *
 *  Created on: Sep 24, 2025
 *      Author: martin
 */

#include <string.h>
#include <stdbool.h>
#include <usbd_cdc_if.h>

#include "version.h"
#include "user_com.h"
#include "str_buf.h"

#define USER_COM_STR_BUF_SIZE (1024)
char user_com_str_buf[USER_COM_STR_BUF_SIZE];
uint16_t user_com_str_len;

#define CMD_GET_HELP_0 (0)
#define CMD_GET_HELP_1 (1)
#define CMD_GET_HELP_2 (2)
#define CMD_GET_TIME (3)
#define CMD_GET_DAY  (4)
#define CMD_SET_TIME (5)
#define CMD_SET_DAY  (6)
#define CMD_MAX_INDEX CMD_SET_DAY

#define INVALID_COMMAND (0xFFFF)

#define CMD_SIZE (16)
static const char commands[][CMD_SIZE] = {
	"?", // CMD_GET_HELP_0
	"h", // CMD_GET_HELP_1
	"help", // CMD_GET_HELP_2
	"get time", // CMD_GET_TIME
	"get day",  // CMD_GET_DAY
	"set time", // CMD_SET_TIME
	"set day",  // CMD_SET_DAY
};

static inline uint32_t get_command_len(uint16_t cmd_index) {
	// check for valid cmd_index
	return strlen(commands[cmd_index]);
}

static uint16_t compare_command(uint16_t cmd_index, uint8_t *buf, uint32_t len) {
	uint16_t n, cmd_len;
	cmd_len = get_command_len(cmd_index);
	if(len > cmd_len) {
		return false;
	}
	// strcmp does not work here, check from beginning up to cmd_len
	for(n = 0; n < cmd_len; n++) {
		if(buf[n] != commands[cmd_index][n]) {
			return false;
		}
	}
	// OK cmd found
	return true;
}

static uint16_t parse_command(uint8_t *buf, uint32_t len) {
	uint16_t n;
	for(n = 0; n < CMD_MAX_INDEX; n++) {
		if(compare_command(n, buf, len) == true) {
			return n;
		}
	}
	return INVALID_COMMAND;
}

static void send_invalid_command(void) {
	user_com_str_len = str_buf_clear(user_com_str_buf, USER_COM_STR_BUF_SIZE);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, "invalid command, send ?, h or help for help");
	CDC_Transmit_FS((uint8_t *)user_com_str_buf, user_com_str_len);
}

static void send_help(void) {
	user_com_str_len = str_buf_clear(user_com_str_buf, USER_COM_STR_BUF_SIZE);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, PROGRAM_NAME);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, " - ");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, PROGRAM_DESCRIPTION);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, "\nversion ");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, VERSION_STRING);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", ");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, VERSION_DATE);
	//CDC_Transmit_FS((uint8_t *)user_com_str_buf, user_com_str_len);

	//user_com_str_len = str_buf_clear(user_com_str_buf, USER_COM_STR_BUF_SIZE);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, "list of valid commands:\n");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, commands[CMD_GET_HELP_0]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", ");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, commands[CMD_GET_HELP_1]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", ");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, commands[CMD_GET_HELP_2]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ": display this help\n");
	//CDC_Transmit_FS((uint8_t *)user_com_str_buf, user_com_str_len);

	//user_com_str_len = str_buf_clear(user_com_str_buf, USER_COM_STR_BUF_SIZE);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, commands[CMD_GET_TIME]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ": get present time in format hh:mm:ss\n");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, commands[CMD_GET_DAY]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ": get present day: MON:1, TUE:2, WEN:3, THU:4, FRY:5, SAT:6, SUN:7\n");
	//CDC_Transmit_FS((uint8_t *)user_com_str_buf, user_com_str_len);

	//user_com_str_len = str_buf_clear(user_com_str_buf, USER_COM_STR_BUF_SIZE);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, commands[CMD_SET_TIME]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ": set present time in format hh:mm:ss\n");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, commands[CMD_GET_DAY]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ": set present day: MON:1, TUE:2, WEN:3, THU:4, FRY:5, SAT:6, SUN:7\n");
	CDC_Transmit_FS((uint8_t *)user_com_str_buf, user_com_str_len);
}

static void send_present_day_time(void) {
	user_com_str_len = str_buf_clear(user_com_str_buf, USER_COM_STR_BUF_SIZE);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, "present time: ");
	user_com_str_len = str_buf_append_uint16(user_com_str_buf, USER_COM_STR_BUF_SIZE, 12);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ":");
	user_com_str_len = str_buf_append_uint16(user_com_str_buf, USER_COM_STR_BUF_SIZE, 34);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ":");
	user_com_str_len = str_buf_append_uint16(user_com_str_buf, USER_COM_STR_BUF_SIZE, 56);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", day: ");
	user_com_str_len = str_buf_append_uint16(user_com_str_buf, USER_COM_STR_BUF_SIZE, 7);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ":SUN");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, "\n");
	CDC_Transmit_FS((uint8_t *)user_com_str_buf, user_com_str_len);
}

void user_com_parse(uint8_t *buf, uint32_t len) {
	uint16_t cmd = parse_command(buf, len);
	switch(cmd) {
	case INVALID_COMMAND:
		send_invalid_command();
	case CMD_GET_HELP_0:
	case CMD_GET_HELP_1:
	case CMD_GET_HELP_2:
		send_help();
		break;
	case CMD_GET_TIME:
		send_present_day_time();
		break;
	case CMD_GET_DAY:
		send_present_day_time();
		break;
	default:
		char error_msg[] = "?!\n";
		CDC_Transmit_FS((uint8_t *)error_msg, strlen(error_msg));
	}
}
