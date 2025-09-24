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

#define USER_COM_STR_BUF_SIZE (256)
char user_com_str_buf[USER_COM_STR_BUF_SIZE];
uint16_t user_com_str_len;

static void send_program_information(void) {
	user_com_str_len = str_buf_clear(user_com_str_buf, USER_COM_STR_BUF_SIZE);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, PROGRAM_NAME);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, " - ");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, PROGRAM_DESCRIPTION);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, "\nversion ");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, VERSION_STRING);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", ");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, VERSION_DATE);
	CDC_Transmit_FS((uint8_t *)user_com_str_buf, user_com_str_len);
}

#define INVALID_COMMAND (0xFFFF)
#define CMD_GET_INFO (0)
#define CMD_GET_ALL (1)

#define CMD_SIZE (16)
static char commands[][CMD_SIZE] = {
	"?", // CMD_GET_INFO
	"get all" // CMD_GET_ALL
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
	if(compare_command(CMD_GET_INFO, buf, len) == true)
		return CMD_GET_INFO;
	if(compare_command(CMD_GET_ALL, buf, len) == true)
			return CMD_GET_ALL;
	return INVALID_COMMAND;
}

void user_com_parse(uint8_t *buf, uint32_t len) {
	uint16_t cmd = parse_command(buf, len);
	switch(cmd) {
	case CMD_GET_INFO:
		send_program_information();
		break;
	case CMD_GET_ALL:
		user_com_str_len = str_buf_clear(user_com_str_buf, USER_COM_STR_BUF_SIZE);
		user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, "ja was den, Rogntütüüü");
		CDC_Transmit_FS((uint8_t *)user_com_str_buf, user_com_str_len);
		break;
	case INVALID_COMMAND:
	default:
		char error_msg[] = "?!\n";
		CDC_Transmit_FS((uint8_t *)error_msg, strlen(error_msg));
	}
}
