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
#include "rtc.h"

#define USER_COM_STR_BUF_SIZE (1024)
char user_com_str_buf[USER_COM_STR_BUF_SIZE];
uint16_t user_com_str_len;

#define CMD_GET_HELP_0 (0)
#define CMD_GET_HELP_1 (1)
#define CMD_GET_HELP_2 (2)
#define CMD_GET_TIME (3)
#define CMD_GET_DATE (4)
#define CMD_SET_TIME (5)
#define CMD_SET_DATE (6)
#define CMD_SET_WEEKDAY (7)
#define CMD_LIST_ALARMS (8)
#define CMD_SET_ALARM (9)
#define CMD_RESTORE (10)
#define CMD_MAX_INDEX CMD_RESTORE

#define INVALID_COMMAND (0xFFFF)

#define CMD_SIZE (16)
static const char commands[][CMD_SIZE] = {
	"?", // CMD_GET_HELP_0
	"h", // CMD_GET_HELP_1
	"help", // CMD_GET_HELP_2
	"get time", // CMD_GET_TIME
	"get date",  // CMD_GET_DATE
	"set time", // CMD_SET_TIME
	"set date",  // CMD_SET_DATE
	"set weekday", // CMD_SET_WEEKDAY
	"list alarms", // CMD_LIST_ALARMS
	"set alarm", // CMD_SET_ALARM
	"restore", // CMD_RESTORE
};

static inline uint32_t get_command_len(uint16_t cmd_index) {
	// check for valid cmd_index
	return strlen(commands[cmd_index]);
}

static uint16_t compare_command(uint16_t cmd_index, uint8_t *buf, uint32_t len) {
	uint16_t n, cmd_len;
	cmd_len = get_command_len(cmd_index);
	if(len < cmd_len) {
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

static void send_message(char *msg) {
	CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
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
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)commands[CMD_GET_HELP_0]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", ");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)commands[CMD_GET_HELP_1]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", ");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)commands[CMD_GET_HELP_2]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ": display this help\n");
	//CDC_Transmit_FS((uint8_t *)user_com_str_buf, user_com_str_len);

	//user_com_str_len = str_buf_clear(user_com_str_buf, USER_COM_STR_BUF_SIZE);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)commands[CMD_GET_TIME]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ": get present time in format hh:mm:ss\n");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)commands[CMD_GET_DATE]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ": get present date in format yyyy-mm-dd, weekday: 1:");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)rtc_weekday_names[1]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", 2;");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)rtc_weekday_names[2]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", 3;");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)rtc_weekday_names[3]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", 4;");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)rtc_weekday_names[4]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", 5;");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)rtc_weekday_names[5]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", 6;");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)rtc_weekday_names[6]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", 7;");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)rtc_weekday_names[7]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, "\n");
	//CDC_Transmit_FS((uint8_t *)user_com_str_buf, user_com_str_len);

	//user_com_str_len = str_buf_clear(user_com_str_buf, USER_COM_STR_BUF_SIZE);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)commands[CMD_SET_TIME]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ": set present time in format hh:mm:ss\n");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)commands[CMD_SET_DATE]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ": get present date in format yyyy-mm-dd\n");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)commands[CMD_SET_WEEKDAY]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ": set present day: 1:");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)rtc_weekday_names[1]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", 2;");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)rtc_weekday_names[2]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", 3;");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)rtc_weekday_names[3]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", 4;");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)rtc_weekday_names[4]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", 5;");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)rtc_weekday_names[5]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", 6;");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)rtc_weekday_names[6]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", 7;");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)rtc_weekday_names[7]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, "\n");

	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)commands[CMD_LIST_ALARMS]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ": get a list of all possible alarms\n");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)commands[CMD_SET_ALARM]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ": set a given alarm\n");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)commands[CMD_RESTORE]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ": restore settings to default, password needed\n");

	CDC_Transmit_FS((uint8_t *)user_com_str_buf, user_com_str_len);
}

static void send_present_date_time(void) {
	uint8_t year, month, day, hour, min, sec, weekday;
	rtc_get_date_time(&year, &month, &day, &hour, &min, &sec, &weekday);
	user_com_str_len = str_buf_clear(user_com_str_buf, USER_COM_STR_BUF_SIZE);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, "present date and time: ");
	user_com_str_len = str_buf_append_uint16(user_com_str_buf, USER_COM_STR_BUF_SIZE, 2000+year);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, "-");
	user_com_str_len = str_buf_append_uint8_lead0(user_com_str_buf, USER_COM_STR_BUF_SIZE, month);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, "-");
	user_com_str_len = str_buf_append_uint8_lead0(user_com_str_buf, USER_COM_STR_BUF_SIZE, day);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", ");
	user_com_str_len = str_buf_append_uint8_lead0(user_com_str_buf, USER_COM_STR_BUF_SIZE, hour);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ":");
	user_com_str_len = str_buf_append_uint8_lead0(user_com_str_buf, USER_COM_STR_BUF_SIZE, min);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ":");
	user_com_str_len = str_buf_append_uint8_lead0(user_com_str_buf, USER_COM_STR_BUF_SIZE, sec);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ", weekday: ");
	user_com_str_len = str_buf_append_uint8(user_com_str_buf, USER_COM_STR_BUF_SIZE, weekday);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, ":");
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, (char *)rtc_weekday_names[weekday]);
	user_com_str_len = str_buf_append_string(user_com_str_buf, USER_COM_STR_BUF_SIZE, "\n");
	CDC_Transmit_FS((uint8_t *)user_com_str_buf, user_com_str_len);
}

static void set_time_from_string(uint8_t *buf, uint32_t len) {
	uint16_t pos = get_command_len(CMD_SET_TIME);
	if(len < strlen("set time hh:mm:ss")) {
		// error, command was not long enough
		send_message("error, invalid command 0, format must be: set time hh:mm:ss\n");
		return;
	}
	if(buf[pos] != ' ') {
		send_message("error, invalid command 1, format must be: set time hh:mm:ss\n");
		return;
	}
	pos++;
	uint8_t hh = (uint8_t)atoi((char *)&buf[pos]);
	if(hh > 23) {
		send_message("error, invalid command 2, format must be: set time hh:mm:ss\n");
		return;
	}
	pos += 2;
	if(buf[pos] != ':') {
		send_message("error, invalid command 3, format must be: set time hh:mm:ss\n");
		return;
	}
	pos++;
	uint8_t mm = (uint8_t)atoi((char *)&buf[pos]);
	if(mm > 59) {
		send_message("error, invalid command 4, format must be: set time hh:mm:ss\n");
		return;
	}
	pos += 2;
	if(buf[pos] != ':') {
		send_message("error, invalid command 5, format must be: set time hh:mm:ss\n");
		return;
	}
	pos++;
	uint8_t ss = (uint8_t)atoi((char *)&buf[pos]);
	if(ss > 59) {
		send_message("error, invalid command 6, format must be: set time hh:mm:ss\n");
		return;
	}
	uint8_t year, month, day, hour, min, sec, weekday;
	rtc_get_date_time(&year, &month, &day, &hour, &min, &sec, &weekday);
	hour = hh;
	min = mm;
	sec = ss;
	rtc_set_date_time(year, month, day, hour, min, sec, weekday);

	send_message("set time ok:\n");
	send_present_date_time();
}

static void set_date_from_string(uint8_t *buf, uint32_t len) {
	uint16_t pos = get_command_len(CMD_SET_DATE);
	if(len < strlen("set date yyyy-mm-dd")) {
		// error, command was not long enough
		send_message("error, invalid command 0, format must be: set date yyyy-mm-dd\n");
		return;
	}
	if(buf[pos] != ' ') {
		send_message("error, invalid command 1, format must be: set date yyyy-mm-dd\n");
		return;
	}
	pos++;
	uint16_t yyyy = (uint16_t)atoi((char *)&buf[pos]);
	if((yyyy < 2000) || (yyyy > 2099)) {
		send_message("error, invalid command 2, format must be: set date yyyy-mm-dd\n");
		return;
	}
	pos += 4;
	if(buf[pos] != '-') {
		send_message("error, invalid command 3, format must be: set date yyyy-mm-dd\n");
		return;
	}
	pos++;
	uint8_t mm = (uint8_t)atoi((char *)&buf[pos]);
	if((mm == 0) || (mm > 12)) {
		send_message("error, invalid command 4, format must be: set date yyyy-mm-dd\n");
		return;
	}
	pos += 2;
	if(buf[pos] != '-') {
		send_message("error, invalid command 5, format must be: set date yyyy-mm-dd\n");
		return;
	}
	pos++;
	uint8_t dd = (uint8_t)atoi((char *)&buf[pos]);
	if((dd == 0) || (dd > 31)) {
		send_message("error, invalid command 6, format must be: set date yyyy-mm-dd\n");
		return;
	}
	uint8_t year, month, day, hour, min, sec, weekday;
	rtc_get_date_time(&year, &month, &day, &hour, &min, &sec, &weekday);
	year = yyyy - 2000;
	month = mm;
	day = dd;
	rtc_set_date_time(year, month, day, hour, min, sec, weekday);

	send_message("set date ok:\n");
	send_present_date_time();
}

static void set_weekday_from_string(uint8_t *buf, uint32_t len) {
	uint16_t pos = get_command_len(CMD_SET_WEEKDAY);
	if(len < strlen("set weekday n")) {
		// error, command was not long enough
		send_message("error, invalid command 0, format must be: set weekday n (1 ... 7)\n");
		return;
	}
	if(buf[pos] != ' ') {
		send_message("error, invalid command 1, format must be: set weekday n (1 ... 7)\n");
		return;
	}
	pos++;
	uint8_t wd = (uint8_t)atoi((char *)&buf[pos]);
	if((wd == 0) || (wd > 7)) {
		send_message("error, invalid command 2, format must be: set weekday n (1 ... 7)\n");
		return;
	}

	uint8_t year, month, day, hour, min, sec, weekday;
	rtc_get_date_time(&year, &month, &day, &hour, &min, &sec, &weekday);
	weekday = wd;
	rtc_set_date_time(year, month, day, hour, min, sec, weekday);

	send_message("set weekday ok:\n");
	send_present_date_time();
}

void user_com_parse(uint8_t *buf, uint32_t len) {
	uint16_t cmd = parse_command(buf, len);
	switch(cmd) {
	case INVALID_COMMAND:
		send_message("invalid command, send ?, h or help for help\n");
	case CMD_GET_HELP_0:
	case CMD_GET_HELP_1:
	case CMD_GET_HELP_2:
		send_help();
		break;
	case CMD_GET_TIME:
	case CMD_GET_DATE:
		send_present_date_time();
		break;
	case CMD_SET_TIME:
		set_time_from_string(buf, len);
		break;
	case CMD_SET_DATE:
		set_date_from_string(buf, len);
		break;
	case CMD_SET_WEEKDAY:
		set_weekday_from_string(buf, len);
		break;
	case CMD_LIST_ALARMS:
	case CMD_SET_ALARM:
	case CMD_RESTORE:
	default:
		send_message("invalid command, send ?, h or help for help, default? why?");
	}
}
