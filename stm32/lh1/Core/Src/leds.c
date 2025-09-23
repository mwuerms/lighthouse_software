/**
 * Martin Egli
 * 2024-11-13
 * leds for light house
 * I2C led driver IS31FL3731
 *   front display
 *   back display
 */

// - includes ------------------------------------------------------------------
#include "main.h"
#include "leds.h"

/* - definitions ------------------------------------------------------------ */
//chip: IS31FL3731
#define cI2C_ADDR_WR 0xE8
#define cI2C_ADDR_RD 0xE9

// register addresses
#define cFRAME_1_REG_ADDR			0x00
#define cFRAME_2_REG_ADDR			0x01
#define cFRAME_3_REG_ADDR			0x02
#define cFRAME_4_REG_ADDR			0x03
#define cFRAME_5_REG_ADDR			0x04
#define cFRAME_6_REG_ADDR			0x05
#define cFRAME_7_REG_ADDR			0x06
#define cFRAME_8_REG_ADDR			0x07
#define cFUNC_REG_ADDR				0x0B

// funciton register addresses
#define cFUNC_CONFIG_REG			0x00
#define cFUNC_PICTURE_DISP_REG		0x01
#define cFUNC_AUTOPLAY1_REG			0x02
#define cFUNC_AUTOPLAY2_REG			0x03
#define cFUNC_RESERVE_REG			0x04
#define cFUNC_DISP_OPTION_REG		0x05
#define cFUNC_AUDIO_SYNC_REG		0x06
#define cFUNC_BREATH_CTRL1_REG		0x08
#define cFUNC_BREATH_CTRL2_REG		0x09
#define cFUNC_SHUTDOWN_REG			0x0a
#define cFUNC_AGC_CTRL_REG			0x0b
#define cFUNC_AUDIO_ADC_RATE_REG	0x0c


// from main.c/h I2C_HandleTypeDef hi2c1;

// - private functions ---------------------------------------------------------

static void i2c_Send(uint8_t reg_addr, uint8_t *buffer, uint32_t size) {
	uint8_t tx_buf[] = {0xFD, reg_addr};
	HAL_I2C_Master_Transmit(&hi2c1, cI2C_ADDR_WR, tx_buf, sizeof(tx_buf), HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(&hi2c1, cI2C_ADDR_WR, buffer, size, HAL_MAX_DELAY);
}

/**
 * send configuration for normal operation mode
 */
void i2cLED_PowerUp(void) {
	HAL_GPIO_WritePin(IS_SDB_GPIO_Port, IS_SDB_Pin, GPIO_PIN_SET);
	// 0x0A, shutdown register = 1: normal operation
	uint8_t tx_buf[] = {cFUNC_SHUTDOWN_REG, 1};
	i2c_Send(cFUNC_REG_ADDR, tx_buf, sizeof(tx_buf));
}

/**
 * send configuration for shutdown mode
 */
void i2cLED_PowerDown(void) {
	HAL_GPIO_WritePin(IS_SDB_GPIO_Port, IS_SDB_Pin, GPIO_PIN_RESET);
	// 0x0A, shutdown register = 0: shutdown mode
	uint8_t tx_buf[] = {cFUNC_SHUTDOWN_REG, 0};
	i2c_Send(cFUNC_REG_ADDR, tx_buf, sizeof(tx_buf));
}

// - public functions ----------------------------------------------------------
void leds_init(void) {
	return;
}

void leds_front_dsiplay(char c) {
	uint8_t tx_buf[] = {0xFD, c};
	HAL_I2C_Master_Transmit(&hi2c1, cI2C_ADDR_WR, tx_buf, sizeof(tx_buf), HAL_MAX_DELAY);
}


/**
 * holding a frame for all 144 possible led
 * write frame register
 * 0x00 ... 0x11: led control, bit.x =1: led.x enabled, =0: disabled
 * 0x11 ... 0x23: led blink,   bit.x =1: led.x: blink enabled, =0: disabled
 * 0x24 ... 0xAB: led pwm 8-bit pwm for every led
 */
#define I2C_LED_FRAME_REG_ADDR_SIZE (1)
#define I2C_LED_FRAME_LED_CTRL_SIZE (18)
#define I2C_LED_FRAME_BLINK_CTRL_SIZE (18)
#define I2C_LED_FRAME_PWM_REG_SIZE (144)
#define I2C_LED_FRAME_SIZE (I2C_LED_FRAME_REG_ADDR_SIZE+I2C_LED_FRAME_LED_CTRL_SIZE+I2C_LED_FRAME_BLINK_CTRL_SIZE+I2C_LED_FRAME_PWM_REG_SIZE)
static struct {
	uint8_t reg_addr; // 1, I2C_LED_FRAME_REG_ADDR_SIZE
	uint8_t led_ctrl[I2C_LED_FRAME_LED_CTRL_SIZE]; // 18
	uint8_t blink_ctrl[I2C_LED_FRAME_BLINK_CTRL_SIZE]; // 18
	uint8_t pwm_reg[I2C_LED_FRAME_PWM_REG_SIZE];  // 144
} __attribute__((packed)) i2c_led_frame_buffer;

static uint8_t led_ctrl_is_used_maks[I2C_LED_FRAME_LED_CTRL_SIZE] = {
		// 0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   10,   11,   12,   13,   14,   15,   16,   17
		0xFF, 0xFF, 0x1F, 0x1F, 0xFF, 0xFF, 0x1F, 0x1F, 0x0F, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*
 * leds mapping for AB:CD (to display time), w (weekdays)
 * A:  D1 ... D13
 * B: D14 ... D26
 * :  D27 ... D30
 * C: D31 ... D43
 * D: D44 ... D56
 * w: D57 ... D63
 */

/*
  4
0   9
  5
1   A
  6
2   B
  7
3   C
  8
  */
// position of the LED inside PWM REGISTER, see i2c_led_frame_buffer.pwm_reg[I2C_LED_FRAME_PWM_REG_SIZE], 0 ... (144-1)
static const uint8_t pwm_reg_pos_num_0[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x10, 0x11, 0x12, 0x13, 0x14}; // reg addr = {0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x34, 0x35, 0x36, 0x37, 0x38};
static const uint8_t pwm_reg_pos_num_1[] = {0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x30, 0x31, 0x32, 0x33, 0x34}; // reg addr = {0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x54, 0x55, 0x56, 0x57, 0x58};
static const uint8_t pwm_reg_pos_colon[] = {0x40, 0x41, 0x42, 0x43}; // reg addr = {0x64, 0x65, 0x66, 0x67};
static const uint8_t pwm_reg_pos_num_2[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x18, 0x19, 0x1A, 0x1B, 0x1C}; // reg addr = {0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x3C, 0x3D, 0x3E, 0x3F, 0x40};
static const uint8_t pwm_reg_pos_num_3[] = {0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x38, 0x39, 0x3A, 0x3B, 0x3C}; // reg addr = {0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53, 0x5C, 0x5D, 0x5E, 0x5F, 0x60};
static const uint8_t pwm_reg_pos_days[]  = {0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E}; // reg addr = {0x6C, 0x6D, 0x6E, 0x6F, 0x70, 0x71, 0x72};
// define types, numbers 0 ... 9
/*static const uint8_t char_0_len = 10;
static const uint8_t char_0_pos[] = {0, 1, 2, 3, 4, 8, 9, 10, 11, 12};
static const uint8_t char_1_len = 8;
static const uint8_t char_1_pos[] = {0, 3, 4, 5, 6, 7, 8, 12};
static const uint8_t char_2_len = 9;
static const uint8_t char_2_pos[] = {0, 2, 3, 4, 6, 7, 9, 10, 12};
static const uint8_t char_3_len = 9;
static const uint8_t char_3_pos[] = {0, 3, 4, 6, 8, 9, 10, 11, 12};
static const uint8_t char_4_len = 6;
static const uint8_t char_4_pos[] = {0, 1, 6, 10, 11, 12};
static const uint8_t char_5_len = 9;
static const uint8_t char_5_pos[] = {0, 1, 3, 5, 6, 8, 9, 11, 12};
static const uint8_t char_6_len = 9;
static const uint8_t char_6_pos[] = {0, 1, 2, 3, 4, 6, 8, 11, 12};
static const uint8_t char_7_len = 7;
static const uint8_t char_7_pos[] = {0, 5, 7, 8, 9, 10, 11};
static const uint8_t char_8_len = 11;
static const uint8_t char_8_pos[] = {0, 1, 2, 3, 4, 6, 8, 9, 10, 11, 12};
static const uint8_t char_9_len = 9;
static const uint8_t char_9_pos[] = {0, 1, 4, 6, 8, 9, 10, 11, 12};
static const uint8_t char_colon_len = 2;
static const uint8_t char_9_pos[] = {1, 3};
*/

static const uint8_t char_num_len[] = {10, 8, 9, 9, 6, 9, 10, 7, 11, 10};
static const uint8_t char_num_pos[10][13] = {
		{0, 1, 2, 3, 4, 8, 9, 10, 11, 12}, // 0
		{0, 3, 4, 5, 6, 7, 8, 12}, // 1
		{0, 2, 3, 4, 6, 7, 9, 10, 12}, // 2
		{0, 3, 4, 6, 8, 9, 10, 11, 12}, // 3
		{0, 1, 6, 10, 11, 12}, // 4
		{0, 1, 3, 5, 6, 8, 9, 11, 12}, // 5
		{0, 1, 2, 3, 4, 6, 8, 9, 11, 12}, // 6
		{0, 5, 7, 8, 9, 10, 11}, // 7
		{0, 1, 2, 3, 4, 6, 8, 9, 10, 11, 12}, // 8
		{0, 1, 3, 4, 6, 8, 9, 10, 11, 12} // 9
};
static const uint8_t char_colon_len = 2;
static const uint8_t char_colon_pos[] = {1, 3};

void leds_front_display_time(uint8_t hour, uint8_t min, uint8_t colon, uint8_t days_mask, uint8_t pwm) {
	uint8_t n, c, pos;
	i2c_led_frame_buffer.reg_addr = 0;
	for(n = 0; n < I2C_LED_FRAME_LED_CTRL_SIZE; n++) {
		i2c_led_frame_buffer.led_ctrl[n] = led_ctrl_is_used_maks[n];
	}
	for(n = 0; n < I2C_LED_FRAME_BLINK_CTRL_SIZE; n++) {
		i2c_led_frame_buffer.blink_ctrl[n] = 0x00;
	}
	for(n = 0; n < I2C_LED_FRAME_PWM_REG_SIZE; n++) {
		i2c_led_frame_buffer.pwm_reg[n] = 0;//pwm;
	}

	// pwm_reg_pos_num_0
	c = hour/10;
	hour -= c*10;
	for(n = 0; n < char_num_len[c]; n++) {
		pos = pwm_reg_pos_num_0[char_num_pos[c][n]];
		i2c_led_frame_buffer.pwm_reg[pos] = pwm;
	}
	// pwm_reg_pos_num_1
	c = hour;
	for(n = 0; n < char_num_len[c]; n++) {
		pos = pwm_reg_pos_num_1[char_num_pos[c][n]];
		i2c_led_frame_buffer.pwm_reg[pos] = pwm;
	}
	// pwm_reg_pos_colon
	if(colon) {
		for(n = 0; n < char_colon_len; n++) {
			pos = pwm_reg_pos_colon[char_colon_pos[n]];
			i2c_led_frame_buffer.pwm_reg[pos] = pwm;
		}
	}
	// pwm_reg_pos_num_2
	c = min/10;
	min -= c*10;
	for(n = 0; n < char_num_len[c]; n++) {
		pos = pwm_reg_pos_num_2[char_num_pos[c][n]];
		i2c_led_frame_buffer.pwm_reg[pos] = pwm;
	}
	// pwm_reg_pos_num_3
	c = min;
	for(n = 0; n < char_num_len[c]; n++) {
		pos = pwm_reg_pos_num_3[char_num_pos[c][n]];
		i2c_led_frame_buffer.pwm_reg[pos] = pwm;
	}
	// pwm_reg_pos_days
	for(n = 0, c = 0x01; n < 7; n++, c <<= 1) {
		if(days_mask & c) {
			pos = pwm_reg_pos_days[n];
			i2c_led_frame_buffer.pwm_reg[pos] = pwm;
		}
	}

	i2c_Send(cFRAME_1_REG_ADDR, (uint8_t *)&i2c_led_frame_buffer, I2C_LED_FRAME_SIZE);
}
