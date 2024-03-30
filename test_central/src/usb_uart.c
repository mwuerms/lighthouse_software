/**
 * Martin Egli
 * 2024-03-24
 * usb uart module
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
//#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/sys/ring_buffer.h>

#include "usb_uart.h"

// do NOT use printk
//#include <zephyr/sys/printk.h>
#define printk(...)

// - private functions --------------------------------------
const struct device *usb_uart_dev;

#define RING_BUF_SIZE (1024UL*2)
uint8_t ring_buffer[RING_BUF_SIZE];

struct ring_buf ringbuf;

#define USB_UART_TEMP_STR_BUFFER_SIZE	(64)
static char usb_uart_temp_str_buffer[USB_UART_TEMP_STR_BUFFER_SIZE];

static void interrupt_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&ringbuf),
					 sizeof(buffer));

			recv_len = uart_fifo_read(dev, buffer, len);
			if (recv_len < 0) {
				printk("Failed to read UART FIFO");
				recv_len = 0;
			};

			rb_len = ring_buf_put(&ringbuf, buffer, recv_len);
			if (rb_len < recv_len) {
				printk("Drop %u bytes", recv_len - rb_len);
			}

			printk("tty fifo -> ringbuf %d bytes", rb_len);
			if (rb_len) {
				uart_irq_tx_enable(dev);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buffer[64];
			int rb_len, send_len;

			rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			if (!rb_len) {
				printk("Ring buffer empty, disable TX IRQ");
				uart_irq_tx_disable(dev);
				continue;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) {
				printk("Drop %d bytes", rb_len - send_len);
			}

			printk("ringbuf -> tty fifo %d bytes", send_len);
		}
	}
}

// - public functions ---------------------------------------

void usb_uart_start(void) {
	uint32_t baudrate = 0U;
	int ret;

	usb_uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (!device_is_ready(usb_uart_dev)) {
		printk("CDC ACM device not ready");
		return;
	}

    ret = usb_enable(NULL);
	if (ret != 0) {
		printk("Failed to enable USB");
		return;
	}

	ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);
	
	/* Wait 100ms for the host to do all settings */
	k_msleep(100);

	ret = uart_line_ctrl_get(usb_uart_dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret) {
		printk("Failed to get baudrate, ret code %d", ret);
	} else {
		printk("Baudrate detected: %d", baudrate);
	}

	uart_irq_callback_set(usb_uart_dev, interrupt_handler);

	/* Enable rx interrupts */
	//uart_irq_rx_enable(usb_uart_dev);
}

void usb_uart_stop(void) {
    return;
}

void usb_uart_print_data_types(void) {
	usb_uart_print_message("print data types\n");

	snprintf(usb_uart_temp_str_buffer, USB_UART_TEMP_STR_BUFFER_SIZE, "char, min: %d, max: %d, nb bytes %d\n", CHAR_MIN, CHAR_MAX, sizeof(char));
	usb_uart_print_message(usb_uart_temp_str_buffer);
    
	snprintf(usb_uart_temp_str_buffer, USB_UART_TEMP_STR_BUFFER_SIZE, "int, min: %d, max: %d, nb bytes %d\n", INT_MIN, INT_MAX, sizeof(int));
	usb_uart_print_message(usb_uart_temp_str_buffer);
    
	snprintf(usb_uart_temp_str_buffer, USB_UART_TEMP_STR_BUFFER_SIZE, "long int, min: %ld, max: %ld, nb bytes %d\n", LONG_MIN, LONG_MAX, sizeof(long int));
	usb_uart_print_message(usb_uart_temp_str_buffer);
    
	//snprintf(usb_uart_temp_str_buffer, USB_UART_TEMP_STR_BUFFER_SIZE, "float, min: %f, max: %f, nb bytes %d\n", FLT_MIN, FLT_MAX, sizeof(float));
	//usb_uart_print_message(usb_uart_temp_str_buffer);

	snprintf(usb_uart_temp_str_buffer, USB_UART_TEMP_STR_BUFFER_SIZE, "int8_t, min: %d, max: %d, nb bytes %d\n", INT8_MIN, INT8_MAX, sizeof(int8_t));
	usb_uart_print_message(usb_uart_temp_str_buffer);

	snprintf(usb_uart_temp_str_buffer, USB_UART_TEMP_STR_BUFFER_SIZE, "uint8_t, min: %d, max: %d, nb bytes %d\n", 0, UINT8_MAX, sizeof(uint8_t));
	usb_uart_print_message(usb_uart_temp_str_buffer);

	snprintf(usb_uart_temp_str_buffer, USB_UART_TEMP_STR_BUFFER_SIZE, "uint16_t, min: %d, max: %d, nb bytes %d\n", 0, UINT16_MAX, sizeof(uint16_t));
	usb_uart_print_message(usb_uart_temp_str_buffer);
    
	snprintf(usb_uart_temp_str_buffer, USB_UART_TEMP_STR_BUFFER_SIZE, "uint32_t, min: %d, max: %ld, nb bytes %d\n", 0, UINT32_MAX, sizeof(uint32_t));
	usb_uart_print_message(usb_uart_temp_str_buffer);

	snprintf(usb_uart_temp_str_buffer, USB_UART_TEMP_STR_BUFFER_SIZE, "uint64_t, min: %d, max: %lld, nb bytes %d\n", 0, UINT64_MAX, sizeof(uint64_t));
	usb_uart_print_message(usb_uart_temp_str_buffer);

	return;
}

void usb_uart_print_message(char *msg) {
    if (!device_is_ready(usb_uart_dev)) {
        // error, device is not ready
        return;
    }
	int rb_len = 0;
    rb_len += ring_buf_put(&ringbuf, msg, strlen(msg));
    if (rb_len) {
        uart_irq_tx_enable(usb_uart_dev);
    }
    return;
}

static const char str_new_line_cr_lf[] = {0x0D, 0x0A, 0x00};
void usb_uart_print_adv_data(char* msg, char* addr_str, int8_t rssi, uint8_t *adv_data, uint16_t adv_len) {
    if (!device_is_ready(usb_uart_dev)) {
        // error, device is not ready
        return;
    }
	int rb_len = 0;
	
    rb_len += ring_buf_put(&ringbuf, msg, strlen(msg));
	rb_len += ring_buf_put(&ringbuf, addr_str, strlen(addr_str));
	snprintf(usb_uart_temp_str_buffer, USB_UART_TEMP_STR_BUFFER_SIZE, ", rssi: %02d, adv_data (hex, %02d):", rssi, adv_len);
	rb_len += ring_buf_put(&ringbuf, usb_uart_temp_str_buffer, strlen(usb_uart_temp_str_buffer));
	uint16_t n;
	for(n = 0; n < adv_len; n++) {
		snprintf(usb_uart_temp_str_buffer, USB_UART_TEMP_STR_BUFFER_SIZE, "%02X", adv_data[n]);
		rb_len += ring_buf_put(&ringbuf, usb_uart_temp_str_buffer, strlen(usb_uart_temp_str_buffer));
	}
	// add new line
	rb_len += ring_buf_put(&ringbuf, str_new_line_cr_lf, strlen(str_new_line_cr_lf));
    if (rb_len) {
        uart_irq_tx_enable(usb_uart_dev);
    }
}

void usb_uart_print_adv_data_timestamp(uint32_t now_s, uint32_t now_ms, char* msg, char* addr_str, int8_t rssi, uint8_t *adv_data, uint16_t adv_len) {
    if (!device_is_ready(usb_uart_dev)) {
        // error, device is not ready
        return;
    }
	int rb_len = 0;
	
	snprintf(usb_uart_temp_str_buffer, USB_UART_TEMP_STR_BUFFER_SIZE, "%d.%03d: ", now_s, now_ms);
	rb_len += ring_buf_put(&ringbuf, usb_uart_temp_str_buffer, strlen(usb_uart_temp_str_buffer));

	usb_uart_print_adv_data(msg, addr_str, rssi, adv_data, adv_len);
}
