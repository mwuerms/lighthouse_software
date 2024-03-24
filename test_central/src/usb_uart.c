/**
 * Martin Egli
 * 2024-03-24
 * usb uart module
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
//#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/sys/ring_buffer.h>

#include "usb_uart.h"

// - private functions --------------------------------------
const struct device *usb_uart_dev;

#define RING_BUF_SIZE 1024
uint8_t ring_buffer[RING_BUF_SIZE];

struct ring_buf ringbuf;

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
    return;
}

void usb_uart_print_message(char *msg) {
    if (!device_is_ready(usb_uart_dev)) {
        // error, device is not ready
        return;
    }

    int rb_len = ring_buf_put(&ringbuf, msg, strlen(msg));
    printk("tty fifo -> ringbuf %d bytes", rb_len);
    if (rb_len) {
        uart_irq_tx_enable(usb_uart_dev);
    }
    return;
}

void usb_uart_print_adv_data(void) {
    return;
}
