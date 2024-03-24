/**
 * Martin Egli
 * 2024-03-24
 * usb uart module
 */

#ifndef _USB_UART_H
#define _USB_UART_H

void usb_uart_start(void);
void usb_uart_stop(void);
void usb_uart_print_data_types(void);
void usb_uart_print_message(char *msg);
void usb_uart_print_adv_data(void);

#endif // _USB_UART_H
