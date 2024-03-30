/**
 * Martin Egli
 * 2024-03-29
 * module timestamp
 * using sample alarm using RTC
 */
#include <stdint.h>
#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>

#define DELAY 2000000
#define ALARM_CHANNEL_ID 0

struct counter_alarm_cfg alarm_cfg;

#if defined(CONFIG_COUNTER_NRF_RTC)
#define TIMER DT_NODELABEL(rtc0)
#else
#error rtc0 not defined, check device tree overlay
#endif

static const struct device *const counter_dev = DEVICE_DT_GET(TIMER);

void timestamp_start_counter(void) {
    counter_start(counter_dev);
}

void timestamp_stop_counter(void) {
    counter_stop(counter_dev);
}

void timestamp_get_counter(uint32_t *now_s, uint32_t *now_ms) {
    uint32_t now_ticks;
	uint64_t now_usec;
	int now_sec, err;

    err = counter_get_value(counter_dev, &now_ticks);
	if (err) {
		printk("Failed to read counter value (err %d)", err);
		return;
	}

	now_usec = counter_ticks_to_us(counter_dev, now_ticks);
	now_sec = (int)(now_usec / USEC_PER_SEC);
	now_usec -= now_sec * USEC_PER_SEC;

	*now_s  = (uint32_t)now_sec;
	*now_ms = (uint32_t)(now_usec/1000);
}
