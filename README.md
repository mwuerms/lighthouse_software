# Lighthouse Software

a light alarm clock

## lh1

+ 1st version for light house
+ git@github.com:mwuerms/lighthouse_software.git

### planing

1. [scheduler](git@github.com:mwuerms/mmscheduler.git)
2. buttons to test mmscheduler
3. front display, so I can see something
4. int RTC
5. ext RTC
6. power_modes

#### modules

+ front display
  + red led driver
  + RH sensor SHTC3
    + I2C1
+ back display
  + red, yellow led driver
    + I2C1
  + white leds
    + PWM
+ base
  + ext RTC
    + I2C1
  + int RTC
  + sound
    + USART2
  + USB CDC
  + BLE central
  + log, mem
    + SPI1
  + buttons

## buoy

floating like a buoy a bit off shore the lighthouse

optional, a small sender for the sleeper, monitoring the waking state, so the alarm will go off just on time

using advertising data
+ name: "buoy_beacon"
+ manufacturer spec:
  + uint8_t length
  + uint8_t version
    + version: 0x01
      + uint16_t SN 10000
      + uint16_t interrupt_counter
      + uint16_t input_states (buttons, etc)
      + uint16_t vbat in 1 mV (0 ... 16 V)

## test_central

nRF52840dongle as testing tool. Log all specified advertisment packets on host over USB.
