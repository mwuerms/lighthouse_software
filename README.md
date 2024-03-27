# Lighthouse Software

a light alarm clock

## Lighthouse

the light alarm clock

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
