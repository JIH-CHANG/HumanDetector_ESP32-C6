# HumanDetector - ESP32-C6 Occupancy Sensor

A smart occupancy sensor for ESP32-C6 that combines multiple detection methods:
- **24 GHz Millimeter-Wave Radar (C4001)** for human presence detection
- **Light Sensor (MAX4400)** for ambient light level
- **Matter/Thread Protocol** for smart home integration with Google Home

## Overview

This project implements an occupancy sensor that triggers when a room is both dark AND occupied (person detected by radar). The sensor reports its state via the Matter protocol over Thread (802.15.4) wireless network.

**Key Features:**
- Dual-sensor occupancy detection (radar + light)
- Thread network connectivity with BLE commissioning
- Onboard WS2812B RGB LED for visual feedback (blue = occupied)
- Matter OccupancySensing cluster (0x0107) compatible with Google Home
- Low power consumption suitable for battery operation

## Hardware

**Target Device:** ESP32-C6-DevKitC-1

**Sensors:**
- **MAX4400**: I2C ambient light sensor (0x4A addr)
  - Pins: SDA=GPIO7, SCL=GPIO6
- **C4001**: 24 GHz millimeter-wave human presence sensor (UART)
  - Pins: RX=GPIO5, TX=GPIO4, Baudrate=9600
- **WS2812B**: Programmable RGB LED
  - Pin: GPIO8 (RMT driver)
- **BOOT Button**: GPIO9 (long press > 5s for factory reset)

## Building

### Prerequisites
- ESP-IDF v5.0 or later
- ESP-Matter SDK
- CMake 3.16+

### Build Steps

```bash
cd HumanDetector
idf.py build
```

### Flash to Device

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

(Replace `/dev/ttyUSB0` with your serial port)

## Configuration

Edit `main/app_main.cpp` to tune these parameters:

```c
#define LIGHT_THRESHOLD_LUX     50.0f    // Radar active only when lux < this
#define SENSOR_POLL_INTERVAL_MS 5000     // Update frequency (ms)
```

## Commissioning

1. Power on the device
2. Open Google Home app
3. Add Device > Set up device > Matter device
4. Scan QR code or use Thread border router
5. Complete BLE commissioning flow

Once commissioned, the device joins your Thread network and appears as an occupancy sensor in Google Home.

## Architecture

```
app_main.cpp           -- Matter device lifecycle, commissioning, attribute handling
app_driver.cpp         -- Hardware abstraction (I2C, UART, RMT)
app_priv.h             -- Shared definitions and driver API
```

### Data Flow

1. **Sensor Task** (5s interval):
   - Read MAX4400 light level
   - Read C4001 radar presence
   - Decide occupancy: (lux < threshold) AND (radar present) = occupied
   - Update LED and Matter attribute on state change

2. **Matter Callback**:
   - Google Home subscribes to OccupancySensing attribute
   - Triggers light automation rules based on occupancy state

## Troubleshooting

### "C4001 UART empty" in logs
- Check UART connection (RX/TX pins)
- Verify C4001 power supply (check red LED on sensor)

### "MAX4400 config write failed"
- Check I2C connection (SDA/SCL lines and pull-ups)
- Verify MAX4400 power supply

### Device not appearing in Google Home
- Ensure Thread border router is active
- Check device logs for commissioning errors
- Re-commission if needed

## License

MIT License - See LICENSE file for details

## References

- [ESP-Matter Documentation](https://docs.espressif.com/projects/esp-matter/en/latest/)
- [ESP32-C6 Technical Reference](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/index.html)
- [Matter Specification](https://csa-iot.org/csa_iot_wp_matter_specification_v1_1.pdf)
