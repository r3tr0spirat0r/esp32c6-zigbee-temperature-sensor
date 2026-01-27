# esp32c6-zigbee-temperature-sensor

# ESP32-C6 Zigbee Temperature Sensor (DS18B20)

A lightweight Zigbee temperature sensor implementation for **ESP32-C6** using **ESP-IDF v5.2**. This project features a DS18B20 probe connected via OneWire (RMT) and reports data to a Zigbee Coordinator (like Zigbee2MQTT or Home Assistant).

## Features
- **Zigbee 3.0** support (End Device role).
- **DS18B20** integration using the modern `onewire_bus` RMT driver.
- **Custom Partition Table** to prevent bootloops caused by Zigbee stack size.
- **Robustness**: Task handles missing sensor gracefully without blocking the Zigbee stack.

## Hardware Setup
- **MCU**: ESP32-C6 (DevKit)
- **Sensor**: DS18B20 (Waterproof probe)
- **Pinout**: GPIO 4 (OneWire Data)
- **Pull-up**: 4.7kΩ resistor between GPIO 4 and 3.3V (if not using a module with built-in resistor).

## Key Technical Fixes
During development, we solved two critical issues:
1. **Bootloop Fix**: Increased partition sizes using a custom `partitions.csv` to accommodate the Zigbee stack.
2. **Initialization Timing**: Added a `vTaskDelay` before OneWire bus scanning to ensure the sensor is ready after power-on.

## How to build
1. Set up ESP-IDF v5.2 environment.
2. Clone the repository.
3. Build and flash:
   ```bash
   idf.py build
   idf.py flash monitor
