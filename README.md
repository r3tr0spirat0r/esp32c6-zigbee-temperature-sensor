# esp32c6-zigbee-temperature-sensor

# ESP32-C6 Zigbee Temperature Sensor (DS18B20)

A lightweight Zigbee temperature sensor implementation for **ESP32-C6** using **ESP-IDF v5.2**. This project features a DS18B20 probe connected via OneWire (RMT) and reports data to a Zigbee Coordinator (like Zigbee2MQTT or Home Assistant).

## Prerequisites

To build this project, you need the **ESP-IDF** (Espressif IoT Development Framework) installed on your system.

1. **Install ESP-IDF v5.2**:
   - Follow the [Official Installation Guide](https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32c6/get-started/index.html).
   - Ensure you use **version 5.2** (higher versions might require minor code adjustments).
   
2. **ESP-IDF Component Manager**:
   - This project uses managed components (`ds18b20`, `onewire_bus`, `esp-zigbee-lib`). 
   - They are automatically downloaded by the IDF Component Manager during the first build based on the `idf_component.yml` file.

3. **Python & Dependencies**:
   - Ensure you have Python 3.10+ and the necessary build tools (`ninja`, `cmake`).
  
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

## Outcome
via bash Terminal
   ```bash
   idf.py monitor

<img width="1762" height="1330" alt="image" src="https://github.com/user-attachments/assets/f8c773a2-49e1-48c5-bfb2-630912ceed67" />

