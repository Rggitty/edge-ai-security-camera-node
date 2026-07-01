# Edge AI Security Camera Node with Power Profiling

An ESP32 based edge AI security camera node that detects motion, captures a camera frame, runs an on device person/no_person classifier, measures real power consumption with an INA219 current sensor, and logs results through a Wi-Fi dashboard and CSV export.

## Project Overview

This project combines embedded systems, computer vision, edge AI, and power profiling into one complete IoT prototype.

The system uses a PIR motion sensor to trigger the ESP32 camera, runs a trained Edge Impulse person-detection model locally on the ESP32, measures voltage/current/power using an INA219 sensor, and displays the results on a browser based dashboard.

## Features

- PIR triggered motion detection
- OV2640 camera capture using the SunFounder ESP32 camera extension board
- Edge Impulse person/no_person image classifier
- On device TensorFlow Lite inference
- INA219 voltage, current, and power measurement
- Wi-Fi dashboard with latest AI frame
- AI confidence thresholding with `PERSON_DETECTED`, `NO_PERSON`, and `UNCERTAIN`
- CSV event logging
- Active dashboard mode and Wi-Fi-off low power comparison mode
- Power profiling graphs and analysis

## Hardware Used

- ESP32 development board
- SunFounder ESP32 camera extension board
- OV2640 camera module
- PIR motion sensor
- INA219 current/voltage sensor
- Power bank
- Modified USB-C cable for INA219 inline current measurement
- Jumper wires

![Hardware Setup](Media/Project Pic (1).jpg), (Media/Project Pic (2).png)

![INA219 Power Measurement Path](Media/Project Pic (1).jpg), (Media/Project Pic (2).png)

![Dashboard Demo](Media/Project Pic (1).png), (Media/Project Pic (2).png)

> Note: The `Media/` folder contains public project photos only. Private training images are excluded from this repository.


## Wiring Summary

### PIR Sensor

| PIR Pin | ESP32 Pin |
|---|---|
| VCC | 5V |
| GND | GND |
| OUT | GPIO 4 |

### INA219 I2C

| INA219 Pin | ESP32 Pin |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO 13 |
| SCL | GPIO 14 |

### INA219 Power Path

| Connection | Destination |
|---|---|
| Power bank red wire | INA219 VIN+ |
| INA219 VIN- | ESP32 USB-C red wire |
| Power bank blue wire | ESP32 USB-C blue wire |

The INA219 is placed inline with the ESP32 power input so the system can measure real current draw during camera capture and AI inference.

## Camera Pin Mapping

The SunFounder camera extension uses an AI Thinker style OV2640 mapping, but the camera reset pin must be set to GPIO 33 because that is what I found to work.

```cpp
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    33
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22