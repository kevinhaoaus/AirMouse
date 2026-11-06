# Comet AirMouse

A battery-powered wireless air mouse: tilt or turn your hand and the cursor follows. Built on an Adafruit Feather 32u4 Bluefruit LE and a BMI160 IMU, and it pairs with a laptop over Bluetooth LE as **"COMET AIRMOUSE"**.

The Feather has no native Bluetooth mouse support, so every cursor move is sent to the BLE module as an `AT+BLEHIDMOUSEMOVE` text command. The firmware accumulates motion between sends and transmits the total in one step. That keeps the cursor smooth despite the several-millisecond cost of each command.

## Features

- Gyro mode (point like a laser pointer), with an automatic fallback to tilt/joystick mode if the gyro won't start
- Self-calibration at startup, automatic reconnect once paired
- Left/right click buttons on A0 / A1
- Tunable cursor speed and smoothing
- Battery level reported over BLE; runs from a LiPo and charges over USB

## Hardware

| BMI160 | Feather |
|--------|---------|
| 3V3    | 3V      |
| GND    | GND     |
| SCL    | pin 3   |
| SDA    | pin 5   |
| CS     | 3V      |

Buttons: A0 (left click) and A1 (right click), each to GND.

## Build

- Board: Adafruit Feather 32u4
- Library: Adafruit BluefruitLE nRF51
- Open `Feather_AirMouse_BMI160_v2.ino` in the Arduino IDE, upload, then pair "COMET AIRMOUSE" in your OS Bluetooth settings.

## Notes

Expect roughly 30-60 cursor updates per second, which is the practical ceiling for a text-command BLE module. A board with built-in BLE HID, such as the Feather nRF52840, would allow 100+.
