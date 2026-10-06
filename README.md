# Virtual ESP32

A console-first virtual ESP32 for testing Robot Control Core without physical hardware.

## Build

g++ -std=c++17 -O2 -pthread virtual_esp32.cpp -o virtual-esp32

## Run

./virtual-esp32

TCP server: 0.0.0.0:5000

For local testing, configure Robot Control Core with:
IP: 127.0.0.1
Port: 5000

## Supported protocol

PING
GET_HARDWARE
GET_TELEMETRY
COMMAND DRIVE_FORWARD 70
COMMAND DRIVE_BACKWARD 50
COMMAND TURN_LEFT 50
COMMAND TURN_RIGHT 50
COMMAND SET_MOTOR_L 40
COMMAND SET_MOTOR_R 40
COMMAND STOP 0
COMMAND E_STOP 0

The console prints received requests and transmitted responses live.

The virtual device also maintains basic simulated telemetry such as front distance, motor values, battery and temperature.