#!/usr/bin/env bash
set -euo pipefail
g++ -std=c++17 -O2 -pthread virtual_esp32.cpp -o /tmp/virtual-esp32
echo "Virtual ESP32 build: PASS"
