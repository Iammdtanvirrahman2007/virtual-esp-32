#!/usr/bin/env bash
set -euo pipefail

binary="$(mktemp /tmp/virtual-esp32.XXXXXX)"
log="$(mktemp /tmp/virtual-esp32-log.XXXXXX)"
cleanup() {
  if [[ -n "${server_pid:-}" ]]; then
    kill "$server_pid" 2>/dev/null || true
    wait "$server_pid" 2>/dev/null || true
  fi
  rm -f "$binary" "$log"
}
trap cleanup EXIT

g++ -std=c++17 -O2 -pthread virtual_esp32.cpp -o "$binary"
echo "Virtual ESP32 build: PASS"

"$binary" >"$log" 2>&1 &
server_pid=$!

python3 - <<'PY'
import json
import socket
import time

deadline = time.monotonic() + 8
sock = None
while time.monotonic() < deadline:
    try:
        sock = socket.create_connection(("127.0.0.1", 5000), timeout=0.5)
        break
    except OSError:
        time.sleep(0.1)
if sock is None:
    raise SystemExit("Virtual ESP32 TCP server did not start")

sock.settimeout(2)
stream = sock.makefile("rwb", buffering=0)

def request(line):
    stream.write((line + "\\n").encode())
    response = stream.readline()
    if not response:
        raise AssertionError(f"No response for {line}")
    return response.decode().strip()

assert request("PING") == "PONG"
hardware = json.loads(request("GET_HARDWARE"))
assert hardware["type"] == "wheeled"
assert any(sensor["id"] == "front_distance" for sensor in hardware["sensors"])

ack = request("COMMAND SET_MOTOR_L -35")
assert ack == "ACK SET_MOTOR_L -35", ack
state = json.loads(request("GET_TELEMETRY"))["state"]
assert state["actuators"]["motor_l"] == -35, state["actuators"]
assert request("COMMAND STOP 0") == "ACK STOP 0"
print("Virtual ESP32 TCP protocol and signed motor command: PASS")
sock.close()
PY
