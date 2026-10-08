# Virtual ESP32 Rover

Software-only ESP32-compatible digital twin for claude-robot.

The rover provides TCP on port 5000 and UDP discovery on port 4210, differential-drive physics, ultrasonic distance, heading, battery, collision detection, and a 600 x 400 cm virtual world with a 20 cm grid.

The telemetry includes the rover pose, obstacles, collision state, and travelled path so claude-robot can build and display its own navigation map.

Build: `g++ -std=c++17 -O2 -pthread virtual_esp32.cpp -o virtual-esp32`

Run: `./virtual-esp32`

JSON protocol: `{"type":"hello"}` and `{"type":"command","cmd":"FORWARD","arg":70}`.

Legacy requests remain supported: PING, GET_HARDWARE, GET_TELEMETRY, GET_WORLD, and COMMAND commands.
