# Virtual ESP32 Rover

Software-only ESP32-compatible digital twin for claude-robot.

The rover provides TCP on port 5000 and UDP discovery on port 4210, differential-drive physics, ultrasonic distance, heading, battery, collision detection, and a 600 x 400 cm virtual world with a 20 cm grid.

The telemetry includes the rover pose, obstacles, collision state, and travelled path so claude-robot can build and display its own navigation map.

Build: `g++ -std=c++17 -O2 -pthread virtual_esp32.cpp -o virtual-esp32`

Run: `./virtual-esp32`

JSON protocol: `{"type":"hello"}` and `{"type":"command","cmd":"FORWARD","arg":70}`.

Legacy requests remain supported: PING, GET_HARDWARE, GET_TELEMETRY, GET_WORLD, and COMMAND commands.

## Firestore control contract

The browser simulator uses the shared Firestore documents:

- `robots/VESP32-01` advertises identity and the hardware description.
- `robots/VESP32-01/control/current` carries sequenced, expiring commands.
- `robots/VESP32-01/state/current` carries telemetry, pose, actuators, world data, hardware capabilities, and command acknowledgements.

Supported cloud commands are `FORWARD`, `BACKWARD`, `TURN_LEFT`, `TURN_RIGHT`, `STOP`, and `SET_SPEED`. Unknown commands are rejected with an `UNSUPPORTED_COMMAND` acknowledgement/error; expired commands are not executed. The simulator stops the motors when its command watchdog expires.

The matching Firestore rules validate the command schema and range. The rules file lives in the `claude-robot` repository; pushing either GitHub Pages site does not deploy Firestore rules. Deploy and verify the rules in the Firebase project before relying on them. Anonymous browser sign-in is not a trusted identity boundary, so this cloud control path is intended for simulation. Do not connect physical hardware to these public client-write rules without adding server-side authorization and restricting device telemetry writes.

