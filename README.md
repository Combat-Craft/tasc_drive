# tasc_drive

`tasc_drive` is now the ROS 2 workspace root. ROS packages live directly under
`src/`:

- [`src/drive_bringup`](src/drive_bringup): launch files for backend, frontend, teleop, and full drive bringup
- [`src/drive_control`](src/drive_control): helper nodes such as motor position publishing and Jetson relay control
- [`src/drive_description`](src/drive_description): URDF/xacro and RViz configuration
- [`src/phidgets_hardware`](src/phidgets_hardware): `ros2_control` hardware interface, controller config, and PS4 teleop
- [`src/rover_dashboard`](src/rover_dashboard): dashboard GUI and hardware backend
- [`extras/esp32_relay_controller`](extras/esp32_relay_controller): ESP32 micro-ROS relay controller sketch and notes

The previous `backend/` and `frontend/` workspaces are preserved in place with
`COLCON_IGNORE` markers, so `colcon` builds only the combined root workspace.

## Build

From this directory:

```bash
colcon build
source install/setup.bash
```

To run tests:

```bash
colcon test
colcon test-result --verbose
```

## Launch

Backend hardware/control stack:

```bash
ros2 launch drive_bringup backend.launch.py
```

Frontend joystick/dashboard stack:

```bash
ros2 launch drive_bringup frontend.launch.py
```

Full Giskard drive bringup:

```bash
ros2 launch drive_bringup giskard_bringup.launch.py
```

Standalone teleop/dashboard:

```bash
ros2 launch drive_bringup teleop.launch.py
```

## Hardware Notes

The system integrates relay-controlled motor power, Phidgets BLDC telemetry,
`ros2_control`, joystick teleop, RViz model publication, and the PyQt rover
dashboard.

The relay controller sketch is here:

- [`relay_controller_wifi.ino`](extras/esp32_relay_controller/relay_controller_wifi/relay_controller_wifi.ino)

If you are using an Ubuntu VM, use bridged networking and configure the ESP32
with the VM IP address for the micro-ROS agent.
