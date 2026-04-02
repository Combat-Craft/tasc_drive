# Rover Dashboard Prototype

This is a ROS 2 Python package that provides:
- a custom PySide6 GUI dashboard (`dashboard`)
- a ROS 2 aggregation backend node (`hardware_backend`)

## Features
- dark-themed operator dashboard
- relay subsystem cards with ON/OFF commands
- drive motor telemetry cards with position, velocity, and temperature
- software kill button with confirmation dialog
- summary strip for battery, relay current, online motors, and faults
- simplified relay topology visualization
- detail panel for relays and motors
- event log panel
- aggregation backend that combines relay and motor data for the GUI
- internal demo mode if ROS 2 is not available

## ROS 2 topic design
- `/rover/gui/telemetry` (`std_msgs/String`) JSON payload with subsystem data
- `/rover/gui/heartbeat` (`std_msgs/String`) timestamp string
- `/rover/gui/command` (`std_msgs/String`) JSON command payload
- `/rover/relay_board/command` (`std_msgs/String`) JSON relay command payload
- `/rover/relay_board/state` (`std_msgs/String`) JSON relay state payload
- `/rover/relay_board/heartbeat` (`std_msgs/String`) relay controller heartbeat
- `/rover/drive/motor_telemetry` (`std_msgs/String`) JSON motor telemetry payload

## Recommended next steps
1. Replace JSON-over-String with custom messages once the interface stabilizes.
2. Add interlock logic in the backend node, not in the GUI.
3. Add startup dependency ordering (for example motor drivers before drive enable).
4. Add persistent fault history and brownout warnings.
5. Add measured rail voltages/current sensors instead of placeholder relay-side values.

## Build and run
From your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
cp -r /path/to/rover_dashboard .
cd ~/ros2_ws
colcon build --packages-select rover_dashboard
source install/setup.bash
```

Run backend and GUI in separate terminals:

```bash
ros2 run rover_dashboard hardware_backend
ros2 run rover_dashboard dashboard
```

## Standalone GUI only
If you just want to preview the layout and you have PySide6 installed:

```bash
python3 rover_dashboard/rover_dashboard/dashboard.py
```

That launches the built-in demo telemetry generator.
