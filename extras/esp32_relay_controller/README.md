# ESP32 Relay Controller

This folder contains a micro-ROS Wi-Fi sketch for an `ESP32-WROOM-32 DOIT ESP32 DEVKIT V1`
controlling up to 8 relay channels.

The current test setup is:

- `LED 1` to `LED 4` connected to relay channels 1 to 4
- relay channels 5 to 8 present in code but disabled by default
- ESP32 connected to ROS 2 over Wi-Fi through a micro-ROS agent running on the laptop or VM

## Default Behavior

- 8 relay channels are defined in the sketch
- `LED 1` to `LED 4` are enabled by default
- channels 5 to 8 are present but marked `configured = false`
- unconfigured channels do not generate control errors and are ignored by the dashboard backend

## Topics

- subscribes: `/rover/relay_board/command`
- publishes: `/rover/relay_board/state`
- publishes: `/rover/relay_board/heartbeat`

## Wiring Diagram Placeholder

Add your wiring diagram image or schematic here.

Suggested future content:

- ESP32 pin to relay input mapping
- relay board power wiring
- LED test load wiring
- common ground connection
- external power source wiring

## Important Electrical Note

Most 5V relay boards should be powered from a proper 5V rail, not directly from a 3.3V GPIO pin.
The ESP32 GPIOs should only be used as logic control lines.

Also make sure:

- the ESP32 ground and relay board ground are tied together
- each LED test circuit includes a proper resistor
- the relay board input polarity matches the sketch configuration

The sketch currently assumes the relay inputs are `active low`, which is common for many relay boards. It should be set to `false` for our board.

## What This Setup Does

The ESP32:

- joins your Wi-Fi network
- connects to the micro-ROS agent over UDP
- listens for relay commands from ROS 2
- toggles relay channels
- publishes relay state and heartbeat back into ROS 2

The laptop or VM:

- runs the micro-ROS agent
- runs the rover dashboard backend
- runs the GUI
- forwards GUI button presses to the ESP32

## Files Used

- sketch: [relay_controller_wifi.ino](/home/ahmedtabl/gui_ws/tasc_drive/rover_dashboard_pkg/esp32_relay_controller/relay_controller_wifi/relay_controller_wifi.ino)
- backend: [hardware_backend.py](/home/ahmedtabl/gui_ws/tasc_drive/rover_dashboard_pkg/rover_dashboard/rover_dashboard/hardware_backend.py)
- GUI: [dashboard.py](/home/ahmedtabl/gui_ws/tasc_drive/rover_dashboard_pkg/rover_dashboard/rover_dashboard/dashboard.py)

## Beginner Bring-Up Guide

Follow these steps in order.

## 1. Make Sure Your Network Setup Is Correct

Your ESP32 and ROS 2 machine need to be on the same network.

If you are using an Ubuntu VM:

- set the VM network adapter to `Bridged Adapter`
- use the `IP address of the VM`, not the host machine
- make sure the VM can reach other devices on the LAN

This is important because the ESP32 will connect to the micro-ROS agent using the IP address you place in the sketch.

## 2. Install Arduino Support

You can use either:

- Arduino IDE
- VS Code with PlatformIO

For beginners, Arduino IDE is usually the easiest for `.ino` sketches.

If you use Arduino IDE:

1. Open Arduino IDE
2. Go to `File > Preferences`
3. Add this URL to `Additional Boards Manager URLs`

```text
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

4. Open `Tools > Board > Boards Manager`
5. Search for `esp32`
6. Install `esp32 by Espressif Systems`

Then select:

- Board: `DOIT ESP32 DEVKIT V1`

## 3. Install The micro-ROS Arduino Library

In Arduino IDE:

1. Open `Sketch > Include Library > Manage Libraries`
2. Search for `micro_ros_arduino`
3. Install it

If it does not appear in the Library Manager, install it manually from the official micro-ROS Arduino repository ZIP.

## 4. Open The Sketch

Open this file:

[relay_controller_wifi.ino](/home/ahmedtabl/gui_ws/tasc_drive/rover_dashboard_pkg/esp32_relay_controller/relay_controller_wifi/relay_controller_wifi.ino)

The default configured channels are:

- `LED 1` on GPIO `16`
- `LED 2` on GPIO `17`
- `LED 3` on GPIO `18`
- `LED 4` on GPIO `19`

## 5. Set Your Wi-Fi Credentials And Agent IP

In the sketch, find:

```cpp
static const char *kWifiSsid = "YOUR_WIFI_SSID";
static const char *kWifiPassword = "YOUR_WIFI_PASSWORD";
static const char *kAgentIp = "192.168.1.10";
static const uint16_t kAgentPort = 8888;
```

Replace them with your real values.

Example:

```cpp
static const char *kWifiSsid = "MyWifi";
static const char *kWifiPassword = "mypassword";
static const char *kAgentIp = "192.168.1.42";
static const uint16_t kAgentPort = 8888;
```

`kAgentIp` must be the IP of the machine running the micro-ROS agent.

If you are using a VM, this must be the `VM IP`, not the host IP.

## 6. Find The Correct IP Address

On the Ubuntu machine that will run the micro-ROS agent, run:

```bash
hostname -I
```

or:

```bash
ip addr
```

Use the local LAN IP address of the active network interface.

Typical examples:

- `192.168.1.x`
- `10.0.0.x`

If you are on a VM in bridged mode, use the VM LAN IP shown there.

## 7. Confirm Or Adjust The GPIO Mapping

The sketch currently maps:

- relay input 1 -> GPIO 16
- relay input 2 -> GPIO 17
- relay input 3 -> GPIO 18
- relay input 4 -> GPIO 19

If your wiring is different, edit the `kRelayConfig` table in the sketch.

## 8. Wire The Hardware

At minimum for the LED test:

- connect ESP32 `GND` to relay board `GND`
- connect relay input pins for channels 1 to 4 to the configured ESP32 GPIO pins
- power the relay board from a proper 5V supply if required by the module
- wire each LED through a resistor

For each LED relay output, a common beginner-friendly setup is:

- relay `COM` to positive supply
- relay `NO` to the LED positive path
- LED negative path back to supply ground

Using `NO` means the LED is off by default and turns on when the relay energizes.

## 9. Upload The Sketch To The ESP32

In Arduino IDE:

1. Connect the ESP32 by USB
2. Select board: `DOIT ESP32 DEVKIT V1`
3. Select the correct serial port in `Tools > Port`
4. Click `Verify`
5. Click `Upload`

If upload fails:

- press and hold the `BOOT` button on the ESP32
- click `Upload`
- release `BOOT` once flashing starts

## 10. Start The micro-ROS Agent

On the ROS 2 machine, start the micro-ROS agent.

If you use Docker:

```bash
docker run --rm -it --net=host microros/micro-ros-agent:humble udp4 --port 8888
```

If you installed it as a ROS 2 package:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Leave this terminal open.

## 11. Build The ROS 2 Workspace

From the workspace root:

```bash
cd /home/ahmedtabl/gui_ws
colcon build --packages-select rover_dashboard
source install/setup.bash
```

If you want to rebuild everything:

```bash
cd /home/ahmedtabl/gui_ws
colcon build
source install/setup.bash
```

## 12. Start The Dashboard Backend

Open a new terminal:

```bash
cd /home/ahmedtabl/gui_ws
source install/setup.bash
ros2 run rover_dashboard hardware_backend
```

Leave it running.

## 13. Start The GUI

Open another terminal:

```bash
cd /home/ahmedtabl/gui_ws
source install/setup.bash
ros2 run rover_dashboard dashboard
```

The GUI should open and show the relay cards.

## 14. Check That The ESP32 Is Publishing

Open another terminal:

```bash
cd /home/ahmedtabl/gui_ws
source install/setup.bash
ros2 topic list
```

You should see at least:

- `/rover/relay_board/command`
- `/rover/relay_board/state`
- `/rover/relay_board/heartbeat`
- `/rover/gui/command`
- `/rover/gui/telemetry`
- `/rover/gui/heartbeat`

Now inspect the relay state:

```bash
ros2 topic echo /rover/relay_board/state
```

If the ESP32 is connected properly, you should see JSON state messages arriving about every 0.5 seconds.

You can also inspect heartbeat:

```bash
ros2 topic echo /rover/relay_board/heartbeat
```

## 15. Test The Relay From The GUI

In the GUI:

- click `Power ON` for `LED 1`
- the backend forwards the command to `/rover/relay_board/command`
- the ESP32 receives it and toggles the relay
- the LED turns on
- the ESP32 publishes updated state
- the GUI updates

## 16. Test Without The GUI If Needed

If you want to debug from the terminal first, publish a relay command manually:

Turn on `LED 1`:

```bash
ros2 topic pub /rover/relay_board/command std_msgs/msg/String "{data: '{\"type\":\"set_power\",\"target\":\"LED 1\",\"enable\":true}'}" --once
```

Turn off `LED 1`:

```bash
ros2 topic pub /rover/relay_board/command std_msgs/msg/String "{data: '{\"type\":\"set_power\",\"target\":\"LED 1\",\"enable\":false}'}" --once
```

This is a very helpful debugging step because it removes the GUI from the equation.

## Recommended Terminal Layout

Use four terminals:

Terminal 1:

```bash
docker run --rm -it --net=host microros/micro-ros-agent:humble udp4 --port 8888
```

Terminal 2:

```bash
cd /home/ahmedtabl/gui_ws
source install/setup.bash
ros2 run rover_dashboard hardware_backend
```

Terminal 3:

```bash
cd /home/ahmedtabl/gui_ws
source install/setup.bash
ros2 run rover_dashboard dashboard
```

Terminal 4:

```bash
cd /home/ahmedtabl/gui_ws
source install/setup.bash
ros2 topic echo /rover/relay_board/state
```

## What Success Looks Like

The setup is working when:

- the ESP32 powers up
- the ESP32 joins Wi-Fi
- the micro-ROS agent shows a client connection
- `/rover/relay_board/state` publishes regularly
- the dashboard stops waiting for relay data
- clicking `Power ON` toggles the correct LED

## Common Problems

### ESP32 Upload Fails

- wrong board selected
- wrong serial port selected
- USB cable does not support data
- need to hold the `BOOT` button during upload

### ESP32 Connects To Wi-Fi But No ROS Traffic Appears

- wrong IP address in `kAgentIp`
- using host IP instead of VM IP
- VM network adapter is not set to `Bridged Adapter`
- micro-ROS agent is not running
- firewall or network policy blocks UDP traffic
- ESP32 and ROS machine are on different networks

### Relay Clicks But LED Logic Looks Backwards

- relay board trigger polarity is opposite of expected
- `active_low` may need to be changed in the sketch

### GUI Button Does Nothing

- backend is not running
- ESP32 is not publishing to ROS 2
- target names do not match relay names in the sketch

Useful debug commands:

```bash
ros2 topic echo /rover/relay_board/command
ros2 topic echo /rover/relay_board/state
ros2 topic echo /rover/relay_board/heartbeat
```

## Best Bring-Up Order

For the smoothest debugging experience, do it in this order:

1. Flash the ESP32
2. Start the micro-ROS agent
3. Confirm `/rover/relay_board/state` is publishing
4. Manually publish one relay command from the terminal
5. Confirm the LED toggles
6. Start the dashboard backend
7. Start the GUI
8. Confirm the GUI buttons also work
