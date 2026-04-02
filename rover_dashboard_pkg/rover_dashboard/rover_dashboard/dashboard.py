import json
import math
import os
import sys
import threading
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except Exception:
    rclpy = None
    Node = object
    String = object
    ROS_AVAILABLE = False

from PySide6.QtCore import QObject, QProcess, Qt, QTimer, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QProgressBar,
    QScrollArea,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

try:
    from .shared_topics import (
        DEFAULT_MOTORS,
        TOPIC_GUI_COMMAND,
        TOPIC_GUI_HEARTBEAT,
        TOPIC_GUI_TELEMETRY,
    )
except ImportError:
    from shared_topics import (  # type: ignore
        DEFAULT_MOTORS,
        TOPIC_GUI_COMMAND,
        TOPIC_GUI_HEARTBEAT,
        TOPIC_GUI_TELEMETRY,
    )

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def format_elapsed(seconds: int) -> str:
    hours, remainder = divmod(seconds, 3600)
    minutes, secs = divmod(remainder, 60)
    return f'{hours:02d}:{minutes:02d}:{secs:02d}'


def humanize_joint_name(name: str) -> str:
    mapping = {
        'front_left_wheel_joint': 'Front Left Wheel',
        'rear_left_wheel_joint': 'Rear Left Wheel',
        'front_right_wheel_joint': 'Front Right Wheel',
        'rear_right_wheel_joint': 'Rear Right Wheel',
    }
    return mapping.get(name, name.replace('_', ' ').title())


class TelemetryBridge(QObject):
    telemetry_received = Signal(dict)
    heartbeat_received = Signal(str)
    ros_status_changed = Signal(str)


class RosDashboardNode(Node):
    def __init__(self, bridge: TelemetryBridge):
        super().__init__('rover_dashboard_ui')
        self.bridge = bridge
        self.telemetry_sub = self.create_subscription(String, TOPIC_GUI_TELEMETRY, self._telemetry_cb, 10)
        self.heartbeat_sub = self.create_subscription(String, TOPIC_GUI_HEARTBEAT, self._heartbeat_cb, 10)
        self.command_pub = self.create_publisher(String, TOPIC_GUI_COMMAND, 10)
        self.bridge.ros_status_changed.emit('ROS 2 connected')

    def _telemetry_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.bridge.telemetry_received.emit(payload)
        except Exception as exc:
            self.get_logger().error(f'Failed to parse telemetry JSON: {exc}')

    def _heartbeat_cb(self, msg: String) -> None:
        self.bridge.heartbeat_received.emit(msg.data)

    def send_command(self, command: Dict) -> None:
        msg = String()
        msg.data = json.dumps(command)
        self.command_pub.publish(msg)


class RosSpinThread(threading.Thread):
    def __init__(self, node: RosDashboardNode):
        super().__init__(daemon=True)
        self.node = node
        self._running = True

    def run(self) -> None:
        while self._running and ROS_AVAILABLE:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def stop(self) -> None:
        self._running = False


@dataclass
class MotorControlState:
    name: str
    display_name: str
    enabled: bool = False
    attached: bool = False
    channel: int = 0
    voltage: float = 0.0
    current: float = 0.0
    position: float = 0.0
    velocity: float = 0.0
    temperature: float = 0.0
    health: str = 'offline'
    fault: str = 'No data'
    relay_fault: str = 'No data'
    motor_fault: str = 'No data'
    last_command: str = 'None'
    last_update: str = '--'


class MetricPill(QFrame):
    def __init__(self, label: str, value: str = '--') -> None:
        super().__init__()
        self.setObjectName('metricPill')
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        self.title = QLabel(label)
        self.title.setObjectName('metricTitle')
        self.value = QLabel(value)
        self.value.setObjectName('metricValue')
        layout.addWidget(self.title)
        layout.addWidget(self.value)

    def set_value(self, value: str) -> None:
        self.value.setText(value)


class StatusBadge(QLabel):
    COLORS = {
        'healthy': '#1db954',
        'warning': '#f0ad4e',
        'fault': '#e74c3c',
        'offline': '#6c757d',
        'connected': '#1db954',
    }

    def __init__(self, text: str = 'OFFLINE', status_key: str = 'offline') -> None:
        super().__init__(text)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumWidth(96)
        self.set_status(text, status_key)

    def set_status(self, text: str, status_key: str) -> None:
        color = self.COLORS.get(status_key, '#6c757d')
        self.setText(text)
        self.setStyleSheet(
            f'background:{color}; color:white; border-radius:10px; padding:5px 10px; font-weight:700;'
        )


class MotorControlCard(QFrame):
    toggled = Signal(str, bool)
    details_requested = Signal(str)

    def __init__(self, motor_name: str) -> None:
        super().__init__()
        self.motor_name = motor_name
        self.setObjectName('motorCard')
        self.state = MotorControlState(name=motor_name, display_name=humanize_joint_name(motor_name))
        self._build_ui()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(10)

        header = QHBoxLayout()
        title_col = QVBoxLayout()
        title = QLabel(self.state.display_name)
        title.setObjectName('cardTitle')
        subtitle = QLabel(self.motor_name)
        subtitle.setObjectName('cardSubTitle')
        title_col.addWidget(title)
        title_col.addWidget(subtitle)
        self.status_badge = StatusBadge('OFFLINE', 'offline')
        header.addLayout(title_col)
        header.addStretch(1)
        header.addWidget(self.status_badge)
        root.addLayout(header)

        self.metrics = {
            'position': MetricPill('Position', '--'),
            'velocity': MetricPill('Velocity', '--'),
            'temperature': MetricPill('Temp', '--'),
            'current': MetricPill('Relay Current', '--'),
        }
        metrics_layout = QGridLayout()
        metrics_layout.setSpacing(10)
        for idx, pill in enumerate(self.metrics.values()):
            row, col = divmod(idx, 2)
            metrics_layout.addWidget(pill, row, col)
        root.addLayout(metrics_layout)

        self.meta_label = QLabel('Channel -- | Bus -- V | Command None')
        self.meta_label.setObjectName('metaLabel')
        root.addWidget(self.meta_label)

        self.fault_label = QLabel('Fault: No data')
        self.fault_label.setWordWrap(True)
        self.fault_label.setObjectName('faultLabel')
        root.addWidget(self.fault_label)

        button_row = QHBoxLayout()
        self.toggle_button = QPushButton('Power ON')
        self.toggle_button.clicked.connect(self._emit_toggle)
        detail_button = QPushButton('Details')
        detail_button.clicked.connect(lambda: self.details_requested.emit(self.motor_name))
        button_row.addWidget(self.toggle_button)
        button_row.addWidget(detail_button)
        root.addLayout(button_row)

    def _emit_toggle(self) -> None:
        self.toggled.emit(self.motor_name, not self.state.enabled)

    def update_state(self, state: MotorControlState) -> None:
        self.state = state
        self.metrics['position'].set_value(f'{state.position:0.3f} rad')
        self.metrics['velocity'].set_value(f'{state.velocity:0.3f} rad/s')
        self.metrics['temperature'].set_value(f'{state.temperature:0.1f} °C')
        self.metrics['current'].set_value(f'{state.current:0.2f} A')

        self.status_badge.set_status(state.health.upper(), state.health)
        self.meta_label.setText(
            f'Channel {state.channel} | Bus {state.voltage:0.1f} V | Last {state.last_command}'
        )
        self.fault_label.setText(f'Fault: {state.fault}')
        self.toggle_button.setText('Power OFF' if state.enabled else 'Power ON')


class UrdfVisualLauncher(QFrame):
    def __init__(self) -> None:
        super().__init__()
        self.setObjectName('driveVisual')
        self.setMinimumHeight(280)
        self.process: Optional[QProcess] = None
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(12)

        title = QLabel('Actual URDF 3D View')
        title.setObjectName('cardTitle')
        description = QLabel(
            'This dashboard now uses RViz for the real 3D robot model view.\n'
            'Start the drive stack, then open RViz to see the live URDF driven by joint states.'
        )
        description.setWordWrap(True)
        description.setObjectName('metaLabel')
        self.status_label = QLabel('RViz status: not launched')
        self.status_label.setObjectName('metaLabel')
        self.open_button = QPushButton('Open 3D URDF View In RViz')
        self.open_button.clicked.connect(self.launch_rviz)

        layout.addWidget(title)
        layout.addWidget(description)
        layout.addStretch(1)
        layout.addWidget(self.status_label)
        layout.addWidget(self.open_button)

    def launch_rviz(self) -> None:
        if get_package_share_directory is None:
            self.status_label.setText('RViz status: ament index unavailable in this environment')
            return

        try:
            description_share = get_package_share_directory('drive_description')
        except Exception as exc:
            self.status_label.setText(f'RViz status: could not find drive_description ({exc})')
            return

        rviz_config = os.path.join(description_share, 'rviz', 'drive_model.rviz')
        if not os.path.exists(rviz_config):
            self.status_label.setText(f'RViz status: config missing at {rviz_config}')
            return

        launched = QProcess.startDetached('rviz2', ['-d', rviz_config])
        self.status_label.setText(
            'RViz status: launched' if launched else 'RViz status: failed to launch rviz2'
        )


class DetailPanel(QGroupBox):
    def __init__(self) -> None:
        super().__init__('Motor Details')
        layout = QVBoxLayout(self)
        self.title = QLabel('Select a motor card')
        self.title.setObjectName('detailTitle')
        self.info = QTextEdit()
        self.info.setReadOnly(True)
        self.info.setMinimumHeight(260)
        layout.addWidget(self.title)
        layout.addWidget(self.info)

    def render_motor(self, state: MotorControlState) -> None:
        self.title.setText(state.display_name)
        self.info.setPlainText(
            f'Joint name: {state.name}\n'
            f'Powered: {state.enabled}\n'
            f'Attached: {state.attached}\n'
            f'Relay channel: {state.channel}\n'
            f'Bus voltage: {state.voltage:0.2f} V\n'
            f'Relay current: {state.current:0.2f} A\n'
            f'Position: {state.position:0.4f} rad\n'
            f'Velocity: {state.velocity:0.4f} rad/s\n'
            f'Temperature: {state.temperature:0.1f} °C\n'
            f'Health: {state.health}\n'
            f'Combined fault: {state.fault}\n'
            f'Relay fault: {state.relay_fault}\n'
            f'Motor fault: {state.motor_fault}\n'
            f'Last command: {state.last_command}\n'
            f'Last motor update: {state.last_update}\n'
        )


class HeaderBar(QFrame):
    kill_requested = Signal()

    def __init__(self) -> None:
        super().__init__()
        self.setObjectName('headerBar')
        layout = QHBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)

        title_col = QVBoxLayout()
        title = QLabel('Rover Motor Power & Telemetry Dashboard')
        title.setObjectName('mainTitle')
        subtitle = QLabel('Power switching and live wheel telemetry in one view')
        subtitle.setObjectName('subTitle')
        title_col.addWidget(title)
        title_col.addWidget(subtitle)
        layout.addLayout(title_col)

        layout.addStretch(1)
        self.ros_badge = StatusBadge('DEMO MODE', 'offline')
        self.heartbeat_label = QLabel('Elapsed: 00:00:00')
        self.mode_badge = StatusBadge('SAFE', 'warning')
        self.kill_button = QPushButton('SOFTWARE KILL')
        self.kill_button.setObjectName('killButton')
        self.kill_button.clicked.connect(self.kill_requested.emit)
        layout.addWidget(self.ros_badge)
        layout.addWidget(self.heartbeat_label)
        layout.addWidget(self.mode_badge)
        layout.addWidget(self.kill_button)


class SummaryStrip(QFrame):
    def __init__(self) -> None:
        super().__init__()
        self.setObjectName('summaryStrip')
        layout = QHBoxLayout(self)
        self.items = {
            'battery': MetricPill('Battery', '-- V'),
            'current': MetricPill('Total Relay Current', '-- A'),
            'motors': MetricPill('Motors Online', '--'),
            'faults': MetricPill('Fault Count', '--'),
        }
        for item in self.items.values():
            layout.addWidget(item)

        self.battery_bar = QProgressBar()
        self.battery_bar.setFormat('Battery %p%')
        self.battery_bar.setValue(0)
        self.battery_bar.setMinimumWidth(180)
        layout.addWidget(self.battery_bar)

    def update_summary(self, battery_voltage: float, total_current: float, motors_online: int, fault_count: int) -> None:
        self.items['battery'].set_value(f'{battery_voltage:0.2f} V')
        self.items['current'].set_value(f'{total_current:0.2f} A')
        self.items['motors'].set_value(str(motors_online))
        self.items['faults'].set_value(str(fault_count))
        pct = int(clamp((battery_voltage - 20.0) / 8.0 * 100.0, 0.0, 100.0))
        self.battery_bar.setValue(pct)


class LogPanel(QGroupBox):
    def __init__(self) -> None:
        super().__init__('Event Log')
        layout = QVBoxLayout(self)
        self.text = QTextEdit()
        self.text.setReadOnly(True)
        layout.addWidget(self.text)

    def add_entry(self, entry: str) -> None:
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.text.append(f'[{timestamp}] {entry}')


class DashboardWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle('Rover Dashboard')
        self.resize(1600, 980)
        self.bridge = TelemetryBridge()
        self.bridge.telemetry_received.connect(self.on_telemetry)
        self.bridge.heartbeat_received.connect(self.on_heartbeat)
        self.bridge.ros_status_changed.connect(self.on_ros_status)

        self.ros_node: Optional[RosDashboardNode] = None
        self.spin_thread: Optional[RosSpinThread] = None
        self.demo_timer = None
        self.elapsed_timer = QTimer(self)
        self.elapsed_timer.timeout.connect(self._update_elapsed_label)
        self.elapsed_timer.start(1000)
        self.gui_launch_time = datetime.now()
        self.seen_backend_heartbeat = False
        self.current_selected: Optional[str] = None
        self.latest_payload = {}

        self.motor_cards: Dict[str, MotorControlCard] = {}
        self.motor_states: Dict[str, MotorControlState] = {
            name: MotorControlState(name=name, display_name=humanize_joint_name(name))
            for name in DEFAULT_MOTORS
        }

        self._build_ui()
        self._apply_styles()
        self._setup_ros_or_demo()
        self._ensure_motor_cards(DEFAULT_MOTORS)
        self._update_elapsed_label()

    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        self.header = HeaderBar()
        self.header.kill_requested.connect(self.on_kill_requested)
        root.addWidget(self.header)

        self.summary = SummaryStrip()
        root.addWidget(self.summary)

        main_grid = QGridLayout()
        main_grid.setSpacing(12)
        root.addLayout(main_grid, 1)

        visual_box = QGroupBox('Drive Visual')
        visual_layout = QVBoxLayout(visual_box)
        self.rover_visual = UrdfVisualLauncher()
        visual_layout.addWidget(self.rover_visual)
        main_grid.addWidget(visual_box, 0, 0, 1, 2)

        control_box = QGroupBox('Motor Power & Telemetry')
        control_layout = QVBoxLayout(control_box)
        control_scroll = QScrollArea()
        control_scroll.setWidgetResizable(True)
        control_host = QWidget()
        self.motor_grid = QGridLayout(control_host)
        self.motor_grid.setSpacing(12)
        control_scroll.setWidget(control_host)
        control_layout.addWidget(control_scroll)
        main_grid.addWidget(control_box, 1, 0, 1, 2)

        self.detail_panel = DetailPanel()
        main_grid.addWidget(self.detail_panel, 0, 2)

        self.log_panel = LogPanel()
        main_grid.addWidget(self.log_panel, 1, 2)

        main_grid.setColumnStretch(0, 2)
        main_grid.setColumnStretch(1, 2)
        main_grid.setColumnStretch(2, 1)
        main_grid.setRowStretch(1, 1)

        self.setCentralWidget(central)

    def _ensure_motor_cards(self, names) -> None:
        for name in names:
            if name in self.motor_cards:
                continue
            card = MotorControlCard(name)
            card.toggled.connect(self.on_toggle_motor)
            card.details_requested.connect(self.on_show_motor_details)
            self.motor_cards[name] = card
            self.motor_states.setdefault(
                name,
                MotorControlState(name=name, display_name=humanize_joint_name(name)),
            )
            row, col = divmod(len(self.motor_cards) - 1, 2)
            self.motor_grid.addWidget(card, row, col)

    def _apply_styles(self) -> None:
        self.setStyleSheet(
            """
            QWidget { background: #0f1720; color: #e5e7eb; font-family: Arial; font-size: 13px; }
            QMainWindow { background: #0f1720; }
            QGroupBox {
                border: 1px solid #263241; border-radius: 12px; margin-top: 12px;
                font-weight: 700; padding-top: 12px; background: #111827;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px 0 4px; }
            #headerBar, #summaryStrip, #motorCard, #driveVisual, #metricPill {
                background: #111827; border: 1px solid #263241; border-radius: 12px;
            }
            #mainTitle { font-size: 24px; font-weight: 800; }
            #subTitle, #cardSubTitle, #metaLabel { color: #9aa4b2; }
            #cardTitle, #detailTitle { font-size: 16px; font-weight: 700; }
            #metricTitle { color: #93a3b8; font-size: 11px; }
            #metricValue { font-size: 18px; font-weight: 700; }
            #faultLabel { color: #f8d7da; }
            QPushButton {
                background: #1f6feb; border: none; border-radius: 10px; padding: 10px 12px;
                font-weight: 700;
            }
            QPushButton:hover { background: #388bfd; }
            QPushButton#killButton { background: #e74c3c; min-width: 160px; }
            QPushButton#killButton:hover { background: #ff6655; }
            QTextEdit, QScrollArea { background: #0b1220; border: 1px solid #263241; border-radius: 10px; }
            QProgressBar {
                background: #0b1220; border: 1px solid #263241; border-radius: 8px; text-align: center;
                min-height: 24px;
            }
            QProgressBar::chunk { background: #1db954; border-radius: 8px; }
            """
        )

    def _setup_ros_or_demo(self) -> None:
        if ROS_AVAILABLE:
            try:
                rclpy.init(args=None)
                self.ros_node = RosDashboardNode(self.bridge)
                self.spin_thread = RosSpinThread(self.ros_node)
                self.spin_thread.start()
                self.log_panel.add_entry('ROS 2 initialized. Waiting for motor power and telemetry...')
                return
            except Exception as exc:
                self.log_panel.add_entry(f'ROS 2 unavailable, starting demo mode: {exc}')

        self.header.ros_badge.set_status('DEMO MODE', 'offline')
        self.demo_timer = QTimer(self)
        self.demo_timer.timeout.connect(self._emit_demo_payload)
        self.demo_timer.start(500)
        self.log_panel.add_entry('Running internal motor dashboard demo mode.')

    def _emit_demo_payload(self) -> None:
        t = datetime.now().timestamp()
        self.on_telemetry(build_fake_payload(t))
        self.on_heartbeat(datetime.now().strftime('%H:%M:%S'))

    def _update_elapsed_label(self) -> None:
        elapsed = int((datetime.now() - self.gui_launch_time).total_seconds())
        self.header.heartbeat_label.setText(f'Elapsed: {format_elapsed(elapsed)}')

    def on_ros_status(self, text: str) -> None:
        self.header.ros_badge.set_status('ROS 2', 'connected')
        self.log_panel.add_entry(text)

    def on_heartbeat(self, beat: str) -> None:
        if beat and beat != '--' and not self.seen_backend_heartbeat:
            self.seen_backend_heartbeat = True
            self.log_panel.add_entry('Backend heartbeat detected.')

    def on_telemetry(self, payload: Dict) -> None:
        self.latest_payload = payload
        relays = payload.get('subsystems', {})
        motors = payload.get('motors', {})
        all_motor_names = list(dict.fromkeys(list(motors.keys()) + list(relays.keys()) + list(DEFAULT_MOTORS)))
        self._ensure_motor_cards(all_motor_names)

        fault_count = 0
        motors_online = 0

        for name in all_motor_names:
            relay_data = relays.get(name, {})
            motor_data = motors.get(name, {})
            state = self.motor_states.get(
                name,
                MotorControlState(name=name, display_name=humanize_joint_name(name)),
            )

            state.enabled = bool(relay_data.get('enabled', state.enabled))
            state.channel = int(relay_data.get('channel', state.channel))
            state.voltage = float(relay_data.get('voltage', state.voltage))
            state.current = float(relay_data.get('current', state.current))
            state.position = float(motor_data.get('position', state.position))
            state.velocity = float(motor_data.get('velocity', state.velocity))
            state.temperature = float(motor_data.get('temperature', state.temperature))
            state.attached = bool(motor_data.get('attached', state.attached))
            state.last_command = relay_data.get('last_command', state.last_command)
            state.last_update = motor_data.get('last_update', state.last_update)
            state.relay_fault = relay_data.get('fault', 'Relay telemetry unavailable')
            state.motor_fault = motor_data.get('fault', 'Motor telemetry unavailable')

            relay_health = relay_data.get('health', 'offline' if not state.enabled else 'healthy')
            motor_health = motor_data.get('health', 'offline' if not state.attached else 'healthy')

            if relay_health == 'fault' or motor_health == 'fault':
                state.health = 'fault'
            elif relay_health == 'warning' or motor_health == 'warning':
                state.health = 'warning'
            elif state.enabled and state.attached:
                state.health = 'healthy'
            elif state.enabled or state.attached:
                state.health = 'warning'
            else:
                state.health = 'offline'

            fault_parts = []
            if state.relay_fault not in {'None', 'Not energized', 'Relay telemetry unavailable'}:
                fault_parts.append(state.relay_fault)
            if state.motor_fault not in {'None', 'Motor telemetry unavailable'}:
                fault_parts.append(state.motor_fault)
            if not fault_parts:
                if not state.enabled:
                    fault_parts.append('Power path is off')
                elif not state.attached:
                    fault_parts.append('Motor controller not attached')
                else:
                    fault_parts.append('None')
            state.fault = ' | '.join(fault_parts)

            self.motor_states[name] = state
            self.motor_cards[name].update_state(state)

            if state.attached:
                motors_online += 1
            if state.health == 'fault':
                fault_count += 1

        bus = payload.get('bus', {})
        battery = float(bus.get('battery_voltage', 0.0))
        total_current = float(bus.get('total_current', 0.0))
        self.summary.update_summary(battery, total_current, motors_online, fault_count)
        mode = payload.get('meta', {}).get('mode', 'SAFE')
        mode_key = 'healthy' if mode == 'ARMED' else 'warning'
        self.header.mode_badge.set_status(mode, mode_key)

        if self.current_selected and self.current_selected in self.motor_states:
            self.detail_panel.render_motor(self.motor_states[self.current_selected])

    def on_toggle_motor(self, name: str, enable: bool) -> None:
        command = {
            'type': 'set_power',
            'target': name,
            'enable': enable,
            'timestamp': datetime.now().isoformat(),
        }
        self.log_panel.add_entry(f"Command sent: {'ENABLE' if enable else 'DISABLE'} {humanize_joint_name(name)}")
        if name in self.motor_states:
            self.motor_states[name].last_command = (
                f"{'ENABLE' if enable else 'DISABLE'} @ {datetime.now().strftime('%H:%M:%S')}"
            )
        if self.ros_node:
            self.ros_node.send_command(command)

    def on_show_motor_details(self, name: str) -> None:
        self.current_selected = name
        if name in self.motor_states:
            self.detail_panel.render_motor(self.motor_states[name])
            self.log_panel.add_entry(f'Detail view opened for {humanize_joint_name(name)}')

    def on_kill_requested(self) -> None:
        confirm = QMessageBox.question(
            self,
            'Confirm software kill',
            'Send a software kill command to disable motor power relays?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if confirm != QMessageBox.Yes:
            return

        command = {
            'type': 'software_kill',
            'target': 'all',
            'enable': False,
            'timestamp': datetime.now().isoformat(),
        }
        self.log_panel.add_entry('SOFTWARE KILL requested by operator.')
        if self.ros_node:
            self.ros_node.send_command(command)
        self.header.mode_badge.set_status('SAFE', 'warning')

    def closeEvent(self, event) -> None:
        try:
            if self.spin_thread:
                self.spin_thread.stop()
            if self.ros_node:
                self.ros_node.destroy_node()
            if ROS_AVAILABLE and rclpy is not None:
                rclpy.shutdown()
        except Exception:
            pass
        super().closeEvent(event)


def build_fake_payload(t: float) -> Dict:
    battery_voltage = 27.2 + 0.3 * math.sin(t / 9.0)
    payload = {
        'meta': {
            'mode': 'ARMED' if int(t) % 20 > 3 else 'SAFE',
        },
        'bus': {
            'battery_voltage': battery_voltage,
            'total_current': 3.6 + 0.8 * math.sin(t / 4.0),
        },
        'subsystems': {},
        'motors': {},
    }

    for idx, name in enumerate(DEFAULT_MOTORS):
        enabled = (idx + int(t)) % 4 != 0
        payload['subsystems'][name] = {
            'state': 'ON' if enabled else 'OFF',
            'enabled': enabled,
            'channel': idx,
            'voltage': 24.0 if enabled else 0.0,
            'current': 0.6 + 0.15 * math.sin(t / (idx + 2)) if enabled else 0.0,
            'temperature': 30.0 + idx,
            'health': 'healthy' if enabled else 'offline',
            'fault': 'None' if enabled else 'Power path is off',
            'last_command': 'AUTO DEMO',
        }
        payload['motors'][name] = {
            'position': math.sin(t / (idx + 2)) * 2.0,
            'velocity': math.cos(t / (idx + 2)) * 3.0,
            'temperature': 34.0 + idx * 1.8 + 1.5 * math.sin(t / 6.0),
            'attached': True,
            'health': 'healthy',
            'fault': 'None',
            'last_update': datetime.now().strftime('%H:%M:%S'),
        }
    return payload


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName('Rover Dashboard')
    app.setFont(QFont('Arial', 10))
    window = DashboardWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
