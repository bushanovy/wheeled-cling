#!/usr/bin/env python3
"""PyQt dashboard for 3D3S adhesion and surface-test analysis.

The dashboard presents the per-wheel adhesion table, the live force/gap/
torque plot, and the scenario/pose/teleop controls.  The previous
SwerveWidget (top-view triangle, wheel locations, and per-wheel speeds)
was removed; the same information is reported in the wheel parameter
table at the bottom of the window.
"""

import json
import math
import os
import shlex
import signal
import shutil
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


WHEEL_LINKS = ['wheel_1_link', 'wheel_2_link', 'wheel_3_link']
WHEEL_JOINTS = ['wheel_1_joint', 'wheel_2_joint', 'wheel_3_joint']

POSE_STORE_VERSION = 2

SCENARIOS = {
    'Horizontal pipe': {
        'launch': 'horizontal_pipe_test.launch.py',
        'world': 'horizontal_pipe_test',
        'mode': 'horizontal_pipe',
        'defaults': {
            'wall_thickness_mm': 9.0,
            'pipe_radius_m': 1.20,
            'pipe_center_z': 2.00,
            'spawn_x': -1.99,
            'spawn_y': 1.29,
            'spawn_z': 2.00,
            'spawn_roll': -1.5708,
            'spawn_pitch': 1.5708,
            'spawn_yaw': 0.0,
            'approach_theta_deg': 0.0,
            'side_start_theta_deg': 0.0,
            'startup_side_hold_s': 15.0,
            'attachment_guard_method': 'wrench',
            'robot_mass_kg': 25.0,
            'target_angle_deg': 90.0,
        },
    },
    'Vertical pipe': {
        'launch': 'adhesion_model_test.launch.py',
        'world': 'adhesion_model_test',
        'mode': 'vertical_pipe',
        'defaults': {
            'wall_thickness_mm': 9.0,
            'pipe_radius_m': 1.20,
            'spawn_x': 1.17,
            'spawn_y': 0.00,
            'spawn_z': 2.00,
            'spawn_roll': 3.1416,
            'spawn_pitch': -1.5708,
            'spawn_yaw': 0.0,
            'robot_mass_kg': 25.0,
            'target_angle_deg': 0.0,
        },
    },
    'Flat wall + slide': {
        'launch': 'ground_to_wall_climb.launch.py',
        'world': 'climb_course',
        'mode': 'course',
        'defaults': {
            'wall_thickness_mm': 9.0,
            'pipe_radius_m': 0.0,
            'spawn_x': -1.20,
            'spawn_y': 0.0,
            'spawn_z': 0.0,
            'spawn_roll': 0.0,
            'spawn_pitch': 0.0,
            'spawn_yaw': 0.0,
            'robot_mass_kg': 25.0,
            'target_angle_deg': 0.0,
        },
    },
    'Mockup pipe': {
        'launch': 'mockup_contact_test.launch.py',
        'world': 'mockup_pipe_test',
        'mode': 'mockup',
        'defaults': {
            'wall_thickness_mm': 9.0,
            'pipe_radius_m': 0.55,
            'spawn_x': 1.0,
            'spawn_y': -0.75,
            'spawn_z': 0.0,
            'spawn_roll': 0.0,
            'spawn_pitch': 0.0,
            'spawn_yaw': 1.5708,
            'robot_mass_kg': 25.0,
            'target_angle_deg': 0.0,
        },
    },
}


PLOT_SPECS = {
    'force_total_n': ('Total force N', QColor(224, 64, 49)),
    'force_w1_n': ('Wheel 1 force N', QColor(32, 120, 220)),
    'force_w2_n': ('Wheel 2 force N', QColor(60, 170, 75)),
    'force_w3_n': ('Wheel 3 force N', QColor(190, 90, 210)),
    'max_gap_mm': ('Max gap mm', QColor(255, 150, 40)),
    'avg_contact': ('Avg contact', QColor(0, 170, 160)),
    'boundary_risk': ('Boundary risk', QColor(225, 120, 0)),
    'torque_est_total_nm': ('Holding torque Nm', QColor(45, 45, 45)),
    'cmd_speed_mps': ('Cmd speed m/s', QColor(130, 130, 130)),
}


def _classify_motion_action(vx, vy, omega):
    speed = math.hypot(vx, vy)
    if speed < 1e-4 and abs(omega) < 1e-4:
        return 'idle'
    if speed < 1e-4:
        return 'rotate in place'
    if abs(omega) < 1e-4:
        return 'translate'
    return 'curved motion'


def _yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _rpy_from_quat(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 0.9999:
        pitch = math.copysign(math.pi / 2.0, sinp)
        # At +/-90 deg pitch, roll and yaw are coupled.  Use yaw=0 and keep
        # the roll component so side-pipe poses round-trip through the GUI.
        yaw = 0.0
        roll = 2.0 * math.atan2(q.x, q.w)
    else:
        pitch = math.asin(sinp)
        yaw = _yaw_from_quat(q)
    return roll, pitch, yaw


def _rpy_to_quat(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _pose_store_path():
    base = Path(os.environ.get('ROS_HOME', Path.home() / '.ros'))
    path = base / 'robot_3d3s' / 'adhesion_dashboard_poses.json'
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        return path
    except OSError:
        return Path('/tmp') / 'robot_3d3s_adhesion_dashboard_poses.json'


class DashboardNode(Node):
    def __init__(self):
        super().__init__('adhesion_dashboard')
        self.declare_parameter('status_topic', '/robot_3d3s/adhesion_status')
        self.declare_parameter('cmd_topic', '/swerve_controller/cmd_vel')
        self.status_topic = self.get_parameter('status_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value

        self.lock = threading.Lock()
        self.start_time = time.monotonic()
        self.series = {
            key: deque(maxlen=1600)
            for key in PLOT_SPECS
        }
        self.latest_status = {}
        self.latest_joint_state = None
        self.latest_cmd_speed = 0.0
        self.latest_state_text = ''
        self.latest_planner_debug = {}
        self.latest_robot_pose = None
        self.latest_clock_time = 0.0
        self.latest_joint_time = 0.0
        self.latest_pose_time = 0.0
        self.latest_cmds = {
            'direct': {'stamp': 0.0, 'cmd': (0.0, 0.0, 0.0)},
            'climb': {'stamp': 0.0, 'cmd': (0.0, 0.0, 0.0)},
            'dashboard': {'stamp': 0.0, 'cmd': (0.0, 0.0, 0.0)},
        }

        self.create_subscription(String, self.status_topic, self._status_cb, 10)
        self.create_subscription(String, '/climb_controller/state', self._state_cb, 10)
        self.create_subscription(
            String, '/surface_goal_planner/debug',
            self._planner_debug_cb, 10)
        self.create_subscription(
            String, '/horizontal_pipe_planner/debug',
            self._planner_debug_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.create_subscription(
            TwistStamped, '/swerve_controller/cmd_vel',
            lambda msg: self._cmd_cb(msg, 'direct'), 10)
        self.create_subscription(
            TwistStamped, '/climb_controller/cmd_vel_in',
            lambda msg: self._cmd_cb(msg, 'climb'), 10)
        self.create_subscription(
            TFMessage, '/model/robot_3d3s/pose', self._model_pose_cb, 10)
        self.create_subscription(
            TFMessage, '/world/horizontal_pipe_test/pose/info',
            self._world_pose_cb, 10)
        self.create_subscription(
            TFMessage, '/world/adhesion_model_test/pose/info',
            self._world_pose_cb, 10)
        self.create_subscription(
            TFMessage, '/world/adhesion_model_test/dynamic_pose/info',
            self._world_pose_cb, 10)
        self.direct_cmd_pub = self.create_publisher(
            TwistStamped, '/swerve_controller/cmd_vel', 10)
        self.climb_cmd_pub = self.create_publisher(
            TwistStamped, '/climb_controller/cmd_vel_in', 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

    def _now(self):
        return time.monotonic() - self.start_time

    def _status_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return

        wheels = data.get('wheels', [])
        force_by_wheel = {
            w.get('wheel', ''): float(w.get('force_n', 0.0))
            for w in wheels
        }
        gaps = [float(w.get('gap_mm', 0.0)) for w in wheels]
        contacts = [float(w.get('contact_fraction', 0.0)) for w in wheels]
        torques = [float(w.get('holding_torque_nm', 0.0)) for w in wheels]
        t = self._now()
        with self.lock:
            self.latest_status = data
            base_pose = data.get('base_pose')
            if isinstance(base_pose, dict):
                p = base_pose.get('p', [])
                q = base_pose.get('q', [])
                if len(p) >= 3 and len(q) >= 4:
                    class Quat:
                        pass
                    quat = Quat()
                    quat.x, quat.y, quat.z, quat.w = (
                        float(q[0]), float(q[1]), float(q[2]), float(q[3]))
                    roll, pitch, yaw = _rpy_from_quat(quat)
                    self.latest_robot_pose = (
                        float(p[0]), float(p[1]), float(p[2]), roll, pitch, yaw)
                    self.latest_pose_time = t
                    self.latest_clock_time = t
            self.series['force_total_n'].append(
                (t, sum(force_by_wheel.values())))
            for i, wheel in enumerate(WHEEL_LINKS, start=1):
                self.series[f'force_w{i}_n'].append(
                    (t, force_by_wheel.get(wheel, 0.0)))
            self.series['max_gap_mm'].append((t, max(gaps, default=0.0)))
            avg_contact = sum(contacts) / len(contacts) if contacts else 0.0
            self.series['avg_contact'].append((t, avg_contact))
            self.series['boundary_risk'].append((
                t, float(data.get('boundary_risk', 0.0))))
            self.series['torque_est_total_nm'].append((t, sum(torques)))
            self.series['cmd_speed_mps'].append((t, self.latest_cmd_speed))

    def _state_cb(self, msg):
        with self.lock:
            self.latest_state_text = msg.data

    def _planner_debug_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        with self.lock:
            self.latest_planner_debug = data

    def _joint_cb(self, msg):
        with self.lock:
            self.latest_joint_state = msg
            self.latest_joint_time = self._now()

    def _cmd_cb(self, msg, source):
        v = msg.twist.linear
        cmd = (v.x, v.y, msg.twist.angular.z)
        with self.lock:
            self.latest_cmd_speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
            self.latest_cmds[source] = {'stamp': self._now(), 'cmd': cmd}

    def _model_pose_cb(self, msg):
        if not msg.transforms:
            return
        tf = msg.transforms[0]
        for candidate in msg.transforms:
            child = candidate.child_frame_id.lower()
            if 'robot_3d3s' in child or 'base_footprint' in child:
                tf = candidate
                break
        t = tf.transform.translation
        q = tf.transform.rotation
        with self.lock:
            roll, pitch, yaw = _rpy_from_quat(q)
            self.latest_robot_pose = (t.x, t.y, t.z, roll, pitch, yaw)
            self.latest_pose_time = self._now()

    def _world_pose_cb(self, msg):
        # The bridge exposes Gazebo poses as TFMessage. This is enough as a
        # heartbeat that Gazebo pose bridging is alive.
        with self.lock:
            self.latest_clock_time = self._now()

    def snapshot(self):
        with self.lock:
            return {
                'series': {key: list(value) for key, value in self.series.items()},
                'status': dict(self.latest_status),
                'joint_state': self.latest_joint_state,
                'cmd_speed': self.latest_cmd_speed,
                'state_text': self.latest_state_text,
                'planner_debug': dict(self.latest_planner_debug),
                'robot_pose': self.latest_robot_pose,
                'health': {
                    'joint_age': self._now() - self.latest_joint_time if self.latest_joint_time else None,
                    'pose_age': self._now() - self.latest_pose_time if self.latest_pose_time else None,
                    'world_pose_age': self._now() - self.latest_clock_time if self.latest_clock_time else None,
                },
                'cmds': dict(self.latest_cmds),
            }

    def publish_initial_pose(self, x, y, z, roll, pitch, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        qx, qy, qz, qw = _rpy_to_quat(roll, pitch, yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        self.initial_pose_pub.publish(msg)

    def publish_dashboard_cmd(self, topic_key, vx, vy, omega):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = omega
        if topic_key == 'climb':
            self.climb_cmd_pub.publish(msg)
        else:
            self.direct_cmd_pub.publish(msg)
        with self.lock:
            self.latest_cmd_speed = math.hypot(vx, vy)
            self.latest_cmds['dashboard'] = {
                'stamp': self._now(),
                'cmd': (vx, vy, omega),
            }


class PlotWidget(QFrame):
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(310)
        self.setFrameShape(QFrame.StyledPanel)
        self.data = {}
        self.enabled = set()
        self.window_s = 30.0

    def set_data(self, data, enabled):
        self.data = data
        self.enabled = set(enabled)
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect().adjusted(56, 18, -18, -42)
        painter.fillRect(self.rect(), QColor(250, 250, 250))
        painter.setPen(QPen(QColor(210, 210, 210), 1))
        painter.drawRect(rect)

        visible = {
            key: values for key, values in self.data.items()
            if key in self.enabled and values
        }
        if not visible:
            painter.setPen(QColor(100, 100, 100))
            painter.drawText(rect, Qt.AlignCenter, 'Waiting for telemetry')
            return

        max_t = max(values[-1][0] for values in visible.values())
        min_t = max(0.0, max_t - self.window_s)
        values = [
            y for points in visible.values()
            for x, y in points if x >= min_t
        ]
        min_y = min(values) if values else 0.0
        max_y = max(values) if values else 1.0
        if abs(max_y - min_y) < 1e-9:
            max_y += 1.0
            min_y -= 1.0
        pad = 0.08 * (max_y - min_y)
        min_y -= pad
        max_y += pad

        painter.setPen(QPen(QColor(225, 225, 225), 1))
        for i in range(1, 5):
            x = rect.left() + rect.width() * i / 5.0
            y = rect.top() + rect.height() * i / 5.0
            painter.drawLine(int(x), rect.top(), int(x), rect.bottom())
            painter.drawLine(rect.left(), int(y), rect.right(), int(y))

        painter.setPen(QColor(80, 80, 80))
        painter.drawText(8, rect.top() + 8, f'{max_y:.2f}')
        painter.drawText(8, rect.bottom(), f'{min_y:.2f}')
        painter.drawText(rect.left(), self.height() - 14, f'-{self.window_s:.0f}s')
        painter.drawText(rect.right() - 42, self.height() - 14, 'now')

        def to_xy(t, y):
            px = rect.left() + (t - min_t) / max(1e-6, self.window_s) * rect.width()
            py = rect.bottom() - (y - min_y) / (max_y - min_y) * rect.height()
            return int(px), int(py)

        legend_x = rect.left() + 8
        legend_y = rect.top() + 8
        for key, points in visible.items():
            label, color = PLOT_SPECS[key]
            recent = [(t, y) for t, y in points if t >= min_t]
            if len(recent) < 2:
                continue
            painter.setPen(QPen(color, 2))
            last = to_xy(recent[0][0], recent[0][1])
            for t, y in recent[1:]:
                cur = to_xy(t, y)
                painter.drawLine(last[0], last[1], cur[0], cur[1])
                last = cur
            painter.setPen(color)
            painter.drawText(legend_x, legend_y, label)
            legend_y += 17


class DashboardWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.launch_process = None
        self.pose_store_path = _pose_store_path()
        self.pose_store = self._load_pose_store()
        self.teleop_enabled = False
        self.teleop_cmd = (0.0, 0.0, 0.0)
        self.linear_step = 0.08
        self.angular_step = 0.25
        self.setWindowTitle('3D3S Adhesion Analysis Dashboard')
        self.resize(1380, 860)
        self.setFocusPolicy(Qt.StrongFocus)

        # Layout: two columns.
        #   Left column  (col 0): scenario/pose/teleop controls (top) and
        #                        the wheel parameter table (bottom).
        #   Right column (col 1): the live force/gap/torque plot (top) and
        #                        the signal-selection check boxes (bottom).
        # The previous SwerveWidget (top-view triangle, wheel positions,
        # and per-wheel speeds) was removed; the table below provides the
        # same per-wheel information numerically.
        root = QGridLayout()
        root.addLayout(self._scenario_panel(), 0, 0)
        root.addLayout(self._plot_panel(), 0, 1)
        root.addLayout(self._feedback_panel(), 1, 0, 1, 2)
        self.setLayout(root)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(120)
        self.teleop_timer = QTimer(self)
        self.teleop_timer.timeout.connect(self._publish_teleop)
        self.teleop_timer.start(50)
        self._scenario_changed()

    def _scenario_panel(self):
        layout = QVBoxLayout()

        scenario_group = QGroupBox('Scene and placement')
        form = QFormLayout()
        self.scenario = QComboBox()
        self.scenario.addItems(SCENARIOS.keys())
        self.scenario.currentTextChanged.connect(self._scenario_changed)
        form.addRow('Surface preset', self.scenario)

        self.thickness = self._spin(1.0, 40.0, 9.0, 0.1)
        self.pipe_radius = self._spin(0.0, 5.0, 1.2, 0.01)
        self.spawn_x = self._spin(-10.0, 10.0, -2.0, 0.01)
        self.spawn_y = self._spin(-10.0, 10.0, -1.75, 0.01)
        self.spawn_z = self._spin(-2.0, 10.0, 0.0, 0.01)
        self.spawn_roll = self._spin(-3.1416, 3.1416, 0.0, 0.01)
        self.spawn_pitch = self._spin(-3.1416, 3.1416, 0.0, 0.01)
        self.spawn_yaw = self._spin(-3.1416, 3.1416, 1.5708, 0.01)
        self.robot_mass = self._spin(1.0, 80.0, 25.0, 0.5)
        self.target_angle = self._spin(-180.0, 180.0, 90.0, 1.0)
        for widget in (
                self.thickness, self.pipe_radius, self.spawn_x, self.spawn_y,
                self.spawn_z, self.spawn_roll, self.spawn_pitch,
                self.spawn_yaw, self.robot_mass, self.target_angle):
            widget.valueChanged.connect(self._update_command)

        form.addRow('Thickness mm', self.thickness)
        form.addRow('Pipe radius m', self.pipe_radius)
        form.addRow('Spawn X m', self.spawn_x)
        form.addRow('Spawn Y m', self.spawn_y)
        form.addRow('Spawn Z m', self.spawn_z)
        form.addRow('Spawn roll rad', self.spawn_roll)
        form.addRow('Spawn pitch rad', self.spawn_pitch)
        form.addRow('Spawn yaw rad', self.spawn_yaw)
        form.addRow('Robot mass kg', self.robot_mass)
        form.addRow('Pipe target deg', self.target_angle)
        scenario_group.setLayout(form)

        pose_group = QGroupBox('Saved poses')
        pose_layout = QVBoxLayout()
        pose_row = QHBoxLayout()
        self.saved_pose = QComboBox()
        self.saved_pose.currentTextChanged.connect(self._saved_pose_changed)
        self.pose_name = QLineEdit()
        self.pose_name.setPlaceholderText('pose name')
        pose_row.addWidget(self.saved_pose)
        pose_row.addWidget(self.pose_name)
        pose_layout.addLayout(pose_row)
        pose_buttons = QHBoxLayout()
        use_live_btn = QPushButton('Use Live Pose')
        use_live_btn.clicked.connect(self._use_live_pose)
        save_pose_btn = QPushButton('Save')
        save_pose_btn.clicked.connect(self._save_pose)
        default_pose_btn = QPushButton('Set Default')
        default_pose_btn.clicked.connect(self._set_default_pose)
        clear_default_btn = QPushButton('Clear Default')
        clear_default_btn.clicked.connect(self._clear_default_pose)
        reset_defaults_btn = QPushButton('Built-in Default')
        reset_defaults_btn.clicked.connect(self._reset_scenario_defaults)
        delete_pose_btn = QPushButton('Delete')
        delete_pose_btn.clicked.connect(self._delete_pose)
        for button in (
                use_live_btn, save_pose_btn, default_pose_btn,
                clear_default_btn, reset_defaults_btn, delete_pose_btn):
            pose_buttons.addWidget(button)
        pose_layout.addLayout(pose_buttons)
        pose_group.setLayout(pose_layout)

        command_group = QGroupBox('Run and pose')
        cmd_layout = QVBoxLayout()
        self.command = QLineEdit()
        self.command.setReadOnly(True)
        cmd_layout.addWidget(self.command)
        self.terminal_box = QCheckBox('Open launch terminal')
        self.terminal_box.setChecked(False)
        cmd_layout.addWidget(self.terminal_box)
        row = QHBoxLayout()
        start_btn = QPushButton('Start Launch')
        start_btn.clicked.connect(self._start_launch)
        restart_btn = QPushButton('Restart Here')
        restart_btn.clicked.connect(self._restart_launch)
        stop_btn = QPushButton('Stop')
        stop_btn.clicked.connect(self._stop_launch)
        clean_btn = QPushButton('Clean All')
        clean_btn.clicked.connect(self._clean_all_processes)
        copy_btn = QPushButton('Copy')
        copy_btn.clicked.connect(self._copy_command)
        pose_btn = QPushButton('Set Gazebo Pose')
        pose_btn.clicked.connect(self._set_gazebo_pose)
        initial_btn = QPushButton('Publish /initialpose')
        initial_btn.clicked.connect(self._publish_initial_pose)
        for button in (
                start_btn, restart_btn, stop_btn, clean_btn,
                copy_btn, pose_btn, initial_btn):
            row.addWidget(button)
        cmd_layout.addLayout(row)
        self.app_status = QLabel('Ready')
        cmd_layout.addWidget(self.app_status)
        command_group.setLayout(cmd_layout)

        teleop_group = QGroupBox('Command source and teleop')
        teleop_layout = QVBoxLayout()
        teleop_row = QHBoxLayout()
        self.teleop_box = QCheckBox('Dashboard teleop')
        self.teleop_box.stateChanged.connect(self._teleop_toggled)
        self.command_source = QComboBox()
        self.command_source.addItem('Direct swerve', 'direct')
        self.command_source.addItem('Climb controller', 'climb')
        self.command_source.addItem('Passive monitor', 'passive')
        self.source_label = QLabel('● passive')
        teleop_row.addWidget(self.teleop_box)
        teleop_row.addWidget(self.command_source)
        teleop_row.addWidget(self.source_label)
        teleop_layout.addLayout(teleop_row)
        self.teleop_hint = QLabel('Keys: W/S forward, A/D lateral, Q/E yaw, Space stop, Ctrl+R restart, Ctrl+S save, Ctrl+G set pose')
        teleop_layout.addWidget(self.teleop_hint)
        teleop_group.setLayout(teleop_layout)

        layout.addWidget(scenario_group)
        layout.addWidget(pose_group)
        layout.addWidget(command_group)
        layout.addWidget(teleop_group)
        layout.addStretch(1)
        return layout

    def _plot_panel(self):
        layout = QVBoxLayout()
        self.plot = PlotWidget()
        layout.addWidget(self.plot)

        check_group = QGroupBox('Signals')
        check_layout = QGridLayout()
        self.checkboxes = {}
        defaults = {
            'force_total_n', 'force_w1_n', 'force_w2_n', 'force_w3_n',
            'max_gap_mm', 'boundary_risk', 'torque_est_total_nm',
        }
        for idx, (key, (label, _color)) in enumerate(PLOT_SPECS.items()):
            box = QCheckBox(label)
            box.setChecked(key in defaults)
            self.checkboxes[key] = box
            check_layout.addWidget(box, idx // 4, idx % 4)
        check_group.setLayout(check_layout)
        layout.addWidget(check_group)
        return layout

    def _feedback_panel(self):
        layout = QVBoxLayout()
        summary = QHBoxLayout()
        self.summary_labels = {
            'surface': QLabel('surface: none'),
            'attached': QLabel('attached: 0'),
            'model': QLabel('model: n/a'),
            'thickness': QLabel('thickness: n/a'),
            'cmd': QLabel('cmd: 0.000 m/s'),
            'action': QLabel('action: idle'),
            'boundary': QLabel('boundary: n/a'),
            'state': QLabel('state: n/a'),
            'planner': QLabel('planner: n/a'),
            'route': QLabel('route: n/a'),
            'health': QLabel('health: waiting'),
        }
        for label in self.summary_labels.values():
            summary.addWidget(label)
        summary.addStretch(1)
        layout.addLayout(summary)

        # Per-wheel parameter table.  This is the primary wheel telemetry
        # view (replaces the previous triangle/wheel-location/wheel-speed
        # SwerveWidget).  Columns: wheel name, surface, attached, magnetic
        # force, air gap, contact fraction, contact points, depth, edge
        # risk, holding torque, and measured joint torque.
        self.table = QTableWidget(3, 11)
        self.table.setHorizontalHeaderLabels([
            'wheel', 'surface', 'attached', 'force N',
            'gap mm', 'contact', 'pts', 'depth mm', 'risk',
            'hold torque Nm', 'joint torque Nm',
        ])
        self.table.verticalHeader().setVisible(False)
        layout.addWidget(self.table)
        return layout

    @staticmethod
    def _spin(low, high, value, step):
        spin = QDoubleSpinBox()
        spin.setRange(low, high)
        spin.setValue(value)
        spin.setSingleStep(step)
        spin.setDecimals(4 if step < 0.01 else 2)
        return spin

    @staticmethod
    def _empty_pose_store():
        return {'_format_version': POSE_STORE_VERSION}

    def _load_pose_store(self):
        try:
            with self.pose_store_path.open('r', encoding='utf-8') as f:
                data = json.load(f)
            if not isinstance(data, dict):
                return self._empty_pose_store()
            if data.get('_format_version') != POSE_STORE_VERSION:
                return self._empty_pose_store()
            return data
        except (OSError, ValueError, json.JSONDecodeError):
            return self._empty_pose_store()

    def _write_pose_store(self):
        try:
            self.pose_store['_format_version'] = POSE_STORE_VERSION
            self.pose_store_path.parent.mkdir(parents=True, exist_ok=True)
            with self.pose_store_path.open('w', encoding='utf-8') as f:
                json.dump(self.pose_store, f, indent=2, sort_keys=True)
            return True
        except OSError as exc:
            self.app_status.setText(f'Pose save failed: {exc}')
            return False

    def _current_pose_dict(self):
        return {
            'wall_thickness_mm': self.thickness.value(),
            'pipe_radius_m': self.pipe_radius.value(),
            'spawn_x': self.spawn_x.value(),
            'spawn_y': self.spawn_y.value(),
            'spawn_z': self.spawn_z.value(),
            'spawn_roll': self.spawn_roll.value(),
            'spawn_pitch': self.spawn_pitch.value(),
            'spawn_yaw': self.spawn_yaw.value(),
            'robot_mass_kg': self.robot_mass.value(),
            'target_angle_deg': self.target_angle.value(),
        }

    def _apply_pose_dict(self, pose):
        self.thickness.setValue(float(pose.get('wall_thickness_mm', self.thickness.value())))
        self.pipe_radius.setValue(float(pose.get('pipe_radius_m', self.pipe_radius.value())))
        self.spawn_x.setValue(float(pose.get('spawn_x', self.spawn_x.value())))
        self.spawn_y.setValue(float(pose.get('spawn_y', self.spawn_y.value())))
        self.spawn_z.setValue(float(pose.get('spawn_z', self.spawn_z.value())))
        self.spawn_roll.setValue(float(pose.get('spawn_roll', self.spawn_roll.value())))
        self.spawn_pitch.setValue(float(pose.get('spawn_pitch', self.spawn_pitch.value())))
        self.spawn_yaw.setValue(float(pose.get('spawn_yaw', self.spawn_yaw.value())))
        self.robot_mass.setValue(float(pose.get('robot_mass_kg', self.robot_mass.value())))
        self.target_angle.setValue(float(pose.get('target_angle_deg', self.target_angle.value())))
        self._update_command()

    def _scenario_store(self):
        scenario = self.scenario.currentText()
        if scenario not in self.pose_store:
            self.pose_store[scenario] = {'default': '', 'poses': {}}
        if 'poses' not in self.pose_store[scenario]:
            self.pose_store[scenario]['poses'] = {}
        return self.pose_store[scenario]

    def _refresh_pose_combo(self, select_name=None):
        store = self._scenario_store()
        names = sorted(store.get('poses', {}).keys())
        self.saved_pose.blockSignals(True)
        self.saved_pose.clear()
        self.saved_pose.addItems(names)
        if select_name and select_name in names:
            self.saved_pose.setCurrentText(select_name)
        elif store.get('default') in names:
            self.saved_pose.setCurrentText(store['default'])
        else:
            self.saved_pose.setCurrentIndex(-1)
        self.saved_pose.blockSignals(False)

    def _scenario_changed(self):
        store = self._scenario_store()
        self._refresh_pose_combo()
        preset = SCENARIOS[self.scenario.currentText()]
        defaults = preset['defaults']
        if preset['mode'] == 'horizontal_pipe':
            self.saved_pose.blockSignals(True)
            self.saved_pose.setCurrentIndex(-1)
            self.saved_pose.blockSignals(False)
            self._apply_pose_dict(defaults)
            self.pose_name.setText('')
            self.app_status.setText(
                f'{self.scenario.currentText()} uses built-in side-start defaults')
            return

        poses = store.get('poses', {})
        default_name = store.get('default', '')
        if default_name in poses:
            self._apply_pose_dict(poses[default_name])
            self.pose_name.setText(default_name)
            return

        self._apply_pose_dict(defaults)
        self.pose_name.setText('')
        self._update_command()

    def _saved_pose_changed(self, name):
        if not name:
            return
        poses = self._scenario_store().get('poses', {})
        if name in poses:
            self.pose_name.setText(name)
            self._apply_pose_dict(poses[name])

    def _save_pose(self):
        name = self.pose_name.text().strip()
        if not name:
            name, ok = QInputDialog.getText(self, 'Save Pose', 'Pose name:')
            if not ok:
                return
            name = name.strip()
        if not name:
            self.app_status.setText('Pose name is empty')
            return None
        store = self._scenario_store()
        store['poses'][name] = self._current_pose_dict()
        if self._write_pose_store():
            self._refresh_pose_combo(name)
            self.pose_name.setText(name)
            self.app_status.setText(f'Saved pose "{name}"')
            return name
        return None

    def _set_default_pose(self):
        name = self.saved_pose.currentText().strip() or self.pose_name.text().strip()
        if not name:
            name = self._save_pose()
        if not name:
            return
        store = self._scenario_store()
        if name not in store.get('poses', {}):
            store['poses'][name] = self._current_pose_dict()
        store['default'] = name
        if self._write_pose_store():
            self._refresh_pose_combo(name)
            self.app_status.setText(f'Default pose set to "{name}"')

    def _clear_default_pose(self):
        store = self._scenario_store()
        store['default'] = ''
        if self._write_pose_store():
            self.saved_pose.setCurrentIndex(-1)
            self.app_status.setText('Scenario default cleared')

    def _reset_scenario_defaults(self):
        defaults = SCENARIOS[self.scenario.currentText()]['defaults']
        self.saved_pose.blockSignals(True)
        self.saved_pose.setCurrentIndex(-1)
        self.saved_pose.blockSignals(False)
        self.pose_name.clear()
        self._apply_pose_dict(defaults)
        self.app_status.setText('Built-in scenario defaults loaded')

    def _delete_pose(self):
        name = self.saved_pose.currentText().strip()
        if not name:
            return
        store = self._scenario_store()
        store.get('poses', {}).pop(name, None)
        if store.get('default') == name:
            store['default'] = ''
        if self._write_pose_store():
            self._refresh_pose_combo()
            self.pose_name.clear()
            self.app_status.setText(f'Deleted pose "{name}"')

    def _use_live_pose(self):
        snap = self.node.snapshot()
        pose = snap.get('robot_pose')
        if pose is None:
            self.app_status.setText('No live Gazebo robot pose yet')
            return
        x, y, z, roll, pitch, yaw = pose
        self.spawn_x.setValue(x)
        self.spawn_y.setValue(y)
        self.spawn_z.setValue(z)
        self.spawn_roll.setValue(roll)
        self.spawn_pitch.setValue(pitch)
        self.spawn_yaw.setValue(yaw)
        self._update_command()
        self.app_status.setText('Copied live Gazebo pose into spawn fields')

    def _launch_args(self):
        preset = SCENARIOS[self.scenario.currentText()]
        mode = preset['mode']
        args = ['run_sim:=true', 'use_rviz:=true']
        if mode in ('horizontal_pipe', 'vertical_pipe', 'mockup'):
            spawn = preset['defaults'] if mode == 'horizontal_pipe' else None
            spawn_x = self.spawn_x.value() if spawn is None else float(spawn['spawn_x'])
            spawn_y = self.spawn_y.value() if spawn is None else float(spawn['spawn_y'])
            spawn_z = self.spawn_z.value() if spawn is None else float(spawn['spawn_z'])
            spawn_roll = self.spawn_roll.value() if spawn is None else float(spawn['spawn_roll'])
            spawn_pitch = self.spawn_pitch.value() if spawn is None else float(spawn['spawn_pitch'])
            spawn_yaw = self.spawn_yaw.value() if spawn is None else float(spawn['spawn_yaw'])
            args.append(f'wall_thickness_mm:={self.thickness.value():.2f}')
            args.append(f'pipe_radius_m:={self.pipe_radius.value():.3f}')
            args.append(f'spawn_x:={spawn_x:.3f}')
            args.append(f'spawn_y:={spawn_y:.3f}')
            args.append(f'spawn_z:={spawn_z:.3f}')
            args.append(f'spawn_roll:={spawn_roll:.4f}')
            args.append(f'spawn_pitch:={spawn_pitch:.4f}')
            args.append(f'spawn_yaw:={spawn_yaw:.4f}')
            args.append(f'robot_mass_kg:={self.robot_mass.value():.2f}')
        if mode == 'horizontal_pipe':
            args.append('use_gazebo_gui:=true')
            args.append('use_pipe_planner:=true')
            pipe_center_z = float(preset['defaults'].get(
                'pipe_center_z', self.pipe_radius.value()))
            args.append(f'pipe_center_z:={pipe_center_z:.3f}')
            args.append(
                f'approach_theta_deg:={preset["defaults"].get("approach_theta_deg", 0.0):.1f}')
            args.append(
                f'side_start_theta_deg:={preset["defaults"].get("side_start_theta_deg", 0.0):.1f}')
            args.append(
                f'startup_side_hold_s:={preset["defaults"].get("startup_side_hold_s", 15.0):.1f}')
            args.append(
                f'attachment_guard_method:={preset["defaults"].get("attachment_guard_method", "wrench")}')
            args.append(f'target_angle_deg:={self.target_angle.value():.1f}')
        elif mode == 'vertical_pipe':
            args.append('use_gazebo_gui:=true')
            args.append('use_surface_planner:=true')
            args.append(f'target_angle_deg:={self.target_angle.value():.1f}')
        return args

    def _command_parts(self):
        preset = SCENARIOS[self.scenario.currentText()]
        return ['ros2', 'launch', 'robot_3d3s', preset['launch']] + self._launch_args()

    def _terminal_command(self, launch_parts):
        script = (
            f'cd {shlex.quote(str(Path.cwd()))} && '
            'source install/setup.bash && '
            f'exec {" ".join(shlex.quote(part) for part in launch_parts)}'
        )
        candidates = [
            ['xterm', '-T', '3D3S launch', '-e', 'bash', '-lc', script],
            ['x-terminal-emulator', '-e', 'bash', '-lc', script],
            ['gnome-terminal', '--', 'bash', '-lc', script],
            ['konsole', '-e', 'bash', '-lc', script],
            ['xfce4-terminal', '--disable-server', '-e',
             f'bash -lc {shlex.quote(script)}'],
        ]
        for command in candidates:
            if shutil.which(command[0]):
                return command
        return None

    def _update_command(self):
        self.command.setText(' '.join(self._command_parts()))

    def _copy_command(self):
        QApplication.clipboard().setText(self.command.text())
        self.app_status.setText('Launch command copied')

    def _start_launch(self, clean_stale=True):
        if self.launch_process is not None and self.launch_process.poll() is None:
            self.app_status.setText('Launch already running')
            return
        if clean_stale:
            self._clean_all_processes(update_status=False)
        try:
            parts = self._command_parts()
            command = parts
            if self.terminal_box.isChecked():
                terminal_command = self._terminal_command(parts)
                if terminal_command is not None:
                    command = terminal_command
                else:
                    self.app_status.setText(
                        'No terminal emulator found; starting internally')
            self.launch_process = subprocess.Popen(
                command,
                start_new_session=True)
            self.app_status.setText(
                'Launch started in terminal'
                if command is not parts else 'Launch started')
        except OSError as exc:
            self.app_status.setText(f'Launch failed: {exc}')

    def _stop_launch(self, wait=False):
        if self.launch_process is not None and self.launch_process.poll() is None:
            try:
                os.killpg(self.launch_process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            if wait:
                try:
                    self.launch_process.wait(timeout=6.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(self.launch_process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    self.launch_process.wait(timeout=2.0)
            self.app_status.setText('Launch stop requested')
        else:
            self.app_status.setText('No launch process from this dashboard')

    def _clean_all_processes(self, update_status=True):
        self._stop_launch(wait=True)
        patterns = [
            'ros2 launch robot_3d3s',
            'gz sim',
            'parameter_bridge',
            'gazebo_model_tf_relay.py',
            'magnetic_adhesion.py',
            'mockup_contact_monitor.py',
            'surface_goal_planner.py',
            'surface_attachment_guard.py',
            'horizontal_pipe_planner.py',
            'adhesion_scene_rviz.py',
            'experimental_polygon_test.launch.py',
            'experimental_polygon_rviz.py',
            'experimental_polygon_grid.py',
            'rviz2',
        ]
        killed = 0
        for pattern in patterns:
            result = subprocess.run(
                ['pkill', '-f', pattern],
                capture_output=True,
                text=True,
                timeout=2.0)
            if result.returncode == 0:
                killed += 1
        self.launch_process = None
        if update_status:
            self.app_status.setText(
                'Cleaned stale sim processes'
                if killed else 'No stale sim processes found')

    def _restart_launch(self):
        self._clean_all_processes(update_status=False)
        self._start_launch(clean_stale=False)
        self.app_status.setText('Restarted launch with current pose')

    def _set_gazebo_pose(self):
        preset = SCENARIOS[self.scenario.currentText()]
        roll = self.spawn_roll.value()
        pitch = self.spawn_pitch.value()
        yaw = self.spawn_yaw.value()
        qx, qy, qz, qw = _rpy_to_quat(roll, pitch, yaw)
        req = (
            'name: "robot_3d3s", '
            f'position: {{x: {self.spawn_x.value():.4f}, '
            f'y: {self.spawn_y.value():.4f}, z: {self.spawn_z.value():.4f}}}, '
            f'orientation: {{x: {qx:.6f}, y: {qy:.6f}, '
            f'z: {qz:.6f}, w: {qw:.6f}}}'
        )
        cmd = [
            'gz', 'service',
            '-s', f'/world/{preset["world"]}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', req,
        ]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)
            if result.returncode == 0:
                self._publish_initial_pose()
                self.app_status.setText('Gazebo pose request sent and /initialpose published')
            else:
                detail = (result.stderr or result.stdout).strip()
                self.app_status.setText(f'Gazebo pose failed: {detail[:90]}')
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.app_status.setText(f'Gazebo pose failed: {exc}')

    def _publish_initial_pose(self):
        self.node.publish_initial_pose(
            self.spawn_x.value(),
            self.spawn_y.value(),
            self.spawn_z.value(),
            self.spawn_roll.value(),
            self.spawn_pitch.value(),
            self.spawn_yaw.value(),
        )
        self.app_status.setText('Published /initialpose')

    def _refresh(self):
        snap = self.node.snapshot()
        enabled = [key for key, box in self.checkboxes.items() if box.isChecked()]
        self.plot.set_data(snap['series'], enabled)
        self._refresh_summary(snap)
        self._refresh_table(snap)

    def _refresh_summary(self, snap):
        status = snap['status']
        self.summary_labels['surface'].setText(
            f'surface: {status.get("dominant_surface", "none")}')
        self.summary_labels['attached'].setText(
            f'attached: {status.get("attached_count", 0)}')
        self.summary_labels['model'].setText(
            f'model: {status.get("adhesion_model", "n/a")}')
        thickness = status.get('wall_thickness_mm')
        if thickness is None:
            self.summary_labels['thickness'].setText('thickness: n/a')
        else:
            self.summary_labels['thickness'].setText(
                f'thickness: {float(thickness):.1f} mm')
        self.summary_labels['cmd'].setText(
            f'cmd: {snap["cmd_speed"]:.3f} m/s')
        _source, selected_cmd = self._selected_cmd(snap)
        self.summary_labels['action'].setText(
            f'action: {_classify_motion_action(*selected_cmd)}')
        self.summary_labels['boundary'].setText(
            f'boundary: {status.get("boundary_state", "n/a")} '
            f'{float(status.get("boundary_risk", 0.0)):.2f}')
        self.summary_labels['state'].setText(
            f'state: {snap["state_text"] or "n/a"}')
        planner = snap.get('planner_debug', {})
        if planner:
            goal = 'active' if planner.get('goal_active') else 'waiting'
            self.summary_labels['planner'].setText(
                f'planner: {planner.get("mode", "n/a")} ({goal})')
            route_path = planner.get('route_path', [])
            route_text = '>'.join(route_path) if route_path else 'n/a'
            edge_clearance = planner.get('edge_clearance_m')
            edge_scale = planner.get('edge_motion_scale')
            try:
                edge_text = f'{float(edge_clearance):.2f}m'
            except (TypeError, ValueError):
                edge_text = 'n/a'
            try:
                scale_text = f'{float(edge_scale):.2f}x'
            except (TypeError, ValueError):
                scale_text = 'n/a'
            self.summary_labels['route'].setText(
                f'route: {planner.get("route_algorithm", "n/a")} '
                f'{route_text} cost={float(planner.get("route_cost", 0.0)):.2f} '
                f'edge={edge_text} scale={scale_text}')
        else:
            self.summary_labels['planner'].setText('planner: n/a')
            self.summary_labels['route'].setText('route: n/a')
        health = snap.get('health', {})
        joint_ok = health.get('joint_age') is not None and health['joint_age'] < 2.0
        pose_ok = health.get('pose_age') is not None and health['pose_age'] < 2.0
        world_ok = health.get('world_pose_age') is not None and health['world_pose_age'] < 2.0
        self.summary_labels['health'].setText(
            f'health: joint={"ok" if joint_ok else "missing"} '
            f'pose={"ok" if pose_ok else "missing"} '
            f'world={"ok" if world_ok else "missing"}')

        source = self.command_source.currentData()
        active = source if source != 'passive' else self._newest_cmd_source(snap)
        color = {
            'direct': '#1b73e8',
            'climb': '#188038',
            'dashboard': '#d93025',
            'passive': '#666666',
        }.get(active, '#666666')
        self.source_label.setText(
            f'<span style="color:{color};">●</span> {active}')

    def _joint_effort_by_name(self, joint_state):
        if joint_state is None or not joint_state.effort:
            return {}
        return {
            name: joint_state.effort[i]
            for i, name in enumerate(joint_state.name)
            if i < len(joint_state.effort)
        }

    def _refresh_table(self, snap):
        wheels = snap['status'].get('wheels', [])
        by_name = {w.get('wheel', ''): w for w in wheels}
        efforts = self._joint_effort_by_name(snap['joint_state'])
        for row, wheel in enumerate(WHEEL_LINKS):
            w = by_name.get(wheel, {})
            joint_torque = efforts.get(WHEEL_JOINTS[row])
            values = [
                wheel,
                str(w.get('surface', 'none')),
                'yes' if w.get('attached', False) else 'no',
                f'{float(w.get("force_n", 0.0)):.1f}',
                f'{float(w.get("gap_mm", 0.0)):.2f}',
                f'{float(w.get("contact_fraction", 0.0)):.2f}',
                str(int(w.get('contact_points', 0))),
                f'{1000.0 * float(w.get("depth_m", 0.0)):.2f}',
                f'{float(w.get("boundary_risk", 0.0)):.2f}',
                f'{float(w.get("holding_torque_nm", 0.0)):.2f}',
                'n/a' if joint_torque is None else f'{joint_torque:.2f}',
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.table.setItem(row, col, item)
        self.table.resizeColumnsToContents()

    def _newest_cmd_source(self, snap):
        cmds = snap.get('cmds', {})
        if not cmds:
            return 'passive'
        source, data = max(
            cmds.items(), key=lambda item: float(item[1].get('stamp', 0.0)))
        if float(data.get('stamp', 0.0)) <= 0.0:
            return 'passive'
        return source

    def _selected_cmd(self, snap):
        source = self.command_source.currentData()
        cmds = snap.get('cmds', {})
        if self.teleop_enabled:
            return 'dashboard', self.teleop_cmd
        if source == 'passive':
            source = self._newest_cmd_source(snap)
        data = cmds.get(source, {})
        return source, tuple(data.get('cmd', (0.0, 0.0, 0.0)))

    def _teleop_toggled(self, state):
        self.teleop_enabled = state == Qt.Checked
        if not self.teleop_enabled:
            self.teleop_cmd = (0.0, 0.0, 0.0)
            self._publish_teleop()
        self.setFocus()

    def _publish_teleop(self):
        if not self.teleop_enabled:
            return
        target = self.command_source.currentData()
        if target == 'passive':
            return
        self.node.publish_dashboard_cmd(target, *self.teleop_cmd)

    def keyPressEvent(self, event):
        if event.modifiers() & Qt.ControlModifier:
            if event.key() == Qt.Key_S:
                self._save_pose()
                return
            if event.key() == Qt.Key_R:
                self._restart_launch()
                return
            if event.key() == Qt.Key_G:
                self._set_gazebo_pose()
                return
            if event.key() == Qt.Key_L:
                self._use_live_pose()
                return

        focused = QApplication.focusWidget()
        if isinstance(focused, QLineEdit):
            super().keyPressEvent(event)
            return

        key = event.key()
        vx, vy, omega = self.teleop_cmd
        if key == Qt.Key_W:
            vx = self.linear_step
        elif key == Qt.Key_S:
            vx = -self.linear_step
        elif key == Qt.Key_A:
            vy = self.linear_step
        elif key == Qt.Key_D:
            vy = -self.linear_step
        elif key == Qt.Key_Q:
            omega = self.angular_step
        elif key == Qt.Key_E:
            omega = -self.angular_step
        elif key == Qt.Key_Space:
            vx, vy, omega = 0.0, 0.0, 0.0
        elif key == Qt.Key_BracketRight:
            self.linear_step = min(0.50, self.linear_step + 0.02)
            self.app_status.setText(f'Linear teleop step {self.linear_step:.2f} m/s')
            return
        elif key == Qt.Key_BracketLeft:
            self.linear_step = max(0.02, self.linear_step - 0.02)
            self.app_status.setText(f'Linear teleop step {self.linear_step:.2f} m/s')
            return
        else:
            super().keyPressEvent(event)
            return

        self.teleop_cmd = (vx, vy, omega)
        if self.teleop_enabled:
            self._publish_teleop()
            self.app_status.setText(
                f'Dashboard teleop cmd vx={vx:+.2f}, vy={vy:+.2f}, omega={omega:+.2f}')
        else:
            self.app_status.setText('Dashboard teleop is off; command preview updated only')

    def closeEvent(self, event):
        self._stop_launch()
        super().closeEvent(event)


def main():
    rclpy.init()
    node = DashboardNode()

    def spin_node():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    win = DashboardWindow(node)
    win.show()
    code = app.exec_()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
