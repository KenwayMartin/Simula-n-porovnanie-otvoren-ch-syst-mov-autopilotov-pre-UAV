#!/usr/bin/env python3
"""PX4 SITL / ROS 2 Jazzy: native takeoff and velocity-based obstacle-stop experiment.

Requires the simulation, MAVROS, and /depth_camera Gazebo->ROS bridge running.
Only a central forward camera region is checked; this is not a 3D planner.
Run without --fly first. Native takeoff needs /mavros/altitude (AMSL).
Native TAKEOFF/LOITER precede OFFBOARD; no ground-level OFFBOARD ramp.
--speed is now a horizontal velocity target in m/s, with an acceleration ramp.
MAVROS raw local setpoints use ROS ENU values; MAVROS converts to MAVLink NED.
Each run records a CSV in the project-root logs/ directory (override with --csv).
Rows are latest-value snapshots, not synchronized sensor measurements.
Sensor header stamps and receipt ages are included; blanks mean unavailable.
"""
import argparse
import csv
from datetime import datetime, timezone
from pathlib import Path
import math
import signal
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, Altitude, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image


def forward_distance(msg):
    """Return (nearest sampled depth, valid fraction) in the central 60% x 40%.

    Non-finite/zero/negative samples are unknown, never interpreted as free space.
    Respect endianness and row padding. Sampling every fourth pixel is deliberate:
    this first experiment targets broad walls, not wires or narrow objects.
    """
    if msg.encoding != '32FC1' or msg.height < 8 or msg.width < 8:
        return None, 0.0
    if msg.step < msg.width * 4 or len(msg.data) < msg.step * msg.height:
        return None, 0.0
    image = np.ndarray((msg.height, msg.width),
                       dtype='>f4' if msg.is_bigendian else '<f4',
                       buffer=bytes(msg.data), strides=(msg.step, 4))
    roi = image[int(msg.height * .3):int(msg.height * .7):4,
                int(msg.width * .2):int(msg.width * .8):4]
    valid = roi[np.isfinite(roi) & (roi > 0)]
    fraction = valid.size / roi.size
    if fraction < .1:
        return None, fraction
    return float(valid.min()), fraction


def stamp_ns(stamp):
    if stamp is None:
        return ''
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class CsvRecorder:
    FIELDS = [
        'wall_time_utc', 'elapsed_monotonic_s', 'ros_time_ns',
        'stage', 'event_id', 'stage_reason', 'fly_enabled',
        'state_received', 'connected', 'armed', 'mode', 'state_age_s',
        'pose_fresh', 'pose_age_s', 'pose_stamp_ns', 'pose_frame',
        'x_m', 'y_m', 'z_m', 'height_from_start_m',
        'qx', 'qy', 'qz', 'qw',
        'velocity_fresh', 'velocity_age_s', 'velocity_stamp_ns', 'velocity_frame',
        'vx_m_s', 'vy_m_s', 'vz_m_s', 'horizontal_speed_m_s', 'speed_3d_m_s',
        'depth_available', 'depth_age_s', 'depth_stamp_ns', 'depth_frame',
        'depth_min_m', 'depth_valid_fraction',
        'setpoint_published', 'setpoint_x_m', 'setpoint_y_m', 'setpoint_z_m',
        'setpoint_yaw_rad', 'planned_forward_progress_m',
        'height_setting_m', 'speed_setting_m_s', 'stop_distance_setting_m',
        'max_travel_setting_m', 'depth_topic',
        'control_version', 'setpoint_kind', 'setpoint_type_mask',
        'setpoint_vx_m_s', 'setpoint_vy_m_s', 'setpoint_vz_m_s',
        'commanded_horizontal_speed_m_s', 'forward_progress_m',
        'amsl_m', 'amsl_age_s', 'takeoff_altitude_amsl_m',
        'acceleration_setting_m_s2', 'depth_timeout_s', 'depth_hold_timeout_s',
        'input_problem', 'last_service_command', 'last_service_result',
    ]

    def __init__(self, path=None):
        if path is None:
            filename = datetime.now(timezone.utc).strftime('obstacle_stop_%Y%m%dT%H%M%S_%fZ.csv')
            path = Path(__file__).resolve().parent.parent / 'logs' / filename
        self.path = Path(path).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Never overwrite a previous run, including an explicitly named CSV.
        self.file = self.path.open('x', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDS)
        self.writer.writeheader()
        self.file.flush()
        self.flushed_at = time.monotonic()

    def write(self, row):
        self.writer.writerow(row)
        now = time.monotonic()
        if now - self.flushed_at >= 1:
            self.file.flush()
            self.flushed_at = now

    def close(self):
        self.file.close()


class ObstacleStop(Node):
    VERSION = 'native_takeoff_velocity_v2'
    NATIVE_MODES = ('AUTO.TAKEOFF', 'AUTO.LOITER')
    OFFBOARD_STAGES = ('SETTLE', 'FORWARD', 'STOP', 'SENSOR_HOLD')

    def __init__(self, args):
        super().__init__('bp_obstacle_stop')
        self.args = args
        self.state = self.pose = self.velocity = self.amsl = None
        self.state_at = self.pose_at = self.velocity_at = self.amsl_at = None
        self.depth_at = None
        self.depth_stamp = self.depth_frame = ''
        self.distance = None
        self.valid_fraction = 0.
        self.stage = 'WAIT'
        self.stage_reason = 'Waiting for MAVROS, pose, velocity, depth and AMSL altitude.'
        self.stage_at = self.started = self.last_tick = time.monotonic()
        self.report_at = self.request_at = float('-inf')
        self.event_id = 0
        self.pending = None
        self.pending_kind = ''
        self.pending_at = 0.
        self.command_ok = {}
        self.last_service_command = self.last_service_result = ''
        self.origin = self.target = self.forward = self.heading = self.altitude = None
        self.takeoff_amsl = None
        self.forward_start = None
        self.travel = self.progress = self.commanded_speed = 0.
        self.stable_since = None
        self.stop_position_hold = False
        self.own_offboard = False
        self.setpoint_published = False
        self.last_setpoint = None
        self.input_problem = ''
        self.done = self.interrupted = self.csv_failed = False
        self.exit_code = 0
        self.create_subscription(State, '/mavros/state', self.on_state, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self.on_velocity, qos_profile_sensor_data)
        self.create_subscription(Altitude, '/mavros/altitude', self.on_altitude, qos_profile_sensor_data)
        self.create_subscription(Image, args.depth_topic, self.on_depth, qos_profile_sensor_data)
        self.pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.recorder = CsvRecorder(args.csv)
        self.get_logger().info(f'{self.VERSION} | CSV: {self.recorder.path}')
        self.get_logger().info('FLIGHT ENABLED: native takeoff, then velocity OFFBOARD.'
                               if args.fly else 'MONITOR ONLY: no commands will be sent.')
        self.timer = self.create_timer(.05, self.tick)

    @staticmethod
    def age(at, now):
        return None if at is None else max(0., now - at)

    def on_state(self, msg):
        self.state, self.state_at = msg, time.monotonic()

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) and .9 < norm < 1.1:
            self.pose, self.pose_at = msg, time.monotonic()

    def on_velocity(self, msg):
        v = msg.twist.linear
        if all(math.isfinite(value) for value in (v.x, v.y, v.z)):
            self.velocity, self.velocity_at = msg, time.monotonic()

    def on_altitude(self, msg):
        if math.isfinite(msg.amsl):
            self.amsl, self.amsl_at = float(msg.amsl), time.monotonic()

    def on_depth(self, msg):
        self.depth_stamp, self.depth_frame = stamp_ns(msg.header.stamp), msg.header.frame_id
        try:
            self.distance, self.valid_fraction = forward_distance(msg)
        except (ValueError, TypeError, BufferError):
            self.distance, self.valid_fraction = None, 0.
        self.depth_at = time.monotonic()

    def inputs(self, now):
        def fresh(at, limit):
            return at is not None and now - at < limit
        return {
            'link': bool(self.state is not None and self.state.connected and fresh(self.state_at, 3)),
            'pose': self.pose is not None and fresh(self.pose_at, 1),
            'velocity': self.velocity is not None and fresh(self.velocity_at, 1),
            'depth': self.distance is not None and fresh(self.depth_at, self.args.depth_timeout),
            'amsl': self.amsl is not None and fresh(self.amsl_at, 3),
        }

    def change_stage(self, stage, reason):
        self.stage, self.stage_at = stage, time.monotonic()
        self.stage_reason = reason
        self.event_id += 1
        self.stable_since = None
        self.get_logger().info(f'{stage}: {reason}')

    def poll_request(self):
        if self.pending is None:
            return
        if not self.pending.done():
            if time.monotonic() - self.pending_at > 5:
                self.pending.cancel()
                self.command_ok[self.pending_kind] = False
                self.last_service_result = 'TIMEOUT'
                self.get_logger().warning(f'{self.pending_kind}: service response timeout.')
                self.pending = None
            return
        try:
            result = self.pending.result()
            accepted = bool(getattr(result, 'success', getattr(result, 'mode_sent', False)))
            self.command_ok[self.pending_kind] = accepted
            self.last_service_result = str(getattr(result, 'result', 'ACCEPTED' if accepted else 'REJECTED'))
            self.get_logger().info(f'{self.pending_kind}: accepted={accepted}, result={self.last_service_result}')
        except Exception as exc:
            self.command_ok[self.pending_kind] = False
            self.last_service_result = 'ERROR'
            self.get_logger().warning(f'{self.pending_kind}: service error: {exc}')
        self.pending = None

    def request(self, kind):
        now = time.monotonic()
        if self.pending is not None or now - self.request_at < 2:
            return
        if kind == 'ARM':
            client = self.arm_client
            req = CommandBool.Request()
            req.value = True
        elif kind == 'TAKEOFF':
            client = self.takeoff_client
            req = CommandTOL.Request()
            req.min_pitch = 0.
            req.yaw = float('nan')
            req.latitude = req.longitude = float('nan')
            # PX4 NAV_TAKEOFF expects AMSL, not height above ground or GPS ellipsoid.
            req.altitude = self.takeoff_amsl
        else:
            client = self.mode_client
            req = SetMode.Request()
            req.custom_mode = kind
        if not client.service_is_ready():
            return
        self.command_ok[kind] = False
        self.last_service_command = kind
        self.last_service_result = 'PENDING'
        self.pending_kind = kind
        self.pending = client.call_async(req)
        self.request_at = self.pending_at = now

    def capture_reference(self):
        p, q = self.pose.pose.position, self.pose.pose.orientation
        self.origin = (p.x, p.y, p.z)
        self.altitude = p.z + self.args.height
        self.takeoff_amsl = self.amsl + self.args.height
        self.capture_hold()

    def capture_hold(self, update_heading=True):
        p, q = self.pose.pose.position, self.pose.pose.orientation
        self.target = [p.x, p.y, p.z]
        if update_heading:
            self.heading = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            self.forward = (math.cos(self.heading), math.sin(self.heading))

    def stable(self, condition, now, duration):
        if not condition:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = now
        return now - self.stable_since >= duration

    def land(self, reason):
        self.commanded_speed = 0.
        self.change_stage('LAND', reason)

    def release(self, reason):
        self.get_logger().warning(reason + ' Control released; no rearming or mode takeover.')
        self.stage_reason = reason
        self.own_offboard = False
        self.exit_code, self.done = 1, True

    def stop(self, reason):
        self.commanded_speed = 0.
        self.stop_position_hold = False
        self.change_stage('STOP', reason)

    def publish(self, velocity=False):
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        # The MAVROS plugin converts ROS ENU vectors/yaw to MAVLink NED.
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.yaw = self.heading
        ignore_acc = PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
        if velocity:
            # XY velocity + Z position, retaining PX4 altitude control.
            msg.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                             PositionTarget.IGNORE_VZ | ignore_acc | PositionTarget.IGNORE_YAW_RATE)
            msg.position.z = self.altitude
            msg.velocity.x = self.commanded_speed * self.forward[0]
            msg.velocity.y = self.commanded_speed * self.forward[1]
        else:
            msg.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                             PositionTarget.IGNORE_VZ | ignore_acc | PositionTarget.IGNORE_YAW_RATE)
            msg.position.x, msg.position.y, msg.position.z = self.target
        self.pub.publish(msg)
        self.last_setpoint = msg
        self.setpoint_published = True

    def tick(self):
        self.setpoint_published = False
        self.last_setpoint = None
        try:
            self.poll_request()
            self.flight_tick()
        finally:
            self.log_csv()

    def flight_tick(self):
        now = time.monotonic()
        dt = min(max(0., now - self.last_tick), .1)
        self.last_tick = now
        good = self.inputs(now)
        self.input_problem = ','.join(name for name, ok in good.items() if not ok)
        if now - self.report_at >= 1:
            d = f'{self.distance:.2f} m' if good['depth'] else 'UNKNOWN/STALE'
            speed = math.hypot(self.velocity.twist.linear.x, self.velocity.twist.linear.y) if good['velocity'] else None
            measured = f'{speed:.2f}' if speed is not None else '?'
            self.get_logger().info(f'{self.stage} | depth={d}, valid={self.valid_fraction:.0%} '
                                   f'| cmd={self.commanded_speed:.2f} m/s actual={measured} m/s '
                                   f'| missing={self.input_problem or "none"}')
            self.report_at = now
        if not self.args.fly:
            self.done = self.interrupted
            return
        if self.interrupted and self.stage != 'LAND':
            if self.origin is None:
                self.done = True
                return
            self.land('Interrupted: request native landing.')
        if self.stage == 'WAIT':
            ready = all(good.values())
            services = all(client.service_is_ready() for client in
                           (self.arm_client, self.mode_client, self.takeoff_client))
            if self.state is not None and self.state.armed:
                self.release('Start with the vehicle disarmed on the ground.')
                return
            if now - self.started > 30:
                self.get_logger().error('Preflight timeout: missing=' + self.input_problem +
                                        '; need /mavros/altitude and a setpoint_raw subscriber.')
                self.exit_code, self.done = 1, True
                return
            # Publisher discovery must complete before any arming command.
            raw_ready = self.args.takeoff_only or self.pub.get_subscription_count() > 0
            if ready and services and raw_ready:
                if self.stable(True, now, 1):
                    self.capture_reference()
                    self.change_stage('PREPARE', 'Select native AUTO.LOITER before arming.')
            else:
                self.stable_since = None
            return
        if self.stage == 'LAND':
            self.landing_tick(now, good)
            return
        # Native and external phases both need healthy position/velocity feedback.
        core = all(good[k] for k in ('link', 'pose', 'velocity'))
        if not core:
            self.land('Input failure: ' + ','.join(k for k in ('link', 'pose', 'velocity') if not good[k]))
            self.landing_tick(now, good)
            return
        elapsed = now - self.stage_at
        mode = self.state.mode
        if self.stage in self.OFFBOARD_STAGES and mode != 'OFFBOARD':
            self.release('OFFBOARD exited (operator or PX4 failsafe).')
            return
        if self.stage in self.OFFBOARD_STAGES and not self.state.armed:
            self.release('Vehicle disarmed during OFFBOARD experiment.')
            return
        if self.stage in ('TAKEOFF', 'CLIMB', 'NATIVE_HOLD', 'WARMUP'):
            if mode not in self.NATIVE_MODES or not self.state.armed:
                self.release('Native flight mode changed/disarmed.')
                return
        if self.stage == 'PREPARE':
            if mode == 'AUTO.LOITER':
                self.change_stage('ARM', 'Request normal arming; no OFFBOARD setpoints on the ground.')
            elif elapsed > 20:
                self.land('Could not select AUTO.LOITER.')
            else:
                self.request('AUTO.LOITER')
        elif self.stage == 'ARM':
            if mode != 'AUTO.LOITER':
                self.release('Mode changed before takeoff.')
            elif self.state.armed:
                if not good['amsl']:
                    self.land('AMSL altitude missing before takeoff command.')
                else:
                    self.capture_reference()
                    self.change_stage('TAKEOFF', f'Request native takeoff to AMSL {self.takeoff_amsl:.2f} m.')
            elif elapsed > 20:
                self.land('Arming timed out; inspect PX4/QGC preflight messages.')
            else:
                self.request('ARM')
        elif self.stage == 'TAKEOFF':
            if self.command_ok.get('TAKEOFF') or mode == 'AUTO.TAKEOFF':
                self.change_stage('CLIMB', 'PX4 owns the takeoff profile; no external setpoints.')
            elif elapsed > 15:
                self.land('Native takeoff command was not accepted.')
            else:
                self.request('TAKEOFF')
        elif self.stage == 'CLIMB':
            v = self.velocity.twist.linear
            reached = abs(self.pose.pose.position.z - self.altitude) < .2 and abs(v.z) < .15
            if self.stable(reached, now, 1):
                self.change_stage('NATIVE_HOLD', 'Select AUTO.LOITER and settle at takeoff altitude.')
            elif elapsed > 60:
                self.land('Native takeoff altitude not reached within 60 s.')
        elif self.stage == 'NATIVE_HOLD':
            v = self.velocity.twist.linear
            settled = (mode == 'AUTO.LOITER' and math.hypot(v.x, v.y) < .15 and abs(v.z) < .15)
            if mode != 'AUTO.LOITER':
                self.request('AUTO.LOITER')
            if self.stable(settled, now, 2):
                if self.args.takeoff_only:
                    self.land('Native takeoff-only test complete.')
                elif not good['depth']:
                    self.land('Depth unavailable after native takeoff; no forward flight.')
                else:
                    self.capture_hold()
                    self.target[2] = self.altitude
                    self.change_stage('WARMUP', 'Stream hold-position setpoints while PX4 remains in AUTO.LOITER.')
            elif elapsed > 15:
                self.land('Native hover did not stabilize.')
        elif self.stage == 'WARMUP':
            self.publish()
            if elapsed >= 3:
                self.change_stage('OFFBOARD', 'Request OFFBOARD only after native takeoff.')
        elif self.stage == 'OFFBOARD':
            self.publish()
            if mode == 'OFFBOARD':
                self.own_offboard = True
                self.change_stage('SETTLE', 'Hold position after OFFBOARD handover.')
            elif mode != 'AUTO.LOITER':
                self.release('Mode changed during OFFBOARD handover.')
            elif elapsed > 15:
                self.land('OFFBOARD handover timed out.')
            else:
                self.request('OFFBOARD')
        elif self.stage == 'SETTLE':
            self.publish()
            if elapsed >= 1:
                if not good['depth']:
                    self.change_stage('SENSOR_HOLD', 'Depth stale/invalid before forward flight; zero horizontal velocity.')
                else:
                    p = self.pose.pose.position
                    self.forward_start = (p.x, p.y)
                    self.change_stage('FORWARD', f'XY velocity target {self.args.speed:.2f} m/s.')
        elif self.stage == 'FORWARD':
            p = self.pose.pose.position
            self.progress = ((p.x - self.forward_start[0])*self.forward[0] +
                             (p.y - self.forward_start[1])*self.forward[1])
            if not good['depth']:
                self.commanded_speed = 0.
                self.change_stage('SENSOR_HOLD', 'Depth stale/invalid: cancel motion, no automatic resume.')
            elif self.distance <= self.args.stop_distance:
                self.stop(f'Obstacle at {self.distance:.2f} m; command zero XY velocity.')
            elif self.progress >= self.args.max_travel or elapsed > self.args.forward_timeout:
                self.stop('Travel/time limit reached; command zero XY velocity.')
            elif abs(p.z - self.altitude) > .5:
                self.land('Altitude tracking error during velocity flight.')
            else:
                self.commanded_speed = min(self.args.speed,
                                           self.commanded_speed + self.args.acceleration * dt)
                # Legacy CSV column is now the integral of commanded forward speed.
                self.travel += self.commanded_speed * dt
            self.publish(velocity=True)
        elif self.stage == 'SENSOR_HOLD':
            self.commanded_speed = 0.
            self.publish(velocity=True)
            if self.stable(good['depth'], now, .5):
                self.stop('Depth recovered; forward flight stays cancelled, hold then land.')
            elif elapsed >= self.args.depth_hold_timeout:
                self.land('Depth did not recover during zero-velocity hold.')
        elif self.stage == 'STOP':
            self.commanded_speed = 0.
            v = self.velocity.twist.linear
            if not self.stop_position_hold and self.stable(math.hypot(v.x, v.y) < .05 and abs(v.z) < .15, now, .5):
                self.capture_hold(update_heading=False)
                self.target[2] = self.altitude
                self.stop_position_hold = True
            self.publish(velocity=not self.stop_position_hold)
            if (elapsed >= 3 and self.stop_position_hold) or elapsed >= 8:
                self.land('Stop test complete.')

    def landing_tick(self, now, good):
        elapsed = now - self.stage_at
        pending_arm = self.pending is not None and self.pending_kind == 'ARM'
        if good['link'] and not self.state.armed and elapsed >= 3 and not pending_arm:
            self.get_logger().info('Disarmed. Test finished.')
            self.done = True
            return
        if good['link'] and self.state.mode == 'AUTO.LAND':
            self.own_offboard = False
            return
        if elapsed > 30:
            self.get_logger().error('Landing not confirmed. Use QGroundControl Land.')
            self.exit_code, self.done = 1, True
            return
        self.request('AUTO.LAND')
        # Continue only an already-owned OFFBOARD stream until mode acceptance.
        if self.own_offboard and self.state is not None and self.state.mode == 'OFFBOARD':
            self.commanded_speed = 0.
            self.publish(velocity=True)

    def log_csv(self):
        if self.csv_failed:
            return
        now = time.monotonic()
        good = self.inputs(now)
        def age(at):
            value = self.age(at, now)
            return '' if value is None else value
        row = dict(
            wall_time_utc=datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
            elapsed_monotonic_s=now-self.started,
            ros_time_ns=getattr(self.get_clock().now(), 'nanoseconds', ''),
            stage=self.stage, event_id=self.event_id, stage_reason=self.stage_reason,
            fly_enabled=int(self.args.fly), state_received=int(self.state is not None),
            state_age_s=age(self.state_at), pose_fresh=int(good['pose']), pose_age_s=age(self.pose_at),
            velocity_fresh=int(good['velocity']), velocity_age_s=age(self.velocity_at),
            depth_available=int(good['depth']), depth_age_s=age(self.depth_at),
            depth_stamp_ns=self.depth_stamp, depth_frame=self.depth_frame,
            depth_min_m=self.distance if self.distance is not None else '',
            depth_valid_fraction=self.valid_fraction,
            setpoint_published=int(self.setpoint_published),
            planned_forward_progress_m=self.travel,
            height_setting_m=self.args.height, speed_setting_m_s=self.args.speed,
            stop_distance_setting_m=self.args.stop_distance,
            max_travel_setting_m=self.args.max_travel, depth_topic=self.args.depth_topic,
            control_version=self.VERSION, setpoint_kind='none',
            commanded_horizontal_speed_m_s=self.commanded_speed if self.setpoint_published else '',
            forward_progress_m=self.progress if self.forward_start is not None else '',
            amsl_m=self.amsl if self.amsl is not None else '', amsl_age_s=age(self.amsl_at),
            takeoff_altitude_amsl_m=self.takeoff_amsl if self.takeoff_amsl is not None else '',
            acceleration_setting_m_s2=self.args.acceleration,
            depth_timeout_s=self.args.depth_timeout, depth_hold_timeout_s=self.args.depth_hold_timeout,
            input_problem=self.input_problem,
            last_service_command=self.last_service_command, last_service_result=self.last_service_result,
        )
        if self.state is not None:
            row.update(connected=int(self.state.connected), armed=int(self.state.armed), mode=self.state.mode)
        if self.pose is not None:
            p, q = self.pose.pose.position, self.pose.pose.orientation
            row.update(x_m=p.x, y_m=p.y, z_m=p.z, qx=q.x, qy=q.y, qz=q.z, qw=q.w,
                       pose_stamp_ns=stamp_ns(self.pose.header.stamp), pose_frame=self.pose.header.frame_id)
            if self.origin is not None:
                row['height_from_start_m'] = p.z - self.origin[2]
            if self.forward_start is not None:
                row['forward_progress_m'] = ((p.x-self.forward_start[0])*self.forward[0] +
                                             (p.y-self.forward_start[1])*self.forward[1])
        if self.velocity is not None:
            v = self.velocity.twist.linear
            row.update(vx_m_s=v.x, vy_m_s=v.y, vz_m_s=v.z,
                       horizontal_speed_m_s=math.hypot(v.x, v.y),
                       speed_3d_m_s=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),
                       velocity_stamp_ns=stamp_ns(self.velocity.header.stamp),
                       velocity_frame=self.velocity.header.frame_id)
        if self.setpoint_published:
            msg = self.last_setpoint
            row.update(setpoint_yaw_rad=msg.yaw, setpoint_type_mask=msg.type_mask)
            if msg.type_mask & PositionTarget.IGNORE_PX:
                row.update(setpoint_kind='velocity_xy_position_z',
                           setpoint_z_m=msg.position.z,
                           setpoint_vx_m_s=msg.velocity.x, setpoint_vy_m_s=msg.velocity.y)
            else:
                row.update(setpoint_kind='position_xyz',
                           setpoint_x_m=msg.position.x, setpoint_y_m=msg.position.y,
                           setpoint_z_m=msg.position.z, commanded_horizontal_speed_m_s=0.)
        try:
            self.recorder.write(row)
        except (OSError, ValueError, csv.Error) as exc:
            self.csv_failed = True
            self.get_logger().error(f'CSV recording stopped: {exc}')

    def close_csv(self):
        try:
            self.recorder.close()
        except OSError as exc:
            self.csv_failed = True
            self.get_logger().error(f'CSV close failed: {exc}')
        if not self.csv_failed:
            self.get_logger().info(f'CSV saved: {self.recorder.path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fly', action='store_true', help='Execute the PX4 SITL flight test.')
    parser.add_argument('--takeoff-only', action='store_true', help='Native takeoff, hover and land; no OFFBOARD or forward motion.')
    parser.add_argument('--csv', type=Path, help='New CSV path; default: project-root logs/<timestamp>.csv.')
    parser.add_argument('--depth-topic', default='/depth_camera')
    parser.add_argument('--height', type=float, default=2.5)
    parser.add_argument('--speed', type=float, default=.3, help='Horizontal velocity target in m/s, not a moving position.')
    parser.add_argument('--acceleration', type=float, default=.4, help='Ramp of forward velocity target in m/s^2.')
    parser.add_argument('--stop-distance', type=float, default=4.)
    parser.add_argument('--max-travel', type=float, default=5.)
    parser.add_argument('--forward-timeout', type=float, default=30.)
    parser.add_argument('--depth-timeout', type=float, default=.5, help='After this receipt age, cancel forward velocity immediately.')
    parser.add_argument('--depth-hold-timeout', type=float, default=2., help='Zero-velocity hold before landing on persistent depth failure.')
    args = parser.parse_args()
    ranges = {'height': (1, 5), 'speed': (.05, 1), 'acceleration': (.1, .8),
              'stop_distance': (3, 8), 'max_travel': (.1, 10), 'forward_timeout': (5, 120),
              'depth_timeout': (.2, .5), 'depth_hold_timeout': (.5, 5)}
    for key, (low, high) in ranges.items():
        value = getattr(args, key)
        if not math.isfinite(value) or not low <= value <= high:
            parser.error(f'--{key.replace("_", "-")} must be finite and in {low}..{high}.')
    if args.takeoff_only and not args.fly:
        parser.error('--takeoff-only requires --fly.')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ObstacleStop(args)
    signal.signal(signal.SIGINT, lambda *_: setattr(node, 'interrupted', True))
    signal.signal(signal.SIGTERM, lambda *_: setattr(node, 'interrupted', True))
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.1)
    except ExternalShutdownException:
        pass
    finally:
        node.close_csv()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code


if __name__ == '__main__':
    raise SystemExit(main())
