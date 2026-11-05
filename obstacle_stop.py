#!/usr/bin/env python3
"""PX4 SITL / ROS 2 Jazzy: monitor depth, or take off and stop before an obstacle.

Requires the simulation, MAVROS, and /depth_camera Gazebo->ROS bridge running.
Only a central forward camera region is checked; this is not a 3D planner.
Run without --fly first. Coordinates on MAVROS pose topics are ENU.
Each run records a CSV in logs/ next to this script (override with --csv).
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
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
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
    ]

    def __init__(self, path=None):
        if path is None:
            filename = datetime.now(timezone.utc).strftime('obstacle_stop_%Y%m%dT%H%M%S_%fZ.csv')
            path = Path(__file__).resolve().parent / 'logs' / filename
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
    def __init__(self, args):
        super().__init__('bp_obstacle_stop')
        self.args = args
        self.state = None
        self.velocity = None
        self.velocity_at = None
        self.depth_stamp = ''
        self.depth_frame = ''
        self.event_id = 0
        self.stage_reason = 'Waiting for input.'
        self.setpoint_published = False
        self.csv_failed = False
        self.pose = None
        self.state_at = self.pose_at = self.depth_at = 0.0
        self.distance = None
        self.valid_fraction = 0.0
        self.stage = 'WAIT'
        self.stage_at = self.started = time.monotonic()
        self.report_at = self.request_at = 0.0
        self.pending = None
        self.pending_at = 0.0
        self.target = None
        self.origin = None
        self.forward = None
        self.heading = None
        self.travel = 0.0
        self.altitude = None
        self.reached_at = None
        self.done = False
        self.exit_code = 0
        self.interrupted = False
        self.last_tick = self.started
        self.create_subscription(State, '/mavros/state', self.on_state,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.on_pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local',
                                 self.on_velocity, qos_profile_sensor_data)
        self.create_subscription(Image, args.depth_topic, self.on_depth,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(PoseStamped,
                                        '/mavros/setpoint_position/local', 10)
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.recorder = CsvRecorder(getattr(args, 'csv', None))
        self.get_logger().info(f'CSV: {self.recorder.path}')
        self.timer = self.create_timer(.05, self.tick)
        self.get_logger().info('FLIGHT ENABLED' if args.fly else
                               'MONITOR ONLY: no commands will be sent.')

    def on_state(self, msg):
        self.state, self.state_at = msg, time.monotonic()

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if all(math.isfinite(x) for x in values) and .9 < norm < 1.1:
            self.pose, self.pose_at = msg, time.monotonic()

    def on_velocity(self, msg):
        v = msg.twist.linear
        if all(math.isfinite(value) for value in (v.x, v.y, v.z)):
            self.velocity, self.velocity_at = msg, time.monotonic()

    def on_depth(self, msg):
        self.depth_stamp = stamp_ns(msg.header.stamp)
        self.depth_frame = msg.header.frame_id
        try:
            self.distance, self.valid_fraction = forward_distance(msg)
        except (ValueError, TypeError, BufferError):
            self.distance, self.valid_fraction = None, 0.0
        self.depth_at = time.monotonic()

    def change_stage(self, stage, reason):
        self.stage, self.stage_at = stage, time.monotonic()
        self.event_id += 1
        self.stage_reason = reason
        self.get_logger().info(f'{stage}: {reason}')

    def request(self, kind):
        now = time.monotonic()
        if self.pending is not None:
            if not self.pending.done():
                if now - self.pending_at > 5:
                    self.pending.cancel()
                    self.pending = None
                return
            try:
                response = self.pending.result()
                accepted = getattr(response, 'success',
                                   getattr(response, 'mode_sent', False))
                if not accepted:
                    self.get_logger().warning('Command rejected; check PX4/QGC messages.')
            except Exception as exc:
                self.get_logger().warning(f'Service error: {exc}')
            self.pending = None
        if now - self.request_at < 2:
            return
        client = self.arm_client if kind == 'ARM' else self.mode_client
        if not client.service_is_ready():
            return
        if kind == 'ARM':
            req = CommandBool.Request()
            req.value = True
        else:
            req = SetMode.Request()
            req.custom_mode = kind
        self.pending = client.call_async(req)
        self.request_at = self.pending_at = now

    def hold_here(self, now):
        if self.pose is not None and now - self.pose_at < 1:
            p = self.pose.pose.position
            self.target = [p.x, p.y, p.z]

    def land(self, reason):
        self.hold_here(time.monotonic())
        self.change_stage('LAND', reason)

    def publish(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.target
        msg.pose.orientation.z = math.sin(self.heading / 2)
        msg.pose.orientation.w = math.cos(self.heading / 2)
        self.pub.publish(msg)
        self.setpoint_published = True

    def log_csv(self):
        if self.csv_failed:
            return
        now = time.monotonic()
        state_age = now - self.state_at if self.state is not None else ''
        pose_age = now - self.pose_at if self.pose is not None else ''
        velocity_age = now - self.velocity_at if self.velocity_at is not None else ''
        depth_age = now - self.depth_at if self.depth_stamp != '' else ''
        row = {
            'wall_time_utc': datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
            'elapsed_monotonic_s': now - self.started,
            'ros_time_ns': getattr(self.get_clock().now(), 'nanoseconds', ''),
            'stage': self.stage, 'event_id': self.event_id,
            'stage_reason': self.stage_reason, 'fly_enabled': int(self.args.fly),
            'state_received': int(self.state is not None),
            'state_age_s': state_age,
            'pose_fresh': int(pose_age != '' and pose_age < 1), 'pose_age_s': pose_age,
            'velocity_fresh': int(velocity_age != '' and velocity_age < 1),
            'velocity_age_s': velocity_age,
            'depth_available': int(depth_age != '' and depth_age < .5 and self.distance is not None),
            'depth_age_s': depth_age, 'depth_stamp_ns': self.depth_stamp,
            'depth_frame': self.depth_frame,
            'depth_min_m': self.distance if self.distance is not None else '',
            'depth_valid_fraction': self.valid_fraction,
            'setpoint_published': int(self.setpoint_published),
            'planned_forward_progress_m': self.travel,
            'height_setting_m': self.args.height, 'speed_setting_m_s': self.args.speed,
            'stop_distance_setting_m': self.args.stop_distance,
            'max_travel_setting_m': self.args.max_travel, 'depth_topic': self.args.depth_topic,
        }
        if self.state is not None:
            row.update(connected=int(self.state.connected), armed=int(self.state.armed),
                       mode=self.state.mode)
        if self.pose is not None:
            p, q = self.pose.pose.position, self.pose.pose.orientation
            row.update(x_m=p.x, y_m=p.y, z_m=p.z, qx=q.x, qy=q.y, qz=q.z, qw=q.w,
                       pose_stamp_ns=stamp_ns(self.pose.header.stamp),
                       pose_frame=self.pose.header.frame_id)
            if self.origin is not None:
                row['height_from_start_m'] = p.z - self.origin[2]
        if self.velocity is not None:
            v = self.velocity.twist.linear
            row.update(vx_m_s=v.x, vy_m_s=v.y, vz_m_s=v.z,
                       horizontal_speed_m_s=math.hypot(v.x, v.y),
                       speed_3d_m_s=math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z),
                       velocity_stamp_ns=stamp_ns(self.velocity.header.stamp),
                       velocity_frame=self.velocity.header.frame_id)
        if self.setpoint_published:
            row.update(setpoint_x_m=self.target[0], setpoint_y_m=self.target[1],
                       setpoint_z_m=self.target[2], setpoint_yaw_rad=self.heading)
        try:
            self.recorder.write(row)
        except (OSError, ValueError, csv.Error) as exc:
            # A logging failure must not interrupt the OFFBOARD setpoint stream.
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

    def tick(self):
        self.setpoint_published = False
        try:
            self.flight_tick()
        finally:
            self.log_csv()

    def flight_tick(self):
        now = time.monotonic()
        dt = min(now - self.last_tick, .1)
        self.last_tick = now
        link_ok = (self.state is not None and self.state.connected
                   and now - self.state_at < 3)
        pose_ok = self.pose is not None and now - self.pose_at < 1
        depth_ok = now - self.depth_at < .5 and self.distance is not None
        if now - self.report_at >= 1:
            d = f'{self.distance:.2f} m' if depth_ok else 'UNKNOWN/STALE'
            self.get_logger().info(
                f'{self.stage} | depth={d}, valid={self.valid_fraction:.0%} '
                f'| link={link_ok} pose={pose_ok}')
            self.report_at = now
        if not self.args.fly:
            if self.interrupted:
                self.done = True
            return
        if self.interrupted and self.stage != 'LAND':
            if self.target is None:
                self.done = True
                return
            self.land('Interrupted: request landing, keep the script running.')
        if self.stage == 'WAIT':
            if now - self.started > 30:
                self.get_logger().error('Required input missing for 30 s; no flight started.')
                self.exit_code, self.done = 1, True
                return
            if not (link_ok and pose_ok and depth_ok):
                return
            if self.state.armed:
                self.get_logger().error('Start this test with the vehicle disarmed on the ground.')
                self.exit_code, self.done = 1, True
                return
            p, q = self.pose.pose.position, self.pose.pose.orientation
            self.heading = math.atan2(2*(q.w*q.z + q.x*q.y),
                                      1 - 2*(q.y*q.y + q.z*q.z))
            self.forward = (math.cos(self.heading), math.sin(self.heading))
            self.origin = (p.x, p.y, p.z)
            self.target = [p.x, p.y, p.z]
            self.altitude = p.z + self.args.height
            self.change_stage('WARMUP', 'Stream position setpoints for 3 seconds.')
        elif self.stage not in ('LAND', 'STOP'):
            if not (link_ok and pose_ok and depth_ok):
                self.land('Input lost/invalid; cancel forward movement.')
            elif self.stage in ('TAKEOFF', 'HOVER', 'FORWARD') and (
                    self.state.mode != 'OFFBOARD' or not self.state.armed):
                self.get_logger().warning('Mode changed/disarmed: relinquishing control.')
                self.done = True
                return
        elapsed = now - self.stage_at
        if self.stage == 'WARMUP' and elapsed >= 3:
            self.change_stage('OFFBOARD', 'Request external position control.')
        elif self.stage == 'OFFBOARD':
            if self.state.mode == 'OFFBOARD':
                self.change_stage('ARM', 'Request arming through normal PX4 checks.')
            elif elapsed > 20:
                self.land('OFFBOARD request timed out.')
            else:
                self.request('OFFBOARD')
        elif self.stage == 'ARM':
            if self.state.mode != 'OFFBOARD':
                self.land('OFFBOARD was lost before takeoff.')
            elif self.state.armed:
                self.change_stage('TAKEOFF', 'Climb vertically; overhead must be clear.')
            elif elapsed > 20:
                self.land('Arming timed out; inspect PX4 preflight messages.')
            else:
                self.request('ARM')
        elif self.stage == 'TAKEOFF':
            self.target[2] = min(self.altitude, self.target[2] + .4 * dt)
            # Keep the reference near the actual vehicle if the simulator is slow.
            self.target[2] = min(self.target[2], self.pose.pose.position.z + .25)
            if abs(self.pose.pose.position.z - self.altitude) < .15:
                if self.reached_at is None:
                    self.reached_at = now
                elif now - self.reached_at >= 1:
                    self.change_stage('HOVER', 'Hold altitude for 2 seconds.')
            else:
                self.reached_at = None
            if elapsed > 45:
                self.land('Takeoff timed out.')
        elif self.stage == 'HOVER' and elapsed >= 2:
            self.change_stage('FORWARD', 'Move along initial vehicle heading.')
        elif self.stage == 'FORWARD':
            p = self.pose.pose.position
            if self.distance <= self.args.stop_distance:
                self.hold_here(now)
                self.change_stage('STOP', f'Obstacle at {self.distance:.2f} m. Hold, then land.')
            elif self.travel >= self.args.max_travel or elapsed > 25:
                self.hold_here(now)
                self.change_stage('STOP', 'Travel/time limit reached. Hold, then land.')
            elif abs(p.z - self.altitude) > .5:
                self.land('Altitude tracking error.')
            else:
                actual = ((p.x - self.origin[0]) * self.forward[0]
                          + (p.y - self.origin[1]) * self.forward[1])
                # Do not accumulate a distant target when the aircraft lags.
                self.travel = min(self.args.max_travel,
                                  self.travel + self.args.speed * dt,
                                  max(0.0, actual + .2))
                self.target[0] = self.origin[0] + self.forward[0] * self.travel
                self.target[1] = self.origin[1] + self.forward[1] * self.travel
        elif self.stage == 'STOP':
            if self.state is not None and self.state.mode != 'OFFBOARD':
                self.done = True  # Respect operator mode changes.
                return
            if elapsed >= 3:
                self.land('Stop test complete.')
        if self.stage == 'LAND':
            if (link_ok and not self.state.armed and elapsed >= 3
                    and (self.pending is None or self.pending.done())):
                self.get_logger().info('Disarmed. Test finished.')
                self.done = True
                return
            if link_ok and self.state.mode == 'AUTO.LAND':
                # PX4 now owns the descent; do not stream OFFBOARD setpoints.
                return
            if elapsed > 30:
                self.get_logger().error('Landing not confirmed. Use QGroundControl Land.')
                self.exit_code, self.done = 1, True
                return
            self.request('AUTO.LAND')
        if self.target is not None:
            self.publish()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', type=Path, help='New CSV path; default: logs/<timestamp>.csv.')
    parser.add_argument('--fly', action='store_true', help='Execute the SITL flight test.')
    parser.add_argument('--depth-topic', default='/depth_camera')
    parser.add_argument('--height', type=float, default=2.5)
    parser.add_argument('--speed', type=float, default=.3)
    parser.add_argument('--stop-distance', type=float, default=4.0)
    parser.add_argument('--max-travel', type=float, default=5.0)
    args = parser.parse_args()
    if not (1 <= args.height <= 5 and .05 <= args.speed <= .5
            and 3 <= args.stop_distance <= 8 and .1 <= args.max_travel <= 10):
        parser.error('SITL limits: height 1..5 m, speed .05.. .5 m/s, '
                     'stop-distance 3..8 m, max-travel .1..10 m.')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ObstacleStop(args)
    # Ctrl+C requests landing rather than immediately killing the setpoint stream.
    signal.signal(signal.SIGINT, lambda *_: setattr(node, 'interrupted', True))
    signal.signal(signal.SIGTERM, lambda *_: setattr(node, 'interrupted', True))
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.close_csv()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code


if __name__ == '__main__':
    raise SystemExit(main())
