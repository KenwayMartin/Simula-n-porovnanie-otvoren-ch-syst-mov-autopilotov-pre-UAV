#!/usr/bin/env python3
"""Read-only depth viewer for BP obstacle_stop.py (ROS 2 Jazzy + OpenCV).

Matches the controller's central ROI, stride=4 and 10% valid-sample gate.
No publishers, arming calls, mode changes, or flight commands.
"""
import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

WINDOW = 'BP - depth diagnostics (Q / Esc to close)'


def analyse_depth(msg):
    if msg.encoding != '32FC1':
        raise ValueError(f'Expected 32FC1, received {msg.encoding}')
    if msg.width < 8 or msg.height < 8:
        raise ValueError('Depth frame is too small')
    if msg.step < msg.width * 4 or len(msg.data) < msg.step * msg.height:
        raise ValueError('Incomplete frame or invalid row stride')
    image = np.ndarray((msg.height, msg.width),
                       dtype='>f4' if msg.is_bigendian else '<f4',
                       buffer=bytes(msg.data), strides=(msg.step, 4))
    x0, x1 = int(msg.width * .2), int(msg.width * .8)
    y0, y1 = int(msg.height * .3), int(msg.height * .7)
    samples = image[y0:y1:4, x0:x1:4]
    valid = np.isfinite(samples) & (samples > 0)
    fraction = float(valid.mean())
    nearest, pixel = None, None
    if valid.any():
        masked = np.where(valid, samples, np.inf)
        row, col = np.unravel_index(np.argmin(masked), masked.shape)
        nearest = float(samples[row, col])
        pixel = (x0 + int(col) * 4, y0 + int(row) * 4)
    return image, (x0, y0, x1, y1), nearest, pixel, fraction


def colour_depth(image, max_depth):
    # Fixed scale across frames: near=warm, far=cool; invalid values=dark gray.
    valid = np.isfinite(image) & (image > 0)
    scaled = np.zeros(image.shape, dtype=np.uint8)
    scaled[valid] = np.rint(255 * (1 - np.clip(image[valid] / max_depth, 0, 1)))
    result = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    result[~valid] = (35, 35, 35)
    return result


def draw_view(snapshot, age, max_depth, stop_distance, error=''):
    footer_h = 190
    if snapshot is None:
        canvas = np.zeros((480 + footer_h, 800, 3), dtype=np.uint8)
        cv2.putText(canvas, 'Waiting for depth frames ...', (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 220, 255), 1)
        if error:
            cv2.putText(canvas, error[:105], (20, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, (80, 80, 255), 1)
        return canvas
    image, bounds, nearest, pixel, fraction, stamp = snapshot
    h, w = image.shape
    view = colour_depth(image, max_depth)
    x0, y0, x1, y1 = bounds
    cv2.rectangle(view, (x0, y0), (x1 - 1, y1 - 1), (0, 255, 255), 2)
    if pixel is not None:
        cv2.circle(view, pixel, 9, (255, 255, 255), 2)
        cv2.drawMarker(view, pixel, (0, 0, 255), cv2.MARKER_CROSS, 15, 2)
    # Leave small frames unscaled: pixel marker stays in source-image coordinates.
    canvas = np.zeros((h + footer_h, max(w, 800), 3), dtype=np.uint8)
    canvas[:h, :w] = view
    accepted = fraction >= .1 and nearest is not None
    fresh = age < .5 and not error
    if not fresh:
        status = 'STALE / ERROR - controller depth unavailable'
        status_colour = (80, 80, 255)
    elif not accepted:
        status = 'INVALID - fewer than 10% finite positive ROI samples'
        status_colour = (80, 80, 255)
    elif nearest <= stop_distance:
        status = f'WITHIN STOP THRESHOLD ({stop_distance:.2f} m)'
        status_colour = (80, 80, 255)
    else:
        status = f'ABOVE STOP THRESHOLD ({stop_distance:.2f} m)'
        status_colour = (80, 220, 80)
    text = [
        (status, status_colour),
        (f'Nearest sampled depth: {nearest:.3f} m at pixel {pixel}'
         if nearest is not None else 'Nearest sampled depth: none', (255, 255, 255)),
        (f'ROI valid: {fraction:.1%} | receipt age: {age:.2f} s | stamp: {stamp}', (220, 220, 220)),
        (f'ROI: x=[{x0},{x1}), y=[{y0},{y1}); stride=4 | 32FC1 | {w}x{h}', (220, 220, 220)),
        (f'Fixed display scale: 0..{max_depth:g} m; warm=near, cool=far, dark gray=unknown', (220, 220, 220)),
        ('Yellow box = tested ROI; red cross = minimum sample. Q / Esc = close.', (0, 255, 255)),
        ('READ ONLY. A depth threshold alone does not certify a clear flight path.', (220, 220, 220)),
    ]
    for i, (line, colour) in enumerate(text):
        cv2.putText(canvas, line, (12, h + 23 + 24 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, .46, colour, 1, cv2.LINE_AA)
    if not fresh:
        cv2.putText(canvas, 'STALE / ERROR', (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 0, 255), 2)
    return canvas


class DepthDebug(Node):
    def __init__(self, topic):
        super().__init__('bp_depth_debug')
        self.snapshot = None
        self.received_at = 0.
        self.error = ''
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f'Read-only viewer listening on {topic}')

    def on_image(self, msg):
        try:
            result = analyse_depth(msg)
            stamp = f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
            self.snapshot = (*result, stamp)
            self.received_at = time.monotonic()
            self.error = ''
        except (ValueError, TypeError, BufferError) as exc:
            if str(exc) != self.error:
                self.get_logger().error(str(exc))
            self.error = str(exc)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/depth_camera')
    parser.add_argument('--max-depth', type=float, default=12.,
                        help='Visualization scale in metres, not sensor maximum range.')
    parser.add_argument('--stop-distance', type=float, default=4.,
                        help='Display threshold; match obstacle_stop.py if changed.')
    args = parser.parse_args()
    if not (np.isfinite(args.max_depth) and args.max_depth > 0
            and np.isfinite(args.stop_distance) and args.stop_distance > 0):
        parser.error('Distances must be finite and positive.')
    rclpy.init()
    node = DepthDebug(args.topic)
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 960, 800)
        next_frame = 0.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.01)
            now = time.monotonic()
            if now >= next_frame:
                frame = draw_view(node.snapshot, now - node.received_at,
                                  args.max_depth, args.stop_distance, node.error)
                cv2.imshow(WINDOW, frame)
                next_frame = now + .1
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q'), ord('Q')):
                break
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except cv2.error as exc:
        node.get_logger().error(f'OpenCV GUI error: {exc}')
        return 1
    finally:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
