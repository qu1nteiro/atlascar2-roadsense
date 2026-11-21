"""Simulated driver: 2D LiDAR 3.5 m from base_link, on a street with a parked van.

The ATLASCAR2 moves forward at `speed` m/s; the parked van (and, in the "blocked"
scenario, a truck stopped in the other lane) approaches, passes and the scene
repeats. Used to test the perception -> grid -> A* chain without the real car.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

SENSOR_X = 3.5    # sensor position in base_link (in the real project it comes from the URDF)


def rectangle(x0, y0, x1, y1):
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
            ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


class FakeScan(Node):

    def __init__(self):
        super().__init__('fake_scan')
        self.declare_parameter('frame_id', 'lidar2d_frente')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('speed', 4.0)
        self.declare_parameter('scenario', 'swerve')        # swerve | blocked
        self.frame_id = self.get_parameter('frame_id').value
        f = self.get_parameter('rate').value

        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.t0 = self.get_clock().now()
        self.rng = np.random.default_rng(0)
        self.create_timer(1.0 / f, self.publish_scan)

        self.tf_static = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = self.frame_id
        tf.transform.translation.x = SENSOR_X
        tf.transform.translation.z = 0.4
        tf.transform.rotation.w = 1.0
        self.tf_static.sendTransform(tf)

    def scene(self, t):
        """Segments (in base_link) visible at time t.

        Road between a sidewalk on the right (y = -3.5 m) and a wall on the
        left (y = +5 m); the car drives at y = 0.
        """
        v = self.get_parameter('speed').value
        xo = 40.0 - (v * t) % 50.0               # rear of the parked van
        seg = [((-10, 5.0), (60, 5.0)),          # wall on the left
               ((-10, -3.5), (60, -3.5))]        # sidewalk on the right
        seg += rectangle(xo, -3.3, xo + 5.0, -0.8)          # parked van, invading the lane
        if self.get_parameter('scenario').value == 'blocked':
            seg += rectangle(xo + 2.0, 1.0, xo + 9.0, 4.6)  # truck stopped in the other lane
        return seg

    def publish_scan(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = -math.pi / 2
        scan.angle_max = math.pi / 2
        n = 361
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (n - 1)
        scan.range_min = 0.1
        scan.range_max = 40.0

        ang = scan.angle_min + scan.angle_increment * np.arange(n)
        dx, dy = np.cos(ang), np.sin(ang)
        r = np.full(n, np.inf)
        for (ax, ay), (bx, by) in self.scene(t):
            ax -= SENSOR_X
            bx -= SENSOR_X
            ex, ey = bx - ax, by - ay
            den = dx * ey - dy * ex
            with np.errstate(divide='ignore', invalid='ignore'):
                s = (ax * ey - ay * ex) / den          # distance along the ray
                u = (ax * dy - ay * dx) / den          # position along the segment
            ok = (np.abs(den) > 1e-9) & (s > 0) & (u >= 0) & (u <= 1)
            r = np.where(ok & (s < r), s, r)
        r = r + self.rng.normal(0.0, 0.02, n)
        r[~np.isfinite(r) | (r > scan.range_max)] = np.inf     # no echo
        scan.ranges = r.astype(float).tolist()
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = FakeScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
