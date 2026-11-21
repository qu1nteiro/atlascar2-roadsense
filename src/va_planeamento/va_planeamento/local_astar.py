"""Local planning node: A* on the local occupancy grid.

Subscribes
  grelha_local   nav_msgs/OccupancyGrid     grid in the car frame
  objetivo       geometry_msgs/PoseStamped  (optional) local goal
Publishes
  caminho_local  nav_msgs/Path              planned path
  estado         std_msgs/String            LIVRE | DESVIO <m> | BLOQUEADO
  zona_proibida  nav_msgs/OccupancyGrid     inflated obstacles (100)

Names are relative: with --ros-args -r __ns:=/g03 they become /g03/grelha_local, ...
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .astar import PlannerParams, plan
from .grid import LocalGrid


class LocalAStar(Node):

    def __init__(self):
        super().__init__('local_astar')
        d = PlannerParams()
        self.declare_parameter('rate', 5.0)
        self.declare_parameter('goal_x', 25.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('inflation_radius', d.inflation_radius)
        self.declare_parameter('heuristic_weight', d.heuristic_weight)
        self.declare_parameter('unknown_is_free', d.unknown_is_free)
        self.declare_parameter('corridor_y_min', d.corridor_y_min)
        self.declare_parameter('corridor_y_max', d.corridor_y_max)
        self.declare_parameter('swerve_threshold', d.swerve_threshold)

        self.grid = None
        self.goal = None
        self.create_subscription(OccupancyGrid, 'grelha_local', self.on_grid, 10)
        self.create_subscription(PoseStamped, 'objetivo', self.on_goal, 10)
        self.pub_path = self.create_publisher(Path, 'caminho_local', 10)
        self.pub_status = self.create_publisher(String, 'estado', 10)
        self.pub_forbidden = self.create_publisher(OccupancyGrid, 'zona_proibida', 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.cycle)

    def on_grid(self, msg):
        self.grid = msg

    def on_goal(self, msg):
        self.goal = msg

    def cycle(self):
        msg = self.grid
        if msg is None:
            self.get_logger().info('waiting for grelha_local ...', throttle_duration_sec=5.0)
            return
        gp = self.get_parameter
        p = PlannerParams(
            inflation_radius=gp('inflation_radius').value,
            heuristic_weight=gp('heuristic_weight').value,
            unknown_is_free=gp('unknown_is_free').value,
            corridor_y_min=gp('corridor_y_min').value,
            corridor_y_max=gp('corridor_y_max').value,
            swerve_threshold=gp('swerve_threshold').value,
        )
        # goal: the one from the `objetivo` topic (if any) or the one from the parameters
        if self.goal is not None:
            goal = (self.goal.pose.position.x, self.goal.pose.position.y)
        else:
            goal = (gp('goal_x').value, gp('goal_y').value)

        r = plan(LocalGrid.from_msg(msg), (0.0, 0.0), goal, p)

        # path
        path = Path(header=msg.header)
        for x, y in r.path_xy:
            pose = PoseStamped(header=msg.header)
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.pub_path.publish(path)

        # status
        text = f'DESVIO {r.max_deviation:.1f} m' if r.status == 'DESVIO' else r.status
        self.pub_status.publish(String(data=text))

        # forbidden zone (to view in Foxglove / rviz2)
        zone = OccupancyGrid(header=msg.header, info=msg.info)
        zone.data = np.where(r.forbidden, 100, 0).astype(np.int8).ravel().tolist()
        self.pub_forbidden.publish(zone)

        age = self.get_clock().now() - rclpy.time.Time.from_msg(msg.header.stamp)
        self.get_logger().info(
            f'{text:14s} expanded={r.expanded:5d}  A*={r.time_ms:5.1f} ms  '
            f'grid age={age.nanoseconds * 1e-6:5.0f} ms',
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = LocalAStar()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
