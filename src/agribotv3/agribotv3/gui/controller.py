"""ROS 2 controller — thin wrapper around Nav2 action clients.

Runs a *daemon* thread that spins a dedicated rclpy node so the nicegui
asyncio event-loop is never blocked.

Navigation is done **entirely** through Nav2:
  • NavigateToPose   — single goal
  • FollowWaypoints  — multi-point sequence

No custom planner, no obstacle-avoidance state machine, no PID.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data

    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Odometry, Path
    from sensor_msgs.msg import Image, Range
    from std_msgs.msg import String
    from nav2_msgs.action import NavigateToPose, FollowWaypoints
    ROS_AVAILABLE = True
except ImportError:  # standalone GUI mode without ROS 2 installed
    rclpy = None
    Node = object
    ActionClient = None
    qos_profile_sensor_data = None
    Twist = PoseStamped = Odometry = Path = Image = Range = String = None
    NavigateToPose = FollowWaypoints = None
    ROS_AVAILABLE = False

from .config import GRID_CONFIG, CELL_W, CELL_H


# ─── Shared state (read by the GUI thread, written by ROS callbacks) ─────
@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    wz: float = 0.0
    # ultrasonic distances  {sensor_key: metres}
    us: dict = field(default_factory=lambda: {'us1': float('inf'), 'us2': float('inf'),
                                               'us3': float('inf'), 'us4': float('inf')})
    # latest camera frame (raw bytes, encoding, w, h)
    cam_data: bytes | None = None
    cam_w: int = 0
    cam_h: int = 0
    cam_encoding: str = ''
    # nav state
    nav_active: bool = False
    nav_status: str = 'idle'
    # latest planned path [(x,y), …]
    path: list[tuple[float, float]] = field(default_factory=list)
    # current navigation goal
    goal_x: float = 0.0
    goal_y: float = 0.0
    has_goal: bool = False
    # vision detections (json string from vision_node)
    vision_detections: str = '[]'
    # ── active mission ───────────────────────────────────────────
    operation: str | None = None   # None | 'soil_scan' | 'planting'
    mission_waypoints: list = field(default_factory=list)
    mission_wp_idx: int = 0
    mission_cancel: bool = False


# ─── Helper: yaw from quaternion ─────────────────────────────────────────
def _yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# ─── Helper: coordinate conversions ─────────────────────────────────────
def world_to_grid(wx: float, wy: float) -> tuple[int, int]:
    col = int((wx - GRID_CONFIG['min_x']) / CELL_W)
    row = int((wy - GRID_CONFIG['min_y']) / CELL_H)
    return (max(0, min(col, GRID_CONFIG['cols'] - 1)),
            max(0, min(row, GRID_CONFIG['rows'] - 1)))


def grid_to_world(col: int, row: int) -> tuple[float, float]:
    wx = GRID_CONFIG['min_x'] + (col + 0.5) * CELL_W
    wy = GRID_CONFIG['min_y'] + (row + 0.5) * CELL_H
    return wx, wy


# ─── ROS node (lives inside daemon thread) ──────────────────────────────
class _ControllerNode(Node):
    def __init__(self, state: RobotState):
        if not ROS_AVAILABLE:
            raise RuntimeError('ROS 2 libraries are unavailable')
        super().__init__('agribot_gui_controller')
        self.st = state

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        for key in ('us1', 'us2', 'us3', 'us4'):
            self.create_subscription(
                Range, f'/ultra/{key}',
                lambda msg, k=key: self._us_cb(msg, k),
                qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self._cam_cb, qos_profile_sensor_data)
        self.create_subscription(Path, '/plan', self._path_cb, 10)
        self.create_subscription(String, '/vision/plant_detections', self._det_cb, 10)

        # Nav2 action clients
        self._nav_to_pose = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._follow_wp   = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self._goal_handle = None

        self.get_logger().info('GUI controller node ready.')

    # ── callbacks ────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.st.x = p.x
        self.st.y = p.y
        self.st.yaw = _yaw(msg.pose.pose.orientation)
        self.st.vx = msg.twist.twist.linear.x
        self.st.wz = msg.twist.twist.angular.z

    def _us_cb(self, msg: Range, key: str):
        self.st.us[key] = msg.range

    def _cam_cb(self, msg: Image):
        self.st.cam_data = bytes(msg.data)
        self.st.cam_w = msg.width
        self.st.cam_h = msg.height
        self.st.cam_encoding = msg.encoding

    def _path_cb(self, msg: Path):
        self.st.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _det_cb(self, msg: String):
        self.st.vision_detections = msg.data

    # ── manual drive ─────────────────────────────────────────
    def send_vel(self, vx: float, wz: float):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)

    def stop(self):
        self.send_vel(0.0, 0.0)

    # ── Nav2: go to single pose ──────────────────────────────
    def navigate_to(self, x: float, y: float, yaw: float = 0.0):
        if not self._nav_to_pose.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('NavigateToPose server not available')
            self.st.nav_status = 'nav2 offline'
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(x, y, yaw)

        self.st.nav_active = True
        self.st.nav_status = f'navigating → ({x:.1f}, {y:.1f})'
        self.st.goal_x = x
        self.st.goal_y = y
        self.st.has_goal = True

        future = self._nav_to_pose.send_goal_async(
            goal, feedback_callback=self._nav_feedback)
        future.add_done_callback(self._nav_goal_response)

    # ── Nav2: follow waypoints ───────────────────────────────
    def follow_waypoints(self, pts: list[tuple[float, float]]):
        if not self._follow_wp.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('FollowWaypoints server not available')
            self.st.nav_status = 'nav2 offline'
            return

        goal = FollowWaypoints.Goal()
        goal.poses = [self._make_pose(x, y) for x, y in pts]

        self.st.nav_active = True
        self.st.nav_status = f'following {len(pts)} waypoints'

        future = self._follow_wp.send_goal_async(
            goal, feedback_callback=self._wp_feedback)
        future.add_done_callback(self._wp_goal_response)

    # ── cancel ───────────────────────────────────────────────
    def cancel_nav(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self.stop()
        self.st.nav_active = False
        self.st.nav_status = 'cancelled'
        self.st.path = []
        self.st.has_goal = False

    # ── internal helpers ─────────────────────────────────────
    @staticmethod
    def _make_pose(x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.z = math.sin(yaw / 2)
        ps.pose.orientation.w = math.cos(yaw / 2)
        return ps

    def _nav_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        cp = fb.current_pose.pose.position
        self.st.nav_status = (
            f'navigating  dist_rem={fb.distance_remaining:.2f}m  '
            f'pos=({cp.x:.1f},{cp.y:.1f})')

    def _nav_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.st.nav_status = 'goal rejected'
            self.st.nav_active = False
            return
        self._goal_handle = gh
        result_future = gh.get_result_async()
        result_future.add_done_callback(self._nav_result)

    def _nav_result(self, future):
        self._goal_handle = None
        self.st.nav_active = False
        self.st.nav_status = 'idle'
        self.st.path = []
        self.st.has_goal = False

    def _wp_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.st.nav_status = f'waypoint {fb.current_waypoint} of route'

    def _wp_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.st.nav_status = 'waypoints rejected'
            self.st.nav_active = False
            return
        self._goal_handle = gh
        result_future = gh.get_result_async()
        result_future.add_done_callback(self._nav_result)


# ─── Public controller class (manages the thread) ───────────────────────
class RosController:
    """Start with ``ctrl = RosController(); ctrl.start()``"""

    def __init__(self):
        self.state = RobotState()
        self.state.nav_status = ('waiting for ROS topics' if ROS_AVAILABLE
                                 else 'standalone mode (ROS unavailable)')
        self._node: _ControllerNode | None = None
        self._thread: threading.Thread | None = None
        self._running = False

    # ── lifecycle ────────────────────────────────────────────
    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def shutdown(self):
        self._running = False
        if self._node:
            self._node.cancel_nav()
            self._node.stop()
        if self._thread:
            self._thread.join(timeout=3.0)

    def _spin(self):
        if not ROS_AVAILABLE:
            self.state.nav_status = 'standalone mode (ROS unavailable)'
            while self._running:
                time.sleep(0.2)
            return

        try:
            if not rclpy.ok():
                rclpy.init()
            self._node = _ControllerNode(self.state)
            while self._running and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.05)
        except Exception as exc:
            self.state.nav_status = f'ROS offline: {exc}'
        finally:
            if self._node:
                self._node.destroy_node()
                self._node = None

    # ── public API (thread-safe — called from GUI) ───────────
    def send_vel(self, vx: float, wz: float):
        if self._node:
            self._node.send_vel(vx, wz)
        else:
            self.state.vx = float(vx)
            self.state.wz = float(wz)

    def stop(self):
        if self._node:
            self._node.stop()
        self.state.vx = 0.0
        self.state.wz = 0.0

    def navigate_to(self, x: float, y: float, yaw: float = 0.0):
        if self._node:
            self._node.navigate_to(x, y, yaw)
        else:
            self.state.goal_x = float(x)
            self.state.goal_y = float(y)
            self.state.has_goal = True
            self.state.nav_status = 'waiting for Nav2 / topics'

    def follow_waypoints(self, pts: list[tuple[float, float]]):
        if self._node:
            self._node.follow_waypoints(pts)
        else:
            self.state.nav_status = 'waiting for Nav2 / topics'

    def cancel_nav(self):
        if self._node:
            self._node.cancel_nav()
        self.state.nav_active = False
        self.state.has_goal = False
        self.state.path = []

    # ── mission executors (called as asyncio tasks) ───────────────
    async def run_soil_scan(self, waypoints: list[tuple[float, float]],
                            dwell: float, log_fn):
        """Visit each waypoint, dwell, log. Respects mission_cancel flag."""
        import asyncio
        self.state.operation = 'soil_scan'
        self.state.mission_waypoints = list(waypoints)
        self.state.mission_wp_idx = 0
        self.state.mission_cancel = False
        log_fn(f'Soil scan started \u2014 {len(waypoints)} sample points')

        for i, (wx, wy) in enumerate(waypoints):
            if self.state.mission_cancel:
                break
            self.state.mission_wp_idx = i
            log_fn(f'Soil scan: moving to sample {i + 1}/{len(waypoints)} ({wx:.2f}, {wy:.2f})')
            self.navigate_to(wx, wy)
            await asyncio.sleep(3.0 + dwell)
            if self.state.mission_cancel:
                break
            log_fn(f'Soil scan: dwelt {dwell}s at sample {i + 1}')

        self.state.operation = None
        self.state.mission_waypoints = []
        if self.state.mission_cancel:
            log_fn('Soil scan cancelled')
        else:
            log_fn('Soil scan complete \u2713')
        self.state.nav_status = 'idle'

    async def run_planting(self, waypoints: list[tuple[float, float]], log_fn):
        """Visit each plant-cell centre. Respects mission_cancel flag."""
        import asyncio
        self.state.operation = 'planting'
        self.state.mission_waypoints = list(waypoints)
        self.state.mission_wp_idx = 0
        self.state.mission_cancel = False
        log_fn(f'Planting mission started \u2014 {len(waypoints)} positions')

        for i, (wx, wy) in enumerate(waypoints):
            if self.state.mission_cancel:
                break
            self.state.mission_wp_idx = i
            log_fn(f'Planting: moving to position {i + 1}/{len(waypoints)} ({wx:.2f}, {wy:.2f})')
            self.navigate_to(wx, wy)
            await asyncio.sleep(3.5)

        self.state.operation = None
        self.state.mission_waypoints = []
        if self.state.mission_cancel:
            log_fn('Planting cancelled')
        else:
            log_fn('Planting mission complete \u2713')
        self.state.nav_status = 'idle'

    def abort_mission(self):
        """Stop all active mission and cancel nav."""
        self.state.mission_cancel = True
        self.state.operation = None
        self.cancel_nav()
        if self._node:
            self._node.stop()

