"""
Author: Rashid Alyassi
Date: 2025

This ROS2 node implements a learning-based social navigation planner that
integrates an RL policy for velocity control. The node subscribes to
`/scan`, `/odom`, `/plan`, and `/goal_pose` topics and publishes motion
commands to `/cmd_vel`.

Core functionalities include:
- Downsampling LIDAR data
- Simplifying and following a global path via waypoints
- Computing relative goal distance and heading
- Integrating a trained RL policy (loaded via rlgames_eval_policy)
  to generate robot control commands

Configuration parameters (e.g., model checkpoint, preferences, limits)
are loaded from a YAML file under `config/planner_config.yaml`.
TODO:
1. robot pose in not center of the robot
2. pass action to smooth velocity controller
3. run global planner
"""
import os

import torch
import math
import numpy as np
import yaml
import pathlib

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan

from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


from rl_planner.rlgames_runner.rlgames_eval_policy import RLModel 


class PlannerNode(Node):
    def __init__(self):
        super().__init__('rl_planner')

        current_dir = "/home/segway/ros2_ws/src/rl_planner/rl_planner"
        cfg_path = os.path.join(current_dir, "config", "planner_config.yaml")
        with open(cfg_path, 'r') as f:
            self.config = yaml.safe_load(f)

        # Parameters
        self.lidar_target_size = 40
        self.max_lidar_range = 12  # based on socialnav simulation
        self.device = self.config.get('device', 'cpu')
        self.goal_threshold = self.config.get('goal_threshold', 0.2)  # meters
        self.waypoint_threshold = self.config.get('waypoint_threshold', 0.3)  # meters
        # speed, safety, social-dist, smoothness weights
        self.pref_vec = self.config.get('preference_vector', [1.0, 0.0, 0.0, 0.0])
        self.pref_vec = torch.tensor(self.pref_vec, dtype=torch.float).to(self.device)  # preferred velocity vector
        self.action_deterministic = self.config.get('action_deterministic', True)
        self.max_velocity = self.config.get('max_velocity', 0.5)  # m/s
        self.max_angular_velocity = self.config.get('max_angular_velocity', 1.0)  # rad/s
        self.checkpoint_path = self.config.get('checkpoint_path', os.path.join(current_dir,'rlgames_runner/checkpoint/last_SocialNavHumanPoseMORL_ep_450_rew_446.22223.pth'))
        self.use_dummy_pid = self.config.get('use_dummy_pid', False)


        self.model = None  # Placeholder for RL policy
        self.is_planning = False
        self.waypoints = []
        self.current_waypoint = None
        self.lidar = None
        self.pose = None
        self.goal_pose = None

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.points_pub = self.create_publisher(Marker, '/simplified_plan', 10)

        # RL Model
        self.load_model()
        planner_hz = 10
        self.timer = self.create_timer(1/planner_hz, self.control_loop)
        # always publish zero when not planning
        self.zero_timer = self.create_timer(0.1, self.publish_zero_velocity)
        
        # Actions
        self.global_plan_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        # --- Optional velocity smoothing ---
        self.use_smoothing = self.config.get("use_smoothing", True)
        self.smoothing_alpha = self.config.get("smoothing_alpha", 0.3)  # low-pass filter
        self.max_accel = self.config.get("max_accel", 0.4)  # m/s²
        self.max_angular_accel = self.config.get("max_angular_accel", 1.0)  # rad/s²
        self.max_jerk = self.config.get("max_jerk", 1.5)  # m/s³ (optional)
        self.max_angular_jerk = self.config.get("max_angular_jerk", 3.0)  # rad/s³

        self.simplify_path_rdp_epsilon = self.config.get("simplify_path_rdp_epsilon", 0.05)


        # Internal state for smoothing
        self.last_v = 0.0
        self.last_omega = 0.0
        self.last_accel = 0.0
        self.last_angular_accel = 0.0
        self.dt = 0.1  # matches your 10 Hz control loop

        self.get_logger().info("PlannerNode initialized and ready.")

    # ------------------------------------------------------------------
    # Topic callbacks
    # ------------------------------------------------------------------

    def scan_callback(self, msg: LaserScan):
        """Process lidar scan."""
        ranges = np.array(msg.ranges, dtype=np.float32)
        self.lidar = self.downsample_lidar(ranges, self.lidar_target_size)
        self.lidar = np.clip(self.lidar, 0.0, self.max_lidar_range)  # clip to max range

        self.get_logger().debug(f"Received scan ({len(ranges)}→{len(self.lidar)})")

    def odom_callback(self, msg: Odometry):
        """Read odometry into (x, y, z, yaw)."""
        self.pose = self.odom_to_pose(msg)  # TODO: pose here is base_link not robot center
        self.get_logger().debug(f"Odom: x={self.pose['x']:.2f}, y={self.pose['y']:.2f}, yaw={math.degrees(self.pose['theta']):.1f}°, v={self.pose['v']:.2f}, omega={math.degrees(self.pose['omega']):.1f}")

    def path_callback(self, msg: Path):
        """Receive and simplify global path."""
        simplified = self.simplify_path(msg)
        self.waypoints = self.path_to_xyz_list(simplified)
        self.is_planning = True
        self.get_logger().info(f"Path simplified to {len(self.waypoints)} waypoints.")

    def goal_callback(self, msg: PoseStamped):
        """Set goal pose."""
        pos = msg.pose.position
        self.goal_pose = np.array([pos.x, pos.y, pos.z])
        self.is_planning = True
        self.get_logger().info(f"Goal set: ({pos.x:.2f}, {pos.y:.2f})")
        self.call_nav2_planner(msg)

    def load_model(self):
        self.model = RLModel(checkpoint_path=self.checkpoint_path)
        self.model.to(self.device)


    def rl_plan(self, observation: np.ndarray):
        # TODO: Use RL model to compute action
        # (1) pass observation to input_normalizer
        # (2) get action from model
        # (3) denormalize action to max velocity
        obs_torch = torch.tensor(observation, dtype=torch.float32).unsqueeze(0).to(self.device)

        action = self.model.get_action({'obs': obs_torch, 'pref_vec': self.pref_vec}, deterministic=self.action_deterministic)

        action = action.cpu().numpy().squeeze(0)
        action = action.clip(-1, 1)
        action[0] = action[0] * self.max_velocity  # denormalize linear velocity
        action[1] = action[1] * self.max_angular_velocity  # denormalize angular velocity
        return action

    def pid_plan(self, observation: np.ndarray):
        """
        Simple PID controller:
          - Linear velocity tries to reduce goal distance
          - Angular velocity tries to reduce heading difference
        """
        goal_dist = observation[2]
        heading_diff = observation[3]
        # --- PID gains (tune as needed) ---
        Kp_v = 0.8
        Ki_v = 0.0
        Kd_v = 0.1
        Kp_w = 2.0
        Ki_w = 0.0
        Kd_w = 0.2
        # --- Integral error state ---
        if not hasattr(self, "pid_int_v"):
            self.pid_int_v = 0.0
            self.pid_int_w = 0.0
            self.pid_prev_dist = goal_dist
            self.pid_prev_heading = heading_diff
        # --- Linear velocity PID ---
        error_v = goal_dist
        self.pid_int_v += error_v * self.dt
        deriv_v = (error_v - self.pid_prev_dist) / self.dt
        v = Kp_v * error_v + Ki_v * self.pid_int_v + Kd_v * deriv_v
        self.pid_prev_dist = error_v
        # --- Angular velocity PID ---
        error_w = heading_diff
        self.pid_int_w += error_w * self.dt
        deriv_w = (error_w - self.pid_prev_heading) / self.dt
        omega = Kp_w * error_w + Ki_w * self.pid_int_w + Kd_w * deriv_w
        self.pid_prev_heading = error_w
        # --- Clip to physical limits ---
        v = np.clip(v, -self.max_velocity, self.max_velocity)
        omega = np.clip(omega, -self.max_angular_velocity, self.max_angular_velocity)

        return np.array([v, omega], dtype=np.float32)

    # ------------------------------------------------------------------
    # Core control loop
    # ------------------------------------------------------------------

    def control_loop(self):
        """Main loop: compute velocity command and publish."""
        if self.goal_pose is None:
            return  # not ready yet
        v, omega = self.planner_step()
        self.send_cmd_vel(v, omega)

    def planner_step(self):
        """
        Compute (v, omega) based on relative goal and current waypoint.
        Updates waypoint as robot progresses.
        """
        if not self.is_planning:
            return 0.0, 0.0

        if self.pose is None or self.lidar is None or len(self.waypoints) == 0:
            return 0.0, 0.0

        # --- (1) Select current waypoint ---
        if self.current_waypoint is None:
            self.current_waypoint = np.array(self.waypoints[0])

        # --- (2) Compute distance to current waypoint ---
        dx = self.current_waypoint[0] - self.pose['x']
        dy = self.current_waypoint[1] - self.pose['y']
        dist = math.hypot(dx, dy)

        # --- (3) If close, move to next waypoint ---
        if dist < self.waypoint_threshold and len(self.waypoints) > 1:
            self.waypoints.pop(0)
            self.current_waypoint = np.array(self.waypoints[0])
            self.get_logger().info("Advancing to next waypoint.")

        if dist < self.goal_threshold:
            self.get_logger().info("Final goal reached.")
            self.is_planning = False
            self.goal_pose = None
            return 0.0, 0.0

        # --- (4) Compute heading to goal ---
        goal_angle = math.atan2(dy, dx)
        yaw = self.pose['theta']
        heading_diff = goal_angle - yaw

        # Normalize to [-π, π]
        heading_diff = (heading_diff + math.pi) % (2 * math.pi) - math.pi

        # --- (5) Placeholder velocity control (to be replaced by RL or PID) ---
        current_v = self.pose['v']
        current_omega = self.pose['omega']
        robot_state = np.array([current_v, current_omega, dist, heading_diff])
        observation = np.concatenate((robot_state, self.lidar))

        if self.use_dummy_pid:
            v, omega = self.pid_plan(observation)
        else:
            v, omega = self.rl_plan(observation)
        # PID like
        v, omega = self.smooth_velocity(v, omega)

        return v, omega

    def smooth_velocity(self, v_cmd, omega_cmd):
        """
        Apply optional smoothing to linear and angular velocity commands.
        Includes:
          - Low-pass filtering
          - Acceleration limiting
          - Jerk limiting
        """
        if not self.use_smoothing:
            return v_cmd, omega_cmd

        # 1) Low-pass filtering (EWMA)
        v = self.smoothing_alpha * v_cmd + (1 - self.smoothing_alpha) * self.last_v
        omega = self.smoothing_alpha * omega_cmd + (1 - self.smoothing_alpha) * self.last_omega

        # 2) Acceleration limiting
        dv = v - self.last_v
        domega = omega - self.last_omega

        # limit linear acceleration
        max_dv = self.max_accel * self.dt
        dv = np.clip(dv, -max_dv, max_dv)

        # limit angular acceleration
        max_domega = self.max_angular_accel * self.dt
        domega = np.clip(domega, -max_domega, max_domega)

        v = self.last_v + dv
        omega = self.last_omega + domega

        # 3) Jerk limiting (optional)
        accel = dv / self.dt
        angular_accel = domega / self.dt

        da = accel - self.last_accel
        da_omega = angular_accel - self.last_angular_accel

        da = np.clip(da, -self.max_jerk * self.dt, self.max_jerk * self.dt)
        da_omega = np.clip(da_omega, -self.max_angular_jerk * self.dt, self.max_angular_jerk * self.dt)

        accel = self.last_accel + da
        angular_accel = self.last_angular_accel + da_omega

        v = self.last_v + accel * self.dt
        omega = self.last_omega + angular_accel * self.dt

        # Update history
        self.last_accel = accel
        self.last_angular_accel = angular_accel
        self.last_v = v
        self.last_omega = omega

        return v, omega
    # ------------------------------------------------------------------
    # Nav2 Global planner
    # ------------------------------------------------------------------
    def call_nav2_planner(self, goal_pose: PoseStamped):
        if not self.global_plan_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 planner server not available!")
            return

        start = PoseStamped()
        start.header.frame_id = 'map'
        start.pose.position.x = self.pose['x']
        start.pose.position.y = self.pose['y']
        start.pose.orientation.w = 1.0

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose = goal_pose.pose

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal

        future = self.global_plan_client.send_goal_async(
            goal_msg,
            feedback_callback=None
        )
        future.add_done_callback(self.handle_plan_response)


    def handle_plan_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Global plan rejected!")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.handle_plan_result)

    def handle_plan_result(self, future):
        result = future.result().result
        path = result.path

        self.get_logger().info(f"Global path received: {len(path.poses)} poses")

        # Use your existing path callback logic:
        self.path_callback(path)



    # ------------------------------------------------------------------
    # Core utilities
    # ------------------------------------------------------------------
    def publish_zero_velocity(self):
        if not self.is_planning:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)

    def send_cmd_vel(self, v: float, omega: float):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = omega
        self.cmd_pub.publish(msg)

    @staticmethod
    def downsample_lidar(ranges: np.ndarray, target_size: int):
        """Reduce lidar ranges to target_size by taking min per segment."""
        n = len(ranges)
        if n == 0:
            return np.zeros(target_size)
        ranges = np.nan_to_num(ranges, nan=np.inf)
        group_size = n / target_size
        idx = (np.arange(target_size + 1) * group_size).astype(int)
        idx[-1] = n
        return np.array([np.min(ranges[idx[i]:idx[i + 1]]) for i in range(target_size)], dtype=np.float32)

    @staticmethod
    def odom_to_pose(odom_msg: Odometry):
        """
        Convert a nav_msgs/Odometry message to (x, y, z, theta, v, omega).
        Args:
            odom_msg (Odometry): Odometry message.
        Returns:
            tuple: (x, y, z, theta, v, omega)
                - (x, y, z): position
                - theta: yaw (radians)
                - v: linear velocity (m/s)
                - omega: angular velocity (rad/s)
        """
        # --- Position ---
        pos = odom_msg.pose.pose.position
        x, y, z = pos.x, pos.y, pos.z

        # --- Orientation to yaw ---
        ori = odom_msg.pose.pose.orientation
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        # --- Velocities ---
        twist = odom_msg.twist.twist
        v = twist.linear.x
        omega = twist.angular.z

        return {"x": x, "y": y, "z": z, "theta": theta, "v":v, "omega": omega}

    @staticmethod
    def path_to_xyz_list(path_msg: Path):
        return [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in path_msg.poses]



    def publish_points(self, pts, header):
        marker = Marker()
        marker.header = header
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
    
        # point size
        marker.scale.x = 0.1
        marker.scale.y = 0.1
    
        # color
        marker.color.r = 1.0
        marker.color.g = 0.3
        marker.color.b = 0.0
        marker.color.a = 1.0
    
        # convert points to geometry_msgs/Point
        for (x, y) in pts:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)
    
        self.points_pub.publish(marker)

    
    def simplify_path(self, path_msg: Path):
        pts = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
        #simplified_pts = rdp(pts, self.simplify_path_rdp_epsilon)
        simplified_pts = segmented_rdp(pts, epsilon=self.simplify_path_rdp_epsilon, segment_size=100)


        out = Path()
        out.header = path_msg.header

        for x, y in simplified_pts:
            ps = PoseStamped()
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0  # no orientation needed
            out.poses.append(ps)
        
        # ---- PUBLISH FOR RVIZ ----
        self.publish_points(simplified_pts, path_msg.header)
        return out


#####################################

def segmented_rdp(points, epsilon, segment_size=200):
    simplified = []
    n = len(points)

    for start in range(0, n, segment_size):
        end = min(start + segment_size, n)
        segment = points[start:end]

        # RDP on segment
        reduced = rdp(segment, epsilon)

        # avoid duplicating joints
        if simplified and reduced:
            reduced = reduced[1:]

        simplified.extend(reduced)

    return simplified

def rdp(points, epsilon):
    """Ramer-Douglas-Peucker simplification."""
    if len(points) < 3:
        return points

    # Find the point with biggest deviation from start-end line
    sx, sy = points[0]
    ex, ey = points[-1]

    max_dist = 0
    index = 0

    dx = ex - sx
    dy = ey - sy
    length2 = dx*dx + dy*dy

    for i in range(1, len(points) - 1):
        px, py = points[i]

        if length2 == 0:
            dist = math.hypot(px - sx, py - sy)
        else:
            t = ((px - sx)*dx + (py - sy)) / length2
            proj_x = sx + t * dx
            proj_y = sy + t * dy
            dist = math.hypot(px - proj_x, py - proj_y)

        if dist > max_dist:
            max_dist = dist
            index = i

    if max_dist > epsilon:
        left = rdp(points[:index+1], epsilon)
        right = rdp(points[index:], epsilon)
        return left[:-1] + right
    else:
        return [points[0], points[-1]]







def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
