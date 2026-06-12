#!/usr/bin/env python3
from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Optional, Tuple, List

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(SCRIPT_DIR, "../config/robot.yaml")

with open(CONFIG_PATH, "r", encoding="utf-8") as cfg_file:
    CONFIG = yaml.safe_load(cfg_file)

MAX_V = float(CONFIG.get("max_v", 1.0))
MAX_W = float(CONFIG.get("max_w", 0.6))
VEL_TOPIC = str(CONFIG.get("vel_teleop_topic", "/cmd_vel"))
SCAN_TOPIC = str(CONFIG.get("base_scan_topic", "/front/scan"))
ODOM_TOPIC = str(CONFIG.get("odom_topic", "/gazebo/ground_truth/state"))
FRAME_RATE = float(CONFIG.get("frame_rate", 10.0))
DT = 1 / 10  # Discrete integration timestep.

WAYPOINT_TOPIC = "/waypoint"
REACHED_GOAL_TOPIC = "/topoplan/reached_goal"

# Input timeouts.
WAYPOINT_TIMEOUT = 1
LIDAR_TIMEOUT = 1
ODOM_TIMEOUT = 1

ACC_V = 10
ACC_W = 10

TRAJ_HORIZON = 20
SAMPLES_V = 20
SAMPLES_W = 20

CLEARANCE_HARD = 0.18

WEIGHT_GOAL = 10 
WEIGHT_TIME = 5 
WEIGHT_OBS = 12
WEIGHT_SMOOTH = 2 
WEIGHT_ORIENT = 1

MIN_LINEAR_CMD = 0
ROBOT_LENGTH = float(CONFIG.get("robot_length", 0.42))
ROBOT_WIDTH = float(CONFIG.get("robot_width", 0.30))
ROBOT_HALF_LENGTH = ROBOT_LENGTH * 0.5
ROBOT_HALF_WIDTH = ROBOT_WIDTH * 0.5
LIDAR_OFFSET_X = 0.12
LIDAR_OFFSET_Y = 0.0


@dataclass
class RobotState:
    x: float
    y: float
    yaw: float
    v: float
    w: float


@dataclass
class VelocityCommand:
    linear: float
    angular: float
    cost: float


class TimeStampedData:
    def __init__(self) -> None:
        self.stamp: float = float("-inf")
        self.data = None

    def update(self, data) -> None:
        self.data = data
        self.stamp = rospy.get_time()

    def valid(self, timeout: float) -> bool:
        return (rospy.get_time() - self.stamp) <= timeout


class LocalPlanner:
    def __init__(self) -> None:
        rospy.init_node("LOCAL_PLANNER", anonymous=False)

        self._waypoint = TimeStampedData()
        self._scan = TimeStampedData()
        self._odom = TimeStampedData()
        self._goal_reached = False
        self._goal_stop_sent = False
        
        self._last_cmd = Twist()
        self._recovery_active = False
        self._recovery_spin_dir = 1.0
        self._recovery_clear_count = 0
        self._recovery_clear_needed = 5

        rospy.Subscriber(WAYPOINT_TOPIC, Float32MultiArray, self._waypoint_cb, queue_size=1)
        rospy.Subscriber(SCAN_TOPIC, LaserScan, self._scan_cb, queue_size=1)
        rospy.Subscriber(ODOM_TOPIC, Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber(REACHED_GOAL_TOPIC, Bool, self._goal_cb, queue_size=1)

        self._cmd_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)

        self._path_pub = rospy.Publisher("~planned_path", Path, queue_size=1)

        self._samples_pub = rospy.Publisher("~sampled_paths", MarkerArray, queue_size=1)

        rate_hz = max(int(math.ceil(1.0 / DT)), 10)
        self._rate = rospy.Rate(rate_hz)

        self._front_thresh = float(rospy.get_param("~obs_front_threshold", 0.60))
        self._side_margin = float(rospy.get_param("~obs_side_margin", 0.10))
        rospy.loginfo("Local planner ready. Publishing to %s", VEL_TOPIC)

    # ----------------------------- callbacks -----------------------------

    def _waypoint_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            rospy.logwarn_throttle(2.0, "local_planner: waypoint has fewer than 2 entries, ignoring.")
            return
        self._waypoint.update(np.array(msg.data[:4], dtype=np.float32))

    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan.update(msg)

    def _odom_cb(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist
        yaw = self._quat_to_yaw(pose.orientation)
        self._odom.update(RobotState(
            x=pose.position.x,
            y=pose.position.y,
            yaw=yaw,
            v=twist.linear.x,
            w=twist.angular.z,
        ))

    def _goal_cb(self, msg: Bool) -> None:
        self._goal_reached = msg.data
        if self._goal_reached:
            self._goal_stop_sent = False

    # ----------------------------- utils -----------------------------

    @staticmethod
    def _quat_to_yaw(orientation) -> float:
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_angle(theta: float) -> float:
        return (theta + math.pi) % (2.0 * math.pi) - math.pi

    # ----------------------------- main loop -----------------------------

    def spin(self) -> None:
        while not rospy.is_shutdown():
            cmd = Twist()

            if self._goal_reached:
                self._publish_stop(cmd)
                if not self._goal_stop_sent:
                    rospy.loginfo("Goal reached, sending stop command before shutdown.")
                    rospy.sleep(0.1)
                    self._publish_stop(cmd)
                    rospy.sleep(0.1)
                    self._goal_stop_sent = True
                    rospy.signal_shutdown("Goal reached, local planner stopped.")
                return

            if not self._inputs_ready():
                self._publish_stop(cmd)
                rospy.logwarn_throttle(2.0, "inputs not ready (waypoint/scan/odom).")
                self._rate.sleep()
                continue

            waypoint = self._waypoint.data
            rel_goal = np.array(waypoint[:2], dtype=np.float32)

            desired_heading: Optional[float] = None
            if waypoint is not None and len(waypoint) >= 4:
                heading_vec = waypoint[2:4]
                if np.linalg.norm(heading_vec) > 1e-4:
                    desired_heading = math.atan2(heading_vec[1], heading_vec[0])

            best_cmd = self._plan(self._odom.data, rel_goal, self._scan.data, desired_heading)
            cmd.linear.x = best_cmd.linear
            cmd.angular.z = best_cmd.angular

            self._cmd_pub.publish(cmd)
            rospy.loginfo_throttle(0.2, "v=%.2f m/s, w=%.2f rad/s", cmd.linear.x, cmd.angular.z)
            self._last_cmd = cmd
            self._rate.sleep()

    def _inputs_ready(self) -> bool:
        return (
            self._waypoint.valid(WAYPOINT_TIMEOUT)
            and self._scan.valid(LIDAR_TIMEOUT)
            and self._odom.valid(ODOM_TIMEOUT)
        )

    def _publish_stop(self, cmd: Twist) -> None:
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self._cmd_pub.publish(cmd)
        self._last_cmd = cmd

    # ----------------------------- planner -----------------------------

    def _plan(self, odom_state: RobotState, rel_goal: np.ndarray, scan: LaserScan, desired_heading: Optional[float]) -> VelocityCommand:
        dyn_window = self._compute_dynamic_window(odom_state.v, odom_state.w)
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        ranges[~np.isfinite(ranges)] = np.inf
        angles = scan.angle_min + np.arange(ranges.size, dtype=np.float32) * scan.angle_increment
        front_min, preferred_turn = self._scan_turn_hint(ranges, angles, scan.range_min, scan.range_max)

        v_samples = np.linspace(dyn_window[0], dyn_window[1], SAMPLES_V)
        w_samples = np.linspace(dyn_window[2], dyn_window[3], SAMPLES_W)

        best = VelocityCommand(0.0, 0.0, float("inf"))
        best_traj = None
        feasible_trajs = [] 

        for v in v_samples:
            for w in w_samples:
                traj = self._simulate_band(v, w)
                feasible, min_clear = self._check_clearance(
                    traj, ranges, angles, scan.range_min, scan.range_max
                )
                if not feasible:
                    continue

                cost = self._evaluate_cost(
                    traj=traj,
                    rel_goal=rel_goal,
                    desired_heading=desired_heading,
                    clearance=min_clear,
                    v=v,
                    w=w,
                    prev_cmd=self._last_cmd,
                    front_min=front_min,
                    preferred_turn=preferred_turn,
                )

                feasible_trajs.append((traj, cost, v, w))

                if cost < best.cost:
                    best = VelocityCommand(v, w, cost)
                    best_traj = traj

        forward_candidates = [item for item in feasible_trajs if item[2] >= 0.08]

        if (
            math.isinf(best.cost)
            or best_traj is None
            or (front_min < (CLEARANCE_HARD + 0.10) and not forward_candidates)
        ):
            self._recovery_active = True
            if preferred_turn != 0.0:
                self._recovery_spin_dir = preferred_turn
            self._recovery_clear_count = 0
            self._publish_sampled_trajs([], odom_state)
            return self._fallback_spin(dyn_window, ranges, angles)

        if self._recovery_active:
            clear_enough = front_min > (self._front_thresh + 0.15)
            if clear_enough and forward_candidates:
                self._recovery_clear_count += 1
            else:
                self._recovery_clear_count = 0

            if self._recovery_clear_count < self._recovery_clear_needed:
                self._publish_sampled_trajs([], odom_state)
                return self._fallback_spin(dyn_window, ranges, angles)

            self._recovery_active = False
            self._recovery_clear_count = 0
        
        self._publish_sampled_trajs(feasible_trajs, odom_state)
        self._publish_path(best_traj, odom_state)

        return best

    # ----------------------------- helpers -----------------------------

    def _compute_dynamic_window(self, current_v: float, current_w: float) -> Tuple[float, float, float, float]:
        v_min = max(MIN_LINEAR_CMD, current_v - ACC_V * DT)
        v_max = min(MAX_V, current_v + ACC_V * DT)
        w_min = max(-MAX_W, current_w - ACC_W * DT)
        w_max = min(MAX_W, current_w + ACC_W * DT)
        return (v_min, v_max, w_min, w_max)

    def _scan_turn_hint(
        self,
        ranges: np.ndarray,
        angles: np.ndarray,
        rmin: float,
        rmax: float,
    ) -> Tuple[float, float]:
        valid = np.isfinite(ranges) & (ranges >= rmin) & (ranges <= rmax)
        if not np.any(valid):
            return float("inf"), 0.0

        points = self._scan_points(ranges, angles)
        front_mask = valid & (np.abs(angles) <= math.radians(20.0))
        if np.any(front_mask):
            front_clearances = self._footprint_signed_distance_points(points[front_mask])
            front_min = float(np.min(front_clearances)) if front_clearances.size > 0 else float("inf")
        else:
            front_min = float("inf")

        left_mask = valid & (angles >= math.radians(15.0)) & (angles <= math.radians(90.0))
        right_mask = valid & (angles <= -math.radians(15.0)) & (angles >= -math.radians(90.0))
        left_clear = float(np.mean(ranges[left_mask])) if np.any(left_mask) else 0.0
        right_clear = float(np.mean(ranges[right_mask])) if np.any(right_mask) else 0.0

        if left_clear > right_clear + self._side_margin:
            return front_min, 1.0
        if right_clear > left_clear + self._side_margin:
            return front_min, -1.0
        return front_min, 0.0

    def _simulate_band(self, v: float, w: float) -> np.ndarray:
        poses = np.zeros((TRAJ_HORIZON, 3), dtype=np.float32)
        x = 0.0
        y = 0.0
        yaw = 0.0
        for i in range(TRAJ_HORIZON):
            x += v * math.cos(yaw) * DT
            y += v * math.sin(yaw) * DT
            yaw += w * DT
            yaw = self._wrap_angle(yaw)
            poses[i, 0] = x
            poses[i, 1] = y
            poses[i, 2] = yaw
        return poses

    @staticmethod
    def _scan_points(ranges: np.ndarray, angles: np.ndarray) -> np.ndarray:
        xs = ranges * np.cos(angles) + LIDAR_OFFSET_X
        ys = ranges * np.sin(angles) + LIDAR_OFFSET_Y
        return np.stack([xs, ys], axis=1)

    @staticmethod
    def _footprint_signed_distance_xy(local_x: np.ndarray, local_y: np.ndarray) -> np.ndarray:
        dx = np.abs(local_x) - ROBOT_HALF_LENGTH
        dy = np.abs(local_y) - ROBOT_HALF_WIDTH
        outside_x = np.maximum(dx, 0.0)
        outside_y = np.maximum(dy, 0.0)
        outside_dist = np.hypot(outside_x, outside_y)
        inside_dist = np.minimum(np.maximum(dx, dy), 0.0)
        return outside_dist + inside_dist

    def _footprint_signed_distance_points(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return np.empty((0,), dtype=np.float32)
        return self._footprint_signed_distance_xy(points[:, 0], points[:, 1]).astype(np.float32)

    def _check_clearance(self, traj: np.ndarray, ranges: np.ndarray, angles: np.ndarray, rmin: float, rmax: float) -> Tuple[bool, float]:
        pts = self._scan_points(ranges, angles)

        min_clearance = float("inf")
        feasible = True

        valid = np.isfinite(ranges) & (ranges >= rmin) & (ranges <= rmax)
        if not np.any(valid):
            return True, float("inf")

        for i in range(traj.shape[0]):
            px, py, yaw = traj[i]
            rel = pts[valid] - np.array([px, py], dtype=np.float32)
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            local_x = rel[:, 0] * cos_yaw + rel[:, 1] * sin_yaw
            local_y = -rel[:, 0] * sin_yaw + rel[:, 1] * cos_yaw

            signed_dist = self._footprint_signed_distance_xy(local_x, local_y)
            local_min = float(np.min(signed_dist))
            min_clearance = min(min_clearance, local_min)
            if local_min < CLEARANCE_HARD:
                feasible = False
                break

        if not np.isfinite(min_clearance):
            min_clearance = float("inf")

        return feasible, min_clearance

    def _evaluate_cost(
        self,
        traj: np.ndarray,
        rel_goal: np.ndarray,
        desired_heading: Optional[float],
        clearance: float,
        v: float,
        w: float,
        prev_cmd: Twist,
        front_min: float,
        preferred_turn: float,
    ) -> float:
        dx = rel_goal[0] - traj[-1, 0]
        dy = rel_goal[1] - traj[-1, 1]
        goal_cost = WEIGHT_GOAL * math.hypot(dx, dy)

        time_cost = WEIGHT_TIME * (1.0 - min(max(v, 0.0) / MAX_V, 1.0)) * 0.5

        if clearance == float("inf"):
            obs_cost = 0.0
        else:
            obs_cost = WEIGHT_OBS * math.exp(-3.0 * clearance / CLEARANCE_HARD)

        dv = v - prev_cmd.linear.x
        dw = w - prev_cmd.angular.z
        smooth_cost = WEIGHT_SMOOTH * 0.3 * (abs(dv) + 0.5 * abs(dw))

        orient_cost = 0.0
        if desired_heading is not None:
            yaw_end = traj[-1, 2]
            d_yaw = self._wrap_angle(desired_heading - yaw_end)
            orient_cost = WEIGHT_ORIENT * abs(d_yaw)

        turn_cost = 0.0
        if np.isfinite(front_min) and front_min < self._front_thresh:
            if preferred_turn != 0.0 and preferred_turn * w < 0.0:
                turn_cost += 8.0
            if abs(w) < 0.08:
                turn_cost += 6.0
            if front_min < (CLEARANCE_HARD + 0.08):
                turn_cost += 10.0 * max(v - 0.12, 0.0)

        return goal_cost + time_cost + obs_cost + smooth_cost + orient_cost + turn_cost

    def _fallback_spin(self, dyn_window: Tuple[float, float, float, float], ranges: np.ndarray, angles: np.ndarray) -> VelocityCommand:
        finite = np.isfinite(ranges)
        points = self._scan_points(ranges, angles)
        clearances = self._footprint_signed_distance_points(points)
        free = np.where(finite & (clearances > (CLEARANCE_HARD + 0.05)))[0]
        direction = self._recovery_spin_dir
        if free.size > 0 and direction == 0.0:
            left = np.sum((angles[free] > 0.0) & (angles[free] <= math.radians(90.0)))
            right = np.sum((angles[free] < 0.0) & (angles[free] >= -math.radians(90.0)))
            direction = 1.0 if left >= right else -1.0

        v = 0.0
        w = direction * min(max(abs(dyn_window[3]), abs(dyn_window[2])), 0.4)
        w = max(min(w, dyn_window[3]), dyn_window[2])
        if abs(w) < 0.08:
            w = 0.08 * direction

        return VelocityCommand(v, w, 0.0)

    # ----------------------------- path publish -----------------------------

    def _publish_path(self, band: np.ndarray, odom_state: RobotState) -> None:
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"

        base_x = odom_state.x
        base_y = odom_state.y
        base_yaw = odom_state.yaw
        cos_y = math.cos(base_yaw)
        sin_y = math.sin(base_yaw)

        for i in range(band.shape[0]):
            bx, by, byaw = band[i]

            gx = base_x + bx * cos_y - by * sin_y
            gy = base_y + bx * sin_y + by * cos_y
            gyaw = base_yaw + byaw

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = gx
            pose.pose.position.y = gy
            pose.pose.position.z = 0.0

            qz = math.sin(gyaw * 0.5)
            qw = math.cos(gyaw * 0.5)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path_msg.poses.append(pose)

        self._path_pub.publish(path_msg)

    def _publish_sampled_trajs(self, traj_cost_list, _odom_state):
        ma = MarkerArray()
        now = rospy.Time.now()
        frame_id = "base_link"

        if not traj_cost_list:
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = frame_id
            m.action = Marker.DELETEALL
            ma.markers.append(m)
            self._samples_pub.publish(ma)
            return

        traj_cost_list = sorted(traj_cost_list, key=lambda x: x[1])

        MAX_DRAW = 8
        for idx, item in enumerate(traj_cost_list[:MAX_DRAW]):
            band, _cost = item[0], item[1]
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = frame_id
            m.ns = "sampled_trajs"
            m.id = idx
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.01
            m.color.r = 0.2
            m.color.g = 0.6
            m.color.b = 1.0
            m.color.a = 0.5

            for i in range(band.shape[0]):
                bx, by, _ = band[i]
                pt = Point()
                pt.x = float(bx)
                pt.y = float(by)
                pt.z = 0.0
                m.points.append(pt)

            ma.markers.append(m)

        self._samples_pub.publish(ma)



def main() -> None:
    planner = LocalPlanner()
    planner.spin()


if __name__ == "__main__":
    main()
