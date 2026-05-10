#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
intersection_supervisor.py

通用动态障碍物通行决策节点（机器人相对运动增强版）

核心改进：
1. 从 world 文件读取 moving_obstacle_plugin 的运动路径；
2. 从 /dynamic_scan 中提取动态障碍物质心；
3. 同时估计机器人速度、动态障碍物二维速度；
4. 不只判断“障碍物是否远离路径线”，还判断：
   - 障碍物和机器人之间的距离是否越来越近；
   - 相对 TTC 是否预测会碰撞；
   - 障碍物是否接近运动端点，可能马上掉头；
5. 只有满足“障碍物已经越过机器人必经线，并且也在远离机器人”时才真正放行；
6. 区分“运动中进入 commit zone”和“启动时就在 commit zone”：
   - 运动中进入：继续通过，避免停在障碍物路径上；
   - 启动就在 commit zone：先做相对 TTC 安全判断，危险则等待。

输出：
  /intersection_state
  /intersection_conflict
  /intersection_decision
  /intersection_obstacle_motion
  /dynamic_obstacle_state
  /dynamic_obstacle_decision

典型决策：
  NORMAL_NAVIGATION
  SLOW_DOWN
  YIELD_WAIT
  PASS_INTERSECTION
  RECOVERY
"""

import math
import os
import xml.etree.ElementTree as ET

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

import tf2_ros


class DynamicObstacleSupervisor(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_supervisor')

        # ============================================================
        # 基础参数
        # ============================================================
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('dynamic_scan_topic', '/dynamic_scan')

        self.declare_parameter('world_file_path', '')
        self.declare_parameter('moving_model_name', '')

        # world 读取失败时的默认动态障碍物路径
        self.declare_parameter('default_motion_axis', 'y')
        self.declare_parameter('default_fixed_x', 2.2)
        self.declare_parameter('default_fixed_y', 0.0)
        self.declare_parameter('default_start_x', 0.0)
        self.declare_parameter('default_end_x', 2.0)
        self.declare_parameter('default_start_y', -1.0)
        self.declare_parameter('default_end_y', 1.2)
        self.declare_parameter('default_obstacle_speed', 0.08)

        # ============================================================
        # 区域参数
        # ============================================================
        # dynamic_scan 中只统计动态障碍物运动轨迹附近的点，避免其他动态点干扰。
        self.declare_parameter('path_detect_half_width', 0.75)
        self.declare_parameter('path_detect_margin', 0.35)

        # 机器人距离动态障碍物运动路径的区域判断。
        self.declare_parameter('approach_path_distance', 1.2)
        self.declare_parameter('near_path_distance', 0.75)

        # 通过承诺区：小车真正进入动态障碍物运动路径附近后，不突然停车。
        self.declare_parameter('commit_path_half_width', 0.45)
        self.declare_parameter('exit_path_distance', 1.0)

        # 机器人与动态障碍物本体的距离阈值。
        self.declare_parameter('slow_distance', 1.2)
        self.declare_parameter('stop_distance', 0.55)

        # 判断动态障碍物是否挡住机器人预计穿越点。
        self.declare_parameter('block_path_half_width', 0.35)
        self.declare_parameter('release_path_distance', 0.65)

        # 传统路径 TTC 阈值：障碍物多久到达机器人预计穿越线。
        self.declare_parameter('ttc_stop_time', 1.8)
        self.declare_parameter('ttc_slow_time', 3.0)

        # 动态点数量阈值。
        self.declare_parameter('conflict_points_threshold', 3)

        # 障碍物沿运动路径的速度估计阈值。
        self.declare_parameter('obstacle_velocity_threshold', 0.03)

        # ============================================================
        # 机器人感知 TTC / 相对运动参数
        # ============================================================
        self.declare_parameter('enable_robot_aware_ttc', True)

        # 距离变化率：负数代表越来越近；正数代表越来越远。
        self.declare_parameter('distance_closing_threshold', 0.03)
        self.declare_parameter('distance_opening_threshold', 0.02)

        # 障碍物越过机器人预计穿越线多远，且远离机器人，才认为已经通过。
        self.declare_parameter('passed_path_release_distance', 0.45)

        # 相对 TTC 参数。
        self.declare_parameter('relative_ttc_stop_time', 1.8)
        self.declare_parameter('relative_ttc_slow_time', 3.0)
        self.declare_parameter('ttc_prediction_horizon', 4.0)
        self.declare_parameter('collision_radius', 0.48)
        self.declare_parameter('slow_collision_radius', 0.75)
        self.declare_parameter('min_relative_speed', 0.03)

        # 机器人速度估计阈值。
        self.declare_parameter('robot_velocity_threshold', 0.02)

        # 启动时如果已经在 commit zone，是否先做安全检查。
        self.declare_parameter('check_startup_in_commit_zone', True)

        # ============================================================
        # 掉头风险参数
        # ============================================================
        # 动态障碍物靠近运动端点时，可能马上掉头。
        self.declare_parameter('enable_turnaround_risk', True)
        self.declare_parameter('turnaround_margin', 0.25)
        self.declare_parameter('turnaround_path_risk_distance', 0.85)
        self.declare_parameter('turnaround_robot_distance', 1.1)

        # ============================================================
        # 状态确认时间
        # ============================================================
        self.declare_parameter('clear_confirm_time', 0.4)
        self.declare_parameter('release_confirm_time', 0.5)
        self.declare_parameter('max_yield_wait_time', 8.0)
        self.declare_parameter('timer_period', 0.1)

        # ============================================================
        # 读取参数
        # ============================================================
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.dynamic_scan_topic = self.get_parameter('dynamic_scan_topic').value
        self.world_file_path = self.get_parameter('world_file_path').value
        self.moving_model_name = self.get_parameter('moving_model_name').value

        self.path_detect_half_width = float(self.get_parameter('path_detect_half_width').value)
        self.path_detect_margin = float(self.get_parameter('path_detect_margin').value)

        self.approach_path_distance = float(self.get_parameter('approach_path_distance').value)
        self.near_path_distance = float(self.get_parameter('near_path_distance').value)
        self.commit_path_half_width = float(self.get_parameter('commit_path_half_width').value)
        self.exit_path_distance = float(self.get_parameter('exit_path_distance').value)

        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)

        self.block_path_half_width = float(self.get_parameter('block_path_half_width').value)
        self.release_path_distance = float(self.get_parameter('release_path_distance').value)

        self.ttc_stop_time = float(self.get_parameter('ttc_stop_time').value)
        self.ttc_slow_time = float(self.get_parameter('ttc_slow_time').value)

        self.conflict_points_threshold = int(self.get_parameter('conflict_points_threshold').value)
        self.obstacle_velocity_threshold = float(self.get_parameter('obstacle_velocity_threshold').value)

        self.enable_robot_aware_ttc = bool(self.get_parameter('enable_robot_aware_ttc').value)
        self.distance_closing_threshold = float(self.get_parameter('distance_closing_threshold').value)
        self.distance_opening_threshold = float(self.get_parameter('distance_opening_threshold').value)
        self.passed_path_release_distance = float(self.get_parameter('passed_path_release_distance').value)

        self.relative_ttc_stop_time = float(self.get_parameter('relative_ttc_stop_time').value)
        self.relative_ttc_slow_time = float(self.get_parameter('relative_ttc_slow_time').value)
        self.ttc_prediction_horizon = float(self.get_parameter('ttc_prediction_horizon').value)
        self.collision_radius = float(self.get_parameter('collision_radius').value)
        self.slow_collision_radius = float(self.get_parameter('slow_collision_radius').value)
        self.min_relative_speed = float(self.get_parameter('min_relative_speed').value)

        self.robot_velocity_threshold = float(self.get_parameter('robot_velocity_threshold').value)
        self.check_startup_in_commit_zone = bool(self.get_parameter('check_startup_in_commit_zone').value)

        self.enable_turnaround_risk = bool(self.get_parameter('enable_turnaround_risk').value)
        self.turnaround_margin = float(self.get_parameter('turnaround_margin').value)
        self.turnaround_path_risk_distance = float(self.get_parameter('turnaround_path_risk_distance').value)
        self.turnaround_robot_distance = float(self.get_parameter('turnaround_robot_distance').value)

        self.clear_confirm_time = float(self.get_parameter('clear_confirm_time').value)
        self.release_confirm_time = float(self.get_parameter('release_confirm_time').value)
        self.max_yield_wait_time = float(self.get_parameter('max_yield_wait_time').value)
        self.timer_period = float(self.get_parameter('timer_period').value)

        # 从 world 文件读取动态障碍物路径。
        self.load_moving_obstacle_from_world()
        self.recompute_path_bounds()

        # ============================================================
        # TF
        # ============================================================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ============================================================
        # 发布器：保留旧话题名，兼容你的 speed_gate
        # ============================================================
        self.state_pub = self.create_publisher(String, '/intersection_state', 10)
        self.conflict_pub = self.create_publisher(Bool, '/intersection_conflict', 10)
        self.decision_pub = self.create_publisher(String, '/intersection_decision', 10)
        self.motion_pub = self.create_publisher(String, '/intersection_obstacle_motion', 10)

        self.dynamic_state_pub = self.create_publisher(String, '/dynamic_obstacle_state', 10)
        self.dynamic_decision_pub = self.create_publisher(String, '/dynamic_obstacle_decision', 10)

        # ============================================================
        # 订阅 /dynamic_scan
        # ============================================================
        self.dynamic_scan_sub = self.create_subscription(
            LaserScan,
            self.dynamic_scan_topic,
            self.dynamic_scan_callback,
            10
        )

        # ============================================================
        # 机器人状态
        # ============================================================
        self.robot_x = None
        self.robot_y = None
        self.robot_vx = 0.0
        self.robot_vy = 0.0
        self.robot_speed = 0.0

        self.prev_robot_x = None
        self.prev_robot_y = None
        self.prev_robot_time = None

        self.robot_state = 'NORMAL_NAVIGATION'
        self.prev_robot_state_for_transition = None
        self.pass_committed = False

        # ============================================================
        # 动态障碍物状态
        # ============================================================
        self.conflict_detected = False
        self.conflict_points_count = 0

        self.obstacle_centroid_x = None
        self.obstacle_centroid_y = None
        self.obstacle_vx = 0.0
        self.obstacle_vy = 0.0
        self.obstacle_speed_2d = 0.0

        self.prev_obstacle_x = None
        self.prev_obstacle_y = None
        self.prev_obstacle_time = None

        self.robot_obstacle_distance = None
        self.distance_rate = None

        self.obstacle_motion_state = 'NO_OBSTACLE'
        self.obstacle_ttc_to_robot_path = None
        self.obstacle_velocity_along_path = 0.0
        self.obstacle_rel_to_robot_path = None

        # 机器人相对 TTC 相关
        self.obstacle_moving_towards_robot = False
        self.obstacle_moving_away_from_robot = False
        self.obstacle_passed_robot_path = False

        self.relative_ttc_to_robot = None
        self.predicted_min_distance = None
        self.relative_collision_risk = False
        self.relative_slow_risk = False

        self.turnaround_risk = False
        self.distance_to_path = None

        # ============================================================
        # 计时器
        # ============================================================
        self.decision_state = 'NORMAL_NAVIGATION'
        self.yield_start_time = None
        self.clear_start_time = None
        self.release_start_time = None

        # 日志状态记录
        self.last_print_robot_state = None
        self.last_print_conflict = None
        self.last_print_decision_state = None
        self.last_print_motion_state = None
        self.last_print_passed_path = None
        self.last_print_turnaround = None

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('intersection_supervisor robot-aware TTC started.')
        self.get_logger().info(f'map_frame: {self.map_frame}')
        self.get_logger().info(f'robot_frame: {self.robot_frame}')
        self.get_logger().info(f'dynamic_scan_topic: {self.dynamic_scan_topic}')
        self.get_logger().info(f'world_file_path: {self.world_file_path}')
        self.get_logger().info(
            f'moving obstacle: model={self.moving_model_name or "<first plugin>"}, '
            f'axis={self.motion_axis}, fixed_x={self.fixed_x:.2f}, fixed_y={self.fixed_y:.2f}, '
            f'start_x={self.start_x:.2f}, end_x={self.end_x:.2f}, '
            f'start_y={self.start_y:.2f}, end_y={self.end_y:.2f}, speed={self.obstacle_speed:.2f}'
        )
        self.get_logger().info(
            f'path zone: {self.path_zone_description()}, '
            f'commit_path_half_width={self.commit_path_half_width:.2f}, '
            f'stop_distance={self.stop_distance:.2f}, slow_distance={self.slow_distance:.2f}'
        )

    # ============================================================
    # world 文件读取
    # ============================================================
    def get_default_float(self, name):
        return float(self.get_parameter(name).value)

    def load_moving_obstacle_from_world(self):
        self.motion_axis = str(self.get_parameter('default_motion_axis').value).lower()
        self.fixed_x = self.get_default_float('default_fixed_x')
        self.fixed_y = self.get_default_float('default_fixed_y')
        self.start_x = self.get_default_float('default_start_x')
        self.end_x = self.get_default_float('default_end_x')
        self.start_y = self.get_default_float('default_start_y')
        self.end_y = self.get_default_float('default_end_y')
        self.obstacle_speed = self.get_default_float('default_obstacle_speed')

        if not self.world_file_path:
            self.get_logger().warn('world_file_path is empty, use default moving obstacle params.')
            return

        if not os.path.exists(self.world_file_path):
            self.get_logger().warn(
                f'world_file_path does not exist: {self.world_file_path}, use default moving obstacle params.'
            )
            return

        try:
            tree = ET.parse(self.world_file_path)
            root = tree.getroot()
        except Exception as e:
            self.get_logger().warn(f'failed to parse world file: {e}, use default moving obstacle params.')
            return

        selected_model = None
        selected_plugin = None

        for model in root.findall('.//model'):
            model_name = model.attrib.get('name', '')
            if self.moving_model_name and model_name != self.moving_model_name:
                continue

            for plugin in model.findall('plugin'):
                plugin_name = plugin.attrib.get('name', '')
                filename = plugin.attrib.get('filename', '')
                if 'moving_obstacle_plugin' in plugin_name or 'moving_obstacle_plugin' in filename:
                    selected_model = model
                    selected_plugin = plugin
                    break

            if selected_plugin is not None:
                break

        if selected_plugin is None:
            self.get_logger().warn('no moving_obstacle_plugin found in world file, use default moving obstacle params.')
            return

        self.moving_model_name = selected_model.attrib.get('name', self.moving_model_name)

        def plugin_text(tag, default):
            elem = selected_plugin.find(tag)
            if elem is None or elem.text is None:
                return default
            return elem.text.strip()

        self.motion_axis = plugin_text('motion_axis', self.motion_axis).lower()
        self.fixed_x = float(plugin_text('x', self.fixed_x))
        self.fixed_y = float(plugin_text('y', self.fixed_y))
        self.start_x = float(plugin_text('start_x', self.start_x))
        self.end_x = float(plugin_text('end_x', self.end_x))
        self.start_y = float(plugin_text('start_y', self.start_y))
        self.end_y = float(plugin_text('end_y', self.end_y))
        self.obstacle_speed = float(plugin_text('speed', self.obstacle_speed))

    def recompute_path_bounds(self):
        self.start_x, self.end_x = min(self.start_x, self.end_x), max(self.start_x, self.end_x)
        self.start_y, self.end_y = min(self.start_y, self.end_y), max(self.start_y, self.end_y)

        if self.motion_axis == 'y':
            self.path_fixed_coord = self.fixed_x
            self.path_min_coord = self.start_y
            self.path_max_coord = self.end_y
        else:
            self.path_fixed_coord = self.fixed_y
            self.path_min_coord = self.start_x
            self.path_max_coord = self.end_x

    def path_zone_description(self):
        if self.motion_axis == 'y':
            return (
                f'x≈{self.fixed_x:.2f}, y=[{self.start_y:.2f}, {self.end_y:.2f}], '
                f'detect_x_half_width={self.path_detect_half_width:.2f}'
            )
        return (
            f'y≈{self.fixed_y:.2f}, x=[{self.start_x:.2f}, {self.end_x:.2f}], '
            f'detect_y_half_width={self.path_detect_half_width:.2f}'
        )

    # ============================================================
    # 基础工具
    # ============================================================
    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def path_cross_axis_distance(self, x, y):
        if x is None or y is None:
            return None
        if self.motion_axis == 'y':
            return abs(x - self.fixed_x)
        return abs(y - self.fixed_y)

    def robot_target_coord_on_obstacle_path(self):
        if self.robot_x is None or self.robot_y is None:
            if self.motion_axis == 'y':
                return 0.5 * (self.start_y + self.end_y)
            return 0.5 * (self.start_x + self.end_x)

        return self.robot_y if self.motion_axis == 'y' else self.robot_x

    def obstacle_coord_along_path(self, x, y):
        return y if self.motion_axis == 'y' else x

    def point_in_dynamic_path_zone(self, x, y):
        if self.motion_axis == 'y':
            return (
                abs(x - self.fixed_x) <= self.path_detect_half_width
                and (self.start_y - self.path_detect_margin) <= y <= (self.end_y + self.path_detect_margin)
            )

        return (
            abs(y - self.fixed_y) <= self.path_detect_half_width
            and (self.start_x - self.path_detect_margin) <= x <= (self.end_x + self.path_detect_margin)
        )

    def robot_in_commit_zone(self, x, y):
        if self.motion_axis == 'y':
            return (
                abs(x - self.fixed_x) <= self.commit_path_half_width
                and (self.start_y - self.path_detect_margin) <= y <= (self.end_y + self.path_detect_margin)
            )

        return (
            abs(y - self.fixed_y) <= self.commit_path_half_width
            and (self.start_x - self.path_detect_margin) <= x <= (self.end_x + self.path_detect_margin)
        )

    def obstacle_distance_to_endpoints(self):
        """障碍物沿运动轴距离端点的最近距离。"""
        if self.obstacle_centroid_x is None or self.obstacle_centroid_y is None:
            return None

        coord = self.obstacle_coord_along_path(self.obstacle_centroid_x, self.obstacle_centroid_y)
        return min(abs(coord - self.path_min_coord), abs(coord - self.path_max_coord))

    # ============================================================
    # dynamic_scan 处理
    # ============================================================
    def dynamic_scan_callback(self, scan_msg: LaserScan):
        result = self.check_dynamic_obstacle(scan_msg)
        (
            self.conflict_detected,
            self.conflict_points_count,
            self.obstacle_centroid_x,
            self.obstacle_centroid_y,
            self.robot_obstacle_distance,
            self.obstacle_motion_state,
            self.obstacle_ttc_to_robot_path,
            self.obstacle_velocity_along_path,
            self.obstacle_rel_to_robot_path,
        ) = result

        self.update_relative_risk()

    def check_dynamic_obstacle(self, scan_msg: LaserScan):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                scan_msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )
        except Exception as e:
            self.get_logger().warn(
                f'Cannot get TF {self.map_frame} -> {scan_msg.header.frame_id}: {e}',
                throttle_duration_sec=1.0
            )
            self.reset_obstacle_motion()
            return False, 0, None, None, None, 'NO_OBSTACLE', None, 0.0, None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        yaw = self.yaw_from_quaternion(tf.transform.rotation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        dynamic_points = []

        for i, r in enumerate(scan_msg.ranges):
            if not math.isfinite(r):
                continue
            if r < scan_msg.range_min or r > scan_msg.range_max:
                continue

            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x_laser = r * math.cos(angle)
            y_laser = r * math.sin(angle)

            x_map = tx + cos_yaw * x_laser - sin_yaw * y_laser
            y_map = ty + sin_yaw * x_laser + cos_yaw * y_laser

            if self.point_in_dynamic_path_zone(x_map, y_map):
                dynamic_points.append((x_map, y_map))

        point_count = len(dynamic_points)
        has_obstacle = point_count >= self.conflict_points_threshold

        if not has_obstacle:
            self.prev_obstacle_x = None
            self.prev_obstacle_y = None
            self.prev_obstacle_time = None
            self.reset_obstacle_motion()
            return False, point_count, None, None, None, 'NO_OBSTACLE', None, 0.0, None

        cx = sum(p[0] for p in dynamic_points) / point_count
        cy = sum(p[1] for p in dynamic_points) / point_count

        robot_obstacle_distance = None
        if self.robot_x is not None and self.robot_y is not None:
            robot_obstacle_distance = math.hypot(cx - self.robot_x, cy - self.robot_y)

        now = self.now_sec()

        vx = 0.0
        vy = 0.0
        if self.prev_obstacle_x is not None and self.prev_obstacle_y is not None and self.prev_obstacle_time is not None:
            dt = now - self.prev_obstacle_time
            if dt > 0.001:
                vx = (cx - self.prev_obstacle_x) / dt
                vy = (cy - self.prev_obstacle_y) / dt

        self.obstacle_vx = vx
        self.obstacle_vy = vy
        self.obstacle_speed_2d = math.hypot(vx, vy)

        obstacle_coord = self.obstacle_coord_along_path(cx, cy)
        target_coord = self.robot_target_coord_on_obstacle_path()
        rel_to_robot_path = obstacle_coord - target_coord

        velocity_along_path = vy if self.motion_axis == 'y' else vx
        ttc = None
        motion_state = 'UNKNOWN'

        if self.obstacle_speed_2d < self.obstacle_velocity_threshold:
            motion_state = 'STATIC_OR_SLOW'
        else:
            approaching_robot_path = (
                (rel_to_robot_path > 0.0 and velocity_along_path < -self.obstacle_velocity_threshold) or
                (rel_to_robot_path < 0.0 and velocity_along_path > self.obstacle_velocity_threshold)
            )

            leaving_robot_path = (
                (rel_to_robot_path > 0.0 and velocity_along_path > self.obstacle_velocity_threshold) or
                (rel_to_robot_path < 0.0 and velocity_along_path < -self.obstacle_velocity_threshold)
            )

            if abs(rel_to_robot_path) <= self.block_path_half_width:
                motion_state = 'IN_ROBOT_CROSSING_PATH'
            elif approaching_robot_path:
                ttc = abs(rel_to_robot_path) / max(abs(velocity_along_path), 1e-6)
                if ttc <= self.ttc_stop_time:
                    motion_state = 'APPROACHING_DANGEROUS'
                elif ttc <= self.ttc_slow_time:
                    motion_state = 'APPROACHING_NEAR'
                else:
                    motion_state = 'APPROACHING_BUT_FAR'
            elif leaving_robot_path:
                if abs(rel_to_robot_path) >= self.release_path_distance:
                    motion_state = 'LEAVING_CLEAR'
                else:
                    motion_state = 'LEAVING_BUT_NOT_CLEAR'
            else:
                motion_state = 'STATIC_OR_SLOW'

        self.prev_obstacle_x = cx
        self.prev_obstacle_y = cy
        self.prev_obstacle_time = now

        return (
            True,
            point_count,
            cx,
            cy,
            robot_obstacle_distance,
            motion_state,
            ttc,
            velocity_along_path,
            rel_to_robot_path,
        )

    def reset_obstacle_motion(self):
        self.obstacle_centroid_x = None
        self.obstacle_centroid_y = None
        self.obstacle_vx = 0.0
        self.obstacle_vy = 0.0
        self.obstacle_speed_2d = 0.0
        self.robot_obstacle_distance = None
        self.distance_rate = None
        self.obstacle_motion_state = 'NO_OBSTACLE'
        self.obstacle_ttc_to_robot_path = None
        self.obstacle_velocity_along_path = 0.0
        self.obstacle_rel_to_robot_path = None
        self.obstacle_moving_towards_robot = False
        self.obstacle_moving_away_from_robot = False
        self.obstacle_passed_robot_path = False
        self.relative_ttc_to_robot = None
        self.predicted_min_distance = None
        self.relative_collision_risk = False
        self.relative_slow_risk = False
        self.turnaround_risk = False

    # ============================================================
    # 机器人速度估计
    # ============================================================
    def update_robot_velocity(self, x, y):
        now = self.now_sec()

        if self.prev_robot_x is not None and self.prev_robot_y is not None and self.prev_robot_time is not None:
            dt = now - self.prev_robot_time
            if dt > 0.001:
                self.robot_vx = (x - self.prev_robot_x) / dt
                self.robot_vy = (y - self.prev_robot_y) / dt
                self.robot_speed = math.hypot(self.robot_vx, self.robot_vy)

        self.prev_robot_x = x
        self.prev_robot_y = y
        self.prev_robot_time = now

    # ============================================================
    # 相对运动 / 相对 TTC / 掉头风险
    # ============================================================
    def update_relative_risk(self):
        self.obstacle_moving_towards_robot = False
        self.obstacle_moving_away_from_robot = False
        self.obstacle_passed_robot_path = False
        self.relative_ttc_to_robot = None
        self.predicted_min_distance = None
        self.relative_collision_risk = False
        self.relative_slow_risk = False
        self.turnaround_risk = False
        self.distance_rate = None

        if not self.conflict_detected:
            return

        if (
            self.robot_x is None or self.robot_y is None or
            self.obstacle_centroid_x is None or self.obstacle_centroid_y is None
        ):
            return

        rx = self.obstacle_centroid_x - self.robot_x
        ry = self.obstacle_centroid_y - self.robot_y
        dist = math.hypot(rx, ry)

        if dist < 1e-6:
            self.distance_rate = -999.0
            self.obstacle_moving_towards_robot = True
            self.relative_collision_risk = True
            self.predicted_min_distance = 0.0
            self.relative_ttc_to_robot = 0.0
            return

        rvx = self.obstacle_vx - self.robot_vx
        rvy = self.obstacle_vy - self.robot_vy
        rel_speed = math.hypot(rvx, rvy)

        # 距离变化率：负数代表越来越近，正数代表越来越远。
        self.distance_rate = (rx * rvx + ry * rvy) / dist

        self.obstacle_moving_towards_robot = self.distance_rate < -self.distance_closing_threshold
        self.obstacle_moving_away_from_robot = self.distance_rate > self.distance_opening_threshold

        # 判断障碍物是否已经越过机器人预计穿越点，并且在远离该穿越点。
        if self.obstacle_rel_to_robot_path is not None:
            rel_path = self.obstacle_rel_to_robot_path
            v_path = self.obstacle_velocity_along_path

            leaving_path = (
                (rel_path > 0.0 and v_path > self.obstacle_velocity_threshold) or
                (rel_path < 0.0 and v_path < -self.obstacle_velocity_threshold)
            )

            self.obstacle_passed_robot_path = (
                abs(rel_path) >= self.passed_path_release_distance and leaving_path
            )

        # 相对 TTC：预测最近接近时间和最近距离。
        if self.enable_robot_aware_ttc and rel_speed >= self.min_relative_speed:
            dot = rx * rvx + ry * rvy
            t_cpa = -dot / max(rel_speed * rel_speed, 1e-6)

            # 只关心未来 horizon 内的最近接近点。
            if 0.0 <= t_cpa <= self.ttc_prediction_horizon:
                px = rx + rvx * t_cpa
                py = ry + rvy * t_cpa
                min_dist = math.hypot(px, py)

                self.relative_ttc_to_robot = t_cpa
                self.predicted_min_distance = min_dist

                # 只有两者正在靠近时，才把相对 TTC 作为停车/减速依据。
                if self.obstacle_moving_towards_robot:
                    if t_cpa <= self.relative_ttc_stop_time and min_dist <= self.collision_radius:
                        self.relative_collision_risk = True
                    elif t_cpa <= self.relative_ttc_slow_time and min_dist <= self.slow_collision_radius:
                        self.relative_slow_risk = True

        # 掉头风险：障碍物接近端点，可能马上反向；如果小车正在靠近路径，就不应直接 PASS。
        if self.enable_turnaround_risk:
            end_dist = self.obstacle_distance_to_endpoints()
            path_dist = self.path_cross_axis_distance(self.robot_x, self.robot_y)

            near_endpoint = end_dist is not None and end_dist <= self.turnaround_margin
            robot_near_path = path_dist is not None and path_dist <= self.turnaround_path_risk_distance
            robot_close_to_obstacle = (
                self.robot_obstacle_distance is not None and
                self.robot_obstacle_distance <= self.turnaround_robot_distance
            )

            # 小车靠近障碍物运动路径，同时障碍物快掉头，且机器人与障碍物并没有明显远离。
            self.turnaround_risk = (
                near_endpoint and
                robot_near_path and
                robot_close_to_obstacle and
                not self.obstacle_moving_away_from_robot
            )

    # ============================================================
    # 主循环
    # ============================================================
    def timer_callback(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )
        except Exception as e:
            self.get_logger().warn(
                f'Cannot get TF {self.map_frame} -> {self.robot_frame}: {e}',
                throttle_duration_sec=1.0
            )
            return

        old_state = self.robot_state

        self.robot_x = tf.transform.translation.x
        self.robot_y = tf.transform.translation.y
        self.update_robot_velocity(self.robot_x, self.robot_y)

        self.distance_to_path = self.path_cross_axis_distance(self.robot_x, self.robot_y)

        # robot 速度更新后，再刷新一次相对风险，保证用最新 robot_vx/vy。
        self.update_relative_risk()

        self.robot_state = self.judge_robot_state(self.robot_x, self.robot_y)
        self.prev_robot_state_for_transition = old_state

        self.update_decision_state()
        self.publish_outputs()
        self.print_debug()

    # ============================================================
    # 机器人区域判断
    # ============================================================
    def judge_robot_state(self, x, y):
        dist_to_path = self.path_cross_axis_distance(x, y)

        if self.robot_in_commit_zone(x, y):
            return 'COMMIT_PASS_ZONE'

        # 已经承诺通过后，在完全离开动态路径前都不能因为点云抖动而反复停车。
        if self.pass_committed and dist_to_path is not None and dist_to_path <= self.exit_path_distance:
            return 'LEAVING_DYNAMIC_PATH'

        if dist_to_path is not None and dist_to_path <= self.near_path_distance:
            return 'NEAR_DYNAMIC_PATH'

        if dist_to_path is not None and dist_to_path <= self.approach_path_distance:
            return 'APPROACH_DYNAMIC_PATH'

        return 'NORMAL_NAVIGATION'

    # ============================================================
    # 风险判断函数
    # ============================================================
    def relative_danger_now(self):
        if not self.conflict_detected:
            return False

        if self.relative_collision_risk:
            return True

        if self.robot_obstacle_distance is not None:
            if self.robot_obstacle_distance <= self.stop_distance and self.obstacle_moving_towards_robot:
                return True

        return False

    def obstacle_is_blocking_or_dangerous(self):
        if not self.conflict_detected:
            return False

        # 最高优先级：机器人与障碍物相对运动预测会撞。
        if self.relative_collision_risk:
            return True

        # 如果障碍物已经通过机器人必经线并且也在远离机器人，不再认为危险。
        if self.obstacle_passed_robot_path and self.obstacle_moving_away_from_robot:
            return False

        # 障碍物虽然远离路径线，但机器人斜着靠近它，不能直接放行。
        if self.obstacle_motion_state == 'LEAVING_CLEAR':
            if self.obstacle_moving_towards_robot:
                if self.predicted_min_distance is not None and self.predicted_min_distance <= self.collision_radius:
                    return True
                if self.robot_obstacle_distance is not None and self.robot_obstacle_distance <= self.stop_distance:
                    return True
            return False

        # 障碍物在机器人预计穿越线上，如果同时正在靠近机器人，停车。
        if self.obstacle_motion_state == 'IN_ROBOT_CROSSING_PATH':
            return self.obstacle_moving_towards_robot or (
                self.robot_obstacle_distance is not None and self.robot_obstacle_distance <= self.stop_distance
            )

        if self.obstacle_motion_state == 'APPROACHING_DANGEROUS':
            return self.obstacle_moving_towards_robot or (
                self.robot_obstacle_distance is not None and self.robot_obstacle_distance <= self.stop_distance
            )

        if self.obstacle_motion_state == 'LEAVING_BUT_NOT_CLEAR':
            return self.obstacle_moving_towards_robot and (
                self.robot_obstacle_distance is not None and self.robot_obstacle_distance <= self.slow_distance
            )

        if self.obstacle_motion_state in ['UNKNOWN', 'STATIC_OR_SLOW']:
            return (
                self.robot_state == 'NEAR_DYNAMIC_PATH' and
                self.robot_obstacle_distance is not None and
                self.robot_obstacle_distance <= self.stop_distance
            )

        return False

    def obstacle_requires_slowdown(self):
        if not self.conflict_detected:
            return False

        if self.relative_slow_risk:
            return True

        if self.turnaround_risk:
            return True

        # 距离在变近时，即使障碍物状态是 LEAVING_CLEAR，也要慢行观察。
        if self.obstacle_moving_towards_robot:
            if self.robot_obstacle_distance is not None and self.robot_obstacle_distance <= self.slow_distance:
                return True
            if self.predicted_min_distance is not None and self.predicted_min_distance <= self.slow_collision_radius:
                return True

        # 障碍物已通过路径并远离机器人，则无需慢行。
        if self.obstacle_passed_robot_path and self.obstacle_moving_away_from_robot:
            return False

        if self.robot_obstacle_distance is not None and self.robot_obstacle_distance <= self.slow_distance:
            return True

        if self.robot_state in ['APPROACH_DYNAMIC_PATH', 'NEAR_DYNAMIC_PATH']:
            if self.obstacle_motion_state in [
                'APPROACHING_NEAR',
                'APPROACHING_BUT_FAR',
                'UNKNOWN',
                'STATIC_OR_SLOW',
                'LEAVING_BUT_NOT_CLEAR',
            ]:
                return True

        return False

    def obstacle_clear_to_pass(self):
        if not self.conflict_detected:
            return True

        # 核心放行条件：
        # 不只是“障碍物远离路径线”，还必须确认它也在远离小车。
        if self.obstacle_passed_robot_path and self.obstacle_moving_away_from_robot:
            if not self.relative_collision_risk and not self.relative_slow_risk and not self.turnaround_risk:
                return True

        if self.obstacle_motion_state == 'LEAVING_CLEAR':
            if self.obstacle_moving_away_from_robot and not self.turnaround_risk:
                return True
            return False

        if self.obstacle_motion_state == 'APPROACHING_BUT_FAR':
            # 如果机器人也在斜向靠近障碍物，不直接放行。
            if self.obstacle_moving_towards_robot:
                return False
            if self.robot_obstacle_distance is None:
                return False
            return self.robot_obstacle_distance > self.slow_distance

        return False

    def startup_commit_zone_is_dangerous(self):
        """小车启动时已经在 commit zone 内时，不能无条件通过。"""
        if not self.conflict_detected:
            return False

        if self.relative_collision_risk:
            return True

        if self.turnaround_risk and self.obstacle_moving_towards_robot:
            return True

        if self.obstacle_moving_towards_robot:
            if self.robot_obstacle_distance is not None and self.robot_obstacle_distance <= self.slow_distance:
                return True
            if self.predicted_min_distance is not None and self.predicted_min_distance <= self.slow_collision_radius:
                return True

        if self.obstacle_motion_state in ['IN_ROBOT_CROSSING_PATH', 'APPROACHING_DANGEROUS']:
            if not self.obstacle_moving_away_from_robot:
                return True

        return False

    # ============================================================
    # 决策状态机
    # ============================================================
    def update_decision_state(self):
        now = self.now_sec()

        # ========================================================
        # 1. 进入 COMMIT_PASS_ZONE
        # ========================================================
        if self.robot_state == 'COMMIT_PASS_ZONE':

            # 已经承诺通过，则继续通过。
            if self.pass_committed:
                self.decision_state = 'PASS_INTERSECTION'
                return

            came_from_outside = self.prev_robot_state_for_transition in [
                'APPROACH_DYNAMIC_PATH',
                'NEAR_DYNAMIC_PATH',
            ]

            # 小车运动中从外部进入 commit zone：进入通过承诺，避免停在障碍物路径上。
            if came_from_outside:
                self.decision_state = 'PASS_INTERSECTION'
                self.pass_committed = True
                self.yield_start_time = None
                self.clear_start_time = None
                self.release_start_time = None
                return

            # 小车启动时已经在 commit zone：先判断是否安全。
            if self.check_startup_in_commit_zone and self.startup_commit_zone_is_dangerous():
                if self.decision_state != 'YIELD_WAIT':
                    self.yield_start_time = now
                    self.clear_start_time = None
                self.decision_state = 'YIELD_WAIT'
                return

            # 已经在 commit zone，但当前无明显风险：允许通过并锁存。
            self.decision_state = 'PASS_INTERSECTION'
            self.pass_committed = True
            self.yield_start_time = None
            self.clear_start_time = None
            self.release_start_time = None
            return

        # ========================================================
        # 2. 离开动态路径但还未完全离开：继续通过。
        # ========================================================
        if self.robot_state == 'LEAVING_DYNAMIC_PATH':
            self.decision_state = 'PASS_INTERSECTION'
            self.pass_committed = True
            self.yield_start_time = None
            self.clear_start_time = None
            self.release_start_time = None
            return

        # ========================================================
        # 3. 完全远离动态路径：解除通过承诺。
        # ========================================================
        if self.robot_state == 'NORMAL_NAVIGATION':
            self.pass_committed = False
            self.yield_start_time = None
            self.clear_start_time = None
            self.release_start_time = None

            if self.relative_collision_risk:
                self.decision_state = 'YIELD_WAIT'
            elif self.obstacle_requires_slowdown():
                self.decision_state = 'SLOW_DOWN'
            else:
                self.decision_state = 'NORMAL_NAVIGATION'
            return

        # ========================================================
        # 4. 已经承诺通过，则继续通过。
        #    注意：这里不再被 LEAVING_CLEAR 等状态干扰。
        # ========================================================
        if self.pass_committed:
            self.decision_state = 'PASS_INTERSECTION'
            return

        # ========================================================
        # 5. 没有检测到动态障碍物。
        # ========================================================
        if not self.conflict_detected:
            self.release_start_time = None

            if self.decision_state == 'YIELD_WAIT':
                if self.clear_start_time is None:
                    self.clear_start_time = now

                if now - self.clear_start_time < self.clear_confirm_time:
                    self.decision_state = 'YIELD_WAIT'
                    return

            self.clear_start_time = None
            self.yield_start_time = None
            self.decision_state = 'NORMAL_NAVIGATION'
            return

        # ========================================================
        # 6. 障碍物已经通过机器人必经线，且也远离机器人：确认后放行。
        # ========================================================
        if self.obstacle_clear_to_pass():
            if self.release_start_time is None:
                self.release_start_time = now

            if now - self.release_start_time >= self.release_confirm_time:
                self.decision_state = 'PASS_INTERSECTION'
                self.yield_start_time = None
                self.clear_start_time = None
                self.release_start_time = None
                return

            self.decision_state = 'SLOW_DOWN'
            return

        self.release_start_time = None

        # ========================================================
        # 7. 相对运动预测会撞，或者确实阻塞：停车等待。
        # ========================================================
        if self.obstacle_is_blocking_or_dangerous():
            if self.decision_state != 'YIELD_WAIT':
                self.yield_start_time = now
                self.clear_start_time = None

            self.decision_state = 'YIELD_WAIT'

            if self.yield_start_time is not None:
                wait_time = now - self.yield_start_time
                if wait_time >= self.max_yield_wait_time:
                    self.decision_state = 'RECOVERY'
            return

        # ========================================================
        # 8. 有接近风险 / 掉头风险：减速观察。
        # ========================================================
        if self.obstacle_requires_slowdown():
            self.decision_state = 'SLOW_DOWN'
            self.yield_start_time = None
            self.clear_start_time = None
            return

        # ========================================================
        # 9. 默认不急停，慢行观察。
        # ========================================================
        self.decision_state = 'SLOW_DOWN'

    # ============================================================
    # 发布输出
    # ============================================================
    def publish_outputs(self):
        state_msg = String()
        state_msg.data = self.robot_state
        self.state_pub.publish(state_msg)
        self.dynamic_state_pub.publish(state_msg)

        conflict_msg = Bool()
        conflict_msg.data = self.conflict_detected
        self.conflict_pub.publish(conflict_msg)

        decision_msg = String()
        decision_msg.data = self.decision_state
        self.decision_pub.publish(decision_msg)
        self.dynamic_decision_pub.publish(decision_msg)

        motion_msg = String()
        motion_msg.data = self.obstacle_motion_state
        self.motion_pub.publish(motion_msg)

    # ============================================================
    # 调试打印
    # ============================================================
    def fmt(self, v, precision=2):
        if v is None:
            return 'None'
        return f'{v:.{precision}f}'

    def print_debug(self):
        self.get_logger().info(
            f'robot_state={self.robot_state}, '
            f'prev_state={self.prev_robot_state_for_transition}, '
            f'decision={self.decision_state}, '
            f'conflict={self.conflict_detected}, '
            f'points={self.conflict_points_count}, '
            f'motion={self.obstacle_motion_state}, '
            f'obs=({self.fmt(self.obstacle_centroid_x)}, {self.fmt(self.obstacle_centroid_y)}), '
            f'robot=({self.fmt(self.robot_x)}, {self.fmt(self.robot_y)}), '
            f'dist={self.fmt(self.robot_obstacle_distance)}, '
            f'dist_rate={self.fmt(self.distance_rate, 3)}, '
            f'towards_robot={self.obstacle_moving_towards_robot}, '
            f'away_robot={self.obstacle_moving_away_from_robot}, '
            f'passed_path={self.obstacle_passed_robot_path}, '
            f'rel_path={self.fmt(self.obstacle_rel_to_robot_path)}, '
            f'v_path={self.obstacle_velocity_along_path:.3f}, '
            f'path_ttc={self.fmt(self.obstacle_ttc_to_robot_path)}, '
            f'rel_ttc={self.fmt(self.relative_ttc_to_robot)}, '
            f'pred_min_dist={self.fmt(self.predicted_min_distance)}, '
            f'rel_collision={self.relative_collision_risk}, '
            f'rel_slow={self.relative_slow_risk}, '
            f'turnaround={self.turnaround_risk}, '
            f'pass_committed={self.pass_committed}, '
            f'dist_to_path={self.fmt(self.distance_to_path)}',
            throttle_duration_sec=1.0
        )

        if self.robot_state != self.last_print_robot_state:
            self.get_logger().warn(f'ROBOT STATE CHANGED: {self.last_print_robot_state} -> {self.robot_state}')
            self.last_print_robot_state = self.robot_state

        if self.conflict_detected != self.last_print_conflict:
            self.get_logger().warn(f'CONFLICT CHANGED: {self.last_print_conflict} -> {self.conflict_detected}')
            self.last_print_conflict = self.conflict_detected

        if self.decision_state != self.last_print_decision_state:
            self.get_logger().warn(f'DECISION CHANGED: {self.last_print_decision_state} -> {self.decision_state}')
            self.last_print_decision_state = self.decision_state

        if self.obstacle_motion_state != self.last_print_motion_state:
            self.get_logger().warn(f'OBSTACLE MOTION CHANGED: {self.last_print_motion_state} -> {self.obstacle_motion_state}')
            self.last_print_motion_state = self.obstacle_motion_state

        if self.obstacle_passed_robot_path != self.last_print_passed_path:
            self.get_logger().warn(f'PASSED PATH CHANGED: {self.last_print_passed_path} -> {self.obstacle_passed_robot_path}')
            self.last_print_passed_path = self.obstacle_passed_robot_path

        if self.turnaround_risk != self.last_print_turnaround:
            self.get_logger().warn(f'TURNAROUND RISK CHANGED: {self.last_print_turnaround} -> {self.turnaround_risk}')
            self.last_print_turnaround = self.turnaround_risk


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleSupervisor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()