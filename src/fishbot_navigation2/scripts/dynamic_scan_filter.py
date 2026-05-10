#!/usr/bin/env python3
import math
import copy

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

import tf2_ros


class DynamicScanFilter(Node):
    def __init__(self):
        super().__init__('dynamic_scan_filter')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('dynamic_scan_topic', '/dynamic_scan')
        self.declare_parameter('map_frame', 'map')

        # 地图中占用概率大于该值，认为是静态障碍物
        self.declare_parameter('occupied_thresh', 65)

        # 静态地图膨胀检查半径，单位：栅格数
        # 设为 1~2 可以减少地图误差导致的墙体漏检
        self.declare_parameter('static_margin_cells', 2)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.dynamic_scan_topic = self.get_parameter('dynamic_scan_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.occupied_thresh = int(self.get_parameter('occupied_thresh').value)
        self.static_margin_cells = int(self.get_parameter('static_margin_cells').value)

        self.map_msg = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.dynamic_scan_pub = self.create_publisher(
            LaserScan,
            self.dynamic_scan_topic,
            10
        )

        self.get_logger().info('dynamic_scan_filter started')
        self.get_logger().info(f'subscribe scan: {self.scan_topic}')
        self.get_logger().info(f'subscribe map: {self.map_topic}')
        self.get_logger().info(f'publish dynamic scan: {self.dynamic_scan_topic}')

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def is_static_obstacle_in_map(self, x_map, y_map):
        """
        判断某个 map 坐标点是否落在静态地图障碍物上。
        """
        if self.map_msg is None:
            return False

        info = self.map_msg.info
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        width = info.width
        height = info.height

        mx = int((x_map - origin_x) / resolution)
        my = int((y_map - origin_y) / resolution)

        if mx < 0 or mx >= width or my < 0 or my >= height:
            return False

        # 检查周围若干格，避免地图和雷达轻微错位导致墙体点被误判为动态障碍
        margin = self.static_margin_cells

        for dx in range(-margin, margin + 1):
            for dy in range(-margin, margin + 1):
                nx = mx + dx
                ny = my + dy

                if nx < 0 or nx >= width or ny < 0 or ny >= height:
                    continue

                index = ny * width + nx
                value = self.map_msg.data[index]

                if value >= self.occupied_thresh:
                    return True

        return False

    def scan_callback(self, scan: LaserScan):
        if self.map_msg is None:
            return

        try:
            # 使用最新 TF，避免仿真时间下出现 extrapolation 问题
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                scan.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )
        except Exception as e:
            self.get_logger().warn(
                f'Cannot transform {scan.header.frame_id} to {self.map_frame}: {e}',
                throttle_duration_sec=1.0
            )
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        yaw = self.yaw_from_quaternion(tf.transform.rotation)

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        dynamic_scan = copy.deepcopy(scan)

        dynamic_count = 0

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                dynamic_scan.ranges[i] = float('inf')
                continue

            if r < scan.range_min or r > scan.range_max:
                dynamic_scan.ranges[i] = float('inf')
                continue

            angle = scan.angle_min + i * scan.angle_increment

            # 激光点在 laser frame 下的坐标
            x_laser = r * math.cos(angle)
            y_laser = r * math.sin(angle)

            # 转到 map frame
            x_map = tx + cos_yaw * x_laser - sin_yaw * y_laser
            y_map = ty + sin_yaw * x_laser + cos_yaw * y_laser

            # 如果这个点落在静态地图障碍物上，过滤掉
            if self.is_static_obstacle_in_map(x_map, y_map):
                dynamic_scan.ranges[i] = float('inf')
            else:
                # 地图上本来是空的，但雷达扫到了，认为可能是动态障碍物
                dynamic_scan.ranges[i] = r
                dynamic_count += 1

        self.dynamic_scan_pub.publish(dynamic_scan)

        self.get_logger().info(
            f'publish /dynamic_scan, dynamic points: {dynamic_count}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = DynamicScanFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()