#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class IntersectionSpeedGate(Node):
    """
    交叉路口速度门控节点：只做“停 / 走”，不做减速。

    输入：
      /cmd_vel_collision
        collision_monitor 输出后的速度

      /intersection_decision
        路口决策状态

    输出：
      /cmd_vel_safe
        最终给底盘执行的速度

    策略：
      NORMAL_NAVIGATION:
        正常放行

      APPROACH_INTERSECTION:
        正常放行，不减速

      PASS_INTERSECTION:
        正常放行

      LEAVING_INTERSECTION:
        正常放行

      YIELD_WAIT:
        停车等待

      RECOVERY:
        停车
    """

    def __init__(self):
        super().__init__('intersection_speed_gate')

        self.declare_parameter('cmd_vel_in_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_out_topic', '/cmd_vel_risk')
        self.declare_parameter('decision_topic', '/intersection_decision')

        self.declare_parameter('stop_on_recovery', True)

        self.cmd_vel_in_topic = self.get_parameter('cmd_vel_in_topic').value
        self.cmd_vel_out_topic = self.get_parameter('cmd_vel_out_topic').value
        self.decision_topic = self.get_parameter('decision_topic').value
        self.stop_on_recovery = bool(self.get_parameter('stop_on_recovery').value)

        self.current_decision = 'NORMAL_NAVIGATION'
        self.last_decision = None

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_in_topic,
            self.cmd_callback,
            10
        )

        self.decision_sub = self.create_subscription(
            String,
            self.decision_topic,
            self.decision_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_out_topic,
            10
        )

        self.get_logger().info('intersection_speed_gate started.')
        self.get_logger().info(f'cmd_vel_in_topic: {self.cmd_vel_in_topic}')
        self.get_logger().info(f'cmd_vel_out_topic: {self.cmd_vel_out_topic}')
        self.get_logger().info(f'decision_topic: {self.decision_topic}')
        self.get_logger().info('mode: STOP_OR_GO_ONLY')

    def decision_callback(self, msg: String):
        self.current_decision = msg.data

        if self.current_decision != self.last_decision:
            self.get_logger().warn(
                f'DECISION: {self.last_decision} -> {self.current_decision}'
            )
            self.last_decision = self.current_decision

    def cmd_callback(self, msg: Twist):
        out = self.copy_twist(msg)

        # 只在明确等待/恢复时停车，其余状态全部放行
        if self.current_decision == 'YIELD_WAIT':
            out = self.zero_twist()

        elif self.current_decision == 'RECOVERY' and self.stop_on_recovery:
            out = self.zero_twist()

        else:
            # NORMAL_NAVIGATION
            # APPROACH_INTERSECTION
            # PASS_INTERSECTION
            # LEAVING_INTERSECTION
            # 以及其他未知状态，全部直接放行
            pass

        self.cmd_pub.publish(out)

    @staticmethod
    def copy_twist(msg: Twist):
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z
        return out

    @staticmethod
    def zero_twist():
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = IntersectionSpeedGate()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()