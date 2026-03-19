#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float):
    """只绕 z 轴旋转"""
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qx, qy, qz, qw


class MobileBaseTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('mobile_base_trajectory_publisher')
        self.br = TransformBroadcaster(self)

        # ====== 可调参数 ======
        self.world_frame = 'world'
        self.base_frame = 'mobile_base_link'

        # 初始位姿
        self.x0 = 0.0
        self.y0 = 0.0
        self.z0 = 0.0
        self.yaw0 = 0.0

        # 轨迹参数
        self.straight_dist = 2.2             # 先沿 x 正方向走 2.2 m
        self.radius = 0.5325                 # 左转半径
        self.arc_deg = 89.0                  # 左转角度（度）
        self.v = 1.0                        # 线速度 m/s

        # 计算派生量
        self.arc_rad = math.radians(self.arc_deg)
        self.omega = self.v / self.radius
        self.t_straight = self.straight_dist / self.v
        self.t_arc = self.arc_rad / self.omega
        self.total_time = self.t_straight + self.t_arc

        # 记录开始时间
        self.t0 = self.get_clock().now()

        # 50 Hz 发布
        self.timer = self.create_timer(0.02, self.on_timer)

        self.get_logger().info(
            f"Trajectory started: straight {self.straight_dist} m, "
            f"then left arc R={self.radius} m, angle={self.arc_deg} deg"
        )

    def on_timer(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        # 默认保持终点
        x, y, yaw = self.compute_pose(min(t, self.total_time))

        msg = TransformStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.world_frame
        msg.child_frame_id = self.base_frame

        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = self.z0

        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw

        self.br.sendTransform(msg)

    def compute_pose(self, t: float):
        """
        分段轨迹：
        1) 先直行
        2) 再做左转圆弧
        """
        # -------- 第一段：直线 --------
        if t <= self.t_straight:
            x = self.x0 + self.v * t
            y = self.y0
            yaw = self.yaw0
            return x, y, yaw

        # 直线末端
        x1 = self.x0 + self.straight_dist
        y1 = self.y0

        # -------- 第二段：左转圆弧 --------
        tau = t - self.t_straight
        phi = min(self.omega * tau, self.arc_rad)  # 当前已转过的角度

        # 初始朝向沿 +x，左转圆弧参数方程
        x = x1 + self.radius * math.sin(phi)
        y = y1 + self.radius * (1.0 - math.cos(phi))
        yaw = self.yaw0 + phi

        return x, y, yaw


def main():
    rclpy.init()
    node = MobileBaseTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()