#!/usr/bin/env python3
import math
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


JOINT_NAMES = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
]


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    one_minus_c = 1.0 - c
    return np.array(
        [
            [
                c + x * x * one_minus_c,
                x * y * one_minus_c - z * s,
                x * z * one_minus_c + y * s,
            ],
            [
                y * x * one_minus_c + z * s,
                c + y * y * one_minus_c,
                y * z * one_minus_c - x * s,
            ],
            [
                z * x * one_minus_c - y * s,
                z * y * one_minus_c + x * s,
                c + z * z * one_minus_c,
            ],
        ],
        dtype=float,
    )


def homogeneous(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


@dataclass
class ChainJoint:
    name: str
    joint_type: str
    origin: np.ndarray
    axis: np.ndarray


class SerialKinematics:
    """Minimal URDF serial-chain FK and position Jacobian implementation."""

    def __init__(self, urdf_file: str, base_link: str, tip_link: str) -> None:
        root = ET.parse(urdf_file).getroot()
        joint_by_child: Dict[str, ET.Element] = {}
        for joint in root.findall("joint"):
            child = joint.find("child")
            if child is not None:
                joint_by_child[child.attrib["link"]] = joint

        reverse_chain: List[ET.Element] = []
        current_link = tip_link
        while current_link != base_link:
            if current_link not in joint_by_child:
                raise RuntimeError(
                    f"URDF 中无法从 {tip_link} 回溯到 {base_link}，"
                    f"缺少 child={current_link} 的关节"
                )
            joint = joint_by_child[current_link]
            reverse_chain.append(joint)
            current_link = joint.find("parent").attrib["link"]

        self.chain: List[ChainJoint] = []
        for joint in reversed(reverse_chain):
            origin_element = joint.find("origin")
            xyz = np.zeros(3, dtype=float)
            rpy = np.zeros(3, dtype=float)
            if origin_element is not None:
                xyz = np.array(
                    [float(value) for value in origin_element.attrib.get(
                        "xyz", "0 0 0"
                    ).split()],
                    dtype=float,
                )
                rpy = np.array(
                    [float(value) for value in origin_element.attrib.get(
                        "rpy", "0 0 0"
                    ).split()],
                    dtype=float,
                )
            axis_element = joint.find("axis")
            axis = np.array([0.0, 0.0, 1.0], dtype=float)
            if axis_element is not None:
                axis = np.array(
                    [float(value) for value in axis_element.attrib.get(
                        "xyz", "0 0 1"
                    ).split()],
                    dtype=float,
                )
            self.chain.append(
                ChainJoint(
                    name=joint.attrib["name"],
                    joint_type=joint.attrib["type"],
                    origin=homogeneous(rpy_matrix(*rpy), xyz),
                    axis=axis,
                )
            )

        self.joint_index = {name: index for index, name in enumerate(JOINT_NAMES)}

    def forward_and_jacobian(
        self, positions: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        transform = np.eye(4, dtype=float)
        origins: Dict[str, np.ndarray] = {}
        axes: Dict[str, np.ndarray] = {}

        for joint in self.chain:
            transform = transform @ joint.origin
            if joint.name in self.joint_index and joint.joint_type in (
                "revolute",
                "continuous",
            ):
                origins[joint.name] = transform[:3, 3].copy()
                axes[joint.name] = transform[:3, :3] @ joint.axis
                angle = positions[self.joint_index[joint.name]]
                transform = transform @ homogeneous(
                    axis_angle_matrix(joint.axis, angle), np.zeros(3)
                )

        tool_position = transform[:3, 3].copy()
        jacobian = np.zeros((3, len(JOINT_NAMES)), dtype=float)
        for name, index in self.joint_index.items():
            if name in origins:
                jacobian[:, index] = np.cross(
                    axes[name], tool_position - origins[name]
                )
        return tool_position, jacobian

    def forward(self, positions: np.ndarray) -> np.ndarray:
        return self.forward_and_jacobian(positions)[0]


class HorizontalDiskSweepPublisher(Node):
    """Keep wrist2 fast while the arm fills a horizontal circular area."""

    def __init__(self) -> None:
        super().__init__("horizontal_disk_sweep_publisher")

        self.declare_parameter("disk_radius_m", 0.30)
        self.declare_parameter("surface_frequency_hz", 0.10)
        self.declare_parameter("wrist2_frequency_hz", 4.0)
        self.declare_parameter("wrist2_amplitude_deg", 14.0)
        self.declare_parameter("wrist2_phase_deg", 0.0)
        self.declare_parameter("backward_sign", -1.0)
        self.declare_parameter("lateral_sign", 1.0)
        self.declare_parameter("center_offset_x_m", 0.0)
        self.declare_parameter("center_offset_y_m", 0.0)
        self.declare_parameter("center_offset_z_m", -1.0)
        self.declare_parameter("base_initial_deg", 0.0)
        self.declare_parameter("shoulder_initial_deg", 0.0)
        self.declare_parameter("elbow_initial_deg", 0.0)
        self.declare_parameter("wrist1_initial_deg", 0.0)
        self.declare_parameter("wrist2_center_deg", 0.0)
        self.declare_parameter("wrist3_initial_deg", 0.0)
        self.declare_parameter("publish_rate_hz", 125.0)
        self.declare_parameter("ik_iterations", 6)
        self.declare_parameter("ik_damping", 0.02)
        self.declare_parameter("ik_gain", 0.85)
        self.declare_parameter("max_ik_step_deg", 8.0)
        self.declare_parameter("trail_points", 6000)
        self.declare_parameter("show_trail", True)
        self.declare_parameter("show_target_disk", True)

        self.disk_radius_m = abs(float(self.get_parameter("disk_radius_m").value))
        self.surface_frequency_hz = float(
            self.get_parameter("surface_frequency_hz").value
        )
        self.wrist2_frequency_hz = float(
            self.get_parameter("wrist2_frequency_hz").value
        )
        self.wrist2_amplitude_rad = math.radians(
            float(self.get_parameter("wrist2_amplitude_deg").value)
        )
        self.wrist2_phase_rad = math.radians(
            float(self.get_parameter("wrist2_phase_deg").value)
        )
        self.backward_sign = float(self.get_parameter("backward_sign").value)
        self.lateral_sign = float(self.get_parameter("lateral_sign").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.ik_iterations = int(self.get_parameter("ik_iterations").value)
        self.ik_damping = float(self.get_parameter("ik_damping").value)
        self.ik_gain = float(self.get_parameter("ik_gain").value)
        self.max_ik_step_rad = math.radians(
            float(self.get_parameter("max_ik_step_deg").value)
        )
        self.show_trail = bool(self.get_parameter("show_trail").value)
        self.show_target_disk = bool(
            self.get_parameter("show_target_disk").value
        )

        initial_degrees = [
            float(self.get_parameter("base_initial_deg").value),
            float(self.get_parameter("shoulder_initial_deg").value),
            float(self.get_parameter("elbow_initial_deg").value),
            float(self.get_parameter("wrist1_initial_deg").value),
            float(self.get_parameter("wrist2_center_deg").value),
            float(self.get_parameter("wrist3_initial_deg").value),
        ]
        self.positions = np.radians(np.array(initial_degrees, dtype=float))
        self.previous_positions = self.positions.copy()
        self.wrist2_center_rad = self.positions[4]

        robot_share = get_package_share_directory("robot")
        urdf_file = robot_share + "/urdf/robot.urdf"
        self.kinematics = SerialKinematics(
            urdf_file=urdf_file,
            base_link="mobile_base_link",
            tip_link="tool",
        )
        initial_tool_position = self.kinematics.forward(self.positions)
        self.disk_center = initial_tool_position + np.array(
            [
                float(self.get_parameter("center_offset_x_m").value),
                float(self.get_parameter("center_offset_y_m").value),
                float(self.get_parameter("center_offset_z_m").value),
            ],
            dtype=float,
        )

        # wrist2 is imposed directly. base/shoulder/elbow/wrist1 compensate XYZ.
        self.compensation_indices = np.array([0, 1, 2, 3], dtype=int)
        trail_points = max(1, int(self.get_parameter("trail_points").value))
        self.actual_trail = deque(maxlen=trail_points)
        self.latest_target_position = initial_tool_position.copy()

        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/disk_sweep_markers", 10
        )
        self.start_time = self.get_clock().now()
        self.previous_time = self.start_time
        self.motion_timer = self.create_timer(
            1.0 / self.publish_rate_hz, self.publish_joint_state
        )
        self.marker_timer = self.create_timer(1.0 / 60.0, self.publish_markers)

        self.get_logger().info(
            "水平圆面扫描已启动："
            f"半径={self.disk_radius_m:.3f} m，"
            f"慢速前后频率={self.surface_frequency_hz:.3f} Hz，"
            f"wrist2 高频={self.wrist2_frequency_hz:.3f} Hz，"
            f"wrist2 最大摆幅=±{math.degrees(self.wrist2_amplitude_rad):.2f}°"
        )
        self.get_logger().info(
            "未启用关节位置、速度、加速度、力矩或碰撞校验；仅用于 RViz 展示。"
        )

    @staticmethod
    def area_equalized_position(area_progress: float) -> float:
        """
        根据圆面积累计比例反求归一化Y位置。

        area_progress:
            0.0 表示圆面一端
            0.5 表示圆心
            1.0 表示圆面另一端

        返回值范围：
            -1.0 到 1.0
        """
        area_progress = max(0.0, min(1.0, area_progress))

        lower = -1.0
        upper = 1.0

        # 二分法反求圆截面积累计函数
        for _ in range(32):
            middle = 0.5 * (lower + upper)

            remaining = max(0.0, 1.0 - middle * middle)

            cumulative_area_ratio = (
                middle * math.sqrt(remaining)
                + math.asin(middle)
                + 0.5 * math.pi
            ) / math.pi

            if cumulative_area_ratio < area_progress:
                lower = middle
            else:
                upper = middle

        return 0.5 * (lower + upper)

    def target_at_time(
        self,
        elapsed: float,
    ) -> Tuple[np.ndarray, float]:

        # 一个完整周期表示：
        # 圆面一端 -> 另一端 -> 原来一端
        cycle_phase = (
            self.surface_frequency_hz * elapsed + 0.25
        ) % 1.0

        # 三角形进度：
        # 0 -> 1 -> 0
        if cycle_phase < 0.5:
            area_progress = 2.0 * cycle_phase
        else:
            area_progress = 2.0 * (1.0 - cycle_phase)

        # 根据累计面积比例反求Y位置。
        # 这样中间移动慢、两端移动快。
        normalized_y = self.area_equalized_position(
            area_progress
        )

        backward_offset = (
            self.backward_sign
            * self.disk_radius_m
            * normalized_y
        )

        # 当前Y位置对应的圆截面半宽
        lateral_half_width = (
            self.disk_radius_m
            * math.sqrt(
                max(
                    0.0,
                    1.0 - normalized_y * normalized_y,
                )
            )
        )

        # wrist2保持原来的高频扫描
        fast_phase = (
            2.0
            * math.pi
            * self.wrist2_frequency_hz
            * elapsed
            + self.wrist2_phase_rad
        )

        # X方向高频左右扫描
        lateral_offset = (
            self.lateral_sign
            * lateral_half_width
            * math.sin(fast_phase)
        )

        # 水平圆面：
        # X方向高频扫描
        # Y方向非匀速前后推进
        # Z方向保持固定
        target = self.disk_center + np.array(
            [
                lateral_offset,
                backward_offset,
                0.0,
            ],
            dtype=float,
        )

        # wrist2角度摆幅随圆截面宽度变化
        if self.disk_radius_m > 1.0e-12:
            envelope = (
                lateral_half_width / self.disk_radius_m
            )
        else:
            envelope = 0.0

        wrist2_target = (
            self.wrist2_center_rad
            + self.wrist2_amplitude_rad
            * envelope
            * math.sin(fast_phase)
        )

        return target, wrist2_target

    def solve_compensation(self, target: np.ndarray) -> np.ndarray:
        positions = self.positions.copy()
        damping_squared = self.ik_damping * self.ik_damping

        for _ in range(max(1, self.ik_iterations)):
            actual, full_jacobian = self.kinematics.forward_and_jacobian(positions)
            error = target - actual
            jacobian = full_jacobian[:, self.compensation_indices]
            system = jacobian @ jacobian.T + damping_squared * np.eye(3)
            try:
                delta = jacobian.T @ np.linalg.solve(system, error)
            except np.linalg.LinAlgError:
                delta = np.linalg.pinv(jacobian) @ error

            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > self.max_ik_step_rad:
                delta *= self.max_ik_step_rad / delta_norm
            positions[self.compensation_indices] += self.ik_gain * delta

        return positions

    def publish_joint_state(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1.0e-9
        delta_time = max((now - self.previous_time).nanoseconds * 1.0e-9, 1.0e-9)
        target, wrist2_target = self.target_at_time(elapsed)

        self.positions[4] = wrist2_target
        self.positions = self.solve_compensation(target)
        self.positions[4] = wrist2_target
        actual = self.kinematics.forward(self.positions)
        velocities = (self.positions - self.previous_positions) / delta_time

        message = JointState()
        message.header.stamp = now.to_msg()
        message.name = JOINT_NAMES
        message.position = self.positions.tolist()
        message.velocity = velocities.tolist()
        self.joint_state_pub.publish(message)

        self.latest_target_position = target.copy()
        self.actual_trail.append(Point(x=actual[0], y=actual[1], z=actual[2]))
        self.previous_positions = self.positions.copy()
        self.previous_time = now

    def publish_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        if self.show_target_disk:
            disk = Marker()
            disk.header.frame_id = "mobile_base_link"
            disk.header.stamp = now
            disk.ns = "horizontal_disk_target"
            disk.id = 0
            disk.type = Marker.CYLINDER
            disk.action = Marker.ADD
            disk.pose.position.x = float(self.disk_center[0])
            disk.pose.position.y = float(self.disk_center[1])
            disk.pose.position.z = float(self.disk_center[2] - 0.004)
            disk.pose.orientation.w = 1.0
            disk.scale.x = 2.0 * self.disk_radius_m
            disk.scale.y = 2.0 * self.disk_radius_m
            disk.scale.z = 0.008
            disk.color.r = 0.16
            disk.color.g = 0.45
            disk.color.b = 0.82
            disk.color.a = 0.16
            marker_array.markers.append(disk)

            boundary = Marker()
            boundary.header.frame_id = "mobile_base_link"
            boundary.header.stamp = now
            boundary.ns = "horizontal_disk_target"
            boundary.id = 1
            boundary.type = Marker.LINE_STRIP
            boundary.action = Marker.ADD
            boundary.pose.orientation.w = 1.0
            boundary.scale.x = 0.009
            boundary.color.r = 0.15
            boundary.color.g = 0.52
            boundary.color.b = 0.92
            boundary.color.a = 0.95
            for index in range(121):
                angle = 2.0 * math.pi * index / 120.0
                boundary.points.append(
                    Point(
                        x=float(
                            self.disk_center[0]
                            + self.disk_radius_m * math.cos(angle)
                        ),
                        y=float(
                            self.disk_center[1]
                            + self.disk_radius_m * math.sin(angle)
                        ),
                        z=float(self.disk_center[2] + 0.006),
                    )
                )
            marker_array.markers.append(boundary)

        if self.show_trail:
            actual_points = Marker()
            actual_points.header.frame_id = "mobile_base_link"
            actual_points.header.stamp = now
            actual_points.ns = "horizontal_disk_actual"
            actual_points.id = 0
            actual_points.type = Marker.POINTS
            actual_points.action = Marker.ADD
            actual_points.pose.orientation.w = 1.0
            actual_points.scale.x = 0.012
            actual_points.scale.y = 0.012
            actual_points.color.r = 0.82
            actual_points.color.g = 0.10
            actual_points.color.b = 0.07
            actual_points.color.a = 0.88
            actual_points.points = list(self.actual_trail)
            marker_array.markers.append(actual_points)

            current_target = Marker()
            current_target.header.frame_id = "mobile_base_link"
            current_target.header.stamp = now
            current_target.ns = "horizontal_disk_actual"
            current_target.id = 1
            current_target.type = Marker.SPHERE
            current_target.action = Marker.ADD
            current_target.pose.position.x = float(self.latest_target_position[0])
            current_target.pose.position.y = float(self.latest_target_position[1])
            current_target.pose.position.z = float(self.latest_target_position[2])
            current_target.pose.orientation.w = 1.0
            current_target.scale.x = 0.035
            current_target.scale.y = 0.035
            current_target.scale.z = 0.035
            current_target.color.r = 0.98
            current_target.color.g = 0.72
            current_target.color.b = 0.08
            current_target.color.a = 1.0
            marker_array.markers.append(current_target)

        self.marker_pub.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HorizontalDiskSweepPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
