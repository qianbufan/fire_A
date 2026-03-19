#!/usr/bin/env python3
import time
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState

from pymoveit2 import MoveIt2


class MobileBaseTFPublisher(Node):
    """底盘轨迹 TF 发布器 - 独立节点"""

    def __init__(self, node_name="mobile_base_tf"):
        super().__init__(node_name)
        self.br = TransformBroadcaster(self)

        # ====== 可调参数 ======
        self.world_frame = "world"
        self.base_frame = "mobile_base_link"

        # 当前底盘全局位姿
        self.current_pose = (0.0, 0.0, 0.0)  # (x, y, yaw)
        self.z0 = 0.0

        # 当前这一段轨迹的起点（每次 start_movement 时更新）
        self.x0 = 0.0
        self.y0 = 0.0
        self.yaw0 = 0.0

        # 轨迹参数
        self.straight_dist = 2.2
        self.radius = 0.5325
        self.arc_deg = 90.0
        self.v = 1.0

        self.arc_rad = math.radians(self.arc_deg)
        self.omega = self.v / self.radius
        self.t_straight = self.straight_dist / self.v
        self.t_arc = self.arc_rad / self.omega
        self.total_time = self.t_straight + self.t_arc

        # 运动状态
        self.is_moving = False
        self.is_paused = False
        self.start_time = None
        self.pause_time = None
        self.current_t = 0.0
        self.paused_pose = self.current_pose

        # 50Hz 持续发布
        self.timer = self.create_timer(0.02, self.on_timer)

        self.get_logger().info("MobileBaseTFPublisher initialized")

    def start_movement(self):
        """从当前位置开始一段新的直线+圆弧运动"""
        self.x0, self.y0, self.yaw0 = self.current_pose

        self.is_moving = True
        self.is_paused = False
        self.start_time = self.get_clock().now()
        self.pause_time = None
        self.current_t = 0.0

        self.get_logger().info(
            f"Base movement started from current pose: "
            f"x0={self.x0:.3f}, y0={self.y0:.3f}, yaw0={math.degrees(self.yaw0):.2f} deg; "
            f"straight={self.straight_dist:.3f} m + arc={self.arc_deg:.2f} deg "
            f"(total time={self.total_time:.2f}s)"
        )

    def pause_movement(self):
        if self.is_moving and not self.is_paused:
            self.is_paused = True
            self.pause_time = self.get_clock().now()
            self.paused_pose = self.current_pose
            self.get_logger().info(
                f"Base movement paused at: "
                f"x={self.paused_pose[0]:.3f}, y={self.paused_pose[1]:.3f}, "
                f"yaw={math.degrees(self.paused_pose[2]):.2f} deg"
            )

    def resume_movement(self):
        if self.is_moving and self.is_paused:
            now = self.get_clock().now()
            if self.pause_time is not None and self.start_time is not None:
                paused_duration = now - self.pause_time
                self.start_time = self.start_time + paused_duration
            self.is_paused = False
            self.pause_time = None
            self.get_logger().info("Base movement resumed")

    def stop_movement(self):
        self.is_moving = False
        self.is_paused = False
        self.start_time = None
        self.pause_time = None
        self.current_t = 0.0
        self.get_logger().info(
            f"Base movement stopped at: "
            f"x={self.current_pose[0]:.3f}, y={self.current_pose[1]:.3f}, "
            f"yaw={math.degrees(self.current_pose[2]):.2f} deg"
        )

    def is_movement_complete(self):
        if not self.is_moving or self.is_paused:
            return False
        return self.current_t >= self.total_time

    def compute_pose(self, t: float):
        """
        从 (x0, y0, yaw0) 出发：
        1) 先沿 yaw0 方向直行
        2) 再左转圆弧
        """
        t = min(t, self.total_time)

        if t <= self.t_straight:
            s = self.v * t
            x = self.x0 + s * math.cos(self.yaw0)
            y = self.y0 + s * math.sin(self.yaw0)
            yaw = self.yaw0
            return x, y, yaw

        x1 = self.x0 + self.straight_dist * math.cos(self.yaw0)
        y1 = self.y0 + self.straight_dist * math.sin(self.yaw0)

        tau = t - self.t_straight
        phi = min(self.omega * tau, self.arc_rad)

        yaw = self.yaw0 + phi
        x = x1 + self.radius * (math.sin(yaw) - math.sin(self.yaw0))
        y = y1 - self.radius * (math.cos(yaw) - math.cos(self.yaw0))

        return x, y, yaw

    def on_timer(self):
        if not self.is_moving:
            x, y, yaw = self.current_pose
        elif self.is_paused:
            x, y, yaw = self.paused_pose
        else:
            now = self.get_clock().now()
            t = (now - self.start_time).nanoseconds * 1e-9
            self.current_t = t
            x, y, yaw = self.compute_pose(t)
            self.current_pose = (x, y, yaw)

            if self.current_t >= self.total_time:
                self.current_t = self.total_time
                self.current_pose = self.compute_pose(self.total_time)
                x, y, yaw = self.current_pose

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.child_frame_id = self.base_frame

        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = self.z0

        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)

        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw

        self.br.sendTransform(msg)


class ArmMover:
    def __init__(self, node: Node) -> None:
        self.node = node
        self._got_joint_state = False
        self._last_joint_state = None

        self._js_sub = self.node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.base_link = "base_link"
        self.ee_link = "tool"   # 改成你实际的末端 link
        self.group_name = "arm"

        self.joint_names = [
            "base_joint",
            "shoulder_joint",
            "elbow_joint",
            "wrist1_joint",
            "wrist2_joint",
            "wrist3_joint",
        ]

        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=self.joint_names,
            base_link_name=self.base_link,
            end_effector_name=self.ee_link,
            group_name=self.group_name,
            use_move_group_action=True,
        )

        # 默认速度
        self.moveit2.max_velocity = 3.0
        self.moveit2.max_acceleration = 2.0

        self.cartesian_client = self.node.create_client(
            GetCartesianPath,
            "/compute_cartesian_path"
        )

        self.execute_client = ActionClient(
            self.node,
            ExecuteTrajectory,
            "/execute_trajectory"
        )

        self.ready_joints = [
            math.radians(90),
            math.radians(-112),
            math.radians(19),
            math.radians(97),
            math.radians(-87),
            math.radians(90),
        ]

    def _on_joint_state(self, msg: JointState):
        self._got_joint_state = True
        self._last_joint_state = msg

    def wait_joint_states(self, timeout_sec=10.0) -> bool:
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout_sec:
            if self._got_joint_state and self._last_joint_state is not None:
                self.node.get_logger().info("Joint states are available.")
                return True
            time.sleep(0.05)
        self.node.get_logger().error("Timeout waiting for /joint_states.")
        return False

    def wait_move_action(self, timeout_sec=10.0) -> bool:
        t0 = time.time()
        cli = getattr(self.moveit2, "_MoveIt2__move_action_client", None)
        if cli is None:
            self.node.get_logger().error("MoveIt2 action client not found.")
            return False

        while rclpy.ok() and (time.time() - t0) < timeout_sec:
            if cli.server_is_ready():
                self.node.get_logger().info("move_action server is ready.")
                return True
            time.sleep(0.1)

        self.node.get_logger().error("Timeout waiting for move_action server.")
        return False

    def wait_cartesian_service(self, timeout_sec=10.0) -> bool:
        ok = self.cartesian_client.wait_for_service(timeout_sec=timeout_sec)
        if ok:
            self.node.get_logger().info("compute_cartesian_path service is ready.")
        else:
            self.node.get_logger().error("Timeout waiting for /compute_cartesian_path.")
        return ok

    def wait_execute_action(self, timeout_sec=10.0) -> bool:
        ok = self.execute_client.wait_for_server(timeout_sec=timeout_sec)
        if ok:
            self.node.get_logger().info("execute_trajectory action is ready.")
        else:
            self.node.get_logger().error("Timeout waiting for /execute_trajectory.")
        return ok

    def wait_all_services(self, timeout_sec=15.0) -> bool:
        checks = [
            self.wait_joint_states(timeout_sec),
            self.wait_move_action(timeout_sec),
            self.wait_cartesian_service(timeout_sec),
            self.wait_execute_action(timeout_sec),
        ]
        return all(checks)

    def get_current_pose(self, timeout_sec=3.0):
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout_sec:
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.base_link,
                    self.ee_link,
                    rclpy.time.Time()
                )
                p = trans.transform.translation
                q = trans.transform.rotation
                position = [p.x, p.y, p.z]
                quat_xyzw = [q.x, q.y, q.z, q.w]
                return position, quat_xyzw
            except (LookupException, ConnectivityException, ExtrapolationException):
                time.sleep(0.05)

        raise RuntimeError(f"Cannot get current pose from {self.base_link} to {self.ee_link}")

    def get_current_joint_positions(self, timeout_sec=3.0):
        """按 self.joint_names 顺序返回当前关节角"""
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout_sec:
            if self._last_joint_state is not None:
                name_to_pos = dict(zip(self._last_joint_state.name, self._last_joint_state.position))
                try:
                    return [name_to_pos[name] for name in self.joint_names]
                except KeyError as e:
                    self.node.get_logger().warn(f"Joint {e} not found in /joint_states, retrying...")
            time.sleep(0.05)

        raise RuntimeError("Cannot get current joint positions from /joint_states")

    def move_to_configuration(self, joint_positions, max_velocity=None, max_acceleration=None) -> bool:
        self.node.get_logger().info(f"Move to joint configuration: {joint_positions}")

        old_velocity = self.moveit2.max_velocity
        old_acceleration = self.moveit2.max_acceleration

        try:
            if max_velocity is not None:
                self.moveit2.max_velocity = max_velocity
            if max_acceleration is not None:
                self.moveit2.max_acceleration = max_acceleration

            self.moveit2.move_to_configuration(joint_positions)
            result = self.moveit2.wait_until_executed()

            if result is False:
                self.node.get_logger().error("Joint motion execution failed.")
                return False
            return True

        except Exception as e:
            self.node.get_logger().error(f"move_to_configuration exception: {repr(e)}")
            return False

        finally:
            # 恢复默认值，避免把后续动作也拖进速度泥潭
            self.moveit2.max_velocity = old_velocity
            self.moveit2.max_acceleration = old_acceleration

    def _build_start_state(self) -> RobotState:
        if self._last_joint_state is None:
            raise RuntimeError("No joint state available.")
        rs = RobotState()
        rs.joint_state = self._last_joint_state
        return rs

    def compute_cartesian_path(
        self,
        waypoints,
        max_step=0.01,
        jump_threshold=0.0,
        avoid_collisions=True,
    ):
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_link
        req.header.stamp = self.node.get_clock().now().to_msg()
        req.start_state = self._build_start_state()
        req.group_name = self.group_name
        req.link_name = self.ee_link
        req.waypoints = waypoints
        req.max_step = max_step
        req.jump_threshold = jump_threshold
        req.avoid_collisions = avoid_collisions

        if hasattr(req, "prismatic_jump_threshold"):
            req.prismatic_jump_threshold = 0.0
        if hasattr(req, "revolute_jump_threshold"):
            req.revolute_jump_threshold = 0.0

        future = self.cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        result = future.result()
        if result is None:
            raise RuntimeError("compute_cartesian_path returned None")
        return result

    def execute_trajectory(self, trajectory) -> bool:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        send_goal_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None:
            self.node.get_logger().error("Failed to send execute_trajectory goal.")
            return False
        if not goal_handle.accepted:
            self.node.get_logger().error("execute_trajectory goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result()

        if result is None:
            self.node.get_logger().error("execute_trajectory returned None.")
            return False

        if result.status != 4:
            self.node.get_logger().error(f"execute_trajectory failed, status={result.status}")
            return False

        return True

    def make_pose(self, xyz, quat_xyzw):
        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        pose.orientation.x = float(quat_xyzw[0])
        pose.orientation.y = float(quat_xyzw[1])
        pose.orientation.z = float(quat_xyzw[2])
        pose.orientation.w = float(quat_xyzw[3])
        return pose

    def build_snake_points_yz(self, base_pos, row_length=0.5, row_step=0.08, num_rows=7):
        x0, y0, z0 = base_pos
        pts = [[x0, y0, z0]]

        current_y = y0
        current_z = z0
        direction = -1.0

        for row in range(num_rows):
            target_y = current_y + direction * row_length
            pts.append([x0, target_y, current_z])

            if row != num_rows - 1:
                current_y = target_y
                current_z = current_z + row_step
                pts.append([x0, current_y, current_z])

            direction *= -1.0

        return pts

    def execute_cartesian_segment_once(
        self,
        p_start,
        p_goal,
        quat_xyzw,
        max_step=0.01,
        avoid_collisions=True,
    ) -> bool:
        waypoints = [
            self.make_pose(p_start, quat_xyzw),
            self.make_pose(p_goal, quat_xyzw),
        ]

        result = self.compute_cartesian_path(
            waypoints=waypoints,
            max_step=max_step,
            jump_threshold=0.0,
            avoid_collisions=avoid_collisions,
        )

        self.node.get_logger().info(
            f"[segment once] {p_start} -> {p_goal}, "
            f"fraction = {result.fraction:.4f}, error_code = {result.error_code.val}"
        )

        if result.fraction < 0.999:
            self.node.get_logger().error("This segment cannot be solved completely.")
            return False

        return self.execute_trajectory(result.solution)

    def execute_cartesian_polyline(self, points_xyz, quat_xyzw, max_step=0.01):
        whole_waypoints = [self.make_pose(p, quat_xyzw) for p in points_xyz]
        result = self.compute_cartesian_path(
            waypoints=whole_waypoints,
            max_step=max_step,
            jump_threshold=0.0,
            avoid_collisions=True,
        )

        self.node.get_logger().info(
            f"[whole] fraction = {result.fraction:.4f}, error_code = {result.error_code.val}"
        )

        if result.fraction > 0.999:
            self.node.get_logger().info("Whole snake path solved. Executing.")
            return self.execute_trajectory(result.solution)

        self.node.get_logger().warn(
            "Whole path incomplete. Fallback to segment-by-segment Cartesian execution."
        )

        for i in range(len(points_xyz) - 1):
            p_start = points_xyz[i]
            p_goal = points_xyz[i + 1]

            self.node.get_logger().info(
                f"[segment {i+1}/{len(points_xyz)-1}] start={p_start}, goal={p_goal}"
            )

            ok = self.execute_cartesian_segment_once(
                p_start=p_start,
                p_goal=p_goal,
                quat_xyzw=quat_xyzw,
                max_step=max_step,
                avoid_collisions=True,
            )
            if not ok:
                self.node.get_logger().error(f"Segment {i+1} execution failed.")
                return False

        return True

    def perform_snake_motion(self):
        self.node.get_logger().info("Starting snake motion...")

        current_pos, current_quat = self.get_current_pose()
        self.node.get_logger().info(f"Current position: {current_pos}")
        self.node.get_logger().info(f"Current quat_xyzw: {current_quat}")

        snake_points = self.build_snake_points_yz(
            base_pos=current_pos,
            row_length=0.5,
            row_step=0.08,
            num_rows=7,
        )
        self.node.get_logger().info(f"Snake polyline points: {snake_points}")

        ok = self.execute_cartesian_polyline(
            points_xyz=snake_points,
            quat_xyzw=current_quat,
            max_step=0.01,
        )

        if not ok:
            self.node.get_logger().error("Snake Cartesian execution failed.")
            return False

        self.node.get_logger().info("Snake path completed successfully!")
        return True

    def move_to_ready(self):
        self.node.get_logger().info("Moving to ready joint configuration...")
        return self.move_to_configuration(self.ready_joints)

    def swing_base_joint_once(
        self,
        delta_deg=30.0,
        go_velocity=0.4,
        go_acceleration=0.3,
        back_velocity=1.2,
        back_acceleration=0.8,
    ):
        """
        只让 base_joint 相对当前角度 +delta_deg，再回到原角度
        去的时候慢一点，回来的时候快一点
        """
        try:
            current_joints = self.get_current_joint_positions()
            start_joints = list(current_joints)
            target_joints = list(current_joints)

            delta_rad = math.radians(delta_deg)
            target_joints[0] = start_joints[0] + delta_rad  # base_joint

            self.node.get_logger().info(
                f"[ARM-BASE-JOINT] Swing start: "
                f"{math.degrees(start_joints[0]):.2f} deg -> "
                f"{math.degrees(target_joints[0]):.2f} deg -> "
                f"{math.degrees(start_joints[0]):.2f} deg | "
                f"go(v={go_velocity}, a={go_acceleration}), "
                f"back(v={back_velocity}, a={back_acceleration})"
            )

            # 去程：慢
            if not self.move_to_configuration(
                target_joints,
                max_velocity=go_velocity,
                max_acceleration=go_acceleration,
            ):
                self.node.get_logger().error("[ARM-BASE-JOINT] Failed to move to target")
                return False

            # 回程：快
            if not self.move_to_configuration(
                start_joints,
                max_velocity=back_velocity,
                max_acceleration=back_acceleration,
            ):
                self.node.get_logger().error("[ARM-BASE-JOINT] Failed to move back to start")
                return False

            self.node.get_logger().info("[ARM-BASE-JOINT] Swing completed")
            return True

        except Exception as e:
            self.node.get_logger().error(f"[ARM-BASE-JOINT] Exception: {repr(e)}")
            return False


class IntegratedController(Node):
    """联合控制器 - 主节点"""

    def __init__(self):
        super().__init__("integrated_controller")

        self.base_tf_pub = MobileBaseTFPublisher()
        self.arm_mover = ArmMover(self)

        self.get_logger().info("Integrated controller initialized")

    def wait_for_services(self):
        self.get_logger().info("Waiting for all services...")
        return self.arm_mover.wait_all_services(timeout_sec=15.0)

    def run_base_motion_with_joint_swing(
        self,
        step_name="base movement",
        delta_deg=30.0,
        go_velocity=0.4,
        go_acceleration=0.3,
        back_velocity=1.2,
        back_acceleration=0.8,
    ):
        """
        底盘运动的同时，让 base_joint 做一次 +delta_deg 再回来
        去程慢一点，回程快一点
        """
        self.get_logger().info(
            f"Starting concurrent {step_name}: "
            f"base motion + base_joint swing ({delta_deg:.1f} deg)"
        )

        swing_result = {"ok": False}

        def _arm_task():
            swing_result["ok"] = self.arm_mover.swing_base_joint_once(
                delta_deg=delta_deg,
                go_velocity=go_velocity,
                go_acceleration=go_acceleration,
                back_velocity=back_velocity,
                back_acceleration=back_acceleration,
            )

        # 启动底盘
        self.base_tf_pub.start_movement()

        # 同时启动机械臂 base_joint 摆动
        arm_thread = threading.Thread(target=_arm_task, daemon=True)
        arm_thread.start()

        # 等待底盘完成
        while rclpy.ok() and not self.base_tf_pub.is_movement_complete():
            time.sleep(0.05)

        self.base_tf_pub.stop_movement()

        # 等待机械臂线程完成
        arm_thread.join()

        if not swing_result["ok"]:
            self.get_logger().error(f"Concurrent {step_name}: base_joint swing failed")
            return False

        self.get_logger().info(
            f"Concurrent {step_name} completed at pose: {self.base_tf_pub.current_pose}"
        )
        return True

    def run_sequence(self):
        """
        序列：
        1. 机械臂到 ready
        2. 底盘运动 + base_joint 去慢回快摆动
        3. 机械臂 snake
        4. 机械臂回 ready
        5. 底盘运动 + base_joint 去慢回快摆动
        6. 机械臂 snake
        7. 最后回 ready
        """
        self.get_logger().info(
            "=== Starting integrated control sequence: "
            "Arm(ready) → [Base + base_joint swing] → Arm(snake) → Arm(ready) "
            "→ [Base + base_joint swing] → Arm(snake) → Arm(ready) ==="
        )

        self.get_logger().info("\n=== Step 1/6: First arm motion (to ready position) ===")
        if not self.arm_mover.move_to_ready():
            self.get_logger().error("Failed to move arm to ready position")
            return False
        
        if not self.arm_mover.perform_snake_motion():
            self.get_logger().error("First snake motion failed")
            return False

        self.get_logger().info("Returning arm to ready position...")
        if not self.arm_mover.move_to_ready():
            self.get_logger().error("Failed to return arm to ready position")
            return False

        self.get_logger().info("\n=== Step 2/6: First base movement + base_joint swing ===")
        if not self.run_base_motion_with_joint_swing(
            step_name="first base movement",
            delta_deg=-40.0,
            go_velocity=5.0,
            go_acceleration=5.0,
            back_velocity=5.0,
            back_acceleration=5.0,
        ):
            self.get_logger().error("First concurrent base movement failed")
            return False

        self.get_logger().info("\n=== Step 3/6: Second arm motion (snake motion) ===")
        if not self.arm_mover.perform_snake_motion():
            self.get_logger().error("Second snake motion failed")
            return False

        self.get_logger().info("Returning arm to ready position...")
        if not self.arm_mover.move_to_ready():
            self.get_logger().error("Failed to return arm to ready position")
            return False

        self.get_logger().info("\n=== Step 4/6: Second base movement + base_joint swing ===")
        if not self.run_base_motion_with_joint_swing(
            step_name="second base movement",
            delta_deg=-40.0,
            go_velocity=5.0,
            go_acceleration=5.0,
            back_velocity=5.0,
            back_acceleration=5.0,
        ):
            self.get_logger().error("Second concurrent base movement failed")
            return False

        self.get_logger().info("\n=== Step 5/6: Third arm motion (snake motion) ===")
        if not self.arm_mover.perform_snake_motion():
            self.get_logger().error("Third snake motion failed")
            return False

        self.get_logger().info("\n=== Step 6/6: Final return to ready configuration ===")
        if not self.arm_mover.move_to_ready():
            self.get_logger().error("Failed final return to ready position")
            return False

        self.get_logger().info("\n=== All steps completed successfully! ===")
        self.get_logger().info(
            f"Final base pose: x={self.base_tf_pub.current_pose[0]:.3f}, "
            f"y={self.base_tf_pub.current_pose[1]:.3f}, "
            f"yaw={math.degrees(self.base_tf_pub.current_pose[2]):.2f} deg"
        )
        return True

    def shutdown(self):
        self.base_tf_pub.stop_movement()
        self.base_tf_pub.destroy_node()
        self.destroy_node()


def main():
    rclpy.init()

    controller = IntegratedController()

    executor = SingleThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(controller.base_tf_pub)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if not controller.wait_for_services():
            controller.get_logger().error("Required services not available")
            return

        success = controller.run_sequence()

        if success:
            controller.get_logger().info("Integrated control sequence completed successfully!")
        else:
            controller.get_logger().error("Integrated control sequence failed")

        time.sleep(2.0)

    except KeyboardInterrupt:
        controller.get_logger().info("Interrupted by user")
    finally:
        controller.shutdown()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()