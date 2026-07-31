# 灭火机械臂水平圆面高频扫描

适用环境：Ubuntu 22.04、ROS 2 Humble、RViz2。

该工作空间保留原灭火机械臂模型。运动目标不再是圆周，而是让末端在固定高度的水平圆面内反复扫过：

- `wrist2_joint` 保持高频左右摆动；
- 末端同时沿机器人前后方向缓慢移动；
- 横向扫描幅度随前后位置按圆截面变化；
- `base_joint`、`shoulder_joint`、`elbow_joint`、`wrist1_joint` 实时补偿末端位置；
- `wrist3_joint` 保持初始角度；
- 不使用 MoveIt，不做碰撞规划；
- 不做关节位置、速度、加速度、力矩或动力学校验；
- 仅发布 `/joint_states`，用于 RViz 演示。

## 1. 轨迹逻辑

水平圆面的圆心为：

```text
C = (center_x, center_y, center_z)
```

慢速前后位置：

```text
x(t) = center_x + sign_back × R × sin(2π f_surface t)
```

当前前后位置对应的圆截面半宽：

```text
half_width(t) = R × |cos(2π f_surface t)|
```

高频横向扫描：

```text
y(t) = center_y
     + sign_lateral × half_width(t)
     × sin(2π f_wrist2 t + phase)
```

高度保持：

```text
z(t) = center_z
```

因此任意时刻均满足：

```text
(x - center_x)² + (y - center_y)² ≤ R²
```

`wrist2_joint` 的角度也使用相同幅度包络：

```text
q_wrist2(t) = q_center
             + A_max × half_width(t) / R
             × sin(2π f_wrist2 t + phase)
```

默认启动时，末端位于圆心，横向摆幅最大；随后向后移动到圆边界，横向摆幅逐渐减小到 0。继续运行后会往复覆盖完整圆面。

## 2. 为什么不需要 MoveIt

该任务没有避障、目标姿态规划或真机控制器执行需求。节点直接：

```text
生成水平圆面内目标点
→ 固定 wrist2 高频目标角
→ 用位置雅可比逆解 shoulder/elbow/wrist1 等补偿关节
→ 发布 JointState
→ RViz 显示
```

引入 MoveIt 会增加规划延迟和工程依赖，不适合持续高频扫描展示。

## 3. 编译

```bash
cd fire_horizontal_disk
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 4. 默认运行

```bash
ros2 launch fire_swing_demo horizontal_disk_rviz.launch.py
```

默认参数：

```text
disk_radius_m            = 0.12
surface_frequency_hz     = 0.10
wrist2_frequency_hz      = 4.0
wrist2_amplitude_deg     = 14.0
wrist2_phase_deg         = 0.0
backward_sign            = -1.0
lateral_sign             = 1.0
publish_rate_hz          = 125.0
```

RViz 中：

- 半透明蓝色圆盘：目标水平圆面；
- 蓝色圆线：圆面边界；
- 红色点云：实际末端扫过的位置；
- 黄色球：当前目标点。

## 5. 常用参数

### 扩大圆面并保持高频 wrist2

```bash
ros2 launch fire_swing_demo horizontal_disk_rviz.launch.py \
  disk_radius_m:=0.18 \
  surface_frequency_hz:=0.08 \
  wrist2_frequency_hz:=4.0 \
  wrist2_amplitude_deg:=18.0
```

### 更快填充圆面

```bash
ros2 launch fire_swing_demo horizontal_disk_rviz.launch.py \
  disk_radius_m:=0.12 \
  surface_frequency_hz:=0.18 \
  wrist2_frequency_hz:=6.0 \
  wrist2_amplitude_deg:=10.0 \
  publish_rate_hz:=250.0
```

### 反转前后方向

```bash
ros2 launch fire_swing_demo horizontal_disk_rviz.launch.py \
  backward_sign:=1.0
```

### 反转左右方向

```bash
ros2 launch fire_swing_demo horizontal_disk_rviz.launch.py \
  lateral_sign:=-1.0
```

### 移动圆面中心

偏移量基于 `mobile_base_link` 坐标系：

```bash
ros2 launch fire_swing_demo horizontal_disk_rviz.launch.py \
  center_offset_x_m:=-0.10 \
  center_offset_y_m:=0.05 \
  center_offset_z_m:=0.02
```

### 修改初始机械臂姿态

```bash
ros2 launch fire_swing_demo horizontal_disk_rviz.launch.py \
  base_initial_deg:=0.0 \
  shoulder_initial_deg:=-10.0 \
  elbow_initial_deg:=20.0 \
  wrist1_initial_deg:=-10.0 \
  wrist2_center_deg:=0.0 \
  wrist3_initial_deg:=0.0
```

### 关闭目标圆面或轨迹

```bash
ros2 launch fire_swing_demo horizontal_disk_rviz.launch.py \
  show_target_disk:=false
```

```bash
ros2 launch fire_swing_demo horizontal_disk_rviz.launch.py \
  show_trail:=false
```

## 6. 参数表

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `disk_radius_m` | `0.12` | 水平圆面半径，单位 m |
| `surface_frequency_hz` | `0.10` | 前后慢运动频率 |
| `wrist2_frequency_hz` | `4.0` | wrist2 高频摆动频率 |
| `wrist2_amplitude_deg` | `14.0` | 圆心位置处 wrist2 最大单侧摆幅 |
| `wrist2_phase_deg` | `0.0` | wrist2 初始相位 |
| `backward_sign` | `-1.0` | 前后扫描方向符号 |
| `lateral_sign` | `1.0` | 左右扫描方向符号 |
| `center_offset_x_m` | `0.0` | 圆心 X 偏移 |
| `center_offset_y_m` | `0.0` | 圆心 Y 偏移 |
| `center_offset_z_m` | `0.0` | 圆面高度偏移 |
| `*_initial_deg` | `0.0` | 各关节初始角度 |
| `wrist2_center_deg` | `0.0` | wrist2 高频摆动中心角 |
| `publish_rate_hz` | `125.0` | `/joint_states` 发布频率 |
| `ik_iterations` | `6` | 每个周期补偿迭代次数 |
| `ik_damping` | `0.02` | 阻尼最小二乘系数 |
| `ik_gain` | `0.85` | 每次逆解更新比例 |
| `max_ik_step_deg` | `8.0` | 单次数字迭代最大角度步长，不是真机关节限制 |
| `trail_points` | `6000` | RViz 保留的末端轨迹点数 |
| `show_trail` | `true` | 显示实际扫描轨迹 |
| `show_target_disk` | `true` | 显示目标圆面 |

## 7. 文件结构

```text
src/
├── robot/
└── fire_swing_demo/
    ├── launch/
    │   └── horizontal_disk_rviz.launch.py
    ├── scripts/
    │   └── horizontal_disk_sweep.py
    ├── rviz/
    │   └── horizontal_disk.rviz
    ├── CMakeLists.txt
    └── package.xml
```

旧文件已经删除：

```text
scripts/end_effector_swing.py
scripts/kinematic_validation.py
launch/swing_rviz.launch.py
```

## 8. 使用边界

该节点没有真机安全限制。输入很大的半径、频率或摆幅时，RViz 仍会尝试播放。数值逆解不保证目标始终可达，也不检查：

- 关节限位；
- 速度、加速度和 jerk；
- 电机力矩、电流、温升；
- 自碰撞和环境碰撞；
- Zu20 真机控制器跟踪能力。

因此该包只用于轨迹形态和演示效果验证，不应直接把同一组参数发送给真机。
