# File: launch/gazebo.launch.py
 
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
 
def generate_launch_description():
    # 获取当前包路径
    robot_package_dir = get_package_share_directory('robot')
 
    # 启动 Gazebo 空世界
    gazebo_ros_package_dir = get_package_share_directory('gazebo_ros')
    empty_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_package_dir, 'launch', 'gazebo.launch.py')
        )
    )
 
    # 加载 URDF 文件路径（需要提前用 xacro 生成）
    urdf_file_path = os.path.join(robot_package_dir, 'urdf', 'robot.urdf')
 
    # 启动 Gazebo 模型生成器节点
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_model',
        arguments=[
            '-entity', 'robot',
            '-file', urdf_file_path,
            '-topic', 'robot_description'
        ],
        output='screen'
    )
 
    # 静态 TF 发布器：base_link -> base_footprint
    tf_footprint_base_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )
 
    return LaunchDescription([
        empty_world_launch,
        spawn_entity_node,
        tf_footprint_base_node
    ])