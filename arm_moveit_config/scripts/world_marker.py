#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class WorldMeshPublisher(Node):
    def __init__(self):
        super().__init__('world_mesh_publisher')
        self.pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.timer = self.create_timer(0.2, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'scene_mesh'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        marker.mesh_resource = 'file:///home/olivier/fire/src/robot/meshes/world.STL'
        marker.mesh_use_embedded_materials = False

        marker.pose.position.x = 1.1
        marker.pose.position.y = 1.6325
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.pub.publish(marker)


def main():
    rclpy.init()
    node = WorldMeshPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()