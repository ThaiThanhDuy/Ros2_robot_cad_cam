#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA

class MeshMarkerPublisher(Node):
    def __init__(self):
        super().__init__('mesh_marker_publisher')
        self.marker_pub = self.create_publisher(Marker, 'mesh_marker', 10)
        self.timer = self.create_timer(0.1, self.publish_marker)  # Publish mỗi 0.1 giây (10 Hz)
        self.marker = Marker()
        self.marker.header.frame_id = "base_link"  # Thay thế bằng frame ID phù hợp
        self.marker.ns = "san_pham_marker"
        self.marker.id = 0
        self.marker.type = Marker.MESH_RESOURCE
        self.marker.action = Marker.ADD
        self.marker.mesh_resource = "package://kuka_kr210_r2700_description/meshes/collision/phoi.stl"
        self.marker.mesh_use_embedded_materials = True  # Sử dụng màu sắc được nhúng (nếu có)
        self.marker.pose = Pose()
        self.marker.pose.position = Point()
        self.marker.pose.position.x = 0.5
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = -0.09
        self.marker.pose.orientation = Quaternion()
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0
        self.marker.color = ColorRGBA()
        self.marker.color.r = 0.5
        self.marker.color.g = 0.5
        self.marker.color.b = 0.5
        self.marker.color.a = 1.0

    def publish_marker(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    mesh_marker_publisher = MeshMarkerPublisher()
    rclpy.spin(mesh_marker_publisher)
    mesh_marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()