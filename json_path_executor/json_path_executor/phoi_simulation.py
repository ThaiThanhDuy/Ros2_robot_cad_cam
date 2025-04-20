#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA


class PhoiBoxMarkerPublisher(Node):
    def __init__(self):
        super().__init__('phoi_box_marker_publisher')
        self.publisher = self.create_publisher(Marker, 'phoi_marker', 10)
        self.timer = self.create_timer(0.1, self.publish_marker)  # 10 Hz

        # Marker dạng hình hộp
        self.marker = Marker()
        self.marker.header.frame_id = "base_link"  # Gắn cố định ở base
        self.marker.ns = "phoi_marker"
        self.marker.id = 0
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD

        # Vị trí đặt phôi (có thể chỉnh nếu cần)
        self.marker.pose = Pose()
        self.marker.pose.position = Point(x=0.9, y=0.225, z=-0.06)  
        self.marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Kích thước hộp phôi (dài, rộng, cao)
        self.marker.scale.x = 0.8  # Chiều dài (X)
        self.marker.scale.y = 0.455  # Chiều rộng (Y)
        self.marker.scale.z = 0.04  # Chiều cao (Z)

        # Màu vàng (đậm hơn)
        self.marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.7)

    def publish_marker(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.marker)


def main(args=None):
    rclpy.init(args=args)
    node = PhoiBoxMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
