#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA


class ToolCylinderMarkerPublisher(Node):
    def __init__(self):
        super().__init__('tool_cylinder_marker_publisher')
        self.publisher = self.create_publisher(Marker, 'tool_cylinder_marker', 10)
        self.timer = self.create_timer(0.1, self.publish_marker)  # 10 Hz

        # Marker dạng hình trụ
        self.marker = Marker()
        self.marker.header.frame_id = "tool_link"  # Gắn theo TCP
        self.marker.ns = "tool_marker"
        self.marker.id = 0
        self.marker.type = Marker.CYLINDER
        self.marker.action = Marker.ADD

        # Vị trí tương đối so với TCP (tool_link)
        self.marker.pose = Pose()
        self.marker.pose.position = Point(x=0.01, y=0.0, z=0.0)  # Dưới TCP 10cm
        self.marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=0.3)

        # Kích thước hình trụ (đường kính và chiều cao)
        self.marker.scale.x = 0.01  # đường kính 1cm
        self.marker.scale.y = 0.01  # nên bằng nhau để hình trụ tròn đều
        self.marker.scale.z = 0.1   # chiều dài dao 10cm


        # Màu sắc
        # Màu vàng
        self.marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.5)


    def publish_marker(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.marker)


def main(args=None):
    rclpy.init(args=args)
    node = ToolCylinderMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()