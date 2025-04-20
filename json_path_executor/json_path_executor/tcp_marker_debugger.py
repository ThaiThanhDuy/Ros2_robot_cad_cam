import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf2_ros import TransformListener, Buffer
import rclpy.duration

class TcpPathMarker(Node):
    def __init__(self):
        super().__init__('tcp_path_marker')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(Marker, '/tcp_path_marker', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)

        self.base_frame = 'base_link'
        self.tool_frame = 'tool_link'
        self.path_points = []

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            point = Point()
            point.x = trans.transform.translation.x
            point.y = trans.transform.translation.y
            point.z = trans.transform.translation.z
            self.path_points.append(point)

            self.publish_path_marker()

        except Exception as e:
            self.get_logger().warn(f"Không tìm thấy TF {self.base_frame} -> {self.tool_frame}: {e}")

    def publish_path_marker(self):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "tcp_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.003  # Độ dày đường
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0

        marker.points = self.path_points[-1000:]  # Giới hạn tối đa 1000 điểm nếu cần

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = TcpPathMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
