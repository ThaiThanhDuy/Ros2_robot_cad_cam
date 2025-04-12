import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import json

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('tcp_path_visualizer')
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        try:
            with open('/home/duy/ws_moveit2/src/json_path_executor/json_path_executor/tcp_path.json', 'r') as f:
                self.path_points = json.load(f)
            self.get_logger().info("Đọc file JSON thành công")
        except Exception as e:
            self.get_logger().error(f"Lỗi đọc file JSON: {e}")
            self.path_points = []

    def timer_callback(self):
        if not self.path_points:
            return

        marker = Marker()
        marker.header.frame_id = "world"  # hoặc "world", tùy theo bạn đang dùng TF gì
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "tcp_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.005  # độ dày của line

        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0

        for pt in self.path_points:
            p = Point()
            p.x = pt["x"] / 1000.0  # nếu đang dùng đơn vị mm thì chia 1000
            p.y = pt["y"] / 1000.0
            p.z = pt["z"] / 1000.0
            marker.points.append(p)

        self.marker_pub.publish(marker)
        self.get_logger().info("Gửi đường line marker lên RViz")

        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()