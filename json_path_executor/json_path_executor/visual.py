import rclpy

from rclpy.node import Node

from visualization_msgs.msg import Marker

from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


class GcodePathVisualizer(Node):
    def __init__(self):
        super().__init__('gcode_path_visualizer')
        self.marker_pub = self.create_publisher(Marker, '/point_marker', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.path_points = []
        self.gcode_file_path = '/home/duy/ws_moveit2/src/json_path_executor/json_path_executor/path.nc'

        self.load_gcode_path()

    def load_gcode_path(self):
        try:
            with open(self.gcode_file_path, 'r') as f:
                content = f.read().upper()
                lines = content.strip().split('\n')
                for line in lines:
                    if line.startswith('G1'):
                        try:
                            parts = line.split()
                            x = None
                            y = None
                            z = None
                            for part in parts:
                                if part.startswith('X'):
                                    x = float(part[1:]) / 1000.0
                                elif part.startswith('Y'):
                                    y = float(part[1:]) / 1000.0
                                elif part.startswith('Z'):
                                    z = float(part[1:]) / 1000.0

                            if x is not None and y is not None and z is not None:
                                point = Point()
                                point.x = x
                                point.y = y
                                point.z = z
                                self.path_points.append(point)
                        except ValueError as e:
                            self.get_logger().warn(f"Lỗi khi phân tích dòng G-code: {line} - {e}")
            self.get_logger().info(f"Đọc thành công {len(self.path_points)} điểm từ file G-code.")
        except Exception as e:
            self.get_logger().error(f"Lỗi đọc file G-code: {e}")

    def timer_callback(self):
        if not self.path_points:
            self.get_logger().warn("Không có điểm nào để hiển thị từ file G-code.")
            return

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gcode_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Thiết lập pose cố định cho marker
        marker.pose.position = Point()
        marker.pose.position.x = 0.5
        marker.pose.position.y = 0.0
        marker.pose.position.z = -0.05
        marker.pose.orientation = Quaternion()
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.005

        marker.color.r = 0.0  # Đặt màu đỏ về 0
        marker.color.g = 1.0  # Đặt màu xanh lá cây về 1
        marker.color.b = 0.0  # Đặt màu xanh lam về 0
        marker.color.a = 1.0

        marker.points = self.path_points

        self.marker_pub.publish(marker)
        self.get_logger().info(f"Đã gửi {len(marker.points)} điểm (từ G-code) lên RViz với màu xanh lá tại vị trí (0.5, 0.0, -0.09).")


def main(args=None):
    rclpy.init(args=args)
    node = GcodePathVisualizer()
    rclpy.spin(node)


if __name__ == '__main__':
    main()