import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import json
import math
import sys

def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
         math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
         math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

class PandaTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('panda_trajectory_publisher')

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/panda_arm_controller/joint_trajectory',
            10
        )

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.current_joint_state = None
        self.current_point_idx = 0
        self.previous_point = None
        self.timer = self.create_timer(1.0, self.timer_callback)

        try:
            json_path = '/home/duy/ws_moveit2/src/json_path_executor/json_path_executor/tcp_path.json'
    
            with open(json_path, 'r') as f:
                self.path_points = json.load(f)
            self.get_logger().info("Đọc file JSON thành công")
        except Exception as e:
            self.get_logger().error(f"Lỗi đọc file JSON: {e}")
            self.path_points = []

        self.target_joint = []
        self.waiting = False
        self.last_sent_time = self.get_clock().now()

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def timer_callback(self):
        if not self.path_points:
            return

        if self.waiting:
            duration = (self.get_clock().now() - self.last_sent_time).nanoseconds * 1e-9
            if duration < 1.0:
                return

            if self.has_reached_target():
                self.get_logger().info(f"Robot đã đến đúng vị trí {self.current_point_idx + 1}")
                self.current_point_idx += 1
                self.waiting = False
            else:
                self.get_logger().warn("Robot chưa tới đúng vị trí")
            return

        if self.current_point_idx >= len(self.path_points):
            self.get_logger().info("Đã hoàn thành tất cả các điểm, kết thúc chương trình.")
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)

        self.send_next_point()

    def send_next_point(self):
        point_data = self.path_points[self.current_point_idx]

        # So sánh với điểm trước để điều chỉnh hướng quay
        if self.previous_point:
            for axis in ['rx', 'ry', 'rz']:
                delta = abs(point_data[axis] - self.previous_point[axis])
                if delta > math.pi:  # tránh quay ngược nếu có thể
                    point_data[axis] = self.previous_point[axis]

        joint_positions = point_data["joint_positions"]

        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 2
        traj_msg.points.append(point)
        self.publisher_.publish(traj_msg)

        self.target_joint = joint_positions
        self.waiting = True
        self.last_sent_time = self.get_clock().now()
        self.previous_point = point_data

        x, y, z = point_data['x'], point_data['y'], point_data['z']
        rx, ry, rz = point_data['rx'], point_data['ry'], point_data['rz']
        qx, qy, qz, qw = quaternion_from_euler(rx, ry, rz)

        self.get_logger().info(f"Gửi điểm {self.current_point_idx + 1}: "
                               f"TCP ({x:.1f}, {y:.1f}, {z:.1f}) | "
                               f"Quat ({qx:.2f}, {qy:.2f}, {qz:.2f}, {qw:.2f})")

    def has_reached_target(self, tol=0.01):
        if self.current_joint_state is None:
            return False

        name_to_pos = dict(zip(self.current_joint_state.name, self.current_joint_state.position))
        for i, name in enumerate([
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']):
            current = name_to_pos.get(name, 0.0)
            target = self.target_joint[i]
            diff = abs(current - target)
            if diff > tol:
                self.get_logger().info(f"Joint {name} lệch {diff:.4f} rad (target: {target:.4f}, actual: {current:.4f})")
                return False
        return True

def main(args=None):
    rclpy.init(args=args)
    node = PandaTrajectoryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
