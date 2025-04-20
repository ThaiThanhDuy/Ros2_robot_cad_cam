import json
import re
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK

# Hàm chuyển Euler -> Quaternion
def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

def pose_from_gcode(x, y, z, a, b, c):
    quat = quaternion_from_euler(a, b, c)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "base_link"
    pose_stamped.pose.position.x = x / 1000.0
    pose_stamped.pose.position.y = y / 1000.0
    pose_stamped.pose.position.z = z / 1000.0
    pose_stamped.pose.orientation.x = quat[0]
    pose_stamped.pose.orientation.y = quat[1]
    pose_stamped.pose.orientation.z = quat[2]
    pose_stamped.pose.orientation.w = quat[3]
    return pose_stamped

class GCodeToRobotController(Node):
    def __init__(self):
        super().__init__('gcode_to_robot_controller')

        self.declare_parameter('start_line', 0)
        self.line_idx = self.get_parameter('start_line').get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.current_joint_state = None
        self.target_joint = []
        self.waiting = False

        self.gcode_lines = self.load_gcode_file('/home/duy/ws_moveit2/src/json_path_executor/json_path_executor/path.nc')
        self.cli = self.create_client(GetPositionIK, "/compute_ik")
        while not self.cli.wait_for_service(timeout_sec=1.5):
            self.get_logger().info('Waiting for IK service...')
        self.get_logger().info('IK service connected.')

        self.timer = self.create_timer(1.0, self.timer_callback)

    def load_gcode_file(self, filepath):
        try:
            with open(filepath, 'r') as f:
                lines = [line.strip() for line in f.readlines()]
                self.get_logger().info(f"Đã load {len(lines)} dòng G-code.")
                return lines
        except FileNotFoundError:
            self.get_logger().error("Không tìm thấy file G-code.")
            return []

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def robot_reached_target(self):
        if not self.current_joint_state or not self.target_joint:
            return False
        tolerance = 0.01
        for i, name in enumerate(self.current_joint_state.name):
            if name in ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']:
                idx = self.current_joint_state.name.index(name)
                actual = self.current_joint_state.position[idx]
                desired = self.target_joint[idx]
                if abs(actual - desired) > tolerance:
                    return False
        return True

    def call_ik_async(self, pose_stamped, line):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.ik_link_name = "tool_link"
        request.ik_request.timeout.sec = 2

        if self.current_joint_state:
            request.ik_request.robot_state.joint_state.name = self.current_joint_state.name
            request.ik_request.robot_state.joint_state.position = self.current_joint_state.position

        future = self.cli.call_async(request)
        future.add_done_callback(lambda f: self.handle_ik_response(f, line))

    def handle_ik_response(self, future, line):
        if future.result() and future.result().error_code.val == 1:
            joint_state = future.result().solution.joint_state
            joint_map = dict(zip(joint_state.name, joint_state.position))
            joint_positions = [joint_map.get(name, 0.0) for name in ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']]
            self.send_trajectory(joint_positions)
        else:
            self.get_logger().warn(f"IK failed hoặc timeout. Bỏ qua dòng: {line}")
            self.waiting = False

    def send_trajectory(self, joint_positions):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * len(joint_positions)
        point.time_from_start.sec = 2
        traj_msg.points.append(point)
        self.publisher_.publish(traj_msg)

        self.target_joint = joint_positions
        self.waiting = True

    def process_gcode_line(self, line):
        if not line.startswith('G0') and not line.startswith('G1'):
            return

        x = float(re.search(r'X([-\d.]+)', line).group(1)) if re.search(r'X([-\d.]+)', line) else 0.0
        y = float(re.search(r'Y([-\d.]+)', line).group(1)) if re.search(r'Y([-\d.]+)', line) else 0.0
        z = float(re.search(r'Z([-\d.]+)', line).group(1)) if re.search(r'Z([-\d.]+)', line) else 0.0
        a = math.radians(float(re.search(r'A([-\d.]+)', line).group(1))) if re.search(r'A([-\d.]+)', line) else 0.0
        b = math.radians(float(re.search(r'B([-\d.]+)', line).group(1))) if re.search(r'B([-\d.]+)', line) else 0.0
        c = math.radians(float(re.search(r'C([-\d.]+)', line).group(1))) if re.search(r'C([-\d.]+)', line) else 0.0

        pose_stamped = pose_from_gcode(x, y, z, a, b, c)
        self.call_ik_async(pose_stamped, line)

    def timer_callback(self):
        if self.waiting:
            if not self.robot_reached_target():
                return
            else:
                self.waiting = False

        while self.line_idx < len(self.gcode_lines):
            line = self.gcode_lines[self.line_idx]
            self.line_idx += 1
            if not line or line.startswith(('(', '%', '#', 'G90', 'G91', 'M', 'G131', 'G133', 'G134')) or '=' in line:
                continue
            self.get_logger().info(f"Xử lý dòng G-code: {line}")
            self.process_gcode_line(line)
            break

def main(args=None):
    rclpy.init(args=args)
    node = GCodeToRobotController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
