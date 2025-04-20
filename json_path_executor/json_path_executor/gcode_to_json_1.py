import json
import re
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
import tf_transformations as tf  # Cài bằng: pip install transforms3d tf-transformations

# ==== Cấu hình mặc định ====
default_orientation = {"A": 0.0, "B": 0.0, "C": 0.0}  # Đặt giá trị mặc định cho A, B, C là 0.0
default_feedrate = 1000.0
ik_service_name = "/compute_ik"
group_name = "arm"  # Sử dụng group name "arm" từ SRDF
ik_link_name = "tool_link"  # Sử dụng tip link "tool_link" từ SRDF
expected_joint_count = 6  # KUKA KR210 có 6 khớp trong group "arm"
tool_offset_z = -0.01  # Không cần offset nếu TCP đã được định nghĩa trong URDF/SRDF

# ==== Hàm chuyển Euler -> Quaternion ====
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

# ==== Hàm offset pose theo Z của TCP ====
def offset_pose_z(pose_stamped, offset_z):
    quat = [
        pose_stamped.pose.orientation.x,
        pose_stamped.pose.orientation.y,
        pose_stamped.pose.orientation.z,
        pose_stamped.pose.orientation.w
    ]
    T = tf.quaternion_matrix(quat)
    T[0:3, 3] = [
        pose_stamped.pose.position.x,
        pose_stamped.pose.position.y,
        pose_stamped.pose.position.z
    ]
    offset_matrix = tf.translation_matrix([0, 0, offset_z])
    T_offset = np.dot(T, offset_matrix)
    new_pos = T_offset[0:3, 3]
    new_quat = tf.quaternion_from_matrix(T_offset)

    new_pose = PoseStamped()
    new_pose.header = pose_stamped.header
    new_pose.pose.position.x = new_pos[0]
    new_pose.pose.position.y = new_pos[1]
    new_pose.pose.position.z = new_pos[2]
    new_pose.pose.orientation.x = new_quat[0]
    new_pose.pose.orientation.y = new_quat[1]
    new_pose.pose.orientation.z = new_quat[2]
    new_pose.pose.orientation.w = new_quat[3]
    return new_pose

class GCodeToJsonConverter(Node):
    def __init__(self):
        super().__init__('gcode_to_json_converter')
        self.cli = self.create_client(GetPositionIK, ik_service_name)
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for IK service...')
        self.get_logger().info('IK service connected.')
        self.last_joint_positions = None

    def call_ik(self, pose_stamped):
        request = GetPositionIK.Request()
        request.ik_request.group_name = group_name
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.ik_link_name = ik_link_name
        request.ik_request.timeout.sec = 2

        if self.last_joint_positions:
            request.ik_request.robot_state.joint_state.name = [
                "joint_a1", "joint_a2", "joint_a3",
                "joint_a4", "joint_a5", "joint_a6"
            ]
            request.ik_request.robot_state.joint_state.position = self.last_joint_positions

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().error_code.val == 1:
            joint_state = future.result().solution.joint_state
            joint_map = dict(zip(joint_state.name, joint_state.position))
            kuka_joint_names = [
                "joint_a1", "joint_a2", "joint_a3",
                "joint_a4", "joint_a5", "joint_a6"
            ]
            try:
                joint_positions = [joint_map[name] for name in kuka_joint_names]
                self.last_joint_positions = joint_positions
                return joint_positions
            except KeyError as e:
                self.get_logger().warn(f'Missing joint: {str(e)} in IK result.')
                return None
        else:
            self.get_logger().warn('IK failed or timed out.')
            return None

    def convert_gcode_to_json(self, gcode_path, output_path):
        tcp_path = []
        current_feedrate = default_feedrate

        with open(gcode_path, "r") as f:
            for line in f:
                line = line.strip().upper()
                if line.startswith("G1") or line.startswith("G0"):
                    x = re.search(r'X([\-\d.]+)', line)
                    y = re.search(r'Y([\-\d.]+)', line)
                    z = re.search(r'Z([\-\d.]+)', line)
                    f_val = re.search(r'F([\-\d.]+)', line)

                    # Thay đổi từ RX, RY, RZ thành A, B, C (tùy thuộc vào quy ước G-code của bạn)
                    a_match = re.search(r'A([\-\d.]+)', line)
                    b_match = re.search(r'B([\-\d.]+)', line)
                    c_match = re.search(r'C([\-\d.]+)', line)

                    a = float(a_match.group(1)) if a_match else default_orientation["A"]
                    b = float(b_match.group(1)) if b_match else default_orientation["B"]
                    c = float(c_match.group(1)) if c_match else default_orientation["C"]

                    if f_val:
                        current_feedrate = float(f_val.group(1))

                    if x or y or z:
                        point = {
                            "x": float(x.group(1)) if x else 0.0,
                            "y": float(y.group(1)) if y else 0.0,
                            "z": float(z.group(1)) if z else 0.0,
                            "a": a,  # Góc quay quanh trục X của tool_link
                            "b": b,  # Góc quay quanh trục Y của tool_link
                            "c": c,  # Góc quay quanh trục Z của tool_link
                            "feedrate": current_feedrate
                        }

                        # Chuyển đổi góc Euler sang Quaternion
                        quat = quaternion_from_euler(a, b, c)

                        pose_stamped = PoseStamped()
                        pose_stamped.header.frame_id = "base_link"  # Sử dụng base_link làm frame tham chiếu
                        pose_stamped.pose.position.x = point["x"] / 1000.0
                        pose_stamped.pose.position.y = point["y"] / 1000.0
                        pose_stamped.pose.position.z = point["z"] / 1000.0
                        pose_stamped.pose.orientation.x = quat[0]
                        pose_stamped.pose.orientation.y = quat[1]
                        pose_stamped.pose.orientation.z = quat[2]
                        pose_stamped.pose.orientation.w = quat[3]

                        # 👉 Không cần offset tool_offset_z nữa nếu tool_link đã bao gồm chiều dài tool
                        # pose_stamped = offset_pose_z(pose_stamped, tool_offset_z)

                        joint_positions = self.call_ik(pose_stamped)
                        if joint_positions and len(joint_positions) == expected_joint_count:
                            point["joint_positions"] = list(joint_positions)
                            point["quaternion"] = {
                                "x": quat[0],
                                "y": quat[1],
                                "z": quat[2],
                                "w": quat[3]
                            }
                            tcp_path.append(point)
                            self.get_logger().info(f'Added point: X={point["x"]} Y={point["y"]} Z={point["z"]} | A={a} B={b} C={c}')
                        else:
                            self.get_logger().warn(
                                f'Skipped point: X={point["x"]} Y={point["y"]} Z={point["z"]} '
                                f'because IK returned {len(joint_positions) if joint_positions else "None"} joints.'
                            )

        with open(output_path, "w") as f:
            json.dump(tcp_path, f, indent=2)
        self.get_logger().info(f'Done. Converted {len(tcp_path)} valid points from {gcode_path} to {output_path}')

def main():
    rclpy.init()
    converter = GCodeToJsonConverter()
    converter.convert_gcode_to_json(
        "/home/duy/ws_moveit2/src/json_path_executor/json_path_executor/path.nc",
        "/home/duy/ws_moveit2/src/json_path_executor/json_path_executor/tcp_path.json"
    )
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()