from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='json_path_executor',
            executable='dao',  # Đã đúng với entry point
            name='dao_simulator_node',
            output='screen'
        ),
        Node(
            package='json_path_executor',
            executable='tcp',  # Đã đúng với entry point
            name='tcp_marker_debugger_node',
            output='screen'
        ),
    ])
