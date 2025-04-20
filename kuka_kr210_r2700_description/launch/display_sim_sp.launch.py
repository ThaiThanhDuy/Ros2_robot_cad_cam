from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='json_path_executor',
            executable='phoi',  # Đã đúng với entry point
            name='phoi_simulator_node',
            output='screen'
        ),
        Node(
            package='json_path_executor',
            executable='san_pham',  # Đã đúng với entry point
            name='san_pham_simulation_node',
            output='screen'
        ),
    ])
