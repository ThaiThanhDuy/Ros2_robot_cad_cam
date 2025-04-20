from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Node để khởi động controller
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            arguments=['arm_controller', '--controller-manager', '/controller_manager'],
            output='screen',
            parameters=[{'controller_config_file': '/home/duy/ws_moveit2/src/demo/config/ros2_controllers.yaml'}],
        ),
        
        # Delay 2 giây để đảm bảo node được load đúng
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    name='controller_spawner_2',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    output='screen',
                    parameters=[{'controller_config_file': '/home/duy/ws_moveit2/src/demo/config/ros2_controllers.yaml'}],
                ),
            ]
        ),
    ])