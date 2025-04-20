from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_pkg = get_package_share_directory('kuka_kr210_r2700_description')
    urdf_path = os.path.join(description_pkg, 'urdf', 'kuka_kr210_r2700.urdf')
    rviz_config_path = os.path.join(description_pkg, 'rviz', 'kuka_kr210_r2700.rviz')  # File này bạn cần tạo thêm

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation/Gazebo clock'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path]),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

    #    Node(
          #  package='rviz2',
         #   executable='rviz2',
           # name='rviz2',
           # output='screen',
           # arguments=['-d', rviz_config_path]
       # ),
    ])
