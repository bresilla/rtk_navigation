from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('param1', default_value='default_value1', description='Description of param1'),
        DeclareLaunchArgument('param2', default_value='default_value2', description='Description of param2'),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        # ),
        # ExecuteProcess(
        #     cmd=[[
        #         FindExecutable(name='ros2'),
        #         "service call",
        #         "/gps/fix_set",
        #         "example_interfaces/srv/Trigger ",
        #     ]],
        #     shell=True
        # ),
        Node(
            package='rtk_navigation',
            executable='rtk_navigation',
            name='rtk_navigation',
            output='screen',
            parameters=[{'param2': LaunchConfiguration('param2')}],
        ),
        Node(
            package='rtk_navigation',
            executable='path_server',
            name='path_server',
            output='screen',
            parameters=[{'param2': LaunchConfiguration('param2')}],
        ),
    ])
