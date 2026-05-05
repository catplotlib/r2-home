from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('DISPLAY', ':0'),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=['/home/puja/ps4.yaml'],
        ),
        Node(
            package='motor_driver',
            executable='serial_drive_node',
            name='serial_drive_node',
        ),
        Node(
            package='voice_cmd',
            executable='voice_cmd_node',
            name='voice_cmd_node',
        ),
        Node(
            package='voice_cmd',
            executable='display_node',
            name='display_node',
        ),
    ])
