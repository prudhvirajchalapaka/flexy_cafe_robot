from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='flexy_butler',
            executable='table_gui',
            name='table_gui',
            output='screen'
        ),
        Node(
            package='flexy_butler',
            executable='cafe_gui',
            name='cafe_gui',
            output='screen'
        ),
    ])
