import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_camera = LaunchConfiguration('enable_camera', default='true')
    enable_tf2_web = LaunchConfiguration('enable_tf2_web', default='false')
    
    # ROSBridge WebSocket Server - Essential for web dashboard communication
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0',
            'use_sim_time': use_sim_time,
            'unregister_timeout': 10.0,
            'max_message_size': 10000000,
            'authenticate': False
        }],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # Web Video Server - For camera streaming
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{
            'port': 8080,
            'address': '0.0.0.0',
            'server_threads': 4,
            'ros_threads': 2,
            'default_stream_type': 'mjpeg',
            'use_sim_time': use_sim_time
        }],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(enable_camera)
    )
    
    # Flask Web Dashboard Server
    web_dashboard_server = Node(
        package='flexy_webdashboard',
        executable='webdashboard_node',
        name='web_dashboard_server',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen',
        respawn=False
    )
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        
        DeclareLaunchArgument(
            'enable_camera',
            default_value='true',
            description='Enable web video server for camera streaming'
        ),
        
        DeclareLaunchArgument(
            'enable_tf2_web',
            default_value='false',
            description='Enable TF2 web republisher (requires tf2_web_republisher package)'
        ),
        
        # Launch all nodes
        rosbridge_server,
        web_video_server,
        web_dashboard_server,
    ])