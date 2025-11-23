import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_flexy_navigation = get_package_share_directory('flexy_navigation')
    pkg_flexy_description = get_package_share_directory('flexy_description')

    gazebo_models_path, _ = os.path.split(pkg_flexy_navigation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='navigation.rviz',
        description='RViz config file'
    )
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Path to robot description (Xacro)
    robot_description_file = os.path.join(pkg_flexy_description, 'urdf', 'flexy.urdf.xacro')
    robot_description_content = Command(['xacro ', robot_description_file])

    # Path to map and params
    map_file_path = os.path.join(pkg_flexy_navigation, 'maps', 'cafe_map.yaml')
    localization_params_path = os.path.join(pkg_flexy_navigation, 'config', 'amcl_params.yaml')
    navigation_params_path = os.path.join(pkg_flexy_navigation, 'config', 'nav2_params.yaml')

    # Nav2 launch files
    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )
    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    # robot_state_publisher node (ensures /tf and /tf_static are present)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': ParameterValue(robot_description_content, value_type=str),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )   

    # RViz node (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_flexy_navigation, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # AMCL + map server
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': localization_params_path,
            'map': map_file_path,
        }.items()
    )

    # Navigation2
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': navigation_params_path,
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(rviz_launch_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(sim_time_arg)
    ld.add_action(robot_state_publisher_node)  # <-- Ensures /tf is present
    ld.add_action(rviz_node)
    ld.add_action(localization_launch)
    ld.add_action(navigation_launch)
    return ld
