import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    pkg_share = get_package_share_directory('nav')

    waypoints_file = os.path.join(pkg_share, 'waypoints', 'waypoint.csv')

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'empty_world.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            os.path.join(pkg_share, 'config', 'sim.rviz')
        ],
        parameters=[{'use_sim_time': True}]
    )

    # Smoothing node
    smoothing = Node(
        package='nav',
        executable='smoothing',
        name='smoothing',
        parameters=[{
            'use_sim_time': True,
            'path_file': waypoints_file,
            'path_resolution': 0.05,
            'frame_id': 'odom',
        }],
        output='screen'
    )

    # MPC tracker
    mpc = Node(
        package='nav',
        executable='mpc_tracker',
        name='mpc_tracker',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rviz,
        smoothing,
        TimerAction(period=3.0, actions=[mpc]),
    ])
