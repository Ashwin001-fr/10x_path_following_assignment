from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to TurtleBot3 Gazebo launch
    tb3_gazebo_launch = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch',
        'turtlebot3_my_world.launch.py'
    )

    return LaunchDescription([
        # Launch TurtleBot3 Gazebo world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_gazebo_launch)
        ),

        # Path Smoother Node
        Node(
            package='path_following',
            executable='path_smoother',
            name='path_smoother',
            output='screen'
        ),

        # Trajectory Generator Node
        Node(
            package='path_following',
            executable='trajectory_generator',
            name='trajectory_generator',
            output='screen'
        ),

        # Follower Node 
        Node(
            package='path_following',
            executable='follower_node',
            name='follower_node',
            output='screen'
        ),
    ])
