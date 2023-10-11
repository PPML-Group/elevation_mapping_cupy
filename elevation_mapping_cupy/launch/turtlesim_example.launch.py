import os

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import launch.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # rviz_config = launch.substitutions.LaunchConfiguration('rviz_config', default_value="$(find elevation_mapping_cupy)/rviz/turtle_example.rviz")

    return launch.LaunchDescription([
        
        DeclareLaunchArgument(
            'world_name',
            default_value='worlds/turtlebot3_world.world',
            description='Name of the Gazebo world file to load'
        ),
        # Start gazebo simulation.   
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('turtlebot3_gazebo'), 'launch',
                'turtlebot3_world.launch.py'
            )]),
            # launch_arguments={'world_name': LaunchConfiguration('world_name')}.items(),
        ),
        

        # Publish turtlebot3 tf's.
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen'
        ),

        # Launch elevation mapping node.
        launch_ros.actions.Node(
            package='elevation_mapping_cupy',
            executable='elevation_mapping_node',
            name='elevation_mapping',
            output='screen',
            parameters=[
                {'map_frame': 'odom'},
                {'base_frame': 'base_footprint'},
                {'pointcloud_topics': ['/camera/depth/points']},
                {'initialize_frame_id': ['base_footprint']},
                {'initialize_tf_offset': [0.0]},
            ],
        ),

        # Launch RViz with the demo configuration.
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            # arguments=['-d', rviz_config],
        ),
    ])
