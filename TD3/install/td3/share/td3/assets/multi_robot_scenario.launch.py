from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Find the package share directory for turtlebot3_gazebo
    turtlebot3_gazebo_dir = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    gazebo_ros_dir = FindPackageShare('gazebo_ros').find('gazebo_ros')

    # Launch Gazebo with the TurtleBot3 world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')])
    )

    # RViz node
    rviz_config_dir = os.path.join(turtlebot3_gazebo_dir, 'rviz', 'tb3_simulation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        gazebo,
        rviz_node,
    ])

if __name__ == '__main__':
    generate_launch_description()

