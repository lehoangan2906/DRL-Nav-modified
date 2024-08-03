from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Define the path to the small_house.world file
    current_dir = os.path.dirname(os.path.realpath(__file__))
    small_house_world_path = os.path.join(current_dir, '..', 'small_house', 'small_house.world')

    # Ensure the file exists
    if not os.path.exists(small_house_world_path):
        raise FileNotFoundError(f"World file not found: {small_house_world_path}")

    # Launch Gazebo with the custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            FindPackageShare('gazebo_ros').find('gazebo_ros'), 'launch', 'gzserver.launch.py')]),
        launch_arguments={'world': small_house_world_path}.items()
    )

    # Launch TurtleBot3 in Gazebo
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo'), 'launch', 'robot_state_publisher.launch.py')]),
        launch_arguments={'use_sim_time': 'true', 'robot_name': 'turtlebot3_waffle'}.items()
    )

    # RViz node
    rviz_config_dir = os.path.join(
        FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo'), 'rviz', 'tb3_simulation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        rviz_node,
    ])

if __name__ == '__main__':
    generate_launch_description()