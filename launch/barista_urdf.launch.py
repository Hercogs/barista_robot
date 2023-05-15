import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('barista_robot_description'), 'launch'),
        '/start_gazebo.launch.py'])
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('barista_robot_description'), 'launch'),
        '/spawn_robot_description.launch.py'])
    )

    urdf_visualize = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('barista_robot_description'), 'launch'),
        '/urdf_visualize_rviz.launch.py'])
    )


    return LaunchDescription([
    gazebo_world,
    spawn_robot,
    urdf_visualize,
    
    ])
