import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('barista_robot_description'), 'launch'),
        '/start_gazebo.launch.py'])
    )


    ####### DATA INPUT ##########
    xacro_file = "barista_robot_model.urdf.xacro"
    package_description = "barista_robot_description"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "xacro", xacro_file)



    # Position and orientation
    # [X, Y, Z]
    position1 = [0.0, 0.0, 0.2]
    position2 = [2.0, 2.0, 0.2]
    # [Roll, Pitch, Yaw]
    orientation1 = [0.0, 0.0, 0.0]
    orientation2 = [0.0, 0.0, 1.0]


    robot_name_1 = "morty"
    robot_name_2 = "rick"


    robot_state_publisher_node1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        namespace=robot_name_1,
        parameters=[{'frame_prefix': robot_name_1+'/', 'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path, ' include_laser:=True', ' robot_name:=', robot_name_1])}],
        output="screen"
    )

    robot_state_publisher_node2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        namespace=robot_name_2,
        parameters=[{'frame_prefix': robot_name_2+'/', 'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path, ' include_laser:=True', ' robot_name:=', robot_name_2])}],
        output="screen"
    )

    # Spawn ROBOT Set Gazebo
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   robot_name_1,
                   '-x', str(position1[0]), '-y', str(position1[1]
                                                     ), '-z', str(position1[2]),
                   '-R', str(orientation1[0]), '-P', str(orientation1[1]
                                                        ), '-Y', str(orientation1[2]),
                   '-topic', robot_name_1 + '/robot_description'
                   ]
    )

    spawn_robot2 = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    name='spawn_entity',
    output='screen',
    arguments=['-entity',
                robot_name_2,
                '-x', str(position2[0]), '-y', str(position2[1]
                                                    ), '-z', str(position2[2]),
                '-R', str(orientation2[0]), '-P', str(orientation2[1]
                                                    ), '-Y', str(orientation2[2]),
                '-topic', robot_name_2 + '/robot_description'
                ]
    )

    # RVIZ config
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'rviz_config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
        )
    

    st_pub_morty = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_morty_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'morty/odom']
    )

    st_pub_rick = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_turtle_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'rick/odom']
    )




    return LaunchDescription([
        gazebo_world,
        robot_state_publisher_node1,
        robot_state_publisher_node2,
        spawn_robot1,
        spawn_robot2,
        st_pub_morty,
        st_pub_rick,
        rviz_node,

    
    ])


