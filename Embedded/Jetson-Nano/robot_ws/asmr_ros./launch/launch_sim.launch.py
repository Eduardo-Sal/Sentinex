import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'asmr_ros'

    # -------------------------------------------------------------------
    # 1. Launch robot_state_publisher
    #    - Publishes TF and joint states from your URDF/XACRO
    #    - Uses simulated time
    # -------------------------------------------------------------------
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        #launch_arguments={'use_sim_time': 'true'}.items()
    )

    # -------------------------------------------------------------------
    # 2. Launch Ignition Gazebo with your empty_world.sdf
    #    - Runs Gazebo simulator with your chosen world
    #    - Starts paused and then runs (-r)
    # -------------------------------------------------------------------
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')
    #     ]),
    #     launch_arguments={
    #         'ign_args': '-r ' + os.path.join(
    #             get_package_share_directory(package_name),
    #             'worlds',
    #             'empty_world.sdf'
    #         )
    #     }.items()
    # )

    # -------------------------------------------------------------------
    # 3. Spawn robot entity into Gazebo
    #    - Uses /robot_description from robot_state_publisher
    #    - Places robot at Z = 1.5 to avoid collision with floor
    # -------------------------------------------------------------------
    # spawn_entity = Node(
    #     package='ros_ign_gazebo',
    #     executable='create',
    #     arguments=['-name', 'my_bot', '-topic', 'robot_description', '-z', '1.5'],
    #     output='screen'
    # )

    # -------------------------------------------------------------------
    # 4. Publish static transform between base_link and laser_frame
    #    - Required if LIDAR is mounted and not publishing its own TF
    # -------------------------------------------------------------------
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_laser_tf_pub',
        arguments=['0.05', '0', '0.025', '0', '0', '0', 'base_link', 'laser_frame']
    )

    # Static transform from odom -> base_footprint
    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_tf_pub',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )

    static_tf_base_footprint_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_link_tf_pub',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )



    # -------------------------------------------------------------------
    # 5. Start RViz with a custom configuration
    #    - Visualizes your robot, LIDAR, and SLAM in real-time
    # -------------------------------------------------------------------
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'view_robot.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # -------------------------------------------------------------------
    # 6. Start sllidar A1 LIDAR driver
    #    - Publishes /scan topic
    #    - Needed for SLAM Toolbox to function
    # -------------------------------------------------------------------
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py')
        ])
    )

    # -------------------------------------------------------------------
    # 7. (Optional) Start SLAM Toolbox in online_async mode
    #    - Builds the map in real-time
    #    - Publishes /map and TF between /map and /odom
    # -------------------------------------------------------------------
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            #'use_sim_time': True
        }],
        remappings=[
            ('/odom', '/odom/unfiltered')
        ]
    )

    # -------------------------------------------------------------------
    # 8. Launch custom joint_state_publisher node
    #    - Publishes joint_states so wheels appear in RViz
    # -------------------------------------------------------------------
    joint_state_publisher_node = Node(
        package='esp32_controller',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )


    # -------------------------------------------------------------------
    # Final launch description
    # All nodes and launch files above will be launched together
    # -------------------------------------------------------------------
    return LaunchDescription([
        rsp,
        # gazebo,
        # spawn_entity,
        static_tf,
        static_tf_base,
        static_tf_base_footprint_to_link,
        sllidar_launch,
        slam_toolbox_node,
        # joint_state_publisher_node,
        rviz_node
    ])