import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process xacro
    pkg_path = get_package_share_directory('asmr_ros')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()
    params = {'robot_description': robot_desc, 'use_sim_time': use_sim_time}

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[params],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        rsp_node
    ])