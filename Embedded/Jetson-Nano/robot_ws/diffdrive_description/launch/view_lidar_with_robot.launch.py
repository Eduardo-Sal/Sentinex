from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution 
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
	# Paths
	lidar_launch_path = os.path.join(
		get_package_share_directory('sllidar_ros2'),
		'launch',
		'sllidar_a1_launch.py'
	)

	robot_description_path = os.path.join(
		get_package_share_directory('diffdrive_description'),
		'urdf',
		'my_robot.urdf.xacro'
	)

	ekf_params = os.path.join(get_package_share_directory("robot_localization"),
		'params',
		'ekf_new.yaml')

	# Launch Description
	return LaunchDescription([
		# Include Lidar Launch
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(lidar_launch_path)
		),
		# Robot State Publisher
		Node(
			package='robot_state_publisher',
			executable='robot_state_publisher',
			output='screen',
			parameters=[{
				'robot_description': Command(['xacro ', robot_description_path])
			}]
		),

		# Joint State Publisher (Optional)
		# Node(
		# 	package='joint_state_publisher_gui',
		# 	executable='joint_state_publisher_gui',
		# 	name='joint_state_publisher_gui',
		# 	output='screen'
		# ),

		# Static TF (map -> odom)
		Node(
			package='tf2_ros',
			executable='static_transform_publisher',
			arguments=['0','0','0','0','0','0','map','odom']
		),

		# Rviz Software
		Node(
			package='rviz2',
			executable='rviz2',
			name='rviz2',
			output='screen',
			arguments=['-d',
				PathJoinSubstitution([
					FindPackageShare('diffdrive_description'),
					'rviz',
					'view_robot.rviz'])]
		),

		# Elk Node (robot_localization)
		Node(
			package='robot_localization',
			executable='ekf_node',
			name='ekf_filter_node',
			output='screen',
			parameters=[ekf_params]
		)

	])
