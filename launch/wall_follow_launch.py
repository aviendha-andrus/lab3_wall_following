#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

def generate_launch_description():
	kp_arg = DeclareLaunchArgument('kp', default_value='0.0')
	kd_arg = DeclareLaunchArgument('kd', default_value='0.0')
	ki_arg = DeclareLaunchArgument('ki', default_value='0.0')
	speed_arg = DeclareLaunchArgument('speed', default_value='0.0')
	dist_arg = DeclareLaunchArgument('dist', default_value='0.0')
	# theta_arg = DeclareLaunchArgument('theta', default_value='0.0')
	# look_arg = DeclareLaunchArgument('look', default_value='0.0')

	kp = LaunchConfiguration('kp')
	kd = LaunchConfiguration('kd')
	ki = LaunchConfiguration('ki')
	speed = LaunchConfiguration('speed')
	dist = LaunchConfiguration('dist')
	# theta = LaunchConfiguration('speed')
	# look = LaunchConfiguration('dist')

	wall_node = Node(
		package='wall_follow',
		executable='wall_follow_node.py',
		name='wall_follow_node',
		# name=LaunchConfiguration('student'),
		output='screen',
		parameters=[
			{'kp': kp},
			{'kd': kd},
			{'ki': ki},
			{'speed': speed},
			{'dist': dist},
			# {'theta': theta},
			# {'look': look},
		]
    )

	return LaunchDescription([
        wall_node,
		kp_arg,
		kd_arg,
		ki_arg,
		speed_arg, 
		dist_arg,
		# theta_arg, 
		# look_arg,
	])




# #!/usr/bin/env python3
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='wall_follow',
#             executable='wall_follow_node.py',
#             name='wall_follow_node'
#         ),
#     ])