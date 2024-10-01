from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    wall_node = Node(
        package='wall_follow',
        executable='wall_follow_node', 
        name='wall_follow_node',
        output='screen'
    )

    return LaunchDescription([
        wall_node
    ])


# # from launch import LaunchDescription
# # from launch_ros.actions import Node # type: ignore
# # from launch.substitutions import Command # type: ignore
# # from launch.substitutions import LaunchConfiguration, PythonExpression # type: ignore
# # from ament_index_python.packages import get_package_share_directory # type: ignore
# # import os
# # import yaml

# # def generate_launch_description():
# #     ld = LaunchDescription()

# #     # allows for command line input
# #     kp = LaunchConfiguration('kp', default=0.0)
# #     kd = LaunchConfiguration('kd', default=0.0)
# #     ki = LaunchConfiguration('ki', default=0.0)
# #     speed = LaunchConfiguration('speed', default=0.8)

# #     wall_node = Node(
# #         package='wall_follow',
# #         executable='wall_follow_node',
# #         name='wall_follow_node',
# #         parameters=[
# #             {'kp': kp},
# #             {'kd': kd}, 
# #             {'ki': ki}, 
# #             {'speed': speed}, 
# #         ],
# #         output='screen',
# #     )

# #     ld.add_action(wall_node)
# #     return ld


# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

# def generate_launch_description():

# 	kp_arg = DeclareLaunchArgument('kp', default_value='0.0')
# 	kd_arg = DeclareLaunchArgument('kd', default_value='0.0')
# 	ki_arg = DeclareLaunchArgument('ki', default_value='0.0')
# 	speed_arg = DeclareLaunchArgument('speed', default_value='0.0')

# 	kp = LaunchConfiguration('kp')
# 	kd = LaunchConfiguration('kd')
# 	ki = LaunchConfiguration('ki')
# 	speed = LaunchConfiguration('speed')
	

# 	wall_node = Node(
# 		package='wall_follow',
# 		executable='wall_follow_node',
# 		name='wall_follow',
# 		output='screen',
# 		parameters=[
# 			{'kp': kp},
# 			{'kd': kd},
# 			{'ki': ki},
# 			{'speed': speed}
# 		]
# 	)

# 	return LaunchDescription([
# 		wall_node,
# 		kp_arg,
# 		kd_arg,
# 		ki_arg,
# 		speed_arg
# 	])


# # # from launch import LaunchDescription
# # # from launch_ros.actions import Node 
# # # from launch.substitutions import Command
# # # from launch.substitutions import LaunchConfiguration, PythonExpression # type: ignore
# # # from ament_index_python.packages import get_package_share_directory 
# # # import os
# # # import yaml

# # # def generate_launch_description():
# # #     ld = LaunchDescription()

# # #     # allows for command line input
# # #     v_parameter = LaunchConfiguration('v', default=0.0)
# # #     d_parameter = LaunchConfiguration('d', default=0.0)

# # # 	kp = LaunchConfiguration('kp', default=0.0)
# # # 	kd = LaunchConfiguration('kd', default=0.0)
# # # 	ki = LaunchConfiguration('ki', default=0.0)
# # # 	speed = LaunchConfiguration('speed', default=0.0)

# # #     talker_node = Node(
# # #         package='lab1_pkg',
# # #         executable='talker',
# # #         name='talker',
# # #         parameters=[
# # #             {'v': v_parameter},
# # #             {'d': d_parameter}
# # #         ],
# # #         output='screen',
# # #     )

# # #     relay_node = Node(
# # #         package='lab1_pkg',
# # #         executable='relay',
# # #         name='relay',
# # #         output='screen',
# # #     )

# # #     ld.add_action(talker_node)
# # #     ld.add_action(relay_node)

# # #     return ld
