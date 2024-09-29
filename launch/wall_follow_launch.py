from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    kp_arg = DeclareLaunchArgument('kp', default_value='0.0')
    kd_arg = DeclareLaunchArgument('kd', default_value='0.0')
    ki_arg = DeclareLaunchArgument('ki', default_value='0.0')
    speed_arg = DeclareLaunchArgument('speed', default_value='0.0')

    # Define launch configurations after declaration
    kp = LaunchConfiguration('kp')
    kd = LaunchConfiguration('kd')
    ki = LaunchConfiguration('ki')
    speed = LaunchConfiguration('speed')

    # Define the node
    wall_node = Node(
        package='wall_follow',
        executable='wall_follow_node',
        name='wall_follow',
        output='screen',
        parameters=[{
            'kp': kp,
            'kd': kd,
            'ki': ki,
            'speed': speed
        }]
    )

    # Return the launch description
    return LaunchDescription([
        kp_arg,
        kd_arg,
        ki_arg,
        speed_arg,
        wall_node
    ])
