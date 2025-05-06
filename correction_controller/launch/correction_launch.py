from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare configurable launch arguments
    correction_topic_arg = DeclareLaunchArgument(
        'correction_topic', default_value='/correction_pose',
        description='Topic to subscribe for correction pose'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/mavic_1/cmd_vel',
        description='Topic to publish velocity commands'
    )

    laser_scan_topic_arg = DeclareLaunchArgument(
        'laser_scan_topic', default_value='/mavic_1/horizontal_obstacle_distance',
        description='Topic to read horizontal obstacle data'
    )

    velocity_arg = DeclareLaunchArgument(
        'velocity', default_value='0.1',
        description='Linear velocity magnitude in m/s'
    )

    ang_velocity_arg = DeclareLaunchArgument(
        'angular_velocity', default_value='0.5',
        description='Angular velocity in rad/s for yaw rotation'
)


    min_obstacle_distance_arg = DeclareLaunchArgument(
        'min_obstacle_distance', default_value='0.5',
        description='Minimum allowed obstacle distance in meters'
    )


    return LaunchDescription([
        correction_topic_arg,
        cmd_vel_topic_arg,
        laser_scan_topic_arg,
        velocity_arg,
        ang_velocity_arg,
        min_obstacle_distance_arg,

        Node(
            package='correction_controller',
            executable='correction_node',
            name='correction_controller_node',
            output='screen',
            remappings=[
                ('/correction_pose', LaunchConfiguration('correction_topic')),
                ('/cmd_vel', LaunchConfiguration('cmd_vel_topic')),
                ('/scan', LaunchConfiguration('laser_scan_topic'))
            ],
            parameters=[{
                'velocity': LaunchConfiguration('velocity'),
                'angular_velocity': LaunchConfiguration('angular_velocity'),
                'min_obstacle_distance': LaunchConfiguration('min_obstacle_distance')
            }]
        )
    ])
