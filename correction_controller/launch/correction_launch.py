from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare configurable launch arguments
    correction_topic_arg = DeclareLaunchArgument(
        'correction_topic',
        default_value='/visual_servo/pose_correction',
        description='Topic to subscribe for correction pose'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/mavic_1/cmd_vel',
        description='Topic to publish velocity commands'
    )

    velocity_arg = DeclareLaunchArgument(
        'velocity',
        default_value='0.10',
        description='Linear velocity magnitude in m/s'
    )

    ang_velocity_arg = DeclareLaunchArgument(
        'angular_velocity',
        default_value='0.10',
        description='Angular velocity in rad/s for yaw rotation'
    )

    return LaunchDescription([
        correction_topic_arg,
        cmd_vel_topic_arg,
        velocity_arg,
        ang_velocity_arg,

        Node(
            package='correction_controller',
            executable='correction_node',
            name='correction_controller_node',
            output='screen',
            remappings=[
                ('/correction_pose', LaunchConfiguration('correction_topic')),
                ('/cmd_vel', LaunchConfiguration('cmd_vel_topic')),
            ],
            parameters=[{
                'velocity': LaunchConfiguration('velocity'),
                'angular_velocity': LaunchConfiguration('angular_velocity'),
            }]
        ),

        Node(
            package='image_transport',
            executable='republish',
            name='image_republish',
            namespace='mavic_1/decoded',
            output='screen',
            arguments=[
                'ffmpeg', 'raw',
                '--ros-args',
                '--remap', 'in/ffmpeg:=/mavic_1/image/ffmpeg',
                '--remap', 'out:=/mavic_1/decoded'
            ],
            parameters=[
                {'ffmpeg_image_transport.decode.threads': 4},
                {'ffmpeg_image_transport.map.hevc_nvenc': 'hevc_nvenc'}
            ]
        ),
    ])
