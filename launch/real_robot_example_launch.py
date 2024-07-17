"""
Run this script in a different terminal window or tab. Be ready to close this, as this activates the real robot if the
connection is successful.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='sas_robot_driver_kuka',
            executable='sas_robot_driver_kuka_node',
            name='kuka_1',
            parameters=[{
                "robot_name": "kuka_1",
                "joint_limits_min": [-170.0, -120.0, -170.0, -120.0, -170.0, -120.0, -175.0],
                "joint_limits_max": [170.0, 120.0, 170.0, 120.0, 170.0, 120.0, 175.0],
                "thread_sampling_time_sec": 0.001
            }]
        ),

    ])
