from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joint_names = []
    for i in range(1, 8):
        joint_names.append(f"LBR_iiwa_14_R820_joint{i}")
    print(joint_names)

    return LaunchDescription([
        Node(
            package='sas_robot_driver',
            executable='sas_robot_driver_ros_composer_node',
            output='screen',
            emulate_tty=True,
            name='sas_robot_driver_kuka_real_with_coppeliasim',
            parameters=[{
                "robot_driver_client_names": ["real_kuka_1"],
                "use_real_robot": True,
                "use_coppeliasim": True,
                "vrep_robot_joint_names": joint_names,
                "vrep_ip": "127.0.0.1",
                "vrep_port": 19997,
                "vrep_dynamically_enabled": True,
                "override_joint_limits_with_robot_parameter_file": False,
                "thread_sampling_time_sec": 0.001
            }]
        )
    ])
