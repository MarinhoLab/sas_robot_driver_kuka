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
            name='sas_robot_driver_kuka_coppeliasim_only',
            parameters=[{
                "robot_driver_client_names": ["dummy_kuka_1"],
                "use_real_robot": True,
                "use_coppeliasim": True,
                "vrep_robot_joint_names": joint_names,
                "vrep_ip": "127.0.0.1",
                "vrep_port": 19997,
                "vrep_dynamically_enabled": True,
                "override_joint_limits_with_robot_parameter_file": False,
                "thread_sampling_time_sec": 0.001
            }]
        ),
        Node(
            package='sas_robot_driver',
            executable='sas_robot_driver_ros_example',
            name='dummy_kuka_1',
            parameters=[{
                "robot_name": "dummy_kuka_1",
                "joint_limits_min": [-170.0, -120.0, -170.0, -120.0, -170.0, -120.0, -175.0],
                "joint_limits_max": [170.0, 120.0, 170.0, 120.0, 170.0, 120.0, 175.0],
                "initial_joint_positions": [0., 0., 0., 0., 0., 0., 0.],
                "thread_sampling_time_sec": 0.001
            }]
        ),
    ])