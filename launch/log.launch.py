import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    delivery_dir = get_package_share_directory('delivery')
    mavros_params_file = os.path.join(delivery_dir, 'config', 'mavros.yaml')
    rosbag2_record_qos_file = os.path.join(delivery_dir, 'config', 'rosbag2_record_qos.yaml')

    return LaunchDescription([
        # Translate messages MAV <-> ROS
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[mavros_params_file],
        ),

        # Bag useful topics
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--qos-profile-overrides-path', rosbag2_record_qos_file,
                '--include-hidden-topics',
                '/battery',
                '/diagnostics',
                '/mavros/imu/data',
                '/mavros/imu/data_raw',
                '/mavros/imu/mag',
                '/mavros/imu/temperature_baro',
                '/mavros/state',
                '/rosout',
                '/tf',
                '/tf_static',
            ],
            output='screen',
        ),
    ])
