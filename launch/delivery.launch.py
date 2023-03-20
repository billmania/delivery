import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('delivery'),
        'config',
        'delivery.yaml'
    )

    delivery_node = Node(
        package="delivery",
        executable="delivery_node.py",
        parameters=[config]
    )

    ld.add_action(delivery_node)

    return ld
