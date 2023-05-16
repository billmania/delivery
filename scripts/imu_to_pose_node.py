#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu


class ImuToPoseNode(Node):
    """
    Subscribe to the /mavros/imu/data message, get the orientation, and republish it as a
    PoseStamped message. This is useful for comparing the Pixhawk and x150 outputs in rviz2.
    """

    def __init__(self):
        super().__init__('imu_to_pose')

        imu_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.SYSTEM_DEFAULT)

        self._imu_sub = self.create_subscription(Imu, '/mavros/imu/data',
                                                 self.imu_callback, imu_qos)

        self._pose_pub = self.create_publisher(PoseStamped, 'imu_pose', 10)

    def imu_callback(self, imu_msg: Imu):
        msg = PoseStamped()
        msg.header = imu_msg.header
        msg.pose.orientation = imu_msg.orientation
        # Leave msg.pose.position as (0, 0, 0), because we don't have any valid position information.

        # Provide a little offset so that the 2 markers don't stack on top of each other in rviz2.
        msg.pose.position.x += 0.1
        msg.pose.position.z += 0.25

        self._pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuToPoseNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
