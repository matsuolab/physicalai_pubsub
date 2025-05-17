#!/usr/bin/env python3

"""
Robot State Subscriber Module

Subscribes to '/joint_states' topic to monitor robot joint positions and velocities.

Example:
    ros2 run physicalai_pubsub state_subscriber
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class RobotStateSubscriber(Node):
    """
    ROS2 node that subscribes to and processes joint state messages.
    """

    def __init__(self):
        """Initialize subscriber for joint state messages."""
        super().__init__('robot_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Robot State Subscriber node has been started')

    def joint_state_callback(self, msg):
        """
        Process incoming joint state messages.

        Args:
            msg: JointState message containing joint names, positions, and velocities
        """
        self.get_logger().info('Received joint states:')
        for i, name in enumerate(msg.name):
            self.get_logger().info(f'Joint: {name}, Position: {msg.position[i]:.2f}, Velocity: {msg.velocity[i]:.2f}')

def main():
    """
    Initialize and run the joint state subscriber node.
    """
    rclpy.init()
    robot_state_subscriber = RobotStateSubscriber()
    rclpy.spin(robot_state_subscriber)
    robot_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
