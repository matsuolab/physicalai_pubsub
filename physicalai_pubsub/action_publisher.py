#!/usr/bin/env python3

"""
Robot Action Publisher Module

Publishes joint trajectory commands to control the robot's movement via
'/crane_plus_arm_controller/joint_trajectory' topic.

Example:
    ros2 run physicalai_pubsub action_publisher
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class RobotActionPublisher(Node):
    """
    ROS2 node that publishes joint trajectory commands for robot control.
    """

    def __init__(self):
        """Initialize publisher for joint trajectory commands."""
        super().__init__('robot_action_publisher')
        self.publisher = self.create_publisher(
            JointTrajectory,
            'crane_plus_arm_controller/joint_trajectory',
            10)
        self.get_logger().info('Robot Action Publisher node has been started')

    def move_to_position(self, positions, time_from_start=2.0):
        """
        Move robot joints to specified positions.

        Args:
            positions: List of joint positions in radians [joint1, joint2, joint3, joint4]
            time_from_start: Time to reach positions in seconds (default: 2.0)
        """
        msg = JointTrajectory()
        msg.joint_names = [
            'crane_plus_joint1',
            'crane_plus_joint2',
            'crane_plus_joint3',
            'crane_plus_joint4'
        ]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.accelerations = [0.0] * len(positions)
        point.time_from_start = Duration(sec=int(time_from_start), nanosec=int((time_from_start % 1) * 1e9))
        
        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published trajectory command: {positions}')

def main():
    """
    Execute a sequence of movements:
    1. Initial position (0.0)
    2. Intermediate position
    3. Return to initial position
    """
    rclpy.init()
    robot_action_publisher = RobotActionPublisher()
    
    try:
        # Move to initial position
        robot_action_publisher.move_to_position([0.0, 0.0, 0.0, 0.0])
        time.sleep(3.0)  # Wait for movement to complete
        
        # Move to a different position
        robot_action_publisher.move_to_position([0.5, 0.3, -0.2, 0.1])
        time.sleep(3.0)
        
        # Return to initial position
        robot_action_publisher.move_to_position([0.0, 0.0, 0.0, 0.0])
        time.sleep(3.0)  # Wait for final movement to complete
        
        robot_action_publisher.get_logger().info('All movements completed. Exiting...')
    except KeyboardInterrupt:
        robot_action_publisher.get_logger().info('Movement interrupted by user')
    finally:
        robot_action_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
