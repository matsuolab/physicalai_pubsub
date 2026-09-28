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
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class RobotActionPublisher(Node):
    """
    ROS2 node that publishes joint trajectory commands for robot control.
    """

    def __init__(self):
        """Initialize publisher for joint trajectory commands."""
        super().__init__('robot_action_publisher')
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            'crane_plus_arm_controller/follow_joint_trajectory')
        
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            'crane_plus_gripper_controller/follow_joint_trajectory')

        self.get_logger().info('Robot Action Publisher node has been started')

    def move_to_position(self, positions, time_from_start=2.0):
        """
        Move robot joints to specified positions.

        Args:
            positions: List of joint positions in radians [joint1, joint2, joint3, joint4]
            time_from_start: Time to reach positions in seconds (default: 2.0)
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = [
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
        
        trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self.arm_client.wait_for_server()
        send_goal_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self,
            send_goal_future
        )
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal was rejected')
            return False

        self.get_logger().info(f'Trajectory started: {positions}')
        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        self.get_logger().info(
            f'Trajectory completed: {positions}'
        )

        return True

    def move_gripper(self,position,time_from_start=2.0):
        
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'crane_plus_joint_hand'
            ]
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities=[0.0]
        point.accelerations = [0.0]

        point.time_from_start = Duration(
            sec=int(time_from_start),
            nanosec=int((time_from_start % 1)*1e9)
        )

        trajectory.points.append(point)
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.gripper_client.wait_for_server()
        send_goal_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self,
            send_goal_future
        )
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal was rejected')
            return False

        self.get_logger().info(
            f'Gripper started: {position}'
        )

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )


        self.get_logger().info(
            f'Gripper completed: {position}'
        )
        
        return True

def main():
    """
    Execute a sequence of movements:
    1. Initial position (0.0)
    2. Intermediate position
    3. Return to initial position
    """
    rclpy.init()
    robot_action_publisher = RobotActionPublisher()
    d2r=math.pi/180.0

    try:
        # Move to initial position 0
        robot_action_publisher.move_gripper(1.0*d2r)

        # Move to initial position 1
        robot_action_publisher.move_to_position([94.0*d2r, 0.0*d2r, 0.0*d2r, 0.0*d2r])
        robot_action_publisher.move_gripper(1.0*d2r)

        # Move to initial position 2
        robot_action_publisher.move_to_position([94.0*d2r, -8.0*d2r, 89.0*d2r, 83.0*d2r])
        robot_action_publisher.move_gripper(1.0*d2r)
               
        # grip the box at initial position
        robot_action_publisher.move_to_position([94.0*d2r, -8.0*d2r, 89.0*d2r, 83.0*d2r])
        robot_action_publisher.move_gripper(9.0*d2r)
        

        # lift up the box at initial position
        robot_action_publisher.move_to_position([94.0*d2r, -8.0*d2r, 77.0*d2r, 96.0*d2r])

        # put the box at initial position
        robot_action_publisher.move_to_position([94.0*d2r, -8.0*d2r, 89.0*d2r, 83.0*d2r])

        # Return to initial position 2
        robot_action_publisher.move_to_position([94.0*d2r, -8.0*d2r, 89.0*d2r, 83.0*d2r])
        robot_action_publisher.move_gripper(1.0*d2r)
        
        # Return to initial position 1
        robot_action_publisher.move_to_position([94.0*d2r, 0.0*d2r, 0.0*d2r, 0.0*d2r])
        robot_action_publisher.move_gripper(1.0*d2r)
        
        # Return to initial position 0
        robot_action_publisher.move_to_position([0.0*d2r, 0.0*d2r, 0.0*d2r, 0.0*d2r])
        robot_action_publisher.move_gripper(0.0*d2r)



        robot_action_publisher.get_logger().info('All movements completed. Exiting...')
    except KeyboardInterrupt:
        robot_action_publisher.get_logger().info('Movement interrupted by user')
    finally:
        robot_action_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
