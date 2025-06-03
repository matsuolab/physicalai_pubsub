#!/usr/bin/env python3

"""
Robot State Subscriber Module

Subscribes to '/joint_states' and 'time' topic to monitor robot joint positions and countdown.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

class RobotStateSubscriber(Node):
    """
    ROS2 node that subscribes to and processes joint state and time messages.
    """

    def __init__(self):
        super().__init__('robot_state_subscriber')

        # /joint_states サブスクライバ
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        
        # /time サブスクライバ
        self.time_sub = self.create_subscription(  
            String,
            'time',
            self.timer_callback,
            10)

        # 最新のtimer値を保持する変数（初期値）
        self.latest_timer = "未受信"

        self.get_logger().info('Robot State Subscriber node has been started')

    def timer_callback(self, msg):
        self.latest_timer = msg.data  # 最新のtimerを保存
        self.get_logger().info(f'タイマー受信: {msg.data}')

    def joint_state_callback(self, msg):
        # self.get_logger().info('Received joint states:')
        # for i, name in enumerate(msg.name):
        #     pos = msg.position[i] if i < len(msg.position) else 0.0
        #     vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            # self.get_logger().info(
            #     f'  Joint: {name}, Position: {pos:.2f}, Velocity: {vel:.2f}')

        # 最新のタイマー値を表示
        self.get_logger().info(f'  最新のタイマー: {self.latest_timer}')

def main():
    rclpy.init()
    node = RobotStateSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

