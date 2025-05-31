#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

def get_key():
    # シンプルなキー入力取得
    try:
        import termios, sys, tty
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(
            JointTrajectory,
            'crane_plus_arm_controller/joint_trajectory',
            10)
        # 初期角度（例：全部ゼロ）
        self.positions = [0.0, 0.0, 0.0, 0.0]
        # 各関節ごとに増減量
        self.step = 0.1
        self.get_logger().info('Keyboard teleop started! [q/a: joint1 +/-, w/s: joint2 +/-, e/d: joint3 +/-, r/f: joint4 +/-]')
        self.publish_position()

    def publish_position(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'crane_plus_joint1',
            'crane_plus_joint2',
            'crane_plus_joint3',
            'crane_plus_joint4'
        ]
        point = JointTrajectoryPoint()
        point.positions = self.positions
        point.time_from_start = Duration(sec=1)
        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {self.positions}")

    def run(self):
        while rclpy.ok():
            key = get_key()
            if key == 'q':
                self.positions[0] += self.step
            elif key == 'a':
                self.positions[0] -= self.step
            elif key == 'w':
                self.positions[1] += self.step
            elif key == 's':
                self.positions[1] -= self.step
            elif key == 'e':
                self.positions[2] += self.step
            elif key == 'd':
                self.positions[2] -= self.step
            elif key == 'r':
                self.positions[3] += self.step
            elif key == 'f':
                self.positions[3] -= self.step
            elif key == '\x03' or key == '\x04':  # Ctrl-C / Ctrl-D
                print("\nExiting...")
                break
            else:
                print("キー操作: q/a:joint1+/-, w/s:joint2+/-, e/d:joint3+/-, r/f:joint4+/-")
                continue
            self.publish_position()

def main():
    rclpy.init()
    teleop = KeyboardTeleop()
    try:
        teleop.run()
    finally:
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
