import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from builtin_interfaces.msg import Duration
import time

class TimerActionPublisher(Node):
    """
    ROS2 node that publishes joint trajectory commands for robot control.
    """

    def __init__(self):
        super().__init__('timer_action_publisher')
        self.publisher = self.create_publisher(
            JointTrajectory,
            'crane_plus_arm_controller/joint_trajectory',
            10)
        self.time_pub = self.create_publisher(String, 'time', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 5
        self.get_logger().info('カウントスタート！！！')

    def timer_callback(self):
        time_msg = String()

        if self.count >0:
            time_msg.data = f'カウント{self.count}'
            self.count -= 1
            
        elif self.count == 0:
            
            for i in range(3):
                num = float(i)
                self.move_to_position([i + 1.0, 1.0, 1.0, 1.0])
                time.sleep(2)
                self.move_to_position([i + 1.0, 0.0, 0.0, 0.0])             
                time.sleep(2)
            self.move_to_position([0.0, 0.0, 0.0, 0.0])
            self.count -= 1
            time_msg.data = 'ストップ！！！'
        else:
            return        

        self.time_pub.publish(time_msg)
        self.get_logger().info(f'パブリッシュ: {time_msg.data}')
        

    def move_to_position(self, positions, time_from_start=0.5):
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
        self.get_logger().info(f'Dancing : {positions}')

def main():
    rclpy.init()
    node = TimerActionPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Movement interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()