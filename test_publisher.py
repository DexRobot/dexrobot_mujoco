import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.start_time = time.time()
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Test Publisher Node has been started.')

    def timer_callback(self):
        current_time = time.time() - self.start_time
        sine_wave_position = -(1 + math.sin(current_time)) * np.pi / 4

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['r_f_joint3_2']  # Add more joint names as needed
        msg.position = [sine_wave_position]  # Sine wave position
        msg.velocity = []
        msg.effort = []

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing Joint State: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
