
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray


class Publisher(Node):

    def __init__(self):
        super().__init__('target')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'target', 2)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float32MultiArray()
        message =[50.0,5.1]
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: dis={},deg={}'.format(message[0],message[1]))


def main(args=None):
    rclpy.init(args=args)

    target = Publisher()

    rclpy.spin(target)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    target.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
