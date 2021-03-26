import rclpy
from rclpy.node import Node

# Import correct msg
from std_msgs.msg import String

class UseSetpointTransfer(Node):

    def __init__(self):
        # Init subscriber and
        super().__init__('setpoint_subscriber')
        super().__init__('setpoint_publisher')

        self.subscription = self.create_subscription(String, 'bs_use_setpoint', self.recv_setpoints, 10)
        self.publisher_ = self.create_publisher(String, 'use_drone_setpoint', 10)

    def recv_setpoints(self, msg):
        self.get_logger().info('I recieved: "%s"' % msg.data)
        self.publisher_.publish(msg)
        self.get_logger().info('Re-transfering: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    use_setpoint_transfer = UseSetpointTransfer()

    rclpy.spin(use_setpoint_transfer)

    use_setpoint_transfer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
