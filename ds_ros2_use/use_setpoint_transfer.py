import rclpy
from rclpy.node import Node

# Import correct msg
from std_msgs.msg import String
from ds_msgs.msg import TrajectorySetpoint

class UseSetpointTransfer(Node):

    def __init__(self):
        # Init subscriber and
        super().__init__('use_setpoint_subscriber')
        super().__init__('use_setpoint_publisher')

        self.subscription = self.create_subscription(TrajectorySetpoint, 'bs_use_setpoint', self.recv_setpoints, 10)
        self.publisher_ = self.create_publisher(TrajectorySetpoint, 'use_drone_setpoint', 10)
        
        self.i = 0

    def recv_setpoints(self, msg):
        self.publisher_.publish(msg)
        self.i += 1
        self.get_logger().info('Transfered TrajectorySetpoint: %d' % self.i)


def main(args=None):
    rclpy.init(args=args)

    use_setpoint_transfer = UseSetpointTransfer()

    rclpy.spin(use_setpoint_transfer)

    use_setpoint_transfer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
