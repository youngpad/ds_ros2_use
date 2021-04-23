import rclpy
from rclpy.node import Node

# Import correct msg
from std_msgs.msg import String
from ds_ros2_msgs.msg import TrajectorySetpoint
from ds_ros2_msgs.msg import DroneControl
#from px4_msgs.msg import TrajectorySetpoint

class Transfer(Node):

    def __init__(self):
        # Init subscriber and
        super().__init__('use_transfer')

        # Trajectory setpoint transfer
        self.trajectory_subscriber = self.create_subscription(TrajectorySetpoint, 'bs_use_setpoint', self.recv_setpoints, 10)
        self.trajectory_publisher = self.create_publisher(TrajectorySetpoint, 'use_drone_setpoint', 10)

        # Drone control transfer
        self.droneControl_subscriber = self.create_subscription(DroneControl, 'bs_use_control', self.recv_control, 10)
        self.droneControl_publisher = self.create_publisher(DroneControl, 'use_drone_control', 10)

        self.i = 0

    def recv_setpoints(self, msg):
        self.trajectory_publisher.publish(msg)
        self.i += 1
        self.get_logger().info('Transfered TrajectorySetpoint: %d' % self.i)

    def recv_control(self, msg):
        self.droneControl_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    use_transfer = Transfer()

    rclpy.spin(use_transfer)

    use_transfer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
