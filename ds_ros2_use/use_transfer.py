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
        self.trajectory_subscriber_ = self.create_subscription(TrajectorySetpoint, 'bs_use_setpoint', self.recv_setpoints, 10)

        self.trajectory_publisher_01_ = self.create_publisher(TrajectorySetpoint, '/drone_01/use_drone_setpoint', 10)
        self.trajectory_publisher_02_ = self.create_publisher(TrajectorySetpoint, '/drone_02/use_drone_setpoint', 10)
        self.trajectory_publisher_03_ = self.create_publisher(TrajectorySetpoint, '/drone_03/use_drone_setpoint', 10)

        # Drone control transfer
        self.droneControl_subscriber = self.create_subscription(DroneControl, 'bs_use_control', self.recv_control, 10)

        self.droneControl_publisher_01_ = self.create_publisher(DroneControl, '/drone_01/use_drone_control', 10)
        self.droneControl_publisher_02_ = self.create_publisher(DroneControl, '/drone_02/use_drone_control', 10)
        self.droneControl_publisher_03_ = self.create_publisher(DroneControl, '/drone_03/use_drone_control', 10)

        # Number of messages sent variables
        self.i_trajectory = 0
        self.i_droneControl = 0


    def recv_setpoints(self, msg):
        # Transfer setpoint to correct drone
        if (msg.drone == 999):
            self.trajectory_publisher_01_.publish(msg)
            self.trajectory_publisher_02_.publish(msg)
            self.trajectory_publisher_03_.publish(msg)
        elif (msg.drone == 1):
            self.trajectory_publisher_01_.publish(msg)
        elif (msg.drone == 2):
            self.trajectory_publisher_02_.publish(msg)
        elif (msg.drone == 3):
            self.trajectory_publisher_03_.publish(msg)

        # Send log
        self.i_trajectory += 1
        self.get_logger().info('Transfered TrajectorySetpoint: %d' % self.i_trajectory)


    def recv_control(self, msg):
        # Transfer control to correct drone
        if (msg.drone == 999):
            self.droneControl_publisher_01_.publish(msg)
            self.droneControl_publisher_02_.publish(msg)
            self.droneControl_publisher_03_.publish(msg)
        elif (msg.drone == 1):
            self.droneControl_publisher_01_.publish(msg)
        elif (msg.drone == 2):
            self.droneControl_publisher_02_.publish(msg)
        elif (msg.drone == 3):
            self.droneControl_publisher_03_.publish(msg)

        # Send log
        self.i_droneControl += 1
        self.get_logger().info('Transfered DroneControl: %d' % self.i_droneControl)


def main(args=None):
    rclpy.init(args=args)

    use_transfer = Transfer()

    rclpy.spin(use_transfer)

    use_transfer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
