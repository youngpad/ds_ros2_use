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

        # Get namespace this node is started in
        self.my_namespace = super().get_namespace()

        # Trajectory setpoint transfer subscriber and publishers
        self.trajectory_subscriber_ = self.create_subscription(TrajectorySetpoint, 'bs_use_setpoint', self.recv_setpoints, 10)
        self.trajectory_publishers = [0]
        for drone in range (1,10):
            self.trajectory_publishers.append(self.create_publisher(TrajectorySetpoint, self.my_namespace + '/drone_0' + str(drone) + '/use_drone_setpoint', 10))
        self.trajectory_publishers.append(self.create_publisher(TrajectorySetpoint, self.my_namespace + '/drone_10' + '/use_drone_setpoint', 10))


        # Drone control transfer subscriber and publishers
        self.droneControl_subscriber = self.create_subscription(DroneControl, 'bs_use_control', self.recv_control, 10)
        self.droneControl_publishers = [0]
        for drone in range (1,10):
            self.droneControl_publishers.append(self.create_publisher(DroneControl, self.my_namespace + '/drone_0' + str(drone) + '/use_drone_control', 10))
        self.droneControl_publishers.append(self.create_publisher(DroneControl, self.my_namespace + '/drone_10' + '/use_drone_control', 10))

        # Number of messages sent variables
        self.i_trajectory = 0
        self.i_droneControl = 0


    def recv_setpoints(self, msg):
        # Transfer setpoint to correct drone
        if (msg.drone == 999):
            for publisher in self.trajectory_publishers:
                if (publisher != 0):
                    publisher.publish(msg)

        elif (msg.drone > 0 and msg.drone < 11):
            self.trajectory_publishers[msg.drone].publish(msg)

        # Send log
        self.i_trajectory += 1
        self.get_logger().info('Transfered TrajectorySetpoint: %d' % self.i_trajectory)


    def recv_control(self, msg):
        # Transfer control to correct drone
        if (msg.drone == 999):
            for publisher in self.droneControl_publishers:
                if (publisher != 0):
                    publisher.publish(msg)

        elif (msg.drone > 0 and msg.drone < 11):
            self.droneControl_publishers[msg.drone].publish(msg)

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
