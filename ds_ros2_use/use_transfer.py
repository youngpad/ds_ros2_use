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
        self.trajectory_publisher_04_ = self.create_publisher(TrajectorySetpoint, '/drone_04/use_drone_setpoint', 10)
        self.trajectory_publisher_05_ = self.create_publisher(TrajectorySetpoint, '/drone_05/use_drone_setpoint', 10)
        self.trajectory_publisher_06_ = self.create_publisher(TrajectorySetpoint, '/drone_06/use_drone_setpoint', 10)
        self.trajectory_publisher_07_ = self.create_publisher(TrajectorySetpoint, '/drone_07/use_drone_setpoint', 10)
        self.trajectory_publisher_08_ = self.create_publisher(TrajectorySetpoint, '/drone_08/use_drone_setpoint', 10)
        self.trajectory_publisher_09_ = self.create_publisher(TrajectorySetpoint, '/drone_09/use_drone_setpoint', 10)
        self.trajectory_publisher_10_ = self.create_publisher(TrajectorySetpoint, '/drone_10/use_drone_setpoint', 10)
        

        # Drone control transfer
        self.droneControl_subscriber = self.create_subscription(DroneControl, 'bs_use_control', self.recv_control, 10)

        self.droneControl_publisher_01_ = self.create_publisher(DroneControl, '/drone_01/use_drone_control', 10)
        self.droneControl_publisher_02_ = self.create_publisher(DroneControl, '/drone_02/use_drone_control', 10)
        self.droneControl_publisher_03_ = self.create_publisher(DroneControl, '/drone_03/use_drone_control', 10)
        self.droneControl_publisher_04_ = self.create_publisher(DroneControl, '/drone_04/use_drone_control', 10)
        self.droneControl_publisher_05_ = self.create_publisher(DroneControl, '/drone_05/use_drone_control', 10)
        self.droneControl_publisher_06_ = self.create_publisher(DroneControl, '/drone_06/use_drone_control', 10)
        self.droneControl_publisher_07_ = self.create_publisher(DroneControl, '/drone_07/use_drone_control', 10)
        self.droneControl_publisher_08_ = self.create_publisher(DroneControl, '/drone_08/use_drone_control', 10)
        self.droneControl_publisher_09_ = self.create_publisher(DroneControl, '/drone_09/use_drone_control', 10)
        self.droneControl_publisher_10_ = self.create_publisher(DroneControl, '/drone_10/use_drone_control', 10)
        

        # Number of messages sent variables
        self.i_trajectory = 0
        self.i_droneControl = 0


    def recv_setpoints(self, msg):
        # Transfer setpoint to correct drone
        if (msg.drone == 999):
            self.trajectory_publisher_01_.publish(msg)
            self.trajectory_publisher_02_.publish(msg)
            self.trajectory_publisher_03_.publish(msg)
            self.trajectory_publisher_04_.publish(msg)
            self.trajectory_publisher_05_.publish(msg)
            self.trajectory_publisher_06_.publish(msg)
            self.trajectory_publisher_07_.publish(msg)
            self.trajectory_publisher_08_.publish(msg)
            self.trajectory_publisher_09_.publish(msg)
            self.trajectory_publisher_10_.publish(msg)
        elif (msg.drone == 1):
            self.trajectory_publisher_01_.publish(msg)
        elif (msg.drone == 2):
            self.trajectory_publisher_02_.publish(msg)
        elif (msg.drone == 3):
            self.trajectory_publisher_03_.publish(msg)
        elif (msg.drone == 4):
            self.trajectory_publisher_04_.publish(msg)
        elif (msg.drone == 5):
            self.trajectory_publisher_05_.publish(msg)
        elif (msg.drone == 6):
            self.trajectory_publisher_06_.publish(msg)
        elif (msg.drone == 7):
            self.trajectory_publisher_07_.publish(msg)
        elif (msg.drone == 8):
            self.trajectory_publisher_08_.publish(msg)
        elif (msg.drone == 9):
            self.trajectory_publisher_09_.publish(msg)
        elif (msg.drone == 10):
            self.trajectory_publisher_10_.publish(msg)

        # Send log
        self.i_trajectory += 1
        self.get_logger().info('Transfered TrajectorySetpoint: %d' % self.i_trajectory)


    def recv_control(self, msg):
        # Transfer control to correct drone
        if (msg.drone == 999):
            self.droneControl_publisher_01_.publish(msg)
            self.droneControl_publisher_02_.publish(msg)
            self.droneControl_publisher_03_.publish(msg)
            self.droneControl_publisher_04_.publish(msg)
            self.droneControl_publisher_05_.publish(msg)
            self.droneControl_publisher_06_.publish(msg)
            self.droneControl_publisher_07_.publish(msg)
            self.droneControl_publisher_08_.publish(msg)
            self.droneControl_publisher_09_.publish(msg)
            self.droneControl_publisher_10_.publish(msg)
        elif (msg.drone == 1):
            self.droneControl_publisher_01_.publish(msg)
        elif (msg.drone == 2):
            self.droneControl_publisher_02_.publish(msg)
        elif (msg.drone == 3):
            self.droneControl_publisher_03_.publish(msg)
        elif (msg.drone == 4):
            self.droneControl_publisher_04_.publish(msg)
        elif (msg.drone == 5):
            self.droneControl_publisher_05_.publish(msg)
        elif (msg.drone == 6):
            self.droneControl_publisher_06_.publish(msg)
        elif (msg.drone == 7):
            self.droneControl_publisher_07_.publish(msg)
        elif (msg.drone == 8):
            self.droneControl_publisher_08_.publish(msg)
        elif (msg.drone == 9):
            self.droneControl_publisher_09_.publish(msg)
        elif (msg.drone == 10):
            self.droneControl_publisher_10_.publish(msg)

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
