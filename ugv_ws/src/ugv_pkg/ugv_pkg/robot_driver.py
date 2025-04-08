import rclpy
from geometry_msgs.msg import Twist


class RobotDriver:
    def __init__(self, webots_node, properties):
        self.robot = webots_node.robot
        self.timestep = int(self.robot.getBasicTimeStep())
        self.node = rclpy.create_node('robot_driver')

        # Initialize devices
        self.left_front_wheel = self.robot.getDevice("joint_4")
        self.right_front_wheel = self.robot.getDevice("joint_5")
        self.left_rear_wheel = self.robot.getDevice("joint_6")
        self.right_rear_wheel = self.robot.getDevice("joint_7")

        self.left_front_wheel.setPosition(float('inf'))
        self.right_front_wheel.setPosition(float('inf'))
        self.left_rear_wheel.setPosition(float('inf'))
        self.right_rear_wheel.setPosition(float('inf'))

        self.left_front_wheel.setVelocity(0)
        self.right_front_wheel.setVelocity(0)
        self.left_rear_wheel.setVelocity(0)
        self.right_rear_wheel.setVelocity(0)

        # Robot parameters
        self.wheel_radius = 0.1  # Radius of the wheels (meters)
        self.wheel_separation = 0.5  # Distance between left and right wheels (meters)

        # Subscribe to /cmd_vel
        self.node.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)



    def cmd_vel_callback(self, twist):
        # Extract velocity commands
        x = twist.linear.x  # Linear velocity in X (m/s)
        y = twist.linear.y  # Linear velocity in Y (m/s) - ignored for differential drive
        theta = twist.angular.z  # Angular velocity around Z (rad/s)

        # Compute wheel velocities
        v_left = (x - theta * self.wheel_separation / 2) / self.wheel_radius
        v_right = (x + theta * self.wheel_separation / 2) / self.wheel_radius

        # Apply velocities to the motors
        self.left_front_wheel.setVelocity(v_left)
        self.right_front_wheel.setVelocity(v_right)
        self.left_rear_wheel.setVelocity(v_left)
        self.right_rear_wheel.setVelocity(v_right)



    def step(self):
        # Process ROS 2 messages
        rclpy.spin_once(self.node, timeout_sec=0)