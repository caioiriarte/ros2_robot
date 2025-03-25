import rclpy
import cv2
import numpy as np
from rclpy.node import Node


iterator = 0


class RobotDriver(Node):
    def __init__(self,webots_node, properties):
        super().__init__('robot_control_node')

        self.robot = webots_node.robot                          # Get robot object from webots_node
        self.timestep = int(self.robot.getBasicTimeStep())      # Time step of the simulation

        # Camera device
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)               # Enable the camera with a sampling period of self.timestep ms

        # LIDAR device
        self.lidar = self.robot.getDevice("lidar")
        self.lidar.enable(self.timestep)                # Enable the LIDAR with a sampling period of 32 ms
        self.lidar.enablePointCloud()

        # Robot motors
        self.left_front_wheel = self.robot.getDevice("joint_4")
        self.right_front_wheel = self.robot.getDevice("joint_5")
        self.left_rear_wheel = self.robot.getDevice("joint_6")
        self.right_rear_wheel = self.robot.getDevice("joint_7")
        self.camera_motor = self.robot.getDevice("joint_9")

        # Set robot motors' initial velocity control
        self.left_front_wheel.setPosition(float('inf'))
        self.right_front_wheel.setPosition(float('inf'))
        self.left_rear_wheel.setPosition(float('inf'))
        self.right_rear_wheel.setPosition(float('inf'))
        self.camera_motor.setPosition(float('inf'))

        # Timer to simulate periodic control and camera capture
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz


    def timer_callback(self):
        global iterator

        # Capture camera image (fetch image as bytes)
        camera_image = self.camera.getImage()
        image_width = self.camera.getWidth()
        image_height = self.camera.getHeight()

        # Convert the Webots image to a format OpenCV can work with
        # Webots image is in RGB format
        image_array = np.frombuffer(camera_image, dtype=np.uint8).reshape((image_height, image_width, 4))
        # Convert from RGBA (Webots format) to BGR (OpenCV format)
        image_bgr = cv2.cvtColor(image_array, cv2.COLOR_RGBA2BGR)

        # Display the image using OpenCV
        cv2.imshow("Camera Feed", image_bgr)
        
        # Wait for a key press for 1 millisecond to allow the image window to refresh
        cv2.waitKey(1)

        # Wheel control commands (example velocity values for wheels)
        velocity = 1.0  # Example: 1 m/s forward velocity

        # Apply velocity to the wheels
        if iterator < 10:
            self.left_front_wheel.setVelocity(velocity)
            self.right_front_wheel.setVelocity(velocity)
            self.left_rear_wheel.setVelocity(velocity)
            self.right_rear_wheel.setVelocity(velocity)
            self.camera_motor.setVelocity(velocity)

            iterator += 1
        else:
            self.left_front_wheel.setVelocity(-velocity)
            self.right_front_wheel.setVelocity(-velocity)
            self.left_rear_wheel.setVelocity(-velocity)
            self.right_rear_wheel.setVelocity(-velocity)
            self.camera_motor.setVelocity(-velocity)

            iterator += 1

            if iterator > 20:
                iterator = 0

        # Optionally, use LIDAR data to make decisions for robot control or obstacle avoidance
        lidar_data = self.lidar.getRangeImage()  # Assuming LIDAR provides distance readings
        self.get_logger().info(f'LIDAR Data: {lidar_data}')


def main(args=None):
    rclpy.init(args=args)
    robot_control_node = RobotDriver()

    rclpy.spin(robot_control_node)

    robot_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
