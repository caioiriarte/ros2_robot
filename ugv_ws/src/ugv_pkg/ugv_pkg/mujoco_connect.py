from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
import mujoco
from mujoco.viewer import launch_passive
import rclpy
from rclpy.node import Node
import transforms3d.euler as euler



class Mujoco_connect(Node):
    def __init__(self, model_path):
        super().__init__('mujoco_model_node')
        
        try:
            # Cargar el modelo de MuJoCo
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
            
            # Lanzar el visor pasivo
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            
            # Creación de publishers
            self.image_publisher = self.create_publisher(Image, '/camera_image', 10)
            self.pose_publisher = self.create_publisher(PoseStamped, '/robot_pose', 10)
            self.laser_publisher = self.create_publisher(LaserScan, '/laser_scan', 10)

            self.bridge = CvBridge()
            
            # Timer para publicar cada 0.5s
            self.timer = self.create_timer(0.5, self.publish_data)
            
        except Exception as e:
            self.get_logger().error(f'Error while loading MuJoCo model: {e}')
            
    def publish_laser_scan(self):
        try:
            laser_msg = LaserScan()
            laser_msg.header.stamp = self.get_clock().now().to_msg()
            laser_msg.header.frame_id = "laser_frame"
            laser_msg.angle_min = -1.57
            laser_msg.angle_max = 1.57
            laser_msg.angle_increment = 0.01
            laser_msg.range_min = 0.1
            laser_msg.range_max = 10.0
            num_points = int((laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment)
            laser_msg.ranges = [np.random.uniform(0.5, 3.0) for _ in range(num_points)]

            self.laser_publisher.publish(laser_msg)
            self.get_logger().info('Published laser scan')
        except Exception as e:
            self.get_logger().error(f'Error while publishing laser scan: {e}')
    
    def publish_image(self):
        try:
            width, height = 640, 480
            
            """
            El ID de la cámara empleada, es el 2. Para obtener otros IDs de otras cámaras
            empleadas en el modelo, usar los siguientes métodos:
            -----------------------------------------------------------------------------
            
            camera_count = self.model.ncam
            self.get_logger().info(f"Number of cameras in model: {camera_count}")

            # Iterar y listar los nombres de las cámaras
            for i in range(camera_count):
                camera_name = self.model.names[self.model.name_camadr[i]:].decode('utf-8')
                self.get_logger().info(f"Camera {i}: {camera_name}")
            """
            
            cam_id = 2      #   Corresponde con el ID de la cámara del modelo mujoco

            # Crear estructuras de visualización
            opt = mujoco.MjvOption()
            pert = mujoco.MjvPerturb()
            scene = mujoco.MjvScene(self.model, maxgeom=1000)
            context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)

            # Configurar la cámara
            cam = mujoco.MjvCamera()
            cam.fixedcamid = cam_id
            cam.type = mujoco.mjtCamera.mjCAMERA_FIXED

            # Actualizar la escena con la cámara seleccionada
            mujoco.mjv_updateScene(self.model, self.data, opt, pert, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)

            # Asegúrate de que el framebuffer se haya actualizado
            # Obtener la imagen directamente del framebuffer de la cámara
            rgb = np.array(scene.camera.rgb, dtype=np.uint8).reshape((height, width, 3))

            # Convertir la imagen a formato de ROS y publicarla
            image_message = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
            self.image_publisher.publish(image_message)

            self.get_logger().info('Published camera image')

        except Exception as e:
            self.get_logger().error(f'Error while publishing image: {e}')


    def publish_pose(self):
        try:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"

            # Extraer la posición del robot desde MuJoCo
            pose_msg.pose.position.x = self.data.qpos[0]
            pose_msg.pose.position.y = self.data.qpos[1]
            pose_msg.pose.position.z = self.data.qpos[2]

            # Extraer la orientación como cuaternión
            quaternion = euler.euler2quat(self.data.qpos[3], self.data.qpos[4], self.data.qpos[5])
            pose_msg.pose.orientation.x = quaternion[1]
            pose_msg.pose.orientation.y = quaternion[2]
            pose_msg.pose.orientation.z = quaternion[3]
            pose_msg.pose.orientation.w = quaternion[0]

            self.pose_publisher.publish(pose_msg)
            self.get_logger().info('Published robot pose')

        except Exception as e:
            self.get_logger().error(f'Error while publishing pose: {e}')
            
    def publish_data(self):
        self.publish_image()
        self.publish_pose()
        self.publish_laser_scan()

def main(args=None):
    rclpy.init(args=args)
    model_path = '/home/caioiriarte/.mujoco/mujoco210/model/ugv_robot.xml'
    mujoco_node = Mujoco_connect(model_path)
    rclpy.spin(mujoco_node)
    mujoco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


