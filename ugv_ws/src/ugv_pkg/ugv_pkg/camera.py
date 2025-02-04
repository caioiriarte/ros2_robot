from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import mujoco
from mujoco.viewer import launch_passive
import rclpy
from rclpy.node import Node


class CameraPublisher(Node):
    def __init__(self, model, data):
        super().__init__('camera_publisher')
        self.model = model
        self.data = data
        self.publisher = self.create_publisher(Image, '/camera_image', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_image)

    def publish_image(self):
        try:
            # Obtener la imagen de la cámara
            width, height = 640, 480
            rgb = np.zeros((height, width, 3), dtype=np.uint8)
            mujoco.mjr_render(width, height, self.model, self.data, rgb)

            # Convertir y publicar la imagen
            image_message = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
            self.publisher.publish(image_message)
            self.get_logger().info('Published camera image')
        except Exception as e:
            self.get_logger().error(f'Error while publishing image: {e}')


def main(args=None):
    # Inicializar el nodo ROS 2
    rclpy.init(args=args)

    # Ruta del modelo de MuJoCo (actualiza con la ruta de tu modelo)
    model_path = '/home/caioiriarte/.mujoco/mujoco210/model/ugv_robot.xml'

    try:
        # Cargar el modelo de MuJoCo
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)

        # Lanzar el visor pasivo de MuJoCo
        viewer = launch_passive(model, data)

        # Crear y ejecutar el nodo CameraPublisher
        camera_publisher = CameraPublisher(model, data)
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Limpiar recursos
        viewer.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
