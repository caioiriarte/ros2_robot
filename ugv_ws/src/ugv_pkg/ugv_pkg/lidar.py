from sensor_msgs.msg import LaserScan
from rclpy.node import Node
import mujoco
import rclpy


class LidarPublisher(Node):
    def __init__(self, model, data):
        super().__init__('lidar_publisher')
        self.model = model
        self.data = data
        self.publisher = self.create_publisher(LaserScan, '/lidar_scan', 10)
        self.timer = self.create_timer(0.1, self.publish_data)

    def publish_data(self):
        """Obtiene datos del LIDAR y los publica."""
        # Realizar un paso de simulación
        mujoco.mj_step(self.model, self.data)

        # Leer datos del LIDAR
        lidar_data = []
        for i, sensor_name in enumerate(self.model.sensor_names):
            if "lidar_ray" in sensor_name:
                lidar_data.append(self.data.sensordata[i])

        # Publicar los datos como un mensaje LaserScan
        self.publish_scan(lidar_data)

    def publish_scan(self, data):
        """Publica los datos LIDAR en formato LaserScan."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_site'
        scan.angle_min = -3.14 / 2  # -90°
        scan.angle_max = 3.14 / 2   # 90°
        scan.angle_increment = 3.14 / len(data)  # Incremento según cantidad de rayos
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = data
        scan.intensities = []  # Si no tienes intensidad, deja vacío

        self.publisher.publish(scan)
        self.get_logger().info("Published LIDAR scan")


def main(args=None):
    # Inicializar ROS 2
    rclpy.init(args=args)

    # Ruta del modelo de MuJoCo (ajusta según tu configuración)
    model_path = 'path_to_your_model.xml'

    try:
        # Cargar el modelo y datos de MuJoCo
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)

        # Crear y ejecutar el nodo LidarPublisher
        lidar_publisher = LidarPublisher(model, data)
        rclpy.spin(lidar_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Finalizar ROS 2
        rclpy.shutdown()


if __name__ == '__main__':
    main()
