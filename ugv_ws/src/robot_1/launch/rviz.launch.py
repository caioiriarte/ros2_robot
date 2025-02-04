from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    ld = LaunchDescription()

    # Rutas del paquete robot_1
    robot_package_path = FindPackageShare('robot_1')
    urdf_path = PathJoinSubstitution([robot_package_path, 'urdf', 'robot_1.urdf'])
    rviz_config_path = PathJoinSubstitution([robot_package_path, 'rviz', 'robot_1_config.rviz'])

    # Argumentos opcionales
    ld.add_action(DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Launch RViz'
    ))
    ld.add_action(DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulated time'
    ))
    

    # Lanzar RViz opcionalmente
    ld.add_action(GroupAction(
        actions=[
            IncludeLaunchDescription(
                PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
                launch_arguments={
                    'urdf_package': 'robot_1',
                    'urdf_package_path': urdf_path,
                    'rviz_config': rviz_config_path,
                }.items(),
            )
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    ))
    
    
    # Definir el espacio de nombres para la cámara
    ld.add_action(PushRosNamespace('camera1'))

    # Nodo static_transform_publisher (para la transformación estática)
    ld.add_action(Node(
        package='tf2_ros',  # Usamos 'tf2_ros' en ROS2 en lugar de 'tf' de ROS1
        executable='static_transform_publisher',
        name='camera_broadcaster',
        arguments=['-15', '0', '15', '1.57', '3.14', '1.1', 'map', 'camera1', '10'],
        output='screen'
    ))

    # Nodo rostopic para publicar la información de la cámara
    ld.add_action(Node(
        package='ros2topic',
        executable='ros2topic',
        name='camera_info',
        arguments=[
            'pub', 'camera_info', 'sensor_msgs/CameraInfo',
            "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: 'camera1'},"
            "height: 480, width: 640, distortion_model: 'plumb_bob',"
            "D: [0],"
            "K: [500.0, 0.0, 320, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],"
            "R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],"
            "P: [500.0, 0.0, 320, 0.0, 0.0, 500, 240, 0.0, 0.0, 0.0, 1.0, 0.0],"
            "binning_x: 0, binning_y: 0,"
            "roi: {x_offset: 0, y_offset: 0, height: 480, width: 640, do_rectify: false}}"
        ],
        output='screen',
        remappings=[('/camera_info', '/camera1/camera_info')]  # Remapear el topic si es necesario
    ))

    return ld
