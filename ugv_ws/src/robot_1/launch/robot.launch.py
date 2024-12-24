from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Obtener el directorio de tu paquete y el archivo URDF
    pkg_robot_1 = get_package_share_directory('robot_1')

    # Declarar el argumento para el archivo del mundo
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value='empty.world',
        description='Mundo para cargar en Gazebo'
    )

    # Lanzar la simulación de Gazebo
    ld = LaunchDescription([

        declare_world_file_cmd,

        # Lanzar el servidor de Gazebo
        Node(
            package='gazebo_ros',
            executable='/usr/bin/gzserver',
            name='gzserver',
            output='screen',
            arguments=[LaunchConfiguration('world_file')]
        ),

        # Lanzar el cliente de Gazebo (gzclient) para la interfaz gráfica
        Node(
            package='gazebo_ros',
            executable='/usr/bin/gzclient',
            name='gzclient',
            output='screen',
        ),

        # Nodo para spawnear el robot en Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-entity', 'robot_1',  # Nombre del robot
                '-urdf', 'true',  # Especifica que el archivo es URDF
                '-file', PathJoinSubstitution([pkg_robot_1, 'urdf', 'robot_1.urdf']),  # Ruta al archivo URDF
                '-x', '0',  # Posición X
                '-y', '0',  # Posición Y
                '-z', '0.05',  # Posición Z
                '-R', '0',  # Rotación en Roll
                '-P', '0',  # Rotación en Pitch
                '-Y', '0',  # Rotación en Yaw
            ]
        ),
    ])

    return ld