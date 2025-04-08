
import os
import launch
import pathlib
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.utils import controller_url_prefix
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('ugv_pkg')
    robot_description_path = os.path.join(package_dir, 'resource', 'ugv_robot.urdf')


    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'robot_world.wbt'),
        ros2_supervisor=True
    )


    # Launch the Webots controller for the robot
    robot_driver = WebotsController(
        robot_name='ugv_robot',
        parameters=[
            {'robot_description': robot_description_path}
        ],
        respawn=True
    )


    # Launch the robot_state_publisher to publish the URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_path}]
    )


    return LaunchDescription([

        webots,
        webots._supervisor,
        robot_driver,
        #robot_state_publisher,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])