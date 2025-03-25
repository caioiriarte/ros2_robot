
import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('ugv_pkg')
    world_path = PathJoinSubstitution([package_dir, 'worlds', LaunchConfiguration('world')])


    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=True
    )

    robot_description_path = os.path.join(
        package_dir, 'resource', 'ugv_robot.urdf')
    robot_driver = WebotsController(
        robot_name='ugv_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        respawn=True
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='robot_world.wbt',
            description='UGV robot world file'
        ),
        webots,
        webots._supervisor,
        robot_driver,

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