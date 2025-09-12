import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_neo_robotino_sim = get_package_share_directory('neo_robotino_sim')
    pkg_robotino_navigation = get_package_share_directory('robotino_navigation')
    pkg_lidar = get_package_share_directory('sllidar_ros2')
    pkg_real_robotino = get_package_share_directory('real_robotino_pkg')
    pkg_robotino = get_package_share_directory('robotino-serial')

    navigation_parameters = os.path.join(pkg_real_robotino, 'config', 'nav2_realbot_params.yaml')
    localizer_mode_params = os.path.join(pkg_real_robotino, 'config', 'localizer_realbot.yaml')
    mapping_mode_params = os.path.join(pkg_real_robotino, 'config', 'mapping_realbot.yaml')

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_neo_robotino_sim, 'launch', 'robot_description.launch.py')),
        launch_arguments={
            'use_sim_time': 'false'
        }.items(),
    )

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robotino_navigation, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': mapping_mode_params,
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robotino_navigation, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'nav2_params_file': navigation_parameters,
        }.items(),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar, 'launch', 'sllidar_a1_launch.py')),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'lidar_link',
            'serial_baudrate': '115200',
        }.items(),
    )

    robotino_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robotino, 'launch', 'robotino_full.launch.py')),
        launch_arguments={
        }.items(),
    )

    return LaunchDescription([
        robot_description,
        lidar_launch,
        robotino_launch,
        TimerAction(
            period=10.0,  # espera 2 segundos
            actions=[mapping_launch]
        ),
        TimerAction(
            period=15.0,  # espera 2 segundos
            actions=[navigation_launch]
        )
    ])
