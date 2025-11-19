import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_vision = get_package_share_directory('yolo_detector')
    pkg_real_robotino = get_package_share_directory('real_robotino_pkg')

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_vision, 'launch', 'vision_objects.launch.py')
        ])
    )

    real_robotino_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_real_robotino, 'launch', 'real_bot.launch.py')
        ])
    )

    mission_node = Node(
        package='real_robotino_pkg',
        executable='missionObjectRecog.py',
        name='missionObjectRecog',
        output='screen'
    )

    # (Opcional) iniciar missão 5s após os outros nós
    delayed_mission = TimerAction(
        period=5.0,
        actions=[mission_node]
    )

    return LaunchDescription([
        vision_launch,
        real_robotino_launch,
        delayed_mission  # inicia após delay
    ])
