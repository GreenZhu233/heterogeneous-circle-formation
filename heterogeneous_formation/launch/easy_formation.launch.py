from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    package_name = 'heterogeneous_formation'
    pkg = FindPackageShare(package_name).find(package_name)
    ld = LaunchDescription()

    # start gazebo world
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('formation_gazebo_worlds').find('formation_gazebo_worlds'),
                'launch',
                'heterogeneous_formation_world.launch.py'
            )
        )
    )

    # summon quadrotors
    summon_quadrotors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'summon_quadrotors.launch.py')
        ),
        launch_arguments={'num': '5', 'x_min': '5', 'x_max': '20', 'y_min': '0', 'y_max': '15', 'z': '0.3'}.items()
    )

    # summon diff_drive_robots
    summon_diff_drive_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'summon_diff_drive_robots.launch.py')
        ),
        launch_arguments={'num': '5', 'x_min': '5', 'x_max': '20', 'y_min': '-15', 'y_max': '0', 'z': '0.3'}.items()
    )

    # start quadrotors formation
    quadrotor_formation = Node(
        package='quadrotor_formation',
        executable='circle_formation_node',
        output='screen',
        parameters=[{'num_of_quadrotors': 5, 'center_x': 9.5, 'center_y': 5.5, 'center_z': 4.0}]
    )

    # start diff_drive_robots formation
    diff_drive_formation = Node(
        package='diff_drive_formation',
        executable='circle_formation_node',
        output='screen',
        parameters=[{'num_of_robots': 5, 'center_x': 9.5, 'center_y': -5.5}]
    )

    ld.add_action(gazebo_world)
    ld.add_action(summon_quadrotors)
    ld.add_action(summon_diff_drive_robots)
    ld.add_action(quadrotor_formation)
    ld.add_action(TimerAction(period=3.0, actions=[diff_drive_formation]))

    return ld