from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('num', default_value='5', description='Number of quadrotors'))
    ld.add_action(DeclareLaunchArgument('x_min', default_value='-20', description='Minimum x value for spawn position'))
    ld.add_action(DeclareLaunchArgument('x_max', default_value='20', description='Maximum x value for spawn position'))
    ld.add_action(DeclareLaunchArgument('y_min', default_value='-20', description='Minimum y value for spawn position'))
    ld.add_action(DeclareLaunchArgument('y_max', default_value='20', description='Maximum y value for spawn position'))
    ld.add_action(DeclareLaunchArgument('z', default_value='-1.0', description='Spawn height'))

    # start gazebo world
    gazebo_world = ExecuteProcess(
        cmd = ['ros2', 'launch', 'uuv_gazebo_worlds', 'ocean_waves.launch'],
        output='screen'
    )

    # summon rexrovs
    rexrov_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('rexrov_formation').find('rexrov_formation'),
                'launch',
                'summon_rexrov_with_velocity_control.launch.py'
            )
        ),
        launch_arguments={
            'num': LaunchConfiguration('num'),
            'x_min': LaunchConfiguration('x_min'),
            'x_max': LaunchConfiguration('x_max'),
            'y_min': LaunchConfiguration('y_min'),
            'y_max': LaunchConfiguration('y_max'),
            'z': LaunchConfiguration('z')
        }.items()
    )

    # formation_node
    formation_node = Node(
        package='rexrov_formation',
        executable='circle_formation_lifecycle_node',
        output='screen',
        parameters=[
            {'num': LaunchConfiguration('num'),
             'center_x': 0.0,
             'center_y': 0.0,
             'center_z': -0.7,
             'radius': 6.0},
            FindPackageShare('rexrov_formation').find('rexrov_formation') + '/config/formation_param.yaml'
        ]
    )

    ld.add_action(gazebo_world)
    ld.add_action(TimerAction(period=3.0, actions=[rexrov_launch]))
    ld.add_action(TimerAction(period=5.0, actions=[formation_node]))

    return ld