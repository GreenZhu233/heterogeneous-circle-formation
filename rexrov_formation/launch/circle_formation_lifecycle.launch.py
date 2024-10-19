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
    ld.add_action(DeclareLaunchArgument('x_min', default_value='-25', description='Minimum x value for spawn position'))
    ld.add_action(DeclareLaunchArgument('x_max', default_value='25', description='Maximum x value for spawn position'))
    ld.add_action(DeclareLaunchArgument('y_min', default_value='-25', description='Minimum y value for spawn position'))
    ld.add_action(DeclareLaunchArgument('y_max', default_value='25', description='Maximum y value for spawn position'))
    ld.add_action(DeclareLaunchArgument('z', default_value='-1', description='Spawn height'))

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
                'summon_rexrov_with_acceleration_control.launch.py'
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
        name='rexrov_formation_lifecycle_node',
        package='rexrov_formation',
        executable='circle_formation_lifecycle_node',
        output='screen',
        parameters=[
            {'num': LaunchConfiguration('num'),
             'center_x': 0.0,
             'center_y': 0.0,
             'center_z': -0.7,
             'radius': 15.0,
             'use_sim_time': True},
            FindPackageShare('rexrov_formation').find('rexrov_formation') + '/config/formation_param.yaml'
        ]
    )

    # configure formation node
    config_formation = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/rexrov_formation_lifecycle_node', 'configure'],
        output='screen'
    )

    # activate formation node
    activate_formation = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/rexrov_formation_lifecycle_node', 'activate'],
        output='screen'
    )

    ld.add_action(gazebo_world)
    ld.add_action(formation_node)
    ld.add_action(TimerAction(period=3.0, actions=[rexrov_launch]))
    ld.add_action(TimerAction(period=10.0, actions=[config_formation]))
    ld.add_action(TimerAction(period=15.0, actions=[activate_formation]))

    return ld