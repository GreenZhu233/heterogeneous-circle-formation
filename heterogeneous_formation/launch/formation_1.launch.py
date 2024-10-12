import launch
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.event_handlers import OnStateTransition
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

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
            os.path.join(
                FindPackageShare('quadrotor_formation').find('quadrotor_formation'),
                'launch',
                'summon_quadrotors.launch.py')
        ),
        launch_arguments={'num': '3', 'x_min': '5', 'x_max': '20', 'y_min': '0', 'y_max': '15', 'z': '0.3'}.items()
    )

    # summon diff_drive_robots
    summon_diff_drive_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('diff_drive_formation').find('diff_drive_formation'),
                'launch',
                'summon_diff_drive_robots.launch.py')
        ),
        launch_arguments={'num': '3', 'x_min': '5', 'x_max': '20', 'y_min': '-15', 'y_max': '0', 'z': '0.3'}.items()
    )

    # summon rexrovs
    summon_rexrovs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('rexrov_formation').find('rexrov_formation'),
                'launch',
                'summon_rexrov_with_acceleration_control.launch.py')
        ),
        launch_arguments={'num': '3', 'x_min': '-30', 'x_max': '-5', 'y_min': '-15', 'y_max': '15', 'z': '-1.0'}.items()
    )

    # start quadrotors formation
    quadrotor_formation = LifecycleNode(
        name='quadrotor_formation_lifecycle_node',
        namespace='/',
        package='quadrotor_formation',
        executable='circle_formation_lifecycle_node',
        output='screen',
        parameters=[{'num': 3, 'center_x': 9.5, 'center_y': 5.5, 'center_z': 4.0, 'use_sim_time': True}]
    )

    # start diff_drive_robots formation
    diff_drive_formation = Node(
        package='diff_drive_formation',
        executable='circle_formation_node',
        output='screen',
        parameters=[{'num': 3, 'center_x': 9.5, 'center_y': -5.5, 'use_sim_time': True}]
    )

    # start rexrovs formation
    rexrov_formation = LifecycleNode(
        name='rexrov_formation_lifecycle_node',
        namespace='/',
        package='rexrov_formation',
        executable='circle_formation_lifecycle_node',
        output='screen',
        parameters=[
            {'num': 3,
             'center_x': -17.5,
             'center_y': 0.0,
             'center_z': -0.7,
             'radius': 10.0,
             'use_sim_time': True},
            FindPackageShare('rexrov_formation').find('rexrov_formation') + '/config/formation_param.yaml'
        ]
    )


    # configure quadrotors formation
    configure_quadrotor_formation = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(quadrotor_formation),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    # activate quadrotors formation
    activate_quadrotor_formation = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=quadrotor_formation,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(quadrotor_formation),
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    # configure rexrovs formation
    configure_rexrov_formation = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(rexrov_formation),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    # activate rexrovs formation
    activate_rexrov_formation = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=rexrov_formation,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(rexrov_formation),
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    ld.add_action(gazebo_world)
    ld.add_action(TimerAction(period=3.0, actions=[
        summon_quadrotors,
        summon_diff_drive_robots,
        summon_rexrovs,
        quadrotor_formation,
        rexrov_formation
    ]))
    ld.add_action(TimerAction(period=10.0, actions=[
        activate_quadrotor_formation,
        activate_rexrov_formation,
    ]))
    ld.add_action(TimerAction(period=14.0, actions=[
        configure_quadrotor_formation,
        configure_rexrov_formation,
        diff_drive_formation
    ]))

    return ld