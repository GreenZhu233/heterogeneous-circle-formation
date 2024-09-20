from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xacro

def generate_launch_description():
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

    # publish xml
    xacro_file = os.path.join(
        FindPackageShare('quadrotor_formation').find('quadrotor_formation'),
        'urdf',
        'quadrotor.xacro'
    )
    doc = xacro.process_file(xacro_file, mappings={'namespace':'quad_0'})
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace='quad_0',
        parameters=[{'use_sim_time': True, 'robot_description': doc.toxml()}]
    )

    # spawn quadrotor
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'quad_0',
            '-topic', 'quad_0/robot_description',
            '-x', '5',
            '-y', '0',
            '-z', '0.3',
            '-Y', '0.0'
        ]
    )

    # start quadrotor position control server
    position_control_server = Node(
        package='quadrotor_control',
        executable='position_control_server',
        output='screen',
        namespace='quad_0',
        parameters=[{'gravity': 1.316*9.8,
                     'odom_topic': '/quad_0/odom',
                     'force_topic': '/quad_0/gazebo_ros_force',
                     'action_service_name': 'position_control_service',}]
    )

    ld.add_action(gazebo_world)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)
    ld.add_action(position_control_server)

    return ld