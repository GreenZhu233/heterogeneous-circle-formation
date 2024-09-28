from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
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

    # gazebo_world = ExecuteProcess(
    #     cmd = ['ros2', 'launch', 'uuv_gazebo_worlds', 'ocean_waves.launch'],
    #     output='screen'
    # )

    # load a quadrotor
    quad_xacro_file = os.path.join(
        FindPackageShare('quadrotor_formation').find('quadrotor_formation'),
        'urdf',
        'quadrotor.xacro'
    )
    quad_doc = xacro.process_file(quad_xacro_file, mappings={'namespace':'quad_0'})
    quad_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace='quad_0',
        parameters=[{'use_sim_time': True, 'robot_description': quad_doc.toxml()}]
    )
    spawn_quad = Node(
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

    # load quadrotor position controller
    quad_pos_controller = Node(
        package='quadrotor_control',
        executable='position_control_server',
        namespace='quad_0',
        parameters=[{'self_odom_topic': 'odom',
                     'tracking_target_odom_topic': '/rexrov_0/pose_gt',
                     'force_topic': 'gazebo_ros_force',
                     'action_service_name': 'position_control_action',
                     'use_sim_time': True},
                     os.path.join(
                         FindPackageShare('quadrotor_control').find('quadrotor_control'),
                         'config',
                         'pids.yaml'
                     )]
    )

    # load a rexrov
    launch_rexrov = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('uuv_descriptions').find('uuv_descriptions'),
                'launch',
                'upload_rexrov_default.launch.py'
            )),
        launch_arguments={
            'x': '-5.0',
            'y': '0.0',
            'z': '-0.7',
            'yaw': '0',
            'mode': 'default',
            'namespace': 'rexrov_0',
            'gazebo_namespace': "''"
        }.items()
    )

    # start uuv thruster manager
    config_file = os.path.join(
        FindPackageShare('uuv_thruster_manager').find('uuv_thruster_manager'), 'config', 'rexrov', 'thruster_manager.yaml'
    )
    tam_file = os.path.join(
        FindPackageShare('uuv_thruster_manager').find('uuv_thruster_manager'), 'config', 'rexrov', 'TAM.yaml'
    )
    uuv_thruster_manager = ExecuteProcess(
        cmd = ['ros2', 'launch', 'uuv_thruster_manager', 'thruster_manager.launch',
               'uuv_name:=rexrov_0', 'model_name:=rexrov_0', 'config_file:=' + config_file,
               'tam_file:=' + tam_file],
    )

    # start uuv acceleration control
    uuv_accel_control = Node(
        package='uuv_control_cascaded_pid',
        executable='acceleration_control.py',
        namespace='rexrov_0',
        parameters=[os.path.join(
            FindPackageShare('uuv_control_cascaded_pid').find('uuv_control_cascaded_pid'),
            'config',
            'rexrov',
            'inertial.yaml'
            )
        ]
    )

    # start uuv velocity control
    uuv_vel_control = Node(
        package='uuv_control_cascaded_pid',
        executable='velocity_control.py',
        namespace='rexrov_0',
        remappings=[('odom', 'pose_gt'),
                    ('cmd_accel', 'cmd_accel')],
        parameters=[os.path.join(
            FindPackageShare('uuv_control_cascaded_pid').find('uuv_control_cascaded_pid'),
            'config',
            'rexrov',
            'vel_pid_control.yaml'),
            {'odom_vel_in_world': False}
        ]
    )

    # start uuv position control
    uuv_pos_control = Node(
        package='uuv_control_cascaded_pid',
        executable='position_control.py',
        namespace='rexrov_0',
        remappings=[('odom', 'pose_gt')],
        parameters=[os.path.join(
            FindPackageShare('uuv_control_cascaded_pid').find('uuv_control_cascaded_pid'),
            'config',
            'rexrov',
            'pos_pid_control.yaml')
        ],
        output='screen'
    )

    # landing behavior
    quadrotor_landing_node = Node(
        package='heterogeneous_formation',
        executable='quadrotor_landing',
        output='screen',
        namespace='quad_0',
        parameters=[{'uuv_odom_topic': '/rexrov_0/pose_gt',
                     'action_service_name': 'position_control_action'}]
    )

    ld.add_action(gazebo_world)
    ld.add_action(quad_robot_state_publisher)
    ld.add_action(quad_pos_controller)
    ld.add_action(spawn_quad)
    ld.add_action(launch_rexrov)
    ld.add_action(uuv_thruster_manager)
    ld.add_action(uuv_accel_control)
    ld.add_action(uuv_vel_control)
    ld.add_action(uuv_pos_control)
    ld.add_action(quadrotor_landing_node)

    return ld