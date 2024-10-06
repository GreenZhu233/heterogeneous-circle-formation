import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import random

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('num', default_value='5', description='Number of quadrotors'))
    ld.add_action(DeclareLaunchArgument('x_min', default_value='-30', description='Minimum x value for spawn position'))
    ld.add_action(DeclareLaunchArgument('x_max', default_value='-5', description='Maximum x value for spawn position'))
    ld.add_action(DeclareLaunchArgument('y_min', default_value='-15', description='Minimum y value for spawn position'))
    ld.add_action(DeclareLaunchArgument('y_max', default_value='15', description='Maximum y value for spawn position'))
    ld.add_action(DeclareLaunchArgument('z', default_value='-1.0', description='Spawn height'))

    def run_node_action(context: LaunchContext):
        actions = []

        num = int(context.launch_configurations['num'])
        x_min = int(context.launch_configurations['x_min'])
        x_max = int(context.launch_configurations['x_max'])
        y_min = int(context.launch_configurations['y_min'])
        y_max = int(context.launch_configurations['y_max'])
        z = float(context.launch_configurations['z'])
        x_range = x_max - x_min
        y_range = y_max - y_min
        rand = random.sample(range(x_range * y_range // 25), num)
        x = [(num // (y_range // 5)) * 5 + x_min for num in rand]
        y = [(num % (y_range // 5)) * 5 + y_min for num in rand]
        spawn_position = [[x[i], y[i], z] for i in range(num)]

        for i in range(num):
            launch_rexrov = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        FindPackageShare('rexrov_formation').find('rexrov_formation'),
                        'launch',
                        'upload_rexrov_easy.launch.py'
                    )
                ),
                launch_arguments={
                    'x': str(spawn_position[i][0]),
                    'y': str(spawn_position[i][1]),
                    'z': str(spawn_position[i][2]),
                    'namespace': 'rexrov_' + str(i),
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
                    'uuv_name:=rexrov_'+str(i), 'model_name:=rexrov_'+str(i), 'config_file:=' + config_file,
                    'tam_file:=' + tam_file],
            )

            # start uuv acceleration control
            uuv_accel_control = Node(
                package='uuv_control_cascaded_pid',
                executable='acceleration_control.py',
                namespace='rexrov_'+str(i),
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
                namespace='rexrov_'+str(i),
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

            actions.append(launch_rexrov)
            actions.append(uuv_thruster_manager)
            actions.append(uuv_accel_control)
            actions.append(uuv_vel_control)
        return actions

    ld.add_action(OpaqueFunction(function=run_node_action))
    return ld