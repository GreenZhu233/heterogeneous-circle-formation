import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import random

def generate_launch_description():
    package_name = 'quadrotor_formation'
    ld = LaunchDescription()
    pkg = FindPackageShare(package_name).find(package_name)
    xacro_file = os.path.join(pkg, 'urdf', 'quadrotor.xacro')

    num_of_quadrotors = 5
    x_min = 25
    x_max = 40
    x_range = x_max - x_min
    y_min = 0
    y_max = 15
    y_range = y_max - y_min
    rand = random.sample(range(x_range * y_range), num_of_quadrotors)
    x = [(num // y_range) + x_min for num in rand]
    y = [(num % y_range) + y_min for num in rand]
    spawn_position = [[x[i], y[i], 0.3] for i in range(num_of_quadrotors)]

    for i in range(num_of_quadrotors):
        # 启动 robot_state_publisher
        doc = xacro.process_file(xacro_file, mappings={'namespace':'quad_'+str(i)})
        robot_state_publisher = Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen',
            namespace='quad_'+str(i),
            parameters=[{'use_sim_time': True, 'robot_description': doc.toxml()}]
        )
        ld.add_action(robot_state_publisher)

        # 生成四旋翼无人机模型
        spawn_entity = Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'quad_'+str(i),
                    '-topic', 'quad_' + str(i) + '/robot_description',
                    '-x', str(spawn_position[i][0]),
                    '-y', str(spawn_position[i][1]),
                    '-z', str(spawn_position[i][2]),
                    '-Y', '0.0'],
            output='screen',
            )
        ld.add_action(spawn_entity)

    return ld