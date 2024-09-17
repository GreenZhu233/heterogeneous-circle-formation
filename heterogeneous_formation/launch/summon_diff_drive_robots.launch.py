from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import os
import random

def generate_launch_description():
    package_name = "diff_drive_formation"
    pkg = FindPackageShare(package_name).find(package_name)
    xacro_file = os.path.join(pkg, 'urdf', 'diff_drive_robot.xacro')
    ld = LaunchDescription()

    num_of_robots = 5
    x_min = 5
    x_max = 20
    x_range = x_max - x_min
    y_min = -15
    y_max = 0
    y_range = y_max - y_min
    rand = random.sample(range(x_range * y_range), num_of_robots)
    x = [(num // y_range) + x_min for num in rand]
    y = [(num % y_range) + y_min for num in rand]
    spawn_position = [[x[i], y[i], 0.0] for i in range(num_of_robots)]

    for i in range(num_of_robots):
        doc = xacro.process_file(xacro_file, mappings={'namespace':'ddrobot_'+str(i)})
        robot_state_publisher = Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen',
            namespace='ddrobot_'+str(i),
            parameters=[{'use_sim_time': True, 'robot_description': doc.toxml()}]
        )
        ld.add_action(robot_state_publisher)

        random_yaw = random.uniform(0, 2 * 3.14159265359)
        spawn_entity = Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'ddrobot_' + str(i),
                    '-topic', 'ddrobot_' + str(i) + '/robot_description',
                    '-x', str(spawn_position[i][0]),
                    '-y', str(spawn_position[i][1]),
                    '-z', str(spawn_position[i][2]),
                    '-Y', str(random_yaw)],
            output='screen',
            )
        ld.add_action(spawn_entity)

    return ld