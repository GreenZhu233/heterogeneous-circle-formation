from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    world_file = 'heterogeneous_formation.world'
    pkg_share = FindPackageShare('formation_gazebo_worlds').find('formation_gazebo_worlds')
    gazebo_world_path = os.path.join(pkg_share, 'worlds', world_file)

    # add gazebo model path
    model_path = os.path.join(pkg_share, 'models')
    os.environ['GAZEBO_MODEL_PATH'] = model_path

    ld = LaunchDescription()

    # plankton global sim time
    plankton_global_sim_time = Node(
        package='plankton_utils',
        executable='plankton_global_sim_time',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # start gazebo
    gazebo_launch_file = os.path.join(FindPackageShare('gazebo_ros').find('gazebo_ros'), 'launch', 'gazebo.launch.py')
    start_gazebo_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'world': gazebo_world_path}.items()
    )

    # publish world ned frame
    world_ned_publisher = ExecuteProcess(
        cmd=['ros2', 'launch', 'uuv_assistants', 'publish_world_ned_frame.launch'],
        output='screen'
    )

    # publish world models
    world_model_publisher = Node(
        package='uuv_assistants',
        executable='publish_world_models.py',
        parameters=[os.path.join(pkg_share, 'config', 'heterogeneous_formation_world.yaml'), {'use_sim_time': True}],
        output='screen'
    )

    ld.add_action(plankton_global_sim_time)
    ld.add_action(start_gazebo_ros)
    # ld.add_action(world_ned_publisher)
    ld.add_action(world_model_publisher)

    return ld