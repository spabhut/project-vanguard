import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'six_wheel_rover'

    # Paths
    pkg_share = get_package_share_directory(package_name)
    
    # 1. Update Xacro Path to the file with the Camera
    xacro_file = os.path.join(pkg_share, 'model', 'six_wheel_rover_camera.xacro')

    # 2. Update World Path to your custom SDF
    # Ensure you added 'world.sdf' to the 'worlds' folder in your package install
    world_file = os.path.join(pkg_share, 'worlds', 'rover_world.sdf')

    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Node: Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    # Node: Gazebo Harmonic (gz_sim)
    # Loading the specific world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Node: Spawn Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'six_wheel_rover',
        ],
        output='screen'
    )

    # Node: ROS GZ Bridge
    # Added Camera topics mappings
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Existing mappings
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    # Node: Controller Spawners
    load_joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    load_diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller"],
    )

    # Node: Teleop Keyboard
    teleop = Node(
        package='six_wheel_rover',
        executable='teleop',
        name='teleop',
        output='screen',
        prefix='gnome-terminal --' 
    )   

    # Node: Custom Controller (if applicable)
    controller = Node(
        package='six_wheel_rover',
        executable='controller',
        name='controller',
        output='screen',
    )   

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        bridge,
        teleop,
        controller,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_broadcaster, load_diff_drive_controller],
            )
        )
    ])
