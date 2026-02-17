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

    pkg_share = get_package_share_directory(package_name)
    
    xacro_file = os.path.join(pkg_share, 'model', 'six_wheel_rover_camera.xacro')

    world_file = os.path.join(pkg_share, 'worlds', 'rover_world.sdf')

    rviz_config_file = os.path.join(pkg_share, 'config', 'six_wheel_rover.rviz')

    robot_description_config = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    rover = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'six_wheel_rover',
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        output='screen'
    )

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

    teleop = Node(
        package='six_wheel_rover',
        executable='teleop',
        name='teleop',
        output='screen',
        prefix='gnome-terminal --' 
    )   

    controller = Node(
        package='six_wheel_rover',
        executable='controller',
        name='controller',
        output='screen',
    )   

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        rsp,
        gazebo,
        rover,
        bridge,
        # teleop, #Uncomment to use teleop 
        controller,
        rviz,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rover,
                on_exit=[load_joint_broadcaster, load_diff_drive_controller],
            )
        )
    ])
