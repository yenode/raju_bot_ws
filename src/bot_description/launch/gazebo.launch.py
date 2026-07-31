import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'bot_description'
    pkg_share = get_package_share_directory(package_name)
    
    # Process the URDF file
    xacro_file = os.path.join(pkg_share, 'urdf', 'bot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Robot State Publisher Node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {"use_sim_time": True}]
    )
    
    # Launch Gazebo (ros_gz_sim)
    world_path = os.path.join(pkg_share, 'worlds', 'sample_world.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )
    
    # Spawn Entity Node for gz-sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'raju_bot',
                   '-allow_renaming', 'true',
                   '-z', '0.2'],
        output='screen'
    )

    # RViz Node
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'bot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Controller Spawners (from bot_controller)
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('bot_controller'), 'launch', 'controller.launch.py')]),
    )

    # ROS-GZ Bridge (Only clock needed for sim time, ros2_control does the rest directly in ROS)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        rviz_node,
        controller_launch
    ])
