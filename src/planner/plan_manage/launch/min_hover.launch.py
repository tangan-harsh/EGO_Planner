import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
#ros2 topic pub -r 5 /a/drone_0_planning/target_position std_msgs/msg/Float32MultiArray "{data: [30.0, 0.0, 40.0, 0.0]}" 
#ros2 topic pub -r 10 /a/drone_0_planning/control_mode std_msgs/msg/UInt8 "{data: 1}"

def generate_launch_description():
    obj_num = LaunchConfiguration('obj_num', default=10)
    drone_id = LaunchConfiguration('drone_id', default=0)
    namespace = LaunchConfiguration('namespace', default='a')

    map_size_x = LaunchConfiguration('map_size_x', default=50.0)
    map_size_y = LaunchConfiguration('map_size_y', default=25.0)
    map_size_z = LaunchConfiguration('map_size_z', default=2.0)
    odom_topic = LaunchConfiguration('odom_topic', default='Odometry')
    cloud_topic = LaunchConfiguration('cloud_topic', default='cloud_registered')

    obj_num_cmd = DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of objects')
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    namespace_cmd = DeclareLaunchArgument('namespace', default_value=namespace, description='ROS namespace')

    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')
    cloud_topic_cmd = DeclareLaunchArgument('cloud_topic', default_value=cloud_topic, description='PointCloud2 topic')

    pid_controller_node = Node(
        package='pid_control_ego_pkg',
        executable='position_pid_controller',
        name=['drone_', drone_id, '_pid_controller'],
        namespace=namespace,
        output='screen',
        remappings=[
            ('target_position', ['drone_', drone_id, '_planning/target_position']),
            ('pid_position_cmd', ['drone_', drone_id, '_planning/pos_cmd_pid']),
            ('odom', ['drone_', drone_id, '_', 'Odom_high_fre']),
        ],
        parameters=[
            {'control_frequency': 50.0},
            {'map_frame': 'a/camera_init'},
            {'laser_link_frame': 'a/body'}
        ],
    )

    control_mux_node = Node(
        package='control_mux_pkg',
        executable='position_cmd_mux_node',
        name=['drone_', drone_id, '_control_mux'],
        namespace=namespace,
        output='screen',
        remappings=[
            ('planner_cmd', ['drone_', drone_id, '_planning/pos_cmd_planner']),
            ('pid_cmd', ['drone_', drone_id, '_planning/pos_cmd_pid']),
            ('control_mode', ['drone_', drone_id, '_planning/control_mode']),
            ('position_cmd_out', ['drone_', drone_id, '_planning/pos_cmd']),
        ],
    )

    service_for_ego_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/home/intelcup/ws_drone/src/total_launch/launch/service_for_ego.launch.py')
    )

    ld = LaunchDescription()
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(cloud_topic_cmd)
    ld.add_action(obj_num_cmd)
    ld.add_action(drone_id_cmd)
    ld.add_action(namespace_cmd)

    ld.add_action(service_for_ego_include)
    ld.add_action(TimerAction(period=0.6, actions=[pid_controller_node]))
    ld.add_action(TimerAction(period=0.7, actions=[control_mux_node]))

    return ld
