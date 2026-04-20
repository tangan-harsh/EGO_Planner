import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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


    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ego_planner'), 'launch', 'advanced_param_to_real.launch.py')
        ),
        launch_arguments={
            'drone_id': drone_id,
            'namespace': namespace,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'obj_num_set': obj_num,
            'camera_pose_topic': 'pcl_render_node/camera_pose',
            'depth_topic': 'pcl_render_node/depth',
            'cloud_topic': cloud_topic,
            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),
            'max_vel': str(1.0),
            'max_acc': str(2.0),
            'planning_horizon': str(7.5),
            'use_distinctive_trajs': 'True',
            'flight_type': str(2),
            'point_num': str(2),
            'point0_x': str(0.3),
            'point0_y': str(0.0),
            'point0_z': str(0.4),
            'point1_x': str(1.3),
            'point1_y': str(0.0),
            'point1_z': str(0.4),
            'point2_x': str(2.0),
            'point2_y': str(0.0),
            'point2_z': str(0.4),
            'point3_x': str(2.0),
            'point3_y': str(0.0),
            'point3_z': str(0.4),
            'point4_x': str(4.5),
            'point4_y': str(0.0),
            'point4_z': str(0.4),
        }.items(),
    )

    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        namespace=namespace,
        output='screen',
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
        ],
        parameters=[{'traj_server/time_forward': 1.0}],
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
    ld.add_action(TimerAction(period=1.0, actions=[advanced_param_include]))
    ld.add_action(TimerAction(period=0.5, actions=[traj_server_node]))

    return ld
