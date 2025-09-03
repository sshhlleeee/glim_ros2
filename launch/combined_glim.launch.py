from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 1    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

livox_config_dir = os.path.join(
    get_package_share_directory('glim_ros2'),
    'config'
    )
user_config_path = os.path.join(livox_config_dir, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]

def generate_launch_description():

    map_dir_arg = DeclareLaunchArgument(
        "map_dir",
        default_value="/root/map/map_default",
        description="Directory of the pcd file, not the path!"
    )
    pc2_topic_arg = DeclareLaunchArgument(
        "pc2_topic",
        default_value="/livox/lidar_front",
        description="pointcloud topic name for lio"
    )
    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="/livox/imu_front",
        description="imu topic name for lio"
    )
    use_concat_arg = DeclareLaunchArgument(
        "use_concat",
        default_value="true",
        description="On/Off pointcloud2 concatnation"
    )
    
    map_dir = LaunchConfiguration("map_dir")
    pc2_topic = LaunchConfiguration("pc2_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    use_concat = LaunchConfiguration("use_concat")
    
    livox_driver_node = ComposableNode(
        package='livox_ros_driver2',
        plugin='livox_ros::DriverNode',
        name='livox_ros_driver',
        parameters=livox_ros2_params,
        remappings=[
            ("/livox/lidar_192.168.0.156", "/livox/lidar_front"),
            ("/livox/imu_192.168.0.156", "/livox/imu_front"),
            ("/livox/lidar_192.168.0.157", "/livox/lidar_back"),
            ("/livox/imu_192.168.0.157", "/livox/imu_back"),
        ]
    )
    
    pointcloud_concat_node = ComposableNode(
        package='pointcloud_concatenate_ros2',
        plugin='pointcloud_concatenate::PointCloudConcatNode',
        name='pointcloud_concatenate_node',
        parameters=[{
            'target_frame': 'livox_frame',
            'clouds': 2,
            'hz': 10.0,
            'use_sim_time': False
        }],
        remappings=[('cloud_in1', '/livox/lidar_front'),
                    ('cloud_in2', '/livox/lidar_back'),
                    ('cloud_out', '/livox/lidar')],
        condition=IfCondition(use_concat)
    )
    
    # 1. base -> imu transform을 위한 노드
    static_tf_imu_node = ComposableNode(
        package='tf2_ros',
        plugin='tf2_ros::StaticTransformBroadcasterNode',
        name='static_tf_publisher_base_to_imu',  # 고유한 이름 부여
        parameters=[{
                'frame_id': 'base',
                'child_frame_id': 'imu',
                'translation.x': 0.37282,
                'translation.y': 0.0,
                'translation.z': 0.12777,
                'rotation.x': 0.0,
                'rotation.y': 0.30071,
                'rotation.z': 0.0,
                'rotation.w': 0.95372
            }]
    )
    
    static_tf_lidar_node = ComposableNode(
        package='tf2_ros',
        plugin='tf2_ros::StaticTransformBroadcasterNode',
        name='static_tf_publisher_base_to_lidar',  # 고유한 이름 부여
        parameters=[{
                'frame_id': 'base',
                'child_frame_id': 'livox_frame',
                'translation.x': 0.37282,
                'translation.y': 0.0,
                'translation.z': 0.12777,
                'rotation.x': 0.0,
                'rotation.y': 0.30071,
                'rotation.z': 0.0,
                'rotation.w': 0.95372
            }]
    )

    # 2. map -> odom transform을 위한 노드
    static_tf_odom_node = ComposableNode(
        package='tf2_ros',
        plugin='tf2_ros::StaticTransformBroadcasterNode',
        name='static_tf_publisher_map_to_odom',  # 고유한 이름 부여
        parameters=[{
                'frame_id': 'map',
                'child_frame_id': 'odom',
                'translation.x': 0.0,
                'translation.y': 0.0,
                'translation.z': 0.0,
                'rotation.x': 0.0,
                'rotation.y': 0.0,
                'rotation.z': 0.0,
                'rotation.w': 1.0
            }]
    )
    
    # LIO3DNode
    lio_node = ComposableNode(
        package='plain_slam_ros2',
        plugin='pslam::LIO3DNode', # CMakeLists.txt에 등록한 클래스 이름
        name='lio_3d_node',
        parameters=[
            lio_config_yaml,
            {'param_files_dir': config_dir,
             'use_as_localizer': True,
             'map_cloud_dir': map_dir,
             'pointcloud_topic': pc2_topic,
             'imu_topic': imu_topic}
        ],
    )
 
    # Composable Node Container
    container = ComposableNodeContainer(
        name='lidar_tf_loc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            livox_driver_node,
            pointcloud_concat_node,
            static_tf_imu_node,
            static_tf_lidar_node,
            static_tf_odom_node,
            lio_node
        ],
        output='screen',
    )

    return LaunchDescription([map_dir_arg,
                              pc2_topic_arg,
                              imu_topic_arg,
                              use_concat_arg,
                              container])
