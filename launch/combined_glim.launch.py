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
    get_package_share_directory('glim_ros'),
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

    ply_filename_arg = DeclareLaunchArgument(
        "ply_filename",
        default_value="map",
        description="File name of contructed point cloud map, without ext"
    )
    points_topic_arg = DeclareLaunchArgument(
        "points_topic",
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
    dump_on_unload_arg = DeclareLaunchArgument(
        "dump_on_unload",
        default_value="true",
        description="Auto save .ply, true or false"
    )
    
    ply_filename = LaunchConfiguration("ply_filename")
    points_topic = LaunchConfiguration("points_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    use_concat = LaunchConfiguration("use_concat")
    dump_on_unload = LaunchConfiguration("dump_on_unload")
    
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
    
    # GLIM Node
    glim_node = ComposableNode(
        package='glim_ros',
        plugin='glim::GlimROS', # CMakeLists.txt에 등록한 클래스 이름
        name='glim_ros',
        parameters=[
            {
             'imu_topic': imu_topic,
             'points_topic': points_topic,
             'ply_filename': ply_filename,
             'dump_on_unload': dump_on_unload,
            }
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
            glim_node
        ],
        output='screen',
    )

    return LaunchDescription([ply_filename_arg,
                              dump_on_unload_arg,
                              points_topic_arg,
                              imu_topic_arg,
                              use_concat_arg,
                              container])
