from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. RealSense Camera Node
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'align_depth': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
        }.items()
    )

    # 2. Visual Odometry
    odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'camera_link',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'approx_sync': True,
            'approx_sync_max_interval': 0.2,
            'qos_image': 2,      # 2 = Best Effort
            'qos_depth': 2,
            'qos_camera_info': 2
        }],
        arguments=['--Odom/MinInliers', '10', '--OdomF2M/BundleAdjustment', '0', '--Odom/ResetCountdown', '1'],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image', '/camera/camera/depth/image_rect_raw'), # Fallback to rect_raw if aligned is missing
            ('odom', '/odom')
        ]
    )

    # 3. RTAB-Map (RGB-D Configuration)
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'camera_link',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'publish_image': True,
            'approx_sync': True,
            'approx_sync_max_interval': 0.1,
            'qos_image': 2,
            'qos_depth': 2,
            'qos_camera_info': 2,
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false'
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image', '/camera/camera/depth/image_rect_raw'),
            ('odom', '/odom')
        ]
    )

    # 4. Our New Bridge Node
    bridge_node = Node(
        package='quest3carv',
        executable='rtabmap_bridge',
        name='rtabmap_bridge',
        output='screen'
    )

    # 5. Incremental Free-Space Carver (C++)
    carving_node = Node(
        package='quest3carv_cpp',
        executable='carving_node',
        name='carving_node',
        output='screen'
    )

    # 6. Mesh Saver
    saver_node = Node(
        package='quest3carv',
        executable='saver',
        name='mesh_saver',
        output='screen'
    )

    # 7. RTAB-Map Visualization (GUI)
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'frame_id': 'camera_link',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'qos_image': 2,
            'qos_depth': 2,
            'qos_camera_info': 2
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image', '/camera/camera/depth/image_rect_raw'),
            ('odom', '/odom')
        ]
    )

    # 8. RViz2 for Carving Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        realsense_launch,
        odom_node,
        rtabmap_node,
        bridge_node,
        carving_node,
        saver_node,
        rtabmap_viz_node,
        rviz_node
    ])
