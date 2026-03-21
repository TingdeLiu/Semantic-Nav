"""
semantic_slam_pure3d.launch.py  (pure vision mode — RGB-D only, no LiDAR)

Full stack:
  wheeltec_robot_rtab  →  RTAB-Map SLAM pure3d (RGB-D only)
  yolo_bringup         →  YOLO detection + tracking + 3D
  wheeltec_semantic_map →  Semantic map overlay

Usage:
  ros2 launch wheeltec_semantic_map semantic_slam_pure3d.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # ── Arguments ────────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('model',         default_value='yolov8m.pt'),
        DeclareLaunchArgument('device',        default_value='cuda:0'),
        DeclareLaunchArgument('threshold',     default_value='0.7'),
        DeclareLaunchArgument('min_score',     default_value='0.8'),
        DeclareLaunchArgument('target_frame',  default_value='map'),
        DeclareLaunchArgument('object_timeout',  default_value='120.0'),
        DeclareLaunchArgument('publish_rate',   default_value='2.0'),
        DeclareLaunchArgument('merge_distance', default_value='0.6'),
    ]

    # ── RTAB-Map SLAM (pure3d: RGB-D only) ───────────────────────────────────
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wheeltec_robot_rtab'),
                'launch', 'wheeltec_slam_rtab_pure3d.launch.py'
            ])
        ]),
    )

    # ── YOLO with 3D ─────────────────────────────────────────────────────────
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('yolo_bringup'), 'launch', 'yolo.launch.py'
            ])
        ]),
        launch_arguments={
            'model':             LaunchConfiguration('model'),
            'device':            LaunchConfiguration('device'),
            'threshold':         LaunchConfiguration('threshold'),
            'use_tracking':           'True',
            'use_3d':                 'True',
            'input_image_topic':      '/camera/color/image_raw',
            'input_depth_topic':      '/camera/depth/image_raw',
            'input_depth_info_topic': '/camera/depth/camera_info',
            'target_frame':           'base_link',
            'half':                   'True',
            'namespace':              '',
            'image_reliability':      '2',
            'depth_image_reliability': '2',
            'depth_info_reliability':  '2',
        }.items(),
    )

    # ── Semantic Map Node ─────────────────────────────────────────────────────
    semantic_map_node = Node(
        package    = 'wheeltec_semantic_map',
        executable = 'semantic_map_node',
        name       = 'semantic_map_node',
        output     = 'screen',
        parameters = [{
            'target_frame':    'map',
            'detection_topic': '/detections_3d',
            'map_topic':       '/map',
            'object_timeout':  LaunchConfiguration('object_timeout'),
            'min_score':       LaunchConfiguration('min_score'),
            'publish_rate':    LaunchConfiguration('publish_rate'),
            'merge_distance':  LaunchConfiguration('merge_distance'),
        }],
    )

    return LaunchDescription(args + [rtabmap_launch, yolo_launch, semantic_map_node])
