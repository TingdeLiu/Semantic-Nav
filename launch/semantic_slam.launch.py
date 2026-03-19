"""
semantic_slam.launch.py  (standard mode — LiDAR + RGB-D)

Full stack:
  wheeltec_robot_rtab  →  RTAB-Map SLAM (LiDAR + RGB-D)
  yolo_bringup         →  YOLO v8 detection + tracking + 3D
  wheeltec_semantic_map →  Semantic map overlay

Usage:
  ros2 launch wheeltec_semantic_map semantic_slam.launch.py
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

    # ── RTAB-Map SLAM (standard: LiDAR + RGB-D) ──────────────────────────────
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wheeltec_robot_rtab'),
                'launch', 'wheeltec_slam_rtab.launch.py'
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
            'use_tracking':      'True',
            'use_3d':            'True',
            'input_image_topic': '/camera/color/image_raw',
            'input_depth_topic': '/camera/depth/image_raw',
            'depth_info_topic':  '/camera/depth/camera_info',
            'target_frame':      'base_link',
            'half':              'True',
        }.items(),
    )

    # ── Semantic Map Node ─────────────────────────────────────────────────────
    semantic_map_node = Node(
        package    = 'wheeltec_semantic_map',
        executable = 'semantic_map_node',
        name       = 'semantic_map_node',
        output     = 'screen',
        parameters = [{
            'target_frame':    LaunchConfiguration('target_frame'),
            'detection_topic': '/detections_3d',
            'map_topic':       '/map',
            'object_timeout':  LaunchConfiguration('object_timeout'),
            'min_score':       LaunchConfiguration('min_score'),
            'publish_rate':    LaunchConfiguration('publish_rate'),
            'merge_distance':  LaunchConfiguration('merge_distance'),
            'marker_lifetime': 3.0,
        }],
    )

    return LaunchDescription(args + [rtabmap_launch, yolo_launch, semantic_map_node])
