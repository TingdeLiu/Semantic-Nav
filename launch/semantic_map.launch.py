"""
semantic_map.launch.py

Launches ONLY the semantic_map_node + YOLO (with 3D).
Use this when RTAB-Map SLAM/Nav is already running in another terminal.

Usage:
  ros2 launch wheeltec_semantic_map semantic_map.launch.py
  ros2 launch wheeltec_semantic_map semantic_map.launch.py model:=yolov8n.pt device:=cpu
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
        DeclareLaunchArgument('model',         default_value='yolov8m.pt',
                              description='YOLO model file'),
        DeclareLaunchArgument('device',        default_value='cuda:0',
                              description='Inference device (cuda:0 / cpu)'),
        DeclareLaunchArgument('threshold',     default_value='0.7',
                              description='Detection confidence threshold'),
        DeclareLaunchArgument('min_score',     default_value='0.8',
                              description='Minimum score to enter semantic map'),
        DeclareLaunchArgument('target_frame',  default_value='map',
                              description='Map frame for semantic objects'),
        DeclareLaunchArgument('object_timeout', default_value='120.0',
                              description='Seconds before stale objects are removed'),
        DeclareLaunchArgument('publish_rate',  default_value='2.0',
                              description='Semantic map publish rate (Hz)'),
    ]

    # ── YOLO with 3D enabled ─────────────────────────────────────────────────
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
            'target_frame':   LaunchConfiguration('target_frame'),
            'detection_topic': '/detections_3d',
            'map_topic':       '/map',
            'object_timeout':  LaunchConfiguration('object_timeout'),
            'min_score':       LaunchConfiguration('min_score'),
            'publish_rate':    LaunchConfiguration('publish_rate'),
            'marker_lifetime': 3.0,
        }],
    )

    return LaunchDescription(args + [yolo_launch, semantic_map_node])
