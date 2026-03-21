"""
wheeltec_semantic_nav.launch.py

Localization + pre-built semantic map + object navigation.

Loads an existing RTAB-Map database (~/.ros/rtabmap.db) for localization
and a saved semantic map (~/.ros/semantic_map.json) as the initial object set.
YOLO continues running so the semantic map is updated with live detections;
objects absent during the session are removed by active disappearance detection.

Prerequisites
-------------
  Run the mapping session first to build both the geometric and semantic maps:
    ros2 launch wheeltec_robot_rtab wheeltec_explore_rtab.launch.py

Usage (2 terminals)
-------------------
  # Terminal 1 — hardware drivers
  ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

  # Terminal 2 — localization + semantic navigation
  conda activate yolo
  source ~/wheeltec_ros2/install/setup.bash
  ros2 launch wheeltec_semantic_map wheeltec_semantic_nav.launch.py

Optional overrides
------------------
  semantic_map_file:=<path>   path to the saved semantic map JSON
  model:=yolov8n.pt           use a smaller YOLO model
  device:=cpu                 CPU inference (slower)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    _default_map_file = os.path.join(
        os.path.expanduser('~'), '.ros', 'semantic_map.json')
    _default_model = os.path.join(
        os.path.expanduser('~'), 'wheeltec_ros2', 'yolov8m.pt')

    # ── Launch arguments ──────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'semantic_map_file', default_value=_default_map_file,
            description='Path to saved semantic map JSON from the mapping session'),
        DeclareLaunchArgument(
            'autosave_period', default_value='30.0',
            description='Seconds between semantic map auto-saves; 0 to disable'),
        DeclareLaunchArgument(
            'model', default_value=_default_model,
            description='YOLO model file'),
        DeclareLaunchArgument(
            'device', default_value='cuda:0',
            description='Inference device (cuda:0 / cpu)'),
        DeclareLaunchArgument(
            'threshold', default_value='0.7',
            description='YOLO detection confidence threshold'),
        DeclareLaunchArgument(
            'min_score', default_value='0.8',
            description='Minimum score to enter semantic map'),
        DeclareLaunchArgument(
            'target_frame', default_value='map'),
        DeclareLaunchArgument(
            'publish_rate', default_value='2.0'),
        DeclareLaunchArgument(
            'merge_distance', default_value='0.6'),
        DeclareLaunchArgument(
            'approach_distance', default_value='0.8'),
        DeclareLaunchArgument(
            'search_timeout', default_value='300.0'),
        DeclareLaunchArgument(
            'search_grace_period', default_value='5.0'),
        DeclareLaunchArgument(
            'web_port', default_value='8080'),
        DeclareLaunchArgument(
            'web_enabled', default_value='true'),
    ]

    # ── 1. RTAB-Map localization (loads ~/.ros/rtabmap.db) ───────────────────
    rtabmap_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wheeltec_robot_rtab'),
                'launch', 'rtabmap_localization.launch.py',
            ])
        ]),
    )

    # ── 2. Nav2 navigation stack ──────────────────────────────────────────────
    # launch_hardware=False  → sensors already started in terminal 1
    # use_rtabmap_localization=True → skip AMCL; RTAB-Map provides map→odom TF
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wheeltec_robot_rtab'),
                'launch', 'wheeltec_nav2_rtab.launch.py',
            ])
        ]),
        launch_arguments={
            'launch_hardware':          'False',
            'use_rtabmap_localization': 'True',
        }.items(),
    )

    # ── 3. YOLO 3D detection ──────────────────────────────────────────────────
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('yolo_bringup'), 'launch', 'yolo.launch.py',
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

    # ── 4. Semantic map node ──────────────────────────────────────────────────
    # load_on_startup=True  → pre-populate map from saved JSON
    # object_timeout=0      → don't expire pre-loaded objects on a timer;
    #                         active disappearance detection still applies
    semantic_map_node = Node(
        package    = 'wheeltec_semantic_map',
        executable = 'semantic_map_node',
        name       = 'semantic_map_node',
        output     = 'screen',
        parameters = [{
            'target_frame':      'map',
            'detection_topic':   '/detections_3d',
            'map_topic':         '/map',
            'object_timeout':    0.0,
            'min_score':         LaunchConfiguration('min_score'),
            'publish_rate':      LaunchConfiguration('publish_rate'),
            'merge_distance':    LaunchConfiguration('merge_distance'),
            'semantic_map_file': LaunchConfiguration('semantic_map_file'),
            'load_on_startup':   True,
            'autosave_period':   LaunchConfiguration('autosave_period'),
        }],
    )

    # ── 5. Object navigator node ──────────────────────────────────────────────
    object_navigator_node = Node(
        package    = 'wheeltec_semantic_map',
        executable = 'object_navigator_node',
        name       = 'object_navigator_node',
        output     = 'screen',
        parameters = [{
            'approach_distance':   LaunchConfiguration('approach_distance'),
            'search_timeout':      LaunchConfiguration('search_timeout'),
            'search_grace_period': LaunchConfiguration('search_grace_period'),
            'web_port':            LaunchConfiguration('web_port'),
            'web_enabled':         LaunchConfiguration('web_enabled'),
        }],
    )

    return LaunchDescription(
        args + [
            rtabmap_localization,
            nav2_launch,
            yolo_launch,
            semantic_map_node,
            object_navigator_node,
        ]
    )
