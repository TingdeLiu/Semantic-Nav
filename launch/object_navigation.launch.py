"""
Object Navigation Launch File

Starts:
  - semantic_map_node  (builds & publishes semantic map)
  - object_navigator_node  (searches map, triggers exploration, navigates)

Usage:
  ros2 launch wheeltec_semantic_map object_navigation.launch.py

Assumes Nav2 and SLAM (or RTAB-Map) are already running.
To also run exploration, launch m-explore-ros2 separately:
  ros2 launch explore_lite explore.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('target_frame',       default_value='map'),
        DeclareLaunchArgument('detection_topic',    default_value='/detections_3d'),
        DeclareLaunchArgument('map_topic',          default_value='/map'),
        DeclareLaunchArgument('min_score',          default_value='0.8'),
        DeclareLaunchArgument('publish_rate',       default_value='2.0'),
        DeclareLaunchArgument('merge_distance',     default_value='0.6'),
        DeclareLaunchArgument('approach_distance',    default_value='0.8'),
        DeclareLaunchArgument('search_timeout',      default_value='300.0'),
        DeclareLaunchArgument('search_grace_period', default_value='5.0'),
        DeclareLaunchArgument('web_port',            default_value='8080'),
        DeclareLaunchArgument('web_enabled',         default_value='true'),
    ]

    # ── Nodes ─────────────────────────────────────────────────────────────────
    semantic_map_node = Node(
        package    = 'wheeltec_semantic_map',
        executable = 'semantic_map_node',
        name       = 'semantic_map_node',
        output     = 'screen',
        parameters = [{
            'target_frame':    LaunchConfiguration('target_frame'),
            'detection_topic': LaunchConfiguration('detection_topic'),
            'map_topic':       LaunchConfiguration('map_topic'),
            'min_score':       LaunchConfiguration('min_score'),
            'publish_rate':    LaunchConfiguration('publish_rate'),
            'merge_distance':  LaunchConfiguration('merge_distance'),
        }],
    )

    object_navigator_node = Node(
        package    = 'wheeltec_semantic_map',
        executable = 'object_navigator_node',
        name       = 'object_navigator_node',
        output     = 'screen',
        parameters = [{
            'approach_distance':    LaunchConfiguration('approach_distance'),
            'search_timeout':      LaunchConfiguration('search_timeout'),
            'search_grace_period': LaunchConfiguration('search_grace_period'),
            'web_port':            LaunchConfiguration('web_port'),
            'web_enabled':         LaunchConfiguration('web_enabled'),
        }],
    )

    return LaunchDescription(args + [semantic_map_node, object_navigator_node])
