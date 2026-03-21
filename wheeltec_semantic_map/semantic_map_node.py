#!/usr/bin/env python3
"""
Semantic Map Node

Integrates YOLO 3D detections with RTAB-Map 2D occupancy grid to build
a persistent 2D semantic map with labeled object positions.

Data flow:
  /yolo/detections_3d (DetectionArray)
        + TF (bbox3d.frame_id → map)
        + /map (OccupancyGrid)
  ──────────────────────────────────
  → /semantic_map/image    (sensor_msgs/Image)               [annotated map image]
  → /semantic_map/objects  (std_msgs/String)                 [JSON object list]
"""

import colorsys
import json
import math
import os
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import tf2_geometry_msgs  # noqa: F401  registers PointStamped transform support
import tf2_ros

from yolo_msgs.msg import DetectionArray

# Minimum footprint size (metres) to keep boxes visible on the map
_MIN_FOOT = 0.3


def class_color(class_name: str) -> Tuple[float, float, float, float]:
    """Return a consistent RGBA color (0–1 range) for a given class name."""
    hue = (hash(class_name) & 0xFFFF) / 0xFFFF
    r, g, b = colorsys.hsv_to_rgb(hue, 0.85, 0.95)
    return (r, g, b, 0.85)


@dataclass
class SemanticObject:
    class_id: int
    class_name: str
    tracking_id: str
    x: float          # map frame (metres)
    y: float
    z: float
    score: float
    last_seen: float  # seconds (monotonic clock)
    foot_w: float = _MIN_FOOT   # footprint lateral width  (bbox x-extent, metres)
    foot_d: float = _MIN_FOOT   # footprint forward depth  (bbox z-extent, metres)
    miss_count: int = 0         # consecutive rescan misses (for grace period)


class SemanticMapNode(Node):
    """Fuses YOLO 3D detections and an OccupancyGrid into a semantic map."""

    def __init__(self) -> None:
        super().__init__('semantic_map_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('detection_topic', '/detections_3d')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('object_timeout', 120.0)  # seconds; 0 = disabled
        self.declare_parameter('min_score', 0.8)
        self.declare_parameter('publish_rate', 2.0)      # Hz
        self.declare_parameter('merge_distance', 0.6)    # metres: spatial dedup radius
        self.declare_parameter('disappear_grace_frames', 3)  # consecutive misses before removal
        # Persistence
        _default_map_file = os.path.join(
            os.path.expanduser('~'), '.ros', 'semantic_map.json')
        self.declare_parameter('semantic_map_file', _default_map_file)
        self.declare_parameter('load_on_startup', False)   # load saved map at startup
        self.declare_parameter('autosave_period', 0.0)     # seconds; 0 = disabled

        self.target_frame       = self.get_parameter('target_frame').value
        detection_topic         = self.get_parameter('detection_topic').value
        map_topic               = self.get_parameter('map_topic').value
        self.object_timeout     = self.get_parameter('object_timeout').value
        self.min_score          = self.get_parameter('min_score').value
        publish_rate            = self.get_parameter('publish_rate').value
        self.merge_distance           = self.get_parameter('merge_distance').value
        self.disappear_grace_frames   = self.get_parameter('disappear_grace_frames').value
        self.semantic_map_file  = self.get_parameter('semantic_map_file').value
        load_on_startup         = self.get_parameter('load_on_startup').value
        autosave_period         = self.get_parameter('autosave_period').value

        # ── TF ──────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── State ───────────────────────────────────────────────────────────
        self.objects: Dict[str, SemanticObject] = {}
        self.map_msg:  Optional[OccupancyGrid]  = None
        self.bridge = CvBridge()
        self._uid = 0          # fallback unique-id counter for un-tracked detections

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(
            DetectionArray, detection_topic, self._on_detections, 10)
        # RTAB-Map publishes /map with TRANSIENT_LOCAL – subscriber must match
        # or ROS2 silently drops the connection and map_msg stays None forever.
        _map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid, map_topic, self._on_map, _map_qos)

        # ── Publishers ──────────────────────────────────────────────────────
        self.image_pub   = self.create_publisher(
            Image, '/semantic_map/image', 10)
        self.objects_pub = self.create_publisher(
            String, '/semantic_map/objects', 10)

        # ── Timer ───────────────────────────────────────────────────────────
        self.create_timer(1.0 / publish_rate, self._publish)
        if autosave_period > 0:
            self.create_timer(autosave_period, self._save_semantic_map)

        # ── Load saved map (localization mode) ───────────────────────────────
        if load_on_startup:
            self._load_semantic_map()

        self.get_logger().info(
            f'SemanticMapNode ready  |  frame={self.target_frame} '
            f'det={detection_topic}  map={map_topic}  '
            f'merge_dist={self.merge_distance}m  '
            f'map_file={self.semantic_map_file}')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _on_detections(self, msg: DetectionArray) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # Keys updated in this batch (for disappearance detection)
        updated_keys: set = set()
        # (class_name, map_x, map_y) for each valid detection in this batch
        scanned_positions: list = []

        for det in msg.detections:
            if det.score < self.min_score:
                continue

            bbox3d = det.bbox3d
            frame  = bbox3d.frame_id
            if not frame:
                continue

            pt = PointStamped()
            pt.header.frame_id = frame
            pt.header.stamp    = msg.header.stamp
            pt.point.x = bbox3d.center.position.x
            pt.point.y = bbox3d.center.position.y
            pt.point.z = bbox3d.center.position.z

            try:
                pt_map = self.tf_buffer.transform(
                    pt, self.target_frame,
                    timeout=Duration(seconds=0.15))
            except Exception as exc:
                self.get_logger().debug(f'TF failed ({frame}→{self.target_frame}): {exc}')
                continue

            mx, my, mz = pt_map.point.x, pt_map.point.y, pt_map.point.z
            scanned_positions.append((det.class_name, mx, my))

            # Footprint: lateral width = bbox x, forward depth = bbox z
            foot_w = max(bbox3d.size.x, _MIN_FOOT)
            foot_d = max(bbox3d.size.z, _MIN_FOOT)

            # ── Spatial deduplication ────────────────────────────────────
            # If the tracking ID is known and already in the dict, update it.
            # Otherwise look for a same-class object within merge_distance to
            # avoid ghost duplicates when tracking is lost and re-acquired.
            key = det.id if det.id and det.id in self.objects else None

            if key is None:
                for k, obj in self.objects.items():
                    if obj.class_name == det.class_name:
                        if math.hypot(obj.x - mx, obj.y - my) < self.merge_distance:
                            key = k
                            break

            if key is None:
                key = det.id if det.id else self._next_uid()

            # Position smoothing: blend new measurement with stored position to
            # reduce per-frame jitter from depth-camera noise (EMA, alpha=0.4).
            if key in self.objects:
                prev   = self.objects[key]
                alpha  = 0.4
                mx     = (1.0 - alpha) * prev.x + alpha * mx
                my     = (1.0 - alpha) * prev.y + alpha * my
                mz     = (1.0 - alpha) * prev.z + alpha * mz
                foot_w = (1.0 - alpha) * prev.foot_w + alpha * foot_w
                foot_d = (1.0 - alpha) * prev.foot_d + alpha * foot_d

            self.objects[key] = SemanticObject(
                class_id    = det.class_id,
                class_name  = det.class_name,
                tracking_id = key,
                x           = mx,
                y           = my,
                z           = mz,
                score       = det.score,
                last_seen   = now_sec,
                foot_w      = foot_w,
                foot_d      = foot_d,
                miss_count  = 0,
            )
            updated_keys.add(key)

        # ── Disappearance detection ──────────────────────────────────────
        # For each stored object: if the robot just scanned a nearby area
        # (same class detected within 2×merge_distance) but this object was
        # NOT matched, increment its miss counter. Only remove after
        # disappear_grace_frames consecutive misses, to tolerate transient
        # detection losses (occlusion, camera angle, YOLO variability).
        rescan_radius = self.merge_distance * 2.0
        for k, obj in list(self.objects.items()):
            if k in updated_keys:
                obj.miss_count = 0  # reset on any successful match
                continue
            nearby_same_class = any(
                cls == obj.class_name
                and math.hypot(sx - obj.x, sy - obj.y) < rescan_radius
                for cls, sx, sy in scanned_positions
            )
            if not nearby_same_class:
                continue
            obj.miss_count += 1
            if obj.miss_count >= self.disappear_grace_frames:
                self.get_logger().info(
                    f'Object disappeared on rescan ({obj.miss_count} misses): '
                    f'{obj.class_name} @ ({obj.x:.2f}, {obj.y:.2f})')
                del self.objects[k]
            else:
                self.get_logger().debug(
                    f'Object miss {obj.miss_count}/{self.disappear_grace_frames}: '
                    f'{obj.class_name} @ ({obj.x:.2f}, {obj.y:.2f})')


    # ── Publishing ───────────────────────────────────────────────────────────

    def _publish(self) -> None:
        self._expire_objects()
        self._publish_objects_json()
        if self.map_msg is not None:
            self._publish_image()

    def _expire_objects(self) -> None:
        """Remove objects that have not been seen within object_timeout seconds."""
        if self.object_timeout <= 0:
            return
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        expired = [
            k for k, obj in self.objects.items()
            if (now_sec - obj.last_seen) > self.object_timeout
        ]
        for k in expired:
            obj = self.objects.pop(k)
            self.get_logger().info(
                f'Object expired ({self.object_timeout:.0f}s): '
                f'{obj.class_name} @ ({obj.x:.2f}, {obj.y:.2f})')

    def _publish_objects_json(self) -> None:
        """Publish all tracked objects as a JSON string for other nodes to query."""
        objs = [
            {
                'class_name':  obj.class_name,
                'class_id':    obj.class_id,
                'tracking_id': obj.tracking_id,
                'x':           obj.x,
                'y':           obj.y,
                'z':           obj.z,
                'score':       obj.score,
            }
            for obj in self.objects.values()
        ]
        msg = String()
        msg.data = json.dumps(objs)
        self.objects_pub.publish(msg)

    def _publish_image(self) -> None:
        """Render the occupancy grid with filled class-coloured bounding boxes."""
        m   = self.map_msg
        w   = m.info.width
        h   = m.info.height
        if w == 0 or h == 0:
            return
        res = m.info.resolution       # metres/pixel
        ox  = m.info.origin.position.x
        oy  = m.info.origin.position.y

        # Base grey-scale map
        grid = np.array(m.data, dtype=np.int8).reshape((h, w))
        img  = np.full((h, w, 3), 200, dtype=np.uint8)
        img[grid == 0]   = [240, 240, 240]
        img[grid == 100] = [30,  30,  30]
        img = cv2.flip(img, 0)   # ROS map: bottom-left origin → flip Y

        overlay = img.copy()

        # Pre-compute per-object draw params once (avoids duplicate coordinate math)
        # Each entry: (x1, y1, x2, y2, px, py, bgr, class_name)
        draw_items = []
        for obj in self.objects.values():
            px = int((obj.x - ox) / res)
            py = h - 1 - int((obj.y - oy) / res)
            hw = max(int(obj.foot_w / res / 2), 6)
            hd = max(int(obj.foot_d / res / 2), 6)
            x1, y1 = px - hw, py - hd
            x2, y2 = px + hw, py + hd
            if x2 < 0 or y2 < 0 or x1 >= w or y1 >= h:
                continue
            r, g, b, _ = class_color(obj.class_name)
            bgr = (int(b * 255), int(g * 255), int(r * 255))
            draw_items.append((x1, y1, x2, y2, px, py, bgr, obj.class_name))

        # Pass 1: semi-transparent filled rectangles
        for x1, y1, x2, y2, _px, _py, bgr, _cls in draw_items:
            cv2.rectangle(overlay, (x1, y1), (x2, y2), bgr, -1)

        # Blend fill (40% opacity) onto map
        cv2.addWeighted(overlay, 0.4, img, 0.6, 0, img)

        # Build a stable class→number mapping (alphabetical order)
        present_classes = sorted({obj.class_name for obj in self.objects.values()})
        class_index = {name: i + 1 for i, name in enumerate(present_classes)}

        # Pass 2: solid outlines and number labels (reuse cached coords)
        for x1, y1, x2, y2, px, py, bgr, class_name in draw_items:
            cv2.rectangle(img, (x1, y1), (x2, y2), bgr, 2)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 0), 1)  # dark outline
            num_str = str(class_index[class_name])
            tx = px - (cv2.getTextSize(num_str, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)[0][0] // 2)
            ty = py + 5
            cv2.putText(img, num_str, (tx, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, num_str, (tx, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        # Legend panel on the right side
        legend_item_h = 20
        legend_padding = 8
        legend_w = 160
        legend_h = legend_padding * 2 + len(present_classes) * legend_item_h
        legend = np.full((max(h, legend_h), legend_w, 3), 50, dtype=np.uint8)

        for i, name in enumerate(present_classes):
            idx = i + 1
            r, g, b, _ = class_color(name)
            bgr = (int(b * 255), int(g * 255), int(r * 255))
            cy = legend_padding + i * legend_item_h + legend_item_h // 2
            cv2.rectangle(legend, (6, cy - 7), (18, cy + 7), bgr, -1)
            cv2.rectangle(legend, (6, cy - 7), (18, cy + 7), (200, 200, 200), 1)
            entry = f'{idx}: {name}'
            cv2.putText(legend, entry, (22, cy + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (220, 220, 220), 1, cv2.LINE_AA)

        # Pad map image height to match legend if needed
        if h < legend.shape[0]:
            pad = np.full((legend.shape[0] - h, w, 3), 200, dtype=np.uint8)
            img = np.vstack([img, pad])

        combined = np.hstack([img, legend])

        ros_img = self.bridge.cv2_to_imgmsg(combined, encoding='bgr8')
        ros_img.header.stamp    = self.get_clock().now().to_msg()
        ros_img.header.frame_id = self.target_frame
        self.image_pub.publish(ros_img)

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _next_uid(self) -> str:
        uid = f'_auto_{self._uid}'
        self._uid += 1
        return uid

    # ── Persistence ──────────────────────────────────────────────────────────

    def _save_semantic_map(self) -> None:
        """Persist current semantic objects to JSON file."""
        if not self.objects:
            return
        path = os.path.expanduser(self.semantic_map_file)
        data = [
            {
                'class_id':    obj.class_id,
                'class_name':  obj.class_name,
                'tracking_id': obj.tracking_id,
                'x':           obj.x,
                'y':           obj.y,
                'z':           obj.z,
                'score':       obj.score,
                'foot_w':      obj.foot_w,
                'foot_d':      obj.foot_d,
            }
            for obj in self.objects.values()
        ]
        try:
            parent = os.path.dirname(path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(path, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(
                f'Saved {len(data)} semantic objects → {path}')
        except Exception as exc:
            self.get_logger().error(f'Failed to save semantic map: {exc}')

    def _load_semantic_map(self) -> None:
        """Load semantic objects from JSON file into the object dictionary."""
        path = os.path.expanduser(self.semantic_map_file)
        try:
            with open(path, 'r') as f:
                data = json.load(f)
        except FileNotFoundError:
            self.get_logger().warning(
                f'Semantic map file not found: {path} (starting with empty map)')
            return
        except Exception as exc:
            self.get_logger().error(f'Failed to load semantic map: {exc}')
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        loaded = 0
        for item in data:
            try:
                obj = SemanticObject(
                    class_id    = item['class_id'],
                    class_name  = item['class_name'],
                    tracking_id = item['tracking_id'],
                    x           = float(item['x']),
                    y           = float(item['y']),
                    z           = float(item['z']),
                    score       = float(item['score']),
                    last_seen   = now_sec,
                    foot_w      = float(item.get('foot_w', _MIN_FOOT)),
                    foot_d      = float(item.get('foot_d', _MIN_FOOT)),
                )
                self.objects[obj.tracking_id] = obj
                loaded += 1
            except (KeyError, TypeError, ValueError) as exc:
                self.get_logger().warning(
                    f'Skipping malformed entry in semantic map: {exc}')

        self.get_logger().info(
            f'Loaded {loaded} semantic objects ← {path}')

    def destroy_node(self) -> None:
        """Save semantic map to disk before shutting down."""
        self._save_semantic_map()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
