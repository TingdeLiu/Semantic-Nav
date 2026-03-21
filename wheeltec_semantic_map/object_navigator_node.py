#!/usr/bin/env python3
"""
Object Navigator Node

Searches for a target object class in the semantic map and navigates to it.
If the object is not found, triggers autonomous exploration (m-explore-ros2)
until the object is detected or the search times out.

Goal interface (topic-based):
  Sub: /navigate_to_object/goal    (std_msgs/String)  – target class name
  Sub: /navigate_to_object/cancel  (std_msgs/Empty)   – cancel current task
  Pub: /navigate_to_object/feedback (std_msgs/String) – JSON status updates
  Pub: /navigate_to_object/result   (std_msgs/String) – JSON final result

Internal:
  Sub: /semantic_map/objects  (std_msgs/String)               – JSON object list
  Sub: /explore/status        (explore_lite_msgs/ExploreStatus)
  Pub: /explore/resume        (std_msgs/Bool)                 – start/stop explore
  Nav2: navigate_to_pose action client
"""

import json
import math
import queue
import threading
import time
from enum import Enum, auto

import rclpy
import rclpy.time
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

try:
    from explore_lite_msgs.msg import ExploreStatus
    from rclpy.type_support import check_for_type_support
    check_for_type_support(ExploreStatus)  # raises UnsupportedTypeSupport if .so missing
    _HAS_EXPLORE_MSGS = True
except Exception:
    _HAS_EXPLORE_MSGS = False


# ── State machine ─────────────────────────────────────────────────────────────

class State(Enum):
    IDLE                 = auto()
    SEARCHING_MAP        = auto()
    NAVIGATING_TO_OBJECT = auto()
    EXPLORING            = auto()
    SUCCESS              = auto()
    FAILED               = auto()


# ── Node ──────────────────────────────────────────────────────────────────────

class ObjectNavigatorNode(Node):

    def __init__(self) -> None:
        super().__init__('object_navigator_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('approach_distance',  0.8)    # metres from object
        self.declare_parameter('search_timeout',    300.0)  # seconds
        self.declare_parameter('search_grace_period', 5.0)  # seconds to wait before exploring
        self.declare_parameter('max_nav_attempts',   3)     # max re-navigation attempts per task
        self.declare_parameter('web_port',           8080)
        self.declare_parameter('web_enabled',        True)

        self.approach_distance   = self.get_parameter('approach_distance').value
        self.search_timeout      = self.get_parameter('search_timeout').value
        self.search_grace_period = self.get_parameter('search_grace_period').value
        self.max_nav_attempts    = self.get_parameter('max_nav_attempts').value
        web_port    = self.get_parameter('web_port').value
        web_enabled = self.get_parameter('web_enabled').value

        # ── TF ──────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── State ────────────────────────────────────────────────────────────
        self._lock           = threading.RLock()
        self._cmd_queue: queue.Queue = queue.Queue()  # thread-safe Flask→ROS channel
        self.state           = State.IDLE
        self.target_class    = ''
        self.current_objects = []   # list[dict] from /semantic_map/objects JSON
        self.explore_status  = ''
        self.task_start_time = 0.0
        self._reset_at       = 0.0  # monotonic time to reset IDLE after result
        self.nav_goal_handle = None
        self._nav_target_lost_since = 0.0  # monotonic time target first disappeared during nav
        self._nav_attempts   = 0           # re-navigation attempts this task

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            String, '/navigate_to_object/goal', self._on_goal, 10)
        self.create_subscription(
            Empty, '/navigate_to_object/cancel', self._on_cancel, 10)
        self.create_subscription(
            String, '/semantic_map/objects', self._on_objects, 10)

        if _HAS_EXPLORE_MSGS:
            self.create_subscription(
                ExploreStatus, '/explore/status', self._on_explore_status, 10)
        else:
            self.get_logger().warning(
                'explore_lite_msgs not found – exploration completion '
                'detection disabled (will rely on timeout instead)')

        # ── Publishers ───────────────────────────────────────────────────────
        self.feedback_pub      = self.create_publisher(
            String, '/navigate_to_object/feedback', 10)
        self.result_pub        = self.create_publisher(
            String, '/navigate_to_object/result', 10)
        self.explore_resume_pub = self.create_publisher(
            Bool, '/explore/resume', 10)

        # ── Nav2 action client ───────────────────────────────────────────────
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Main loop (1 Hz) ─────────────────────────────────────────────────
        self.create_timer(1.0, self._loop)

        # ── Web server ───────────────────────────────────────────────────────
        if web_enabled:
            self._start_web_server(web_port)

        self.get_logger().info(
            f'ObjectNavigatorNode ready | approach={self.approach_distance}m '
            f'timeout={self.search_timeout}s grace={self.search_grace_period}s '
            f'max_nav_attempts={self.max_nav_attempts} '
            f'web={web_port if web_enabled else "off"}')

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _on_goal(self, msg: String) -> None:
        class_name = msg.data.strip()
        if not class_name:
            return
        with self._lock:
            if self.state not in (State.IDLE, State.SUCCESS, State.FAILED):
                self.get_logger().warning(
                    f'Busy (state={self.state.name}), ignoring goal "{class_name}"')
                return
            self._start_task(class_name)

    def _on_cancel(self, msg: Empty) -> None:
        with self._lock:
            if self.state == State.IDLE:
                return
            self.get_logger().info('Cancel received')
            self._cancel_nav()
            self._stop_exploration()
            self.state = State.FAILED
            self._reset_at = time.monotonic() + 2.0
            self._pub_result(False, 'Cancelled by user', None)

    def _on_objects(self, msg: String) -> None:
        try:
            objs = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            self.current_objects = objs

    def _on_explore_status(self, msg) -> None:
        with self._lock:
            self.explore_status = msg.status

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _loop(self) -> None:
        # Drain commands posted from non-ROS threads (Flask web server).
        # All ROS operations (publish, state mutation) happen here in the
        # executor thread, avoiding cross-thread publish races.
        while True:
            try:
                cmd, arg = self._cmd_queue.get_nowait()
            except queue.Empty:
                break
            if cmd == 'goal':
                msg = String()
                msg.data = arg
                self._on_goal(msg)
            elif cmd == 'cancel':
                self._on_cancel(Empty())

        with self._lock:
            state = self.state

            # Auto-reset after result
            if state in (State.SUCCESS, State.FAILED):
                if self._reset_at > 0 and time.monotonic() >= self._reset_at:
                    self.state = State.IDLE
                    self._reset_at = 0.0
                return

            if state == State.IDLE:
                return

            # Global timeout
            if time.monotonic() - self.task_start_time > self.search_timeout:
                self._fail(f'Search timeout ({self.search_timeout:.0f}s)')
                return

            if state == State.SEARCHING_MAP:
                self._do_search()
            elif state == State.EXPLORING:
                self._do_explore_check()
            elif state == State.NAVIGATING_TO_OBJECT:
                self._do_nav_check()

    def _do_search(self) -> None:
        found = self._find_in_map(self.target_class)
        if found:
            self.get_logger().info(
                f'Found "{self.target_class}" in map at '
                f'({found["x"]:.2f}, {found["y"]:.2f})')
            self._navigate_to(found)
        elif time.monotonic() - self.task_start_time < self.search_grace_period:
            # Give the semantic map time to accumulate detections before exploring
            elapsed = time.monotonic() - self.task_start_time
            self._pub_feedback(
                'SEARCHING_MAP',
                f'Scanning map for "{self.target_class}" ({elapsed:.0f}s)...')
        else:
            self.get_logger().info(
                f'"{self.target_class}" not in semantic map – starting exploration')
            self._start_exploration()

    def _do_explore_check(self) -> None:
        found = self._find_in_map(self.target_class)
        if found:
            self.get_logger().info(
                f'Found "{self.target_class}" during exploration! '
                f'({found["x"]:.2f}, {found["y"]:.2f})')
            self._stop_exploration()
            self._navigate_to(found)
        elif self.explore_status in ('exploration_complete', 'returned_to_origin'):
            self._fail(f'Exploration complete – "{self.target_class}" not found')
        else:
            elapsed = time.monotonic() - self.task_start_time
            self._pub_feedback(
                'EXPLORING',
                f'Exploring ({elapsed:.0f}s elapsed), searching for "{self.target_class}"')

    # Grace period before aborting navigation when target disappears from map.
    # Transient detection losses (camera angle change, occlusion) are common
    # during approach; 10 s gives the semantic map time to re-detect the object.
    _NAV_TARGET_LOST_GRACE = 10.0  # seconds

    def _do_nav_check(self) -> None:
        """While navigating, abort if the target disappears from the semantic map."""
        found = self._find_in_map(self.target_class)
        if found:
            self._nav_target_lost_since = 0.0
            # Proximity check: succeed immediately if robot is already close enough,
            # without waiting for the Nav2 goal to formally complete.
            try:
                rx, ry = self._robot_pose()
                dist = math.hypot(found['x'] - rx, found['y'] - ry)
                if dist <= self.approach_distance:
                    self.get_logger().info(
                        f'Robot reached "{self.target_class}" '
                        f'({dist:.2f}m ≤ {self.approach_distance}m) – task complete')
                    self._cancel_nav()
                    self.state     = State.SUCCESS
                    self._reset_at = time.monotonic() + 3.0
                    self._pub_result(True, f'Reached "{self.target_class}"', found)
                    return
            except RuntimeError:
                pass  # TF temporarily unavailable – fall through to normal wait
            return

        now = time.monotonic()
        if self._nav_target_lost_since == 0.0:
            self._nav_target_lost_since = now
            self.get_logger().debug(
                f'Target "{self.target_class}" not in map during navigation – '
                f'grace period started')
            return

        lost_for = now - self._nav_target_lost_since
        if lost_for < self._NAV_TARGET_LOST_GRACE:
            self._pub_feedback(
                'NAVIGATING',
                f'Target "{self.target_class}" temporarily absent from map '
                f'({lost_for:.0f}s/{self._NAV_TARGET_LOST_GRACE:.0f}s grace)')
            return

        self.get_logger().warning(
            f'Target "{self.target_class}" absent from semantic map for '
            f'{lost_for:.0f}s during navigation – cancelling and re-searching '
            f'(attempt {self._nav_attempts}/{self.max_nav_attempts})')
        self._cancel_nav()
        self._nav_target_lost_since = 0.0
        self.state = State.SEARCHING_MAP
        self._pub_feedback(
            'SEARCHING_MAP',
            f'Target lost during navigation, re-searching for "{self.target_class}" '
            f'(attempt {self._nav_attempts}/{self.max_nav_attempts})')

    # ── Task control ──────────────────────────────────────────────────────────

    def _start_task(self, class_name: str) -> None:
        self.target_class           = class_name
        self.task_start_time        = time.monotonic()
        self.explore_status         = ''
        self._nav_target_lost_since = 0.0
        self._nav_attempts          = 0
        self.state                  = State.SEARCHING_MAP
        self.get_logger().info(f'New navigation task: find "{class_name}"')
        self._pub_feedback('SEARCHING_MAP', f'Searching map for "{class_name}"')

    def _start_exploration(self) -> None:
        self.state = State.EXPLORING
        msg = Bool()
        msg.data = True
        self.explore_resume_pub.publish(msg)
        self._pub_feedback('EXPLORING', f'Starting exploration to find "{self.target_class}"')
        self.get_logger().info('Exploration started (/explore/resume = true)')

    def _stop_exploration(self) -> None:
        msg = Bool()
        msg.data = False
        self.explore_resume_pub.publish(msg)
        self.get_logger().info('Exploration paused (/explore/resume = false)')

    def _cancel_nav(self) -> None:
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None

    def _fail(self, reason: str) -> None:
        self._cancel_nav()
        self._stop_exploration()
        self.state     = State.FAILED
        self._reset_at = time.monotonic() + 2.0
        self.get_logger().warning(f'Task failed: {reason}')
        self._pub_result(False, reason, None)

    # ── Navigation ────────────────────────────────────────────────────────────

    def _navigate_to(self, obj: dict) -> None:
        ox, oy = obj['x'], obj['y']

        # Guard against infinite re-navigation loops (e.g. target repeatedly
        # disappearing / reappearing from semantic map while SLAM is running).
        self._nav_attempts += 1
        if self._nav_attempts > self.max_nav_attempts:
            self._fail(
                f'Exceeded max navigation attempts ({self.max_nav_attempts}) '
                f'for "{self.target_class}" – aborting')
            return

        # Approach pose: stop approach_distance metres in front of object.
        # Robot pose must be resolved BEFORE mutating state so that a TF failure
        # triggers _fail() cleanly rather than leaving state as NAVIGATING.
        try:
            rx, ry = self._robot_pose()
        except RuntimeError as e:
            self._fail(f'Cannot compute approach pose: {e}')
            return

        # Already close enough – no need to navigate.
        dist = math.hypot(ox - rx, oy - ry)
        if dist <= self.approach_distance:
            self.get_logger().info(
                f'Already within approach distance of "{self.target_class}" '
                f'({dist:.2f}m ≤ {self.approach_distance}m) – task complete')
            self.state     = State.SUCCESS
            self._reset_at = time.monotonic() + 3.0
            self._pub_result(True, f'Reached "{self.target_class}"', obj)
            return

        dx, dy = ox - rx, oy - ry
        dist = math.hypot(dx, dy) or 1e-6
        dx /= dist
        dy /= dist

        goal_x = ox - self.approach_distance * dx
        goal_y = oy - self.approach_distance * dy
        yaw    = math.atan2(oy - goal_y, ox - goal_x)

        self.state                  = State.NAVIGATING_TO_OBJECT
        self._nav_target_lost_since = 0.0
        self._pub_feedback(
            'NAVIGATING',
            f'Navigating to "{self.target_class}" at ({ox:.2f}, {oy:.2f})')

        if not self.nav_client.server_is_ready():
            self._fail('Nav2 action server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f, o=obj: self._on_goal_accepted(f, o))

    def _on_goal_accepted(self, future, obj: dict) -> None:
        handle = future.result()
        if not handle.accepted:
            with self._lock:
                self._fail('Navigation goal rejected by Nav2')
            return
        with self._lock:
            self.nav_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f, o=obj: self._on_nav_result(f, o))

    def _on_nav_result(self, future, obj: dict) -> None:
        with self._lock:
            self.nav_goal_handle = None
            if self.state != State.NAVIGATING_TO_OBJECT:
                return  # task was cancelled
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.state     = State.SUCCESS
                self._reset_at = time.monotonic() + 3.0
                self.get_logger().info(
                    f'Successfully reached "{self.target_class}"!')
                self._pub_result(True, f'Reached "{self.target_class}"', obj)
            else:
                self._fail(f'Navigation failed (Nav2 status={status})')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _find_in_map(self, class_name: str):
        """Return the highest-score object of class_name, or None."""
        matches = [o for o in self.current_objects
                   if o.get('class_name') == class_name]
        if not matches:
            return None
        return max(matches, key=lambda o: o.get('score', 0.0))

    def _robot_pose(self):
        """Return (x, y) of the robot in the map frame.

        Raises RuntimeError if neither base_footprint nor base_link → map
        transform is available, so callers get an explicit failure instead of
        silently computing approach positions relative to the map origin.
        """
        last_exc: Exception = RuntimeError('TF not yet available')
        for frame in ('base_footprint', 'base_link'):
            try:
                t = self.tf_buffer.lookup_transform(
                    'map', frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5))
                return t.transform.translation.x, t.transform.translation.y
            except Exception as e:
                last_exc = e
        raise RuntimeError(
            f'Robot pose unavailable (base_footprint/base_link → map): {last_exc}'
        )

    # ── Publish helpers ───────────────────────────────────────────────────────

    def _pub_feedback(self, status: str, message: str) -> None:
        msg = String()
        msg.data = json.dumps({
            'status':  status,
            'message': message,
            'target':  self.target_class,
            'elapsed': round(time.monotonic() - self.task_start_time, 1),
        })
        self.feedback_pub.publish(msg)

    def _pub_result(self, success: bool, message: str, obj) -> None:
        data = {
            'success': success,
            'message': message,
            'target':  self.target_class,
        }
        if obj:
            data['object'] = {
                'x':     obj['x'],
                'y':     obj['y'],
                'score': obj.get('score', 0.0),
            }
        msg = String()
        msg.data = json.dumps(data)
        self.result_pub.publish(msg)

    # ── Web server ────────────────────────────────────────────────────────────

    def _start_web_server(self, port: int) -> None:
        try:
            from flask import Flask, jsonify, request
        except ImportError:
            self.get_logger().warning(
                'Flask not installed – web interface disabled. '
                'Install with: pip install flask')
            return

        app  = Flask(__name__)
        node = self

        @app.route('/')
        def index():
            return _WEB_HTML

        @app.route('/navigate', methods=['POST'])
        def navigate():
            data       = request.get_json(force=True, silent=True) or {}
            class_name = data.get('class_name', '').strip()
            if not class_name:
                return jsonify({'error': 'class_name required'}), 400
            node._cmd_queue.put(('goal', class_name))
            return jsonify({'status': 'accepted', 'target': class_name})

        @app.route('/cancel', methods=['POST'])
        def cancel():
            node._cmd_queue.put(('cancel', None))
            return jsonify({'status': 'cancelled'})

        @app.route('/status')
        def status():
            with node._lock:
                state_name  = node.state.name
                is_idle     = node.state == State.IDLE
                target      = node.target_class
                start_time  = node.task_start_time
            elapsed = (round(time.monotonic() - start_time, 1) if not is_idle else 0)
            return jsonify({
                'state':   state_name,
                'target':  target,
                'elapsed': elapsed,
            })

        @app.route('/objects')
        def objects():
            with node._lock:
                objs = list(node.current_objects)
            return jsonify(objs)

        def _run():
            import logging
            logging.getLogger('werkzeug').setLevel(logging.ERROR)
            app.run(host='0.0.0.0', port=port, debug=False, use_reloader=False)

        t = threading.Thread(target=_run, daemon=True)
        t.start()
        self.get_logger().info(f'Web interface: http://0.0.0.0:{port}')


# ── Embedded Web UI ───────────────────────────────────────────────────────────

_WEB_HTML = """<!DOCTYPE html>
<html lang="zh">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Wheeltec Object Navigator</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: 'Segoe UI', sans-serif; background: #1a1a2e; color: #eee; min-height: 100vh; }
    .header { background: #16213e; padding: 16px 24px; border-bottom: 2px solid #0f3460; }
    .header h1 { font-size: 1.4rem; color: #e94560; }
    .header p  { font-size: 0.8rem; color: #888; margin-top: 2px; }
    .container { max-width: 720px; margin: 24px auto; padding: 0 16px; }
    .card { background: #16213e; border-radius: 10px; padding: 20px; margin-bottom: 16px; border: 1px solid #0f3460; }
    .card h2 { font-size: 1rem; color: #a8dadc; margin-bottom: 14px; }
    .row { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
    input[type=text] {
      flex: 1; min-width: 180px; padding: 10px 14px; border-radius: 6px;
      border: 1px solid #0f3460; background: #0f3460; color: #eee; font-size: 0.95rem;
    }
    input[list]::-webkit-calendar-picker-indicator { filter: invert(1); }
    button {
      padding: 10px 22px; border: none; border-radius: 6px; font-size: 0.9rem;
      cursor: pointer; transition: opacity 0.15s;
    }
    button:hover { opacity: 0.85; }
    .btn-go     { background: #e94560; color: #fff; }
    .btn-cancel { background: #555; color: #eee; }
    .status-box {
      display: flex; align-items: center; gap: 12px;
      padding: 14px; border-radius: 8px; background: #0f3460; margin-top: 4px;
    }
    .dot { width: 12px; height: 12px; border-radius: 50%; flex-shrink: 0; }
    .dot.IDLE                 { background: #555; }
    .dot.SEARCHING_MAP        { background: #f4a261; animation: pulse 1s infinite; }
    .dot.EXPLORING            { background: #a8dadc; animation: pulse 1s infinite; }
    .dot.NAVIGATING_TO_OBJECT { background: #2196f3; animation: pulse 1s infinite; }
    .dot.SUCCESS              { background: #4caf50; }
    .dot.FAILED               { background: #e94560; }
    @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:.4} }
    .state-label { font-size: 1rem; font-weight: 600; }
    .state-detail { font-size: 0.8rem; color: #aaa; margin-top: 2px; }
    .objects-table { width: 100%; border-collapse: collapse; font-size: 0.85rem; }
    .objects-table th { color: #a8dadc; text-align: left; padding: 6px 10px; border-bottom: 1px solid #0f3460; }
    .objects-table td { padding: 6px 10px; border-bottom: 1px solid #0f346055; }
    .objects-table tr:hover td { background: #0f346088; }
    .tag { display: inline-block; padding: 2px 8px; border-radius: 4px; font-size: 0.75rem; font-weight: 600; cursor: pointer; }
    datalist option { background: #1a1a2e; }
    .empty { color: #555; font-size: 0.85rem; padding: 10px 0; }
  </style>
</head>
<body>
  <div class="header">
    <h1>Wheeltec Object Navigator</h1>
    <p>语义地图物体导航控制界面</p>
  </div>

  <div class="container">
    <!-- Goal input -->
    <div class="card">
      <h2>目标物体</h2>
      <div class="row">
        <input type="text" id="class_input" list="class_list"
               placeholder="输入物体类名 (如: chair, cup, person)" autocomplete="off">
        <datalist id="class_list"></datalist>
        <button class="btn-go" onclick="doNavigate()">导航</button>
        <button class="btn-cancel" onclick="doCancel()">取消</button>
      </div>
    </div>

    <!-- Status -->
    <div class="card">
      <h2>导航状态</h2>
      <div class="status-box">
        <div class="dot IDLE" id="status_dot"></div>
        <div>
          <div class="state-label" id="status_state">IDLE</div>
          <div class="state-detail" id="status_detail">等待任务</div>
        </div>
      </div>
    </div>

    <!-- Objects in map -->
    <div class="card">
      <h2>当前语义地图物体</h2>
      <div id="objects_container"><p class="empty">暂无检测到的物体</p></div>
    </div>
  </div>

  <script>
    const STATE_LABELS = {
      IDLE:                 '空闲',
      SEARCHING_MAP:        '搜索地图中...',
      EXPLORING:            '主动探索中...',
      NAVIGATING_TO_OBJECT: '正在导航...',
      SUCCESS:              '导航成功！',
      FAILED:               '任务失败',
    };

    async function doNavigate() {
      const cn = document.getElementById('class_input').value.trim();
      if (!cn) { alert('请输入目标物体类名'); return; }
      await fetch('/navigate', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({class_name: cn})
      });
    }

    async function doCancel() {
      await fetch('/cancel', {method: 'POST'});
    }

    function fillClass(name) {
      document.getElementById('class_input').value = name;
    }

    async function pollStatus() {
      try {
        const r = await fetch('/status');
        const d = await r.json();
        const dot   = document.getElementById('status_dot');
        const state = document.getElementById('status_state');
        const detail = document.getElementById('status_detail');
        dot.className = 'dot ' + d.state;
        state.textContent = d.state + (d.target ? ` — "${d.target}"` : '');
        detail.textContent = (STATE_LABELS[d.state] || d.state) +
          (d.elapsed > 0 ? `  (${d.elapsed}s)` : '');
      } catch(e) {}
    }

    function esc(s) {
      return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');
    }

    async function pollObjects() {
      try {
        const r = await fetch('/objects');
        const objs = await r.json();
        const container = document.getElementById('objects_container');
        const dl = document.getElementById('class_list');

        if (!objs.length) {
          container.innerHTML = '<p class="empty">暂无检测到的物体</p>';
          dl.innerHTML = '';
          return;
        }

        // Unique classes for datalist
        const classes = [...new Set(objs.map(o => o.class_name))].sort();
        dl.innerHTML = classes.map(c => `<option value="${esc(c)}">`).join('');

        // Table
        let html = `<table class="objects-table">
          <tr><th>类名</th><th>X (m)</th><th>Y (m)</th><th>置信度</th><th>快捷导航</th></tr>`;
        for (const o of objs) {
          const score = (o.score * 100).toFixed(0);
          html += `<tr>
            <td>${esc(o.class_name)}</td>
            <td>${o.x.toFixed(2)}</td>
            <td>${o.y.toFixed(2)}</td>
            <td>${score}%</td>
            <td><span class="tag" style="background:#e94560"
                onclick="fillClass(${JSON.stringify(o.class_name)})">选择</span></td>
          </tr>`;
        }
        html += '</table>';
        container.innerHTML = html;
      } catch(e) {}
    }

    setInterval(pollStatus,  800);
    setInterval(pollObjects, 1500);
    pollStatus();
    pollObjects();
  </script>
</body>
</html>"""


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
