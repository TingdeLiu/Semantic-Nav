# wheeltec_semantic_map

将 YOLO v8 三维目标检测与 RTAB-Map SLAM 融合，在机器人导航地图上构建持久化语义标注层。检测到的物体以彩色贴地方块 + 文字标签的形式叠加到占用栅格地图上，支持 RViz 3D 可视化、2D 标注图像、JSON 物体列表三种输出。在此基础上，`object_navigator_node` 可接收目标类别名称，自动在语义地图中搜索并驱动 Nav2 导航到该物体；若未找到，则通过内置的 `frontier_explorer_node` 自主探索环境（仅追踪相机前向 ±90° 的前沿，确保语义地图不漏检）。

---

## 目录

- [硬件要求](#硬件要求)
- [软件依赖](#软件依赖)
- [编译安装](#编译安装)
- [启动方式](#启动方式)
- [物体导航](#物体导航)
- [参数说明](#参数说明)
- [RViz 可视化配置](#rviz-可视化配置)
- [地图数据查看](#地图数据查看)
- [话题列表](#话题列表)
- [常见问题排查](#常见问题排查)
- [点云质量调优](#点云质量调优)

---

## 硬件要求

| 硬件 | 说明 |
|------|------|
| Wheeltec 轮式机器人 | 提供底盘里程计（`/odom_combined`） |
| Orbbec Gemini 336L | RGB-D 相机，需接 **USB 3.0** 口（带宽要求） |
| 激光雷达 | 标准 SLAM 模式使用，纯 RGB-D 模式可不接 |

> **注意**：Orbbec Gemini 336L 的有效深度范围为 **0.25 m ~ 4 m**，超出范围的点云噪声显著增大。

---

## 软件依赖

| 依赖 | 版本 / 说明 |
|------|------------|
| ROS 2 | Humble 或更高 |
| rtabmap_ros | `rtabmap_slam`、`rtabmap_sync`、`rtabmap_util` |
| nav2_msgs | NavigateToPose action（frontier_explorer_node 使用） |
| yolo_bringup | YOLO v8 检测 + 3D 定位节点 |
| wheeltec_robot_rtab | Wheeltec RTAB-Map 配置 + 内置前沿探索节点 |
| turn_on_wheeltec_robot | 底盘驱动、激光雷达、相机驱动 |
| python3-opencv | 2D 地图图像渲染 |
| python3-cv-bridge | ROS Image ↔ OpenCV 转换 |
| tf2_ros, tf2_geometry_msgs | 坐标变换 |
| explore_lite_msgs | `/explore/status` 消息类型（可选，缺失时降级为超时） |

YOLO 推理依赖 conda 独立环境，需在启动前激活：

```bash
conda activate yolo
```

---

## 编译安装

```bash
cd ~/wheeltec_ros2

# 编译两个相关包
colcon build --packages-select wheeltec_semantic_map wheeltec_robot_rtab

# 加载环境（每个新终端都需要执行）
source install/setup.bash
```

> **重要**：含 YOLO 的启动方式必须先激活 conda 再 source ROS2，顺序不能颠倒。

```bash
conda activate yolo
source ~/wheeltec_ros2/install/setup.bash
```

---

## 启动方式

### 方式一：仅语义映射（推荐调试时使用）

适用于已有外部 SLAM 系统在运行的情况。

```bash
# 终端1：先启动 SLAM（任选一种）
ros2 launch wheeltec_robot_rtab wheeltec_slam_rtab.launch.py         # LiDAR + RGB-D
ros2 launch wheeltec_robot_rtab wheeltec_slam_rtab_pure3d.launch.py  # 纯 RGB-D

# 终端2：启动语义映射
ros2 launch wheeltec_semantic_map semantic_map.launch.py
```

支持的启动参数：

```bash
ros2 launch wheeltec_semantic_map semantic_map.launch.py \
  model:=yolov8m.pt \
  device:=cuda:0 \
  threshold:=0.7 \
  target_frame:=map \
  publish_rate:=2.0
```

---

### 方式二：完整栈一键启动（LiDAR + RGB-D）

同时启动底盘驱动、激光雷达、相机、RTAB-Map、YOLO 和语义映射。

```bash
ros2 launch wheeltec_semantic_map semantic_slam.launch.py
```

---

### 方式三：纯 RGB-D 完整栈（无激光雷达）

不需要激光雷达，仅用深度相机完成 SLAM + 语义映射。

```bash
ros2 launch wheeltec_semantic_map semantic_slam_pure3d.launch.py
```

---

### 方式四：语义地图 + 物体导航（含自主探索）

使用 `wheeltec_robot_rtab` 内置前沿探索节点，**无需 m-explore-ros2**，仅需 2 个终端：

**终端 1 — 底盘驱动**

```bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

**终端 2 — 完整栈（SLAM + Nav2 + 前沿探索 + YOLO + 语义地图 + 物体导航）**

```bash
conda activate yolo
source ~/wheeltec_ros2/install/setup.bash
ros2 launch wheeltec_robot_rtab wheeltec_explore_rtab.launch.py
```

> 内置的 `frontier_explorer_node` 直接订阅 RTAB-Map 的 `/map`，通过 Nav2 `navigate_to_pose` 驱动探索，并内置前向 ±90° 过滤（只追踪相机能看到的前沿，避免机器人朝相机看不到的方向行进导致语义地图漏检）。
> `object_navigator_node` 无需任何改动，仍通过 `/explore/resume` 话题控制探索启停。

编译（首次或代码更新后）：

```bash
colcon build --packages-select wheeltec_robot_rtab
source install/setup.bash
```

---

### 定位模式（加载已有地图）

已有 `~/.ros/rtabmap.db` 时，无需重新建图，直接加载定位：

```bash
ros2 launch wheeltec_robot_rtab wheeltec_slam_rtab.launch.py Localization:=true
```

---

## 物体导航

`object_navigator_node` 订阅 `/navigate_to_object/goal`（String，物体类别名），在语义地图中查找目标，调用 Nav2 导航过去。找不到时自动触发前沿探索（发布 `/explore/resume = true`），探索中持续监视地图直到发现目标或超时。

> **前置条件**：使用方式四启动时，`frontier_explorer_node` 已内置于 `wheeltec_explore_rtab.launch.py` 中，无需单独启动。

```bash
# 发送导航指令（例：找椅子）
ros2 topic pub --once /navigate_to_object/goal std_msgs/msg/String "data: 'chair'"

# 查看导航状态
ros2 topic echo /navigate_to_object/feedback

# 取消当前任务
ros2 topic pub --once /navigate_to_object/cancel std_msgs/msg/Empty "{}"

# 或直接访问 Web UI（支持点选导航、实时状态、物体列表）
http://<robot_ip>:8080
```

状态机流转：`IDLE → SEARCHING_MAP → NAVIGATING_TO_OBJECT / EXPLORING → SUCCESS / FAILED`

---

## 参数说明

### semantic_map_node 节点参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `target_frame` | `map` | 语义对象的目标坐标系 |
| `detection_topic` | `/yolo/detections_3d` | YOLO 3D 检测输入话题 |
| `map_topic` | `/map` | 占用栅格地图输入话题 |
| `object_timeout` | `120.0` s | 预留参数，当前未实现；物体清除依赖主动消失检测 |
| `min_score` | `0.8` | 最低检测置信度，低于此值丢弃 |
| `publish_rate` | `2.0` Hz | 语义地图输出刷新频率 |
| `marker_lifetime` | `2.0` s | RViz 标注的可见持续时长（超时自动消失） |
| `merge_distance` | `0.6` m | 空间去重半径，范围内同类物体合并为一个 |

### object_navigator_node 节点参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `approach_distance` | `0.8` m | 导航目标点距物体的停止距离 |
| `search_timeout` | `300.0` s | 整体任务超时时间 |
| `search_grace_period` | `5.0` s | 首次搜索等待窗口，超时后才触发自主探索 |
| `web_port` | `8080` | Flask Web UI 端口 |
| `web_enabled` | `true` | 是否启动 Web 控制界面 |

### frontier_explorer_node 节点参数

通过 `wheeltec_explore_rtab.launch.py` 的同名参数传入。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `camera_half_fov` | `π/2` rad | 相机半视角（rad），只探索前向此角度范围内的前沿 |
| `min_frontier_size` | `5` 格 | 最小前沿簇大小（格数），过小的前沿被忽略 |
| `max_frontier_dist` | `8.0` m | 忽略超出此距离的前沿 |
| `goal_tolerance` | `0.5` m | 到达前沿目标的容差 |
| `replanning_period` | `5.0` s | 到达前沿后重新规划的等待时间 |

### RTAB-Map 关键参数（wheeltec_slam_rtab.launch.py）

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `Reg/Force3DoF` | `true` | 强制 2D 平面约束，适合平地机器人 |
| `Grid/RangeMin` | `0.2` m | 忽略近距噪声点（雷达自身遮挡） |
| `Grid/RangeMax` | `4.0` m | 截断 4m 外的深度噪声（Gemini 336L 精度极限） |
| `cloud_voxel_size` | `0.05` m | 5 cm 体素滤波，减少点云噪声和内存 |
| `Grid/NoiseFilteringRadius` | `0.05` m | 噪声过滤搜索半径 |
| `Grid/NoiseFilteringMinNeighbors` | `5` | 邻域内少于此数量的点视为噪声并剔除 |
| `RGBD/LinearUpdate` | `0.1` m | 每移动 10 cm 才插入新地图节点 |
| `RGBD/AngularUpdate` | `0.1` rad | 每旋转约 6° 才插入新地图节点 |
| `RGBD/OptimizeMaxError` | `5.0` | 回环闭合最大误差容忍（默认 3.0 过严） |
| `Vis/MinInliers` | `20` | 接受回环闭合所需最少视觉内点数 |

### rgbd_sync 同步参数

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `approx_sync` | `true` | 启用近似时间戳同步 |
| `approx_sync_max_interval` | `0.1` s | 允许彩色帧与深度帧最大时间戳差（Orbbec 需要 100ms） |
| `queue_size` | `20` | 同步队列大小 |

---

## RViz 可视化配置

启动后在 RViz 中添加以下显示：

| 显示类型 | 话题 | 说明 |
|---------|------|------|
| `MarkerArray` | `/semantic_map/markers` | 3D 彩色贴地方块 + 文字标签 |
| `Image` | `/semantic_map/image` | 带语义标注的 2D 占用栅格俯视图 |
| `Map` | `/map` | 原始占用栅格地图 |
| `PointCloud2` | `/rtabmap/cloud_map` | RTAB-Map 3D 点云地图 |
| `MarkerArray` | `/explore/frontiers` | 当前候选前沿点（绿色球体，仅方式四） |

> **重要**：Global Options → **Fixed Frame** 必须设为 `map`，否则 MapCloudDisplay 会报 TF 外推错误。

---

## 地图数据查看

RTAB-Map 建图数据保存在：

```
~/.ros/rtabmap.db
```

### 使用官方查看器

```bash
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

打开后的常用操作：

| 菜单 | 功能 |
|------|------|
| Edit → View 3D Map | 查看完整 3D 点云地图 |
| Edit → View Occupancy Grid | 查看 2D 占用栅格 |
| 左侧节点列表 | 逐帧浏览每个关键帧的图像和点云 |
| Edit → Export Poses | 导出机器人轨迹 |

### 导出点云为 PLY 文件

```bash
rtabmap-export --cloud --cloud_voxel 0.05 --output_dir ~/map_export ~/.ros/rtabmap.db
# 输出：~/map_export/cloud.ply
```

或在 rtabmap-databaseViewer 图形界面中：`Edit → View 3D Map` 加载完成后 → `File → Export 3D map`

### 导出 2D 语义地图图像

语义地图图像发布在 `/semantic_map/image` 话题，系统运行时保存：

```bash
# 保存单帧（需安装 image_view）
ros2 run image_view image_saver --ros-args \
  -r image:=/semantic_map/image \
  -p filename_format:=/home/wheeltec/map_export/semantic_map_%04d.png
# 按 Ctrl+C 停止，图像保存在指定路径
```

也可录制 bag 后离线提取：

```bash
# 录制
ros2 bag record /semantic_map/image /map -o ~/map_export/semantic_bag

# 离线提取为 PNG（需 Python + cv_bridge）
python3 - <<'EOF'
import rclpy, cv2
from cv_bridge import CvBridge
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

reader = SequentialReader()
reader.open(StorageOptions(uri='/home/wheeltec/map_export/semantic_bag', storage_id='sqlite3'),
            ConverterOptions('', ''))
bridge = CvBridge()
idx = 0
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic == '/semantic_map/image':
        img = bridge.imgmsg_to_cv2(deserialize_message(data, Image), 'bgr8')
        cv2.imwrite(f'/home/wheeltec/map_export/semantic_{idx:04d}.png', img)
        idx += 1
print(f'Exported {idx} frames')
EOF
```

---

## 话题列表

### 订阅

| 话题 | 类型 | 节点 |
|------|------|------|
| `/yolo/detections_3d` | `yolo_msgs/DetectionArray` | `semantic_map_node` |
| `/map` | `nav_msgs/OccupancyGrid` | `semantic_map_node`, `frontier_explorer_node` |
| `/navigate_to_object/goal` | `std_msgs/String` | `object_navigator_node`（物体类别名） |
| `/navigate_to_object/cancel` | `std_msgs/Empty` | `object_navigator_node`（取消当前任务） |
| `/explore/resume` | `std_msgs/Bool` | `frontier_explorer_node`（true = 开始, false = 暂停） |
| `/explore/status` | `explore_lite_msgs/ExploreStatus` | `object_navigator_node`（可选，缺失时降级为超时） |

### 发布

| 话题 | 类型 | 节点 | 说明 |
|------|------|------|------|
| `/semantic_map/markers` | `visualization_msgs/MarkerArray` | `semantic_map_node` | RViz 3D 贴地方块 + 文字标注 |
| `/semantic_map/image` | `sensor_msgs/Image` (bgr8) | `semantic_map_node` | 2D 标注地图图像 |
| `/semantic_map/objects` | `std_msgs/String` (JSON) | `semantic_map_node` | 当前地图中所有物体的列表 |
| `/navigate_to_object/feedback` | `std_msgs/String` (JSON) | `object_navigator_node` | 导航任务实时状态 |
| `/navigate_to_object/result` | `std_msgs/String` (JSON) | `object_navigator_node` | 导航任务最终结果 |
| `/explore/resume` | `std_msgs/Bool` | `object_navigator_node` | 控制探索启停（True = 开始，False = 暂停） |
| `/explore/status` | `explore_lite_msgs/ExploreStatus` | `frontier_explorer_node` | 探索状态（`exploration_complete` 等） |
| `/explore/frontiers` | `visualization_msgs/MarkerArray` | `frontier_explorer_node` | 当前候选前沿点，供 RViz 调试 |

### 验证话题输出

```bash
# 查看语义对象数量（每次刷新应有数据）
ros2 topic echo /semantic_map/markers --once

# 查看话题频率
ros2 topic hz /semantic_map/markers
ros2 topic hz /rgbd_image
ros2 topic hz /yolo/detections_3d

# 确认探索节点已收到地图（方式四）
ros2 topic echo /explore/frontiers --once
```

---

## 常见问题排查

### rtabmap 报 "Did not receive data since 5 seconds"

rgbd_sync 同步失败，彩色帧和深度帧时间戳差超过阈值。

```bash
# 确认三个输入话题都在发布
ros2 topic hz /odom_combined   # 应约 10 Hz
ros2 topic hz /rgbd_image      # 应稳定 6-15 Hz，std dev < 0.05s
ros2 topic hz /scan            # 应约 10 Hz
```

解决：已将 `approx_sync_max_interval` 从 0.05 改为 0.1，重新编译后生效。

---

### RViz 报 "Lookup would require extrapolation into the past"

RViz 的 Fixed Frame 设置为 `base_link` 导致。

**解决**：RViz → Global Options → Fixed Frame 改为 `map`。

---

### TF 树断链（map frame 不存在）

```bash
# 检查 TF 树是否完整
ros2 run tf2_tools view_frames

# 验证 map → base_link 变换是否可用
ros2 run tf2_ros tf2_echo map base_link
```

TF 链应为：`map → odom_combined → base_footprint → base_link`

---

### 物体导航发送目标后机器人不动

节点日志显示 `Exploration started (/explore/resume = true)` 但机器人静止不动。

**解决**：确认使用方式四（`wheeltec_explore_rtab.launch.py`）启动，它内置了 `frontier_explorer_node`。
验证探索节点是否在运行：

```bash
ros2 node list | grep frontier
# 应看到 /frontier_explorer_node
```

若单独使用 `object_navigation.launch.py`，需要手动启动探索节点：

```bash
ros2 run wheeltec_robot_rtab frontier_explorer_node.py
```

---

### 前沿探索节点启动后不发送目标

`frontier_explorer_node` 默认处于 `IDLE` 状态，需要收到 `/explore/resume = true` 才开始探索。使用方式四时由 `object_navigator_node` 自动触发；手动测试：

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "data: true"
```

---

### YOLO 节点无检测输出

99% 是 conda 环境未激活。确保：

```bash
conda activate yolo          # 先激活 conda
source install/setup.bash    # 再 source ROS2
```

---

### 相机深度帧不稳定（抖动）

检查 USB 连接：

```bash
lsusb -t   # Orbbec 应出现在 Bus 002（5000M USB3 总线）
```

若相机在 USB2 口（Bus 001），换插 USB3 口即可解决抖动问题。

---

## 点云质量调优

当 3D 点云效果不理想时，可调整以下参数：

| 问题 | 参数 | 建议值 |
|------|------|--------|
| 点云太密/太慢 | `cloud_voxel_size` | 增大至 `0.08`~`0.1` |
| 远处噪声点多 | `Grid/RangeMax` | 减小至 `3.0` |
| 点云漂移累积 | `RGBD/LinearUpdate` | 增大至 `0.2`，减少节点数 |
| 孤立噪声点多 | `Grid/NoiseFilteringMinNeighbors` | 增大至 `10` |
| z 轴叠影（多层） | `Reg/Force3DoF` | 改为 `false`（仅多层建筑场景） |
| 回环闭合被误拒 | `RGBD/OptimizeMaxError` | 增大至 `8.0` |
| 回环闭合误触发 | `Vis/MinInliers` | 增大至 `30` |

修改参数后重新编译：

```bash
colcon build --packages-select wheeltec_robot_rtab
source install/setup.bash
```

建图时删除旧数据库（参数 `-d` 已在 SLAM 模式中自动传入，重启即清空）。
