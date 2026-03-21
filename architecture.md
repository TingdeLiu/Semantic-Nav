# 项目架构

## 项目概述

`wheeltec_semantic_map` 是一个 ROS2 Python 包，将 YOLO v8 三维目标检测与 RTAB-Map 二维占用栅格地图深度融合，构建持久化语义地图。系统将检测到的物体叠加到占用栅格上，并通过 TF2 坐标变换保持空间一致性，以 RViz 3D 方块标注、二维标注图像、JSON 物体列表三种形式输出语义信息。在此基础上，`object_navigator_node` 可查询语义地图并驱动 Nav2 自动导航到目标物体。

## 技术栈

- **编程语言**: Python 3.8+
- **机器人框架**: ROS2 (ament_python 构建系统)
- **SLAM 系统**: RTAB-Map（LiDAR+RGB-D 或纯 RGB-D 模式）
- **目标检测**: YOLO v8（yolo_bringup 封装）
- **计算机视觉**: OpenCV (python3-opencv)、cv_bridge
- **坐标变换**: TF2 (tf2_ros, tf2_geometry_msgs)
- **数值计算**: NumPy
- **消息类型**: nav_msgs, sensor_msgs, visualization_msgs, geometry_msgs, yolo_msgs

## 项目结构

```
wheeltec_semantic_map/
├── README.md                          # 项目说明文档
├── package.xml                        # ROS2 包元数据与依赖声明
├── setup.py                           # Python 包安装配置
├── setup.cfg                          # 入口点配置（节点可执行文件）
├── resource/
│   └── wheeltec_semantic_map          # ROS2 ament 资源标记文件
├── launch/
│   ├── semantic_map.launch.py           # 仅语义映射（SLAM 单独运行）
│   ├── semantic_slam.launch.py          # 完整栈：LiDAR+RGB-D SLAM + YOLO
│   ├── semantic_slam_pure3d.launch.py   # 完整栈：纯 RGB-D SLAM + YOLO
│   ├── object_navigation.launch.py      # 语义映射 + 物体搜索导航
│   └── wheeltec_semantic_nav.launch.py  # 定位 + 加载语义地图 + 物体导航（方式四B）
└── wheeltec_semantic_map/
    ├── __init__.py                    # Python 包初始化
    ├── semantic_map_node.py           # 核心节点实现（语义地图构建）
    └── object_navigator_node.py       # 物体搜索与自主导航节点
```

## 架构总览

本包采用**双节点协作**架构：`SemanticMapNode` 负责感知融合与地图维护，`ObjectNavigatorNode` 负责任务规划与自主导航。

```
┌──────────────────────────────────────────────────────┐
│                    外部上游系统                        │
│  ┌──────────────┐         ┌─────────────────────┐    │
│  │  YOLO 检测器  │         │   RTAB-Map SLAM      │    │
│  │ (yolo_bringup)│         │(wheeltec_robot_rtab) │    │
│  └──────┬───────┘         └──────────┬───────────┘    │
│         │ /detections_3d             │ /map            │
└─────────┼──────────────────────────-┼────────────────-┘
          │                            │
          ▼                            ▼
┌─────────────────────────────────────────────────────┐
│               SemanticMapNode (核心节点)              │
│                                                      │
│  ┌─────────────┐  ┌──────────────┐  ┌────────────┐  │
│  │  检测回调    │  │  地图回调    │  │  定时发布  │  │
│  │_on_detections│  │  _on_map()  │  │ _publish() │  │
│  └──────┬──────┘  └──────┬───────┘  └─────┬──────┘  │
│         │                │                 │          │
│         ▼                ▼                 ▼          │
│  ┌──────────────────────────────────────────────┐    │
│  │           内部对象追踪存储                    │    │
│  │   Dict[tracking_id → SemanticObject]          │    │
│  └──────────────────────────────────────────────┘    │
└──────────────┬───────────────────┬───────────────────┘
               │                   │
               ▼                   ▼
   ┌──────────────────────┐  ┌─────────────────────┐
   │  /semantic_map/image  │  │ /semantic_map/objects│
   │  (sensor_msgs/Image)  │  │  (std_msgs/String)  │
   │   2D 标注地图图像      │  │   JSON 物体列表      │
   └──────────────────────┘  └──────────┬──────────┘
              │
              ▼
┌─────────────────────────────────────────────┐
│          ObjectNavigatorNode                 │
│  状态机: IDLE→SEARCHING→NAVIGATING/EXPLORING │
│                                              │
│  ┌──────────────────┐  ┌──────────────────┐ │
│  │  查询语义地图     │  │    触发自动探索        │ │
│  │  _find_in_map()  │  │ (frontier_explorer) │ │
│  └──────────────────┘  └──────────────────┘ │
│  ┌──────────────────────────────────────┐   │
│  │  Flask Web UI (端口 8080)             │   │
│  └──────────────────────────────────────┘   │
└──────────────────────┬──────────────────────┘
                       │ NavigateToPose action
                       ▼
               ┌──────────────┐
               │     Nav2     │
               └──────────────┘
```

## 处理流水线

```
摄像头 & 深度传感器
        │
        ▼
┌───────────────┐     ┌──────────────┐
│  YOLO v8 检测 │     │ RTAB-Map SLAM │
│  (目标识别)   │     │  (构建地图)   │
└───────┬───────┘     └──────┬───────┘
        │ DetectionArray     │ OccupancyGrid
        │ (3D 边界框)         │ (/map)
        ▼                    ▼
┌───────────────────────────────────────┐
│         SemanticMapNode               │
│                                       │
│  [置信度过滤] ──▶ [TF2 坐标变换]      │
│        ↓                              │
│  [EMA 位置平滑] ──▶ [对象字典更新]   │
│        ↓                              │
│  [主动消失检测] + [超时过期清除]      │
│        ↓                              │
│  [定时触发发布]                       │
│   ├── [生成 RViz 3D 方块标注]         │
│   ├── [渲染 2D 标注地图图像]          │
│   └── [输出 JSON 物体列表]            │
└───────────────────────────────────────┘
        │                   │
        ▼                   ▼
  /semantic_map/      /semantic_map/
      image               objects
  (Image BGR8)        (String JSON)
  [2D 地图预览]       [物体查询接口]
```

## 核心组件

### SemanticObject（数据类）

- **位置**: `wheeltec_semantic_map/semantic_map_node.py`
- **用途**: 表示地图坐标系中已检测并追踪的语义对象
- **关键字段**:
  - `class_id`, `class_name` - 物体类别标识与名称
  - `tracking_id` - 跨帧唯一追踪 ID
  - `x`, `y`, `z` - 地图坐标系三维坐标（米）
  - `score` - 检测置信度（0.0~1.0）
  - `last_seen` - 最后一次检测时间戳

### SemanticMapNode（主节点）

- **位置**: `wheeltec_semantic_map/semantic_map_node.py`
- **节点名**: `semantic_map_node`
- **用途**: 融合 YOLO 检测与 SLAM 地图，构建并发布语义地图
- **关键文件**:
  - `semantic_map_node.py` - 全部核心逻辑（约 300+ 行）
- **订阅话题**:
  - `/detections_3d` (yolo_msgs/DetectionArray) - YOLO 3D 检测结果
  - `/map` (nav_msgs/OccupancyGrid) - SLAM 占用栅格地图
- **发布话题**:
  - `/semantic_map/image` (sensor_msgs/Image) - 2D 标注地图图像
  - `/semantic_map/objects` (std_msgs/String) - JSON 格式的物体列表（供其他节点查询）
- **关键参数**:
  - `semantic_map_file` - 持久化路径（默认 `~/.ros/semantic_map.json`）
  - `load_on_startup` - 启动时从文件加载已保存对象（方式四B 设为 `true`）
  - `autosave_period` - 定期自动保存间隔（秒）；`0` 仅在关闭时保存
- **关键方法**:
  - `_load_semantic_map()` - 从 JSON 读取并恢复 `SemanticObject` 列表（`load_on_startup=true` 时调用）
  - `_save_semantic_map()` - 将 `self.objects` 序列化为 JSON 写入磁盘
  - `destroy_node()` - 节点关闭时先调用 `_save_semantic_map()`，再调用 `super().destroy_node()`
- **依赖关系**: TF2 变换树（`map` ← `base_link` ← 传感器坐标系）

### 启动配置组件

- **位置**: `launch/`
- **用途**: 提供五种场景的完整系统启动方案
- **关键文件**:
  - `semantic_map.launch.py` - 仅启动语义映射（外部运行 SLAM）
  - `semantic_slam.launch.py` - 完整栈（LiDAR+RGB-D SLAM + YOLO）
  - `semantic_slam_pure3d.launch.py` - 完整栈（纯 RGB-D SLAM + YOLO）
  - `object_navigation.launch.py` - 语义映射 + 物体搜索导航
  - `wheeltec_semantic_nav.launch.py` - **定位模式**：加载已保存的 RTAB-Map 数据库和语义地图 JSON，直接进行语义导航（方式四B）
- **依赖关系**: `yolo_bringup`、`wheeltec_robot_rtab`

## 架构模式

- **发布-订阅模式**: 节点通过 ROS2 话题解耦接收检测与地图数据，异步发布语义输出
- **双重物体清除**: 两种互补机制维护地图新鲜度：① 主动消失检测——机器人重新扫描某区域时若物体未出现则立即删除；② 超时清除——`object_timeout` 秒内未被任何帧检测到的物体定期删除（设为 0 禁用）
- **位置平滑（EMA）**: 已知物体更新时使用指数移动平均（alpha=0.4）融合新旧测量值，抑制深度相机帧间抖动，提高位置稳定性
- **坐标系变换**: 使用 TF2 将传感器坐标系的检测结果统一变换到 `map` 坐标系
- **一致性哈希着色**: 通过类别名的哈希值分配 HSV 颜色，保证同类物体颜色一致且无需配置
- **双路输出**: 同时输出 2D 标注图像（`/semantic_map/image`）和 JSON 物体列表（`/semantic_map/objects`）
- **速率限制发布**: 发布频率由 `publish_rate` 参数独立控制，与输入频率解耦
- **JSON 语义地图持久化**: `SemanticMapNode` 支持启动时从 JSON 加载、运行时定期自动保存、关闭时通过 `destroy_node()` 钩子安全写盘，三重保障数据不丢失；与 RTAB-Map 数据库配合形成**两模式工作流**（方式四A 实时建图→自动保存；方式四B 加载上次保存结果→直接导航）
- **状态机导航**: `ObjectNavigatorNode` 通过显式状态机协调地图查询、Nav2 导航与自动探索三个阶段

## 数据流

```
传感器数据
    │
    ├─[RGB 图像]──────────────────▶ YOLO 检测器
    │                                    │
    ├─[深度图像]──────────────────▶ YOLO 3D 定位
    │                                    │
    │                           /detections_3d
    │                                    │
    ├─[点云/RGB-D]────────────▶ RTAB-Map SLAM ──▶ /map
    │                                    │           │
    │                                    ▼           ▼
    │                          ┌──────────────────────┐
    │                          │    SemanticMapNode    │
    │                          │  1. 置信度过滤        │
    │                          │  2. TF2 坐标变换      │
    │                          │  3. EMA 位置平滑      │
    │                          │  4. 对象字典更新      │
    │                          │  5. 主动消失检测      │
    │                          │  6. 超时过期清除      │
    │                          │  7. 定时发布输出      │
    │                          └──────────┬───────────┘
    │                                     │
    │                          ┌──────────────────────────────┐
    │                          ▼                              ▼
    │              /semantic_map/image          /semantic_map/objects
    │              (2D 标注地图图像)              (JSON 物体列表)
    └──────────────────────────────────────────────────────────
```

## 配置说明

### 节点参数

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `target_frame` | `map` | 语义对象的目标坐标系 |
| `detection_topic` | `/detections_3d` | YOLO 检测输入话题 |
| `map_topic` | `/map` | 占用栅格地图输入话题 |
| `object_timeout` | `120.0` (秒) | 超过此时间未被检测到的物体自动删除；设为 `0` 仅依赖主动消失检测 |
| `min_score` | `0.8` | 进入语义地图的最低置信度；独立于 YOLO 的 `threshold`，可单独调节 |
| `publish_rate` | `2.0` (Hz) | 语义地图发布频率 |
| `merge_distance` | `0.6` (米) | 空间去重半径，距离内同类物体合并为一个 |
| `semantic_map_file` | `~/.ros/semantic_map.json` | 语义地图持久化路径（保存与加载共用） |
| `load_on_startup` | `false` | 启动时从文件加载已保存对象（方式四B 设为 `true`） |
| `autosave_period` | `0.0` (秒) | 定期自动保存间隔；`0` = 仅在关闭时保存 |

### 启动参数（semantic_map.launch.py）

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `model` | `yolov8m.pt` | YOLO 模型文件路径 |
| `device` | `cuda:0` | 推理设备（GPU/CPU） |
| `threshold` | `0.7` | YOLO 发布检测结果的置信度门槛 |
| `min_score` | `0.8` | 进入语义地图的置信度门槛（高于 threshold，二级过滤） |
| `target_frame` | `map` | 目标坐标系 |
| `object_timeout` | `120.0` | 超时自动删除物体；设为 `0` 禁用 |
| `publish_rate` | `2.0` | 发布频率 |

### 启动参数（wheeltec_semantic_nav.launch.py，方式四B）

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `semantic_map_file` | `~/.ros/semantic_map.json` | 方式四A 保存的语义地图 JSON，启动即加载 |
| `autosave_period` | `30.0` | 运行中定期保存（秒） |
| `model` | `~/wheeltec_ros2/yolov8m.pt` | YOLO 模型 |
| `device` | `cuda:0` | 推理设备 |
| `threshold` | `0.7` | YOLO 置信度门槛 |
| `min_score` | `0.8` | 进入语义地图的最低置信度 |
| `approach_distance` | `0.8` | 导航到物体时的停止距离（米） |
| `search_timeout` | `300.0` | 物体搜索总超时（秒） |

### ObjectNavigatorNode 节点参数

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `approach_distance` | `0.8` (米) | 导航目标点距物体的停止距离 |
| `search_timeout` | `300.0` (秒) | 整体任务超时时间 |
| `search_grace_period` | `5.0` (秒) | 首次搜索等待窗口；期间若地图中无目标则继续等待，超时后才触发探索 |
| `web_port` | `8080` | Flask Web UI 端口 |
| `web_enabled` | `true` | 是否启动 Web 控制界面 |

## 构建与部署

> **重要**：YOLO 依赖 conda 独立环境。所有包含 YOLO 的启动方式必须先激活 conda 环境，再 source ROS2，否则 YOLO 节点将无法订阅相机话题，导致无检测输出。

```bash
# 构建包
cd ~/wheeltec_ros2
colcon build --packages-select wheeltec_semantic_map

# 加载环境（必须先激活 conda）
conda activate yolo
source ~/wheeltec_ros2/install/setup.bash

# 方式1：仅语义映射（需外部运行 SLAM）
ros2 launch wheeltec_semantic_map semantic_map.launch.py

# 方式2：完整栈（LiDAR+RGB-D SLAM + YOLO + 语义映射）
ros2 launch wheeltec_semantic_map semantic_slam.launch.py

# 方式3：纯 RGB-D 完整栈
ros2 launch wheeltec_semantic_map semantic_slam_pure3d.launch.py

# 方式四A：实时建图 + 自主探索（SLAM + Nav2 + 前沿探索 + YOLO + 语义地图 + 物体导航）
# 终端1
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
# 终端2
ros2 launch wheeltec_robot_rtab wheeltec_explore_rtab.launch.py
# Ctrl+C 关闭后自动保存 ~/.ros/rtabmap.db 和 ~/.ros/semantic_map.json

# 方式四B：定位 + 加载语义地图（无需重新建图）
# 终端1
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
# 终端2
ros2 launch wheeltec_semantic_map wheeltec_semantic_nav.launch.py
```

## 关键设计决策

1. **双节点分工**: `SemanticMapNode` 专注感知融合，`ObjectNavigatorNode` 专注任务执行，职责清晰、独立可测
2. **双重物体清除**: 主动消失检测（扫描时未出现则立即删除）与 `object_timeout` 超时删除互补：前者响应快，后者兜底处理机器人不重访的区域
3. **EMA 位置平滑**: 已知物体更新时以 alpha=0.4 混合新旧测量值，减少深度相机噪声引起的位置抖动，提升语义地图空间一致性
4. **TF2 延迟容忍**: 坐标变换使用 0.15 秒超时，变换不可用时跳过当前检测而非报错，提高系统稳定性
5. **频率解耦**: 输入话题（YOLO 检测通常 10-30 Hz）与输出发布（默认 2 Hz）频率独立，避免下游系统过载
6. **五种启动模式**: 分离 SLAM 与语义映射，支持在已有 SLAM 系统上直接集成语义层，灵活适配不同硬件配置
7. **两阶段工作流（建图→导航）**: 方式四A（实时 SLAM + 探索）与方式四B（定位 + 加载语义地图）共享同一套 JSON 持久化接口，使机器人只需探索一次环境，此后每次启动直接加载历史语义地图进行导航，大幅缩短部署前的准备时间

## 测试策略

本包为 ROS2 集成包，建议测试方法：

- **话题验证**: 使用 `ros2 topic echo` 验证 `/semantic_map/objects` 和 `/semantic_map/image` 输出
- **RViz 可视化**: 添加 MarkerArray 和 Image 显示类型进行实时验证
- **参数调优**: 通过调整 `min_score`、`merge_distance` 观察语义地图质量变化
- **bag 文件回放**: 录制传感器数据后离线重放，验证检测与地图融合效果
