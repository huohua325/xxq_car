# xxq_host API 文档

## 📋 目录

- [通信模块](#通信模块)
- [SLAM模块](#slam模块)
- [导航模块](#导航模块)
- [可视化模块](#可视化模块)
- [工具模块](#工具模块)

---

## 通信模块

### `communication.protocol`

数据协议定义模块。

#### 数据类

##### `LidarData`
雷达扫描数据。

```python
from communication.protocol import LidarData

lidar = LidarData(
    timestamp=12345,          # 时间戳 (ms)
    total_points=800,         # 总点数
    angle_coverage=360.0,     # 角度覆盖 (度)
    sectors=[                 # 扇区数据列表
        {
            'sector_id': 0,
            'angle_center': 0,
            'count': 100,
            'min_dist': 0.5,
            'avg_dist': 2.0
        }
    ]
)
```

##### `MPUData`
MPU6050姿态数据。

```python
from communication.protocol import MPUData

mpu = MPUData(
    timestamp=12345,          # 时间戳 (ms)
    roll=0.5,                 # 横滚角 (度)
    pitch=-1.2,               # 俯仰角 (度)
    accel=[0.01, -0.02, 9.8], # 加速度 [x, y, z] (m/s²)
    gyro=[0.1, -0.1, 0.05]    # 角速度 [x, y, z] (rad/s)
)
```

##### `OdometryData`
里程计数据。

```python
from communication.protocol import OdometryData

odom = OdometryData(
    timestamp=12345,          # 时间戳 (ms)
    left_speed=1.5,           # 左轮速度 (RPS)
    right_speed=1.5,          # 右轮速度 (RPS)
    left_count=1000,          # 左轮编码器计数
    right_count=1000          # 右轮编码器计数
)
```

##### `RobotMode`
机器人运行模式枚举。

```python
from communication.protocol import RobotMode

mode = RobotMode.AUTO_NAV   # 自动导航模式
# 可选值:
# - STOP: 停止
# - FORWARD: 前进
# - BACKWARD: 后退
# - TURN_LEFT: 左转
# - TURN_RIGHT: 右转
# - AUTO_NAV: 自动导航
```

### `communication.robot_comm`

机器人通信管理模块（需要硬件）。

---

## SLAM模块

### `slam.occupancy_map`

占据栅格地图实现。

#### `MapConfig`
地图配置类。

```python
from slam.occupancy_map import MapConfig

config = MapConfig(
    width=500,              # 宽度 (格)
    height=500,             # 高度 (格)
    resolution=0.1,         # 分辨率 (米/格)
    origin=None,            # 原点 (默认中心)
    log_odds_max=100.0,     # 最大log-odds值
    log_odds_min=-100.0,    # 最小log-odds值
    prob_hit=0.7,           # 击中概率
    prob_miss=0.4           # 未击中概率
)
```

#### `OccupancyGridMap`
占据栅格地图类。

##### 初始化
```python
from slam.occupancy_map import OccupancyGridMap, MapConfig

config = MapConfig(width=500, height=500, resolution=0.1)
map = OccupancyGridMap(config)
```

##### 主要方法

**坐标转换**
```python
# 世界坐标 → 栅格坐标
gx, gy = map.world_to_grid(1.5, 2.3)

# 栅格坐标 → 世界坐标
wx, wy = map.grid_to_world(250, 250)

# 检查栅格有效性
is_valid = map.is_valid_grid(gx, gy)
```

**地图更新**
```python
# 使用雷达数据更新地图
from communication.protocol import LidarData

lidar = LidarData(...)
robot_pose = (0.0, 0.0, 0.0)  # (x, y, theta)
map.update_with_lidar(lidar, robot_pose)

# 直接更新单个栅格
map.update_cell(gx, gy, is_obstacle=True)
```

**地图查询**
```python
# 获取二值地图 (0=自由, 100=障碍, 50=未知)
binary_map = map.get_binary_map()

# 获取自由空间地图 (True=自由)
free_space = map.get_free_space_map()

# 检查占据状态
is_occupied = map.is_occupied(wx, wy)
is_free = map.is_free(wx, wy)

# 获取统计信息
stats = map.get_statistics()
# 返回: {'total_cells', 'free_cells', 'obstacle_cells', 'unknown_cells'}
```

**地图保存/加载**
```python
# 保存地图
map.save_map('data/maps/my_map.npy')

# 加载地图
map.load_map('data/maps/my_map.npy')
```

### `slam.frontier_detector`

前沿点检测与探索。

#### `FrontierDetector`

##### 初始化
```python
from slam.frontier_detector import FrontierDetector

detector = FrontierDetector(
    map=map,
    cluster_distance=0.3,    # 聚类距离 (米)
    min_cluster_size=5       # 最小簇大小
)
```

##### 主要方法

**查找前沿点**
```python
# 基本查找
frontiers = detector.find_frontiers(robot_pose)
# 返回: [(x1, y1), (x2, y2), ...]

# 带详细信息查找
frontiers, info = detector.find_frontiers(robot_pose, return_info=True)
# info = {
#     'clusters': [...],        # 簇列表
#     'cluster_sizes': [...],   # 簇大小
#     'cluster_centers': [...]  # 簇中心
# }
```

**选择最佳前沿点**
```python
# 最近策略
target = detector.select_best_frontier(
    frontiers, robot_pose, 
    strategy='nearest'
)

# 最大策略
target = detector.select_best_frontier(
    frontiers, robot_pose, 
    strategy='largest',
    frontier_info=info
)

# 信息增益策略
target = detector.select_best_frontier(
    frontiers, robot_pose, 
    strategy='information_gain',
    frontier_info=info
)
```

**计算信息增益**
```python
gain = detector.calculate_information_gain(frontier_point)
```

---

## 导航模块

### `navigation.path_planner`

全局路径规划。

#### `PathPlanner`

##### 初始化
```python
from navigation.path_planner import PathPlanner

planner = PathPlanner(
    map=map,
    safety_distance=0.2,     # 安全距离 (米)
    smoothing_epsilon=0.1    # 平滑参数
)
```

##### 主要方法

**A*路径规划**
```python
start = (0.0, 0.0)
goal = (5.0, 5.0)

path = planner.plan_path(start, goal)
# 返回: [(x1, y1), (x2, y2), ...] 或 None (无路径)
```

**路径平滑**
```python
smooth_path = planner.smooth_path(path, epsilon=0.1)
```

**障碍物膨胀**
```python
inflated_map = planner.inflate_obstacles(radius=3)
```

**路径验证**
```python
is_valid = planner.is_path_valid(path)
```

### `navigation.dwa`

动态窗口法局部避障。

#### `DWA`

##### 初始化
```python
from navigation.dwa import DWA

dwa = DWA(
    map=map,
    max_speed=1.0,           # 最大速度 (m/s)
    max_yaw_rate=40.0,       # 最大角速度 (度/s)
    max_accel=0.5,           # 最大加速度 (m/s²)
    max_delta_yaw_rate=20.0, # 最大角加速度 (度/s²)
    v_resolution=0.1,        # 速度分辨率
    yaw_rate_resolution=5.0, # 角速度分辨率 (度)
    dt=0.1,                  # 预测时间步长 (秒)
    predict_time=1.0,        # 预测时间 (秒)
    heading_weight=1.0,      # 航向权重
    distance_weight=1.0,     # 距离权重
    velocity_weight=1.0      # 速度权重
)
```

##### 主要方法

**局部规划**
```python
robot_pose = (0.0, 0.0, 0.0)  # (x, y, theta_deg)
current_v = 0.5               # 当前速度 (m/s)
current_w = 0.0               # 当前角速度 (度/s)
goal = (5.0, 5.0)             # 目标点

# 规划最优速度
best_v, best_w = dwa.plan(robot_pose, current_v, current_w, goal)

# 如果无解
if best_v is None:
    print("无法找到安全路径")
```

**轨迹预测**
```python
trajectory = dwa._predict_trajectory(
    x=0.0, y=0.0, theta=0.0,
    v=0.5, w=10.0
)
# 返回: [(x1, y1, theta1), (x2, y2, theta2), ...]
```

### `navigation.controller`

主控制循环。

#### `RobotController`

##### 初始化
```python
from navigation.controller import RobotController
from slam.occupancy_map import OccupancyGridMap, MapConfig

config = MapConfig(width=500, height=500, resolution=0.1)
slam_map = OccupancyGridMap(config)

controller = RobotController(
    slam_map=slam_map,
    visualize=True
)
```

##### 主要方法

**运行探索**
```python
# 完整探索流程
controller.run_exploration(
    initial_pose=(0.0, 0.0, 0.0),
    max_steps=1000
)
```

**单步控制**
```python
# 执行单步控制循环
state = controller._control_step()
# 返回: RobotState 枚举
```

##### 状态机

```python
from navigation.controller import RobotState

# 状态定义:
# - IDLE: 空闲
# - EXPLORING: 探索中
# - NAVIGATING: 导航中
# - STUCK: 卡住
# - COMPLETED: 完成
```

---

## 可视化模块

### `visualization.map_visualizer`

地图可视化。

#### `MapVisualizer`

##### 初始化
```python
from visualization.map_visualizer import MapVisualizer

viz = MapVisualizer(
    map=slam_map,
    figsize=(10, 10),
    style='default'  # 'default' | 'dark' | 'seaborn'
)
```

##### 主要方法

**更新显示**
```python
robot_pose = (0.0, 0.0, 0.0)

viz.update(
    robot_pose,
    lidar_points=[(1.0, 0.0), (0.0, 1.0)],  # 可选
    path=[(0, 0), (1, 1), (2, 2)],          # 可选
    frontiers=[(3, 3), (4, 4)]              # 可选
)
```

**保存截图**
```python
viz.save_screenshot('output.png')
```

**重置**
```python
viz.reset()
```

**关闭**
```python
viz.close()
```

---

## 工具模块

### `utils.logger`

日志系统。

```python
from utils.logger import setup_logger

logger = setup_logger(
    name='my_module',
    log_file='data/logs/my_module.log',
    level='INFO'  # 'DEBUG' | 'INFO' | 'WARNING' | 'ERROR'
)

logger.info("信息日志")
logger.warning("警告日志")
logger.error("错误日志")
logger.debug("调试日志")
```

### `utils.data_recorder`

数据记录器（需要硬件）。

```python
from utils.data_recorder import DataRecorder

recorder = DataRecorder(output_dir='data/recordings')
recorder.start_recording()
# ... 记录数据 ...
recorder.stop_recording()
```

---

## 完整示例

### 示例1: SLAM建图

```python
from slam.occupancy_map import OccupancyGridMap, MapConfig
from communication.protocol import LidarData

# 创建地图
config = MapConfig(width=500, height=500, resolution=0.1)
slam_map = OccupancyGridMap(config)

# 模拟雷达数据
lidar = LidarData(
    timestamp=1000,
    total_points=800,
    angle_coverage=360.0,
    sectors=[
        {'sector_id': 0, 'angle_center': 0, 'count': 100,
         'min_dist': 1.0, 'avg_dist': 2.0}
    ]
)

# 更新地图
robot_pose = (0.0, 0.0, 0.0)
slam_map.update_with_lidar(lidar, robot_pose)

# 保存地图
slam_map.save_map('data/maps/my_map.npy')
```

### 示例2: Frontier探索

```python
from slam.occupancy_map import OccupancyGridMap, MapConfig
from slam.frontier_detector import FrontierDetector

# 创建地图和探测器
config = MapConfig(width=200, height=200, resolution=0.1)
slam_map = OccupancyGridMap(config)
detector = FrontierDetector(slam_map)

# 查找前沿点
robot_pose = (0.0, 0.0, 0.0)
frontiers, info = detector.find_frontiers(robot_pose, return_info=True)

# 选择目标
target = detector.select_best_frontier(
    frontiers, robot_pose, 
    strategy='information_gain',
    frontier_info=info
)

print(f"目标前沿点: {target}")
```

### 示例3: 路径规划与避障

```python
from slam.occupancy_map import OccupancyGridMap, MapConfig
from navigation.path_planner import PathPlanner
from navigation.dwa import DWA

# 创建地图和规划器
config = MapConfig(width=200, height=200, resolution=0.1)
slam_map = OccupancyGridMap(config)
planner = PathPlanner(slam_map)
dwa = DWA(slam_map)

# 全局路径规划
start = (0.0, 0.0)
goal = (5.0, 5.0)
path = planner.plan_path(start, goal)

# 路径平滑
smooth_path = planner.smooth_path(path)

# 局部避障
robot_pose = (0.0, 0.0, 0.0)
current_v = 0.5
current_w = 0.0
best_v, best_w = dwa.plan(robot_pose, current_v, current_w, goal)

print(f"最优速度: v={best_v} m/s, w={best_w} deg/s")
```

### 示例4: 完整自主探索

```python
from slam.occupancy_map import OccupancyGridMap, MapConfig
from navigation.controller import RobotController

# 创建地图
config = MapConfig(width=500, height=500, resolution=0.1)
slam_map = OccupancyGridMap(config)

# 创建控制器
controller = RobotController(slam_map, visualize=True)

# 运行探索
controller.run_exploration(
    initial_pose=(0.0, 0.0, 0.0),
    max_steps=1000
)
```

---

## 配置参数

主要配置参数在 `config.py` 中：

```python
# SLAM配置
MAP_WIDTH = 500
MAP_HEIGHT = 500
MAP_RESOLUTION = 0.1

# DWA参数
MAX_SPEED = 1.0
MAX_YAW_RATE = 40.0
MAX_ACCEL = 0.5

# Frontier检测
FRONTIER_CLUSTER_DIST = 0.3
MIN_FRONTIER_SIZE = 5
FRONTIER_SELECTION = 'nearest'

# 可视化
VISUALIZE_RATE = 30
SHOW_LIDAR_POINTS = True
SHOW_PATH = True
```

---

## 性能指标

- **地图更新**: < 10ms (500x500栅格)
- **前沿检测**: < 50ms
- **A*路径规划**: < 100ms (典型场景)
- **DWA局部规划**: < 20ms
- **可视化刷新**: 30-60 fps
- **内存占用**: < 200MB (典型场景)

---

## 测试

运行所有测试:
```bash
pytest tests/ -v
```

运行特定模块测试:
```bash
pytest tests/test_slam.py -v
pytest tests/test_frontier.py -v
pytest tests/test_navigation.py -v
pytest tests/test_integration.py -v
```

---

## 演示脚本

```bash
# Frontier探索演示
python scripts/demo_frontier.py

# 路径规划演示
python scripts/demo_path_planning.py

# 完整探索演示
python scripts/demo_exploration.py

# 可视化演示
python scripts/demo_visualization.py
```

---

## 版本信息

- **版本**: 1.0.0
- **Python**: 3.11+
- **主要依赖**: numpy, matplotlib, pytest

---

## 许可证

本项目仅供学习研究使用。

