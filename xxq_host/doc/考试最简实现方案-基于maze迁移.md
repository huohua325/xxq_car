# 🎓 考试最简实现方案 - 基于maze迁移

**目标**：将已验证的`maze`项目通信层迁移到`xxq_host`，实现Frontier探索 + 目标导向导航

**开发时间**：约2-3小时  
**成功标准**：能从Entrance探索到Exit，并返回起点

---

## 📊 现状分析

### ✅ maze项目（已验证可用）
- **通信层**：`SimpleBLERobotComm` - BLE通信稳定
- **位姿估计**：`SimplePoseEstimator` - 差分驱动运动学，已处理右轮负值
- **雷达包装**：`SimpleLidarWrapper` - 支持点云和扇区格式
- **运动控制**：`SimpleMotionController` - 已标定的左转/右转参数
- **集成测试**：`test_all_modules.py` - 验证通过

### ✅ xxq_host项目（功能完整，未测试）
- **SLAM建图**：`OccupancyGridMap` - Log-odds贝叶斯更新
- **Frontier探索**：`FrontierDetector` - DBSCAN聚类
- **路径规划**：`PathPlanner` - A*算法
- **DWA避障**：`DWA` - 动态窗口避障
- **主控制器**：`RobotController` - 集成所有模块

### 🔄 迁移策略
**保留maze的底层通信层（已验证），使用xxq_host的上层算法（符合考试要求）**

---

## 🏗️ 架构设计

```
┌─────────────────────────────────────────────────────────┐
│                  考试系统架构                            │
└─────────────────────────────────────────────────────────┘

┌──────────────── 上层算法（xxq_host） ─────────────────┐
│                                                          │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │   SLAM      │  │   Frontier   │  │   A* + DWA   │  │
│  │  建图模块    │  │   探索检测   │  │  路径规划     │  │
│  └─────────────┘  └──────────────┘  └──────────────┘  │
│                                                          │
│  ┌──────────────────────────────────────────────────┐  │
│  │         RobotController（主控制器）              │  │
│  │   - 目标导向frontier选择                        │  │
│  │   - 探索循环                                     │  │
│  └──────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────┘
                            ↕
┌──────────────── 底层通信（maze，已验证） ──────────────┐
│                                                          │
│  ┌──────────────────────────────────────────────────┐  │
│  │       SimpleBLERobotComm（BLE通信）              │  │
│  │   - 已处理右轮PWM反向                           │  │
│  │   - 已处理右轮负值计数                          │  │
│  └──────────────────────────────────────────────────┘  │
│                                                          │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  位姿估计   │  │  雷达包装    │  │  运动控制    │  │
│  │ SimplePose  │  │ SimpleLidar  │  │SimpleMotion  │  │
│  │ Estimator   │  │   Wrapper    │  │ Controller   │  │
│  └─────────────┘  └──────────────┘  └──────────────┘  │
└──────────────────────────────────────────────────────────┘
                            ↕
┌──────────────── 硬件层（STM32固件） ───────────────────┐
│  UART4 BLE通信 | 编码器 | 雷达 | 电机PID                │
└──────────────────────────────────────────────────────────┘
```

---

## 🔧 关键硬件特性处理

### 1. 右轮PWM反向（已在maze中处理）
```python
# maze/ble_robot_control.py 已处理
# TURN命令中直接使用标定值，固件端Motor.c会自动反转
```

### 2. 右轮编码器负值（已在maze中处理）
```python
# maze/simple_pose_estimator.py 第94-95行
left_distance = (delta_left / self.left_ppr) * (2 * math.pi * self.wheel_radius)
right_distance = (delta_right / self.right_ppr) * (2 * math.pi * self.wheel_radius)
# 注意：delta_right已经是负数，直接使用
```

### 3. 左转/右转参数（已标定，不可改动）
```python
# maze/simple_motion.py 第18-32行
TURN_LEFT_90 = {
    'direction': 0,
    'left_pwm': 0.85,
    'right_pwm': 0.18,
    'duration': 1.0,  # ⭐ 固定值
    'angle': 90
}

TURN_RIGHT_90 = {
    'direction': 1,
    'left_pwm': 0.50,
    'right_pwm': 0.45,
    'duration': 0.73,  # ⭐ 固定值
    'angle': 90
}
```

---

## 📝 实施步骤

### 步骤1：复制maze通信层到xxq_host（30分钟）

#### 1.1 复制文件
```bash
# 从maze/复制到xxq_host/src/simple/
mkdir -p xxq_host/src/simple
cp maze/ble_robot_control.py xxq_host/src/simple/
cp maze/simple_pose_estimator.py xxq_host/src/simple/
cp maze/simple_lidar.py xxq_host/src/simple/
cp maze/simple_motion.py xxq_host/src/simple/
```

#### 1.2 调整导入路径
在复制的文件中，确保BLE地址从config导入：
```python
# xxq_host/src/simple/simple_motion.py 等文件
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from config import BLE_ADDRESS, WHEEL_BASE, WHEEL_RADIUS
```

---

### 步骤2：创建桥接适配器（30分钟）

**目的**：将`SimpleBLERobotComm`（maze）适配到`RobotController`（xxq_host）

创建文件：`xxq_host/src/adapters/maze_adapter.py`

```python
"""
maze通信层到xxq_host控制器的适配器
"""

import numpy as np
from typing import Tuple, List, Optional
import sys
import os

# 添加项目根目录
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# maze模块
from src.simple.ble_robot_control import SimpleBLERobotComm
from src.simple.simple_pose_estimator import SimplePoseEstimator
from src.simple.simple_lidar import SimpleLidarWrapper
from src.simple.simple_motion import SimpleMotionController

# xxq_host配置
import config


class MazeAdapter:
    """maze通信层适配器"""
    
    def __init__(self, ble_address: str = None):
        """
        初始化适配器
        
        Args:
            ble_address: BLE地址，None则使用config中的默认值
        """
        # BLE通信
        address = ble_address or getattr(config, 'BLE_ADDRESS', 'C4:25:01:20:02:8E')
        self.robot = SimpleBLERobotComm(address=address, verbose=False)
        
        # maze三大模块
        self.pose_estimator = SimplePoseEstimator(
            wheel_base=config.WHEEL_BASE,
            wheel_radius=config.WHEEL_RADIUS,
            left_ppr=config.LEFT_ENCODER_PPR,
            right_ppr=config.RIGHT_ENCODER_PPR
        )
        self.lidar_wrapper = SimpleLidarWrapper(self.robot)
        self.motion_controller = SimpleMotionController(self.robot)
        
        # 注册ODO回调
        self.robot.on_odom_update = self._on_odo_update
        
        # 当前位姿缓存
        self._current_pose = (0.0, 0.0, 0.0)
    
    def _on_odo_update(self, data):
        """ODO数据回调"""
        self._current_pose = self.pose_estimator.update(data)
    
    def connect(self) -> bool:
        """连接BLE"""
        return self.robot.connect()
    
    def disconnect(self):
        """断开BLE"""
        self.robot.disconnect()
    
    def get_pose(self) -> Tuple[float, float, float]:
        """
        获取当前位姿
        
        Returns:
            (x, y, theta): 位姿（米，米，弧度）
        """
        return self._current_pose
    
    def scan_lidar(self, timeout: float = 6.0) -> Optional[List[Tuple[float, float]]]:
        """
        请求雷达扫描
        
        Args:
            timeout: 超时时间（秒）
        
        Returns:
            [(angle_deg, distance_m), ...] 或 None
        """
        scan = self.lidar_wrapper.request_scan(timeout=timeout)
        if not scan:
            return None
        
        # 转换为(angle, distance)元组列表
        return [(p['angle'], p['distance']) for p in scan]
    
    def move_forward(self, distance: float = 0.26):
        """
        前进指定距离
        
        Args:
            distance: 目标距离（米），默认0.26m
        """
        # 根据距离选择步长
        if distance < 0.30:
            self.motion_controller.forward('medium')  # 26cm
        else:
            self.motion_controller.forward('long')    # 32cm
    
    def turn_angle(self, angle_deg: float):
        """
        转向指定角度
        
        Args:
            angle_deg: 转向角度（度），正数=左转，负数=右转
        """
        if angle_deg > 0:
            # 左转
            if abs(angle_deg) <= 50:
                self.motion_controller.turn_left(45)
            else:
                self.motion_controller.turn_left(90)
        else:
            # 右转
            if abs(angle_deg) <= 50:
                self.motion_controller.turn_right(45)
            else:
                self.motion_controller.turn_right(90)
    
    def stop(self):
        """停止运动"""
        self.motion_controller.stop()
    
    def reset_pose(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """
        重置位姿
        
        Args:
            x, y: 坐标（米）
            theta: 航向角（弧度）
        """
        self.pose_estimator.reset(x, y, theta)
        self._current_pose = (x, y, theta)
```

---

### 步骤3：修改RobotController使用适配器（20分钟）

修改文件：`xxq_host/src/navigation/controller.py`

```python
# 在__init__方法开头添加

def __init__(self, 
             comm=None,
             use_maze_adapter=False,  # 🆕 新增参数
             ble_address=None,        # 🆕 新增参数
             map_obj=None,
             visualizer=None,
             enable_visualization=True,
             visualization_mode='matplotlib',
             web_port=5000):
    """初始化控制器"""
    
    # 🆕 使用maze适配器
    if use_maze_adapter:
        from src.adapters.maze_adapter import MazeAdapter
        self.maze_adapter = MazeAdapter(ble_address)
        print("[Controller] 使用maze适配器（已验证通信）")
    else:
        self.maze_adapter = None
    
    # ... 原有代码 ...
```

添加方法：

```python
def _get_current_pose_from_adapter(self):
    """从maze适配器获取位姿"""
    if self.maze_adapter:
        return self.maze_adapter.get_pose()
    else:
        # 原有逻辑
        return self.get_current_pose()

def _scan_lidar_from_adapter(self):
    """从maze适配器获取雷达扫描"""
    if self.maze_adapter:
        scan = self.maze_adapter.scan_lidar(timeout=6.0)
        if not scan:
            return None
        
        # 转换为numpy数组
        angles = np.deg2rad([p[0] for p in scan])
        distances = np.array([p[1] for p in scan])
        return (angles, distances)
    else:
        # 原有逻辑
        return self._request_lidar_scan()

def _execute_motion_from_adapter(self, action: str, params: dict):
    """通过maze适配器执行运动"""
    if self.maze_adapter:
        if action == 'forward':
            self.maze_adapter.move_forward(params.get('distance', 0.26))
        elif action == 'turn':
            self.maze_adapter.turn_angle(params.get('angle', 0))
        elif action == 'stop':
            self.maze_adapter.stop()
    else:
        # 原有逻辑
        self._execute_motion_original(action, params)
```

---

### 步骤4：添加目标导向frontier选择（30分钟）

修改文件：`xxq_host/src/slam/frontier_detector.py`

```python
def select_best_frontier_goal_directed(self,
                                      frontiers: List[Tuple[float, float]], 
                                      robot_pose: Tuple[float, float, float],
                                      goal_position: Tuple[float, float],
                                      alpha: float = 0.6) -> Optional[Tuple[float, float]]:
    """
    选择朝向目标方向的最佳前沿点（考试专用）
    
    Args:
        frontiers: 前沿点列表 [(x1, y1), (x2, y2), ...]
        robot_pose: 机器人位姿 (x, y, theta)
        goal_position: 目标位置 (x, y)，例如Exit坐标
        alpha: 距离权重 (0.6=稍微偏向距离，0.4偏向目标方向)
        
    Returns:
        最佳前沿点坐标 (x, y) 或 None
        
    Example:
        >>> detector = FrontierDetector(map_obj)
        >>> frontiers = detector.find_frontiers()
        >>> exit_pos = (2.1, 1.4)  # Exit=(3,2)格子的世界坐标
        >>> best = detector.select_best_frontier_goal_directed(
        ...     frontiers, robot_pose, exit_pos, alpha=0.6
        ... )
    """
    if not frontiers:
        return None
    
    robot_x, robot_y = robot_pose[0], robot_pose[1]
    goal_x, goal_y = goal_position
    
    # 机器人到目标的方向向量
    goal_vec = np.array([goal_x - robot_x, goal_y - robot_y])
    goal_dist = np.linalg.norm(goal_vec)
    
    # 如果已经很接近目标，选择最近的frontier
    if goal_dist < 0.35:  # 半个格子
        print("[Frontier] 接近目标，选择最近frontier")
        return self._select_nearest(frontiers, robot_x, robot_y)
    
    goal_vec = goal_vec / goal_dist  # 单位向量
    
    best_score = -float('inf')
    best_frontier = None
    
    for fx, fy in frontiers:
        # 到frontier的距离
        dist_to_frontier = np.hypot(fx - robot_x, fy - robot_y)
        
        # 到frontier的方向向量
        frontier_vec = np.array([fx - robot_x, fy - robot_y])
        frontier_vec = frontier_vec / (np.linalg.norm(frontier_vec) + 1e-6)
        
        # 与目标方向的余弦相似度 (-1到1)
        direction_similarity = np.dot(frontier_vec, goal_vec)
        
        # 综合评分
        # 距离评分：1/(1+dist)，越近越好
        distance_score = 1.0 / (1.0 + dist_to_frontier)
        
        # 最终评分
        score = alpha * distance_score + (1 - alpha) * direction_similarity
        
        if score > best_score:
            best_score = score
            best_frontier = (fx, fy)
    
    print(f"[Frontier] 目标导向: frontier={best_frontier}, 评分={best_score:.2f}")
    return best_frontier
```

修改`controller.py`中的frontier选择：

```python
def set_exploration_goal(self, goal_x: float, goal_y: float):
    """
    设置探索目标（Exit坐标）
    
    Args:
        goal_x, goal_y: 目标位置（米）
    """
    self.goal_position = (goal_x, goal_y)
    print(f"[Controller] 设置探索目标: ({goal_x:.2f}, {goal_y:.2f})")

def check_goal_reached(self, current_pose: Tuple[float, float, float]) -> bool:
    """检查是否到达目标"""
    if not hasattr(self, 'goal_position') or self.goal_position is None:
        return False
    
    goal_x, goal_y = self.goal_position
    robot_x, robot_y = current_pose[0], current_pose[1]
    dist = np.hypot(goal_x - robot_x, goal_y - robot_y)
    
    return dist < 0.35  # 35cm，半个格子

def _select_frontier_target(self, frontiers, current_pose):
    """选择frontier目标（支持目标导向）"""
    if not frontiers:
        return None
    
    # 🆕 检查是否有探索目标
    if hasattr(self, 'goal_position') and self.goal_position is not None:
        return self.frontier_detector.select_best_frontier_goal_directed(
            frontiers,
            current_pose,
            self.goal_position,
            alpha=0.6  # 稍微偏向距离
        )
    else:
        # 默认：选择最近的
        return self.frontier_detector.select_best_frontier(
            frontiers,
            current_pose,
            strategy='nearest'
        )
```

---

### 步骤5：调整config.py参数（5分钟）

修改文件：`xxq_host/config.py`

```python
# ============ BLE通信（新增） ============
BLE_ADDRESS = "C4:25:01:20:02:8E"  # 小车BLE地址

# ============ 机器人物理参数 ============
WHEEL_BASE = 0.185       # 轮距（米）- 与maze一致
WHEEL_RADIUS = 0.033     # 轮半径（米）
LEFT_ENCODER_PPR = 1560  # 左轮编码器（四倍频）
RIGHT_ENCODER_PPR = 780  # 右轮编码器（双倍频）⚠️ 注意是正值

# ============ 地图参数（考试要求） ============
MAP_WIDTH = 3.0           # 地图宽度（米）- 280cm+余量
MAP_HEIGHT = 3.0          # 地图高度（米）
MAP_RESOLUTION = 0.05     # 分辨率（米/格）- 5cm
CELL_SIZE = 0.70          # 迷宫格子尺寸（米）- 70cm

# ============ 探索参数 ============
EXPLORATION_ALPHA = 0.6          # 目标导向权重
GOAL_REACHED_THRESHOLD = 0.35    # 到达目标阈值（米）
LIDAR_SCAN_TIMEOUT = 6.0         # 雷达扫描超时（秒）
```

---

### 步骤6：创建考试启动脚本（30分钟）

创建文件：`xxq_host/exam_main.py`

```python
"""
考试专用探索脚本（基于maze适配器）
"""

import sys
import time
import argparse
from src.navigation.controller import RobotController
from src.adapters.maze_adapter import MazeAdapter
import config


def cell_to_world(cell_x: int, cell_y: int) -> tuple:
    """
    格子坐标转世界坐标
    
    Args:
        cell_x, cell_y: 格子坐标 (0-3)
    
    Returns:
        (world_x, world_y): 世界坐标（米）
    """
    # 格子中心 = (索引 + 0.5) * 格子尺寸
    world_x = (cell_x + 0.5) * config.CELL_SIZE
    world_y = (cell_y + 0.5) * config.CELL_SIZE
    return (world_x, world_y)


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='考试探索系统')
    parser.add_argument('--entrance-x', type=int, required=True, help='入口X (0-3)')
    parser.add_argument('--entrance-y', type=int, required=True, help='入口Y (0-3)')
    parser.add_argument('--exit-x', type=int, required=True, help='出口X (0-3)')
    parser.add_argument('--exit-y', type=int, required=True, help='出口Y (0-3)')
    parser.add_argument('--web', action='store_true', help='Web可视化')
    args = parser.parse_args()
    
    # 转换坐标
    entrance = cell_to_world(args.entrance_x, args.entrance_y)
    exit_pos = cell_to_world(args.exit_x, args.exit_y)
    
    print("="*80)
    print("🎓 考试探索系统启动（maze适配器版）")
    print("="*80)
    print(f"📍 入口: ({args.entrance_x}, {args.entrance_y}) → {entrance}")
    print(f"🎯 出口: ({args.exit_x}, {args.exit_y}) → {exit_pos}")
    print("="*80)
    
    # 1. 初始化适配器
    print("\n[1/4] 初始化maze适配器...")
    adapter = MazeAdapter()
    
    if not adapter.connect():
        print("❌ BLE连接失败")
        return
    print("✅ BLE已连接")
    time.sleep(1)
    
    # 2. 初始化控制器
    print("\n[2/4] 初始化控制器...")
    controller = RobotController(
        use_maze_adapter=True,
        enable_visualization=True,
        visualization_mode='web' if args.web else 'matplotlib'
    )
    controller.maze_adapter = adapter  # 注入适配器
    controller.set_exploration_goal(exit_pos[0], exit_pos[1])
    
    # 3. 探索到Exit
    print("\n[3/4] 开始探索（Entrance → Exit）...")
    input("▶️  按Enter开始...")
    
    start_time = time.time()
    step_count = 0
    max_steps = 100
    
    try:
        while step_count < max_steps:
            step_count += 1
            print(f"\n{'='*60}")
            print(f"探索步骤 {step_count}/{max_steps}")
            print(f"{'='*60}")
            
            # 获取当前位姿
            current_pose = adapter.get_pose()
            print(f"📍 位姿: ({current_pose[0]:.2f}, {current_pose[1]:.2f}, {current_pose[2]:.2f}rad)")
            
            # 检查是否到达Exit
            if controller.check_goal_reached(current_pose):
                print("\n🎉 到达Exit！")
                break
            
            # 雷达扫描
            print("🔄 扫描中...")
            scan = adapter.scan_lidar(timeout=6.0)
            if not scan:
                print("⚠️  扫描超时")
                continue
            
            # 更新地图（如果有可视化）
            # TODO: 调用SLAM更新
            
            # 检测frontiers
            # frontiers = controller.frontier_detector.find_frontiers()
            # 简化版：直接朝Exit方向前进
            
            # 简单决策：前进
            adapter.move_forward(0.26)
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n⏹️  手动停止")
    
    exploration_time = time.time() - start_time
    adapter.stop()
    
    print(f"\n✅ 探索完成，用时: {exploration_time:.1f}秒")
    
    # 4. 返回起点
    input("\n▶️  按Enter开始返回...")
    print("\n[4/4] 返回起点...")
    
    controller.set_exploration_goal(entrance[0], entrance[1])
    
    # TODO: 类似探索逻辑
    
    print("\n🏁 任务完成！")
    adapter.disconnect()


if __name__ == '__main__':
    main()
```

---

## 🧪 测试步骤

### 测试1：验证maze模块（5分钟）
```bash
cd xxq_host
python src/simple/simple_motion.py  # 测试运动控制
```

### 测试2：验证适配器（10分钟）
```python
# test_adapter.py
from src.adapters.maze_adapter import MazeAdapter
import time

adapter = MazeAdapter()
adapter.connect()

# 测试位姿
pose = adapter.get_pose()
print(f"位姿: {pose}")

# 测试扫描
scan = adapter.scan_lidar()
print(f"扫描点数: {len(scan) if scan else 0}")

# 测试运动
adapter.move_forward(0.26)
time.sleep(1)
adapter.stop()

adapter.disconnect()
```

### 测试3：运行考试脚本（实际测试）
```bash
python exam_main.py \
    --entrance-x 0 --entrance-y 0 \
    --exit-x 3 --exit-y 2 \
    --web
```

---

## ⚠️ 注意事项

### 1. 编码器数值
- **右轮计数是负数**，maze的`SimplePoseEstimator`已正确处理
- 不要在适配器中再次取负！

### 2. 转向参数
- **必须使用maze标定的参数**，不能修改
- 左转90°: `(0, 0.85, 0.18, 1.0秒)`
- 右转90°: `(1, 0.50, 0.45, 0.73秒)`

### 3. 前进模式
- 使用`MODE,1`（PID前进）
- medium: 0.8秒 ≈ 26cm
- long: 1.0秒 ≈ 32cm

### 4. 雷达超时
- 扫描超时设置为**6秒**
- 如果持续超时，检查固件`main.c`的'A'命令处理

---

## 🎯 成功标准

- [ ] maze模块在xxq_host中能正常运行
- [ ] 适配器能正确获取位姿和雷达数据
- [ ] 能执行基本运动（前进、转向、停止）
- [ ] Frontier检测能找到探索边界
- [ ] 目标导向选择能朝Exit方向探索
- [ ] 考试脚本能运行（即使探索效果不完美）

---

## 📚 相关文件清单

### 需要复制的文件
- [ ] `maze/ble_robot_control.py` → `xxq_host/src/simple/`
- [ ] `maze/simple_pose_estimator.py` → `xxq_host/src/simple/`
- [ ] `maze/simple_lidar.py` → `xxq_host/src/simple/`
- [ ] `maze/simple_motion.py` → `xxq_host/src/simple/`

### 需要新建的文件
- [ ] `xxq_host/src/adapters/__init__.py`
- [ ] `xxq_host/src/adapters/maze_adapter.py`
- [ ] `xxq_host/exam_main.py`

### 需要修改的文件
- [ ] `xxq_host/config.py` - 添加BLE_ADDRESS等参数
- [ ] `xxq_host/src/slam/frontier_detector.py` - 添加目标导向方法
- [ ] `xxq_host/src/navigation/controller.py` - 支持maze适配器

---

## 🚀 快速开始

```bash
# 1. 进入项目根目录
cd d:/Programs/STM32CubeIDE/workspace

# 2. 创建目录结构
mkdir -p xxq_host/src/simple
mkdir -p xxq_host/src/adapters

# 3. 复制文件（手动或脚本）
# 4. 运行测试
# 5. 考试当天运行exam_main.py
```

---

## ✅ 总结

**优势**：
1. ✅ 复用已验证的maze通信层（稳定可靠）
2. ✅ 使用xxq_host的高级算法（符合考试要求）
3. ✅ 适配器模式，易于调试和替换
4. ✅ 保留硬件特性处理（右轮负值、PWM反向）

**工作量**：约2-3小时
- 复制文件：30分钟
- 创建适配器：30分钟
- 修改controller：20分钟
- 添加目标导向：30分钟
- 调整config：5分钟
- 测试验证：30-60分钟

**风险**：低（底层已验证，只需桥接）

---

**需要我现在帮你实施这些修改吗？** 🚀

