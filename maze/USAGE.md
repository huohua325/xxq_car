# 使用指南 - 阶段二模块测试

## 📁 文件结构

```
maze/
├── README.md                      # 项目总览
├── USAGE.md                       # 本文件（使用指南）
├── __init__.py                    # 包初始化
├── ble_robot_control.py           # BLE通信模块（从根目录复制）
├── simple_pose_estimator.py       # 位姿估计器 ⭐
├── simple_lidar.py                # 雷达数据包装器 ⭐
├── simple_motion.py               # 运动控制包装器 ⭐
└── tests/
    ├── __init__.py
    ├── test_pose_only.py          # 位姿估计器单元测试（无需硬件）
    └── test_all_modules.py        # 集成测试（需要硬件）
```

---

## 🚀 快速测试

### 1. 位姿估计器单元测试（无需硬件）

```bash
cd maze
python tests/test_pose_only.py
```

**测试内容**：
- 直线前进1米
- 原地左转90度
- 组合运动（前进+转向+前进）
- 统计信息

**期望输出**：显示位姿估计结果，验证算法正确性

---

### 2. 集成测试（需要硬件）

```bash
cd maze
python tests/test_all_modules.py
```

**测试内容**：
- 前进（medium步长）→ 雷达扫描
- 左转45° → 前进 → 雷达扫描
- 右转45° → 前进 → 雷达扫描

**期望结果**：
- 小车执行完整运动序列
- 实时更新位姿估计
- 成功获取雷达数据
- 显示详细统计信息

---

## 📊 模块使用示例

### SimplePoseEstimator - 位姿估计器

```python
from simple_pose_estimator import SimplePoseEstimator
from ble_robot_control import SimpleBLERobotComm

# 创建位姿估计器
pose_estimator = SimplePoseEstimator()

# 创建BLE连接
robot = SimpleBLERobotComm('C4:25:01:20:02:8E')

# 注册ODO回调
def on_odo_update(data):
    pose_estimator.update(data)
    x, y, theta = pose_estimator.get_pose_degrees()
    print(f"位姿: x={x:.3f}m, y={y:.3f}m, θ={theta:.1f}°")

robot.on_odom_update = on_odo_update

# 连接并运动
robot.connect()
robot.send_command("MODE,1\n")  # 前进
time.sleep(1)
robot.send_command("MODE,0\n")  # 停止

# 获取最终位姿
final_pose = pose_estimator.get_pose()
print(f"最终位姿: {final_pose}")

robot.disconnect()
```

---

### SimpleLidarWrapper - 雷达数据包装器

```python
from simple_lidar import SimpleLidarWrapper
from ble_robot_control import SimpleBLERobotComm

robot = SimpleBLERobotComm('C4:25:01:20:02:8E')
robot.connect()

# 创建雷达包装器
lidar = SimpleLidarWrapper(robot)

# 请求扫描
scan = lidar.request_scan(timeout=3.0)

if scan:
    print(f"扫描点数: {len(scan)}")
    
    # 查询特定方向距离
    front = lidar.get_distance_at_angle(0)      # 正前方
    left = lidar.get_distance_at_angle(90)      # 左侧
    right = lidar.get_distance_at_angle(270)    # 右侧
    
    print(f"前方: {front:.2f}m")
    print(f"左侧: {left:.2f}m")
    print(f"右侧: {right:.2f}m")
    
    # 获取障碍物概览
    obstacles = lidar.get_obstacles_summary()
    print(obstacles)
    
    # 终端可视化
    lidar.visualize_scan()

robot.disconnect()
```

---

### SimpleMotionController - 运动控制包装器

```python
from simple_motion import SimpleMotionController
from ble_robot_control import SimpleBLERobotComm

robot = SimpleBLERobotComm('C4:25:01:20:02:8E')
robot.connect()

# 创建运动控制器
motion = SimpleMotionController(robot)

# 基础运动
motion.forward('medium')    # 前进26cm
motion.turn_left_45()       # 左转45度
motion.forward('long')      # 前进32cm
motion.turn_right_45()      # 右转45度
motion.stop()

# 90度转向
motion.turn_left_90()       # 左转90度
motion.turn_right_90()      # 右转90度

# 后退
motion.backward(0.5)        # 后退0.5秒

# 获取统计
stats = motion.get_statistics()
print(f"总运动次数: {stats['total_motions']}")

robot.disconnect()
```

---

## 🎯 三个模块联合使用示例

```python
from ble_robot_control import SimpleBLERobotComm
from simple_pose_estimator import SimplePoseEstimator
from simple_lidar import SimpleLidarWrapper
from simple_motion import SimpleMotionController
import time

# 初始化
robot = SimpleBLERobotComm('C4:25:01:20:02:8E')
pose_estimator = SimplePoseEstimator()
lidar = SimpleLidarWrapper(robot)
motion = SimpleMotionController(robot)

# 注册ODO回调
robot.on_odom_update = lambda data: pose_estimator.update(data)

# 连接
robot.connect()

# 执行一个简单的探索循环
for i in range(3):
    print(f"\n步骤 {i+1}/3")
    
    # 1. 扫描环境
    print("  扫描中...")
    scan = lidar.request_scan()
    
    if scan:
        front = lidar.get_distance_at_angle(0)
        right = lidar.get_distance_at_angle(270)
        
        print(f"  前方距离: {front:.2f}m")
        print(f"  右侧距离: {right:.2f}m")
        
        # 2. 决策
        if front < 0.3:
            print("  前方障碍物，左转45°")
            motion.turn_left_45()
        elif right > 0.6:
            print("  右侧空旷，右转45°")
            motion.turn_right_45()
        else:
            print("  直行")
            motion.forward('medium')
    
    # 3. 显示当前位姿
    x, y, theta = pose_estimator.get_pose_degrees()
    print(f"  当前位姿: ({x:.3f}, {y:.3f}, {theta:.1f}°)")
    
    time.sleep(0.5)

# 断开
robot.disconnect()

# 显示统计
print("\n统计信息:")
print(f"  总行驶距离: {pose_estimator.total_distance:.3f}m")
print(f"  雷达扫描次数: {lidar.get_scan_count()}")
print(f"  运动次数: {motion.get_statistics()['total_motions']}")
```

---

## 📝 注意事项

### 1. 运动参数

**前进步长**：
- `'medium'`：0.8秒，26.3cm（推荐墙跟随）
- `'long'`：1.0秒，31.9cm（快速探索）

**转向角度**：
- 45度转向：推荐用于灵活探索
- 90度转向：用于大角度调整

**转向参数**（已标定）：
```python
# 左转45度
motion.turn_left(45)   # 或 motion.turn_left_45()

# 右转45度
motion.turn_right(45)  # 或 motion.turn_right_45()

# 左转90度
motion.turn_left(90)   # 或 motion.turn_left_90()

# 右转90度
motion.turn_right(90)  # 或 motion.turn_right_90()
```

### 2. 雷达角度定义

```
         0° (正前方)
            ↑
            |
90° (左侧) ← + → 270° (右侧)
            |
            ↓
        180° (后方)
```

### 3. 位姿估计误差

- 位姿估计基于编码器，会有**累积误差**
- 长时间运行后误差可能达到10-20cm
- 这是正常现象，后续阶段会添加修正机制

### 4. BLE连接

- 确保小车电池电量 >50%
- 确保BLE地址正确：`C4:25:01:20:02:8E`
- 如果连接失败，检查：
  - Windows蓝牙是否开启
  - 小车是否开机
  - 是否有其他程序占用BLE连接

---

## ✅ 验收标准

**阶段二完成标志**：

- [x] SimplePoseEstimator 单元测试通过
- [x] SimpleLidarWrapper 能成功获取雷达数据
- [x] SimpleMotionController 能执行45度和90度转向
- [x] 三个模块集成测试通过
- [x] 所有测试脚本能正常运行

**下一步**：开发墙跟随算法（阶段三）

---

## 🐛 故障排查

### 问题1: `ModuleNotFoundError: No module named 'bleak'`

**解决**：
```bash
pip install bleak numpy
```

### 问题2: 位姿估计器显示 (0, 0, 0)

**原因**：ODO数据未更新  
**解决**：
1. 检查是否注册了ODO回调：`robot.on_odom_update = ...`
2. 检查固件是否自动发送ODO数据

### 问题3: 雷达扫描超时

**原因**：
1. 硬件雷达未连接
2. 固件未实现雷达扫描功能
3. BLE连接不稳定

**解决**：
1. 检查雷达硬件连接
2. 运行 `test_lidar.py`（根目录）验证雷达功能
3. 重新连接BLE

### 问题4: 转向角度不准

**原因**：电池电量、地面摩擦力影响  
**解决**：
1. 充电至 >50%
2. 在平坦地面测试
3. 如果持续偏差，调整 `simple_motion.py` 中的 `duration` 参数

---

**创建日期**：2025-10-13  
**文档版本**：v1.0  
**对应阶段**：阶段二

