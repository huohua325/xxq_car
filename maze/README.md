# 墙跟随探索系统 - 主机端代码

> **项目目标**：实现从入口到出口的自主探索 + 路径回放返回  
> **技术方案**：墙跟随算法 + 停顿扫描策略 + 实时SLAM建图  
> **硬件平台**：STM32F446 + 360°激光雷达 + MPU6500

---

## 📁 目录结构

```
maze/
├── README.md                      # 本文件
├── simple_pose_estimator.py       # 位姿估计器（基于ODO数据）
├── simple_lidar.py                # 雷达数据包装器
├── simple_motion.py               # 运动控制包装器
├── simple_wall_follower.py        # 墙跟随算法（待开发）
├── simple_explorer.py             # 探索主循环（待开发）
└── tests/                         # 测试脚本
    ├── test_pose.py
    ├── test_lidar.py
    └── test_motion.py
```

---

## 🚀 快速开始

### 1. 安装依赖

```bash
# 需要bleak库（蓝牙通信）
pip install bleak numpy
```

### 2. 测试基础模块

```python
# 复制 ble_robot_control.py 到 maze/ 目录
# 然后测试各个模块
cd maze
python tests/test_pose.py
python tests/test_lidar.py
python tests/test_motion.py
```

---

## 📊 标定参数

### 转向参数（来自运动参数标定结果.md）

| 转向 | 左轮PWM | 右轮PWM | 持续时间 | 精度 |
|-----|---------|---------|---------|------|
| 左转90° | 0.85 | 0.18 | 1.0秒 | ±2° |
| 右转90° | 0.50 | 0.45 | 0.73秒 | ±3° |
| **左转45°** | 0.85 | 0.18 | **0.5秒** | ±2° |
| **右转45°** | 0.50 | 0.45 | **0.365秒** | ±3° |

**注**：45度转向时间 = 90度时间 / 2

### 前进参数

| 持续时间 | 实际距离 | 实际速度 | 推荐场景 |
|---------|---------|---------|---------|
| 0.8秒 | 26.3 cm | 32.9 cm/s | 墙跟随探索 |
| 1.0秒 | 31.9 cm | 31.9 cm/s | 快速探索 |

---

## 🔧 机器人参数

```python
# 必须与固件同步
WHEEL_BASE = 0.185      # 轮距（米）
WHEEL_RADIUS = 0.033    # 轮半径（米）
LEFT_PPR = 1560         # 左轮编码器分辨率
RIGHT_PPR = 780         # 右轮编码器分辨率
```

---

## 📡 通信协议

### 蓝牙配置
- **设备地址**：C4:25:01:20:02:8E
- **波特率**：115200 bps

### 命令格式

| 命令 | 格式 | 说明 |
|-----|------|------|
| 前进 | `MODE,1\n` | PID速度控制 |
| 后退 | `MODE,2\n` | PID速度控制 |
| 停止 | `MODE,0\n` | 停止运动 |
| 左转 | `TURN,0,{left_pwm},{right_pwm}\n` | 左转 |
| 右转 | `TURN,1,{left_pwm},{right_pwm}\n` | 右转 |
| 雷达扫描 | `A\n` | 请求360°扫描 |

### 数据格式

**ODO数据**（自动发送，10Hz）：
```
ODO,{timestamp},{left_rps},{right_rps},{left_count},{right_count}
```

**雷达数据**（JSON格式）：
```json
{
  "sectors": [
    {"sector_id": 0, "angle_center": 0, "min_dist": 1.2, "avg_dist": 1.5, "count": 15},
    ...
  ]
}
```

---

## 🎯 开发进度

- [x] **阶段一**：硬件端完善（已完成标定）
- [x] **阶段二**：Python基础功能
  - [x] SimplePoseEstimator
  - [x] SimpleLidarWrapper
  - [x] SimpleMotionController
- [ ] **阶段三**：墙跟随算法
- [ ] **阶段四**：实时SLAM可视化
- [ ] **阶段五**：路径回放返回
- [ ] **阶段六**：集成测试与优化

---

## 📝 注意事项

1. **所有命令必须以 `\n` 结尾**
2. **电池电量保持 >50%**，低电量影响转速
3. **地面环境应与标定环境一致**
4. **转向间隔 ≥0.5秒**，避免惯性影响

---

**创建日期**：2025-10-13  
**文档版本**：v1.0

