# xxq智能小车项目

> 🚗 基于STM32的SLAM自主导航智能小车系统

---

## 📁 项目结构

```
workspace/
├── xxq/                    # 🔧 STM32固件端（硬件控制）
│   ├── Core/              # 固件核心代码
│   │   └── Src/main.c    # ⭐ 主程序（包含通信协议）
│   └── README.md          # 固件使用说明
│
├── xxq_host/              # 🐍 Python主机端（智能决策）
│   ├── src/              # 源代码
│   │   ├── slam/         # SLAM建图模块
│   │   ├── navigation/   # 路径规划与导航
│   │   ├── visualization/ # 可视化显示
│   │   └── communication/ # 通信协议
│   ├── tests/            # 测试代码
│   ├── scripts/          # 演示脚本
│   ├── doc/              # 完整文档
│   └── README.md         # 主机端使用说明
│
└── 系统集成总结报告.md   # 📊 集成状态报告
```

---

## 🎯 各部分作用

### 1. **xxq** - STM32固件端（底层硬件控制）

**作用**：
- 🚗 控制电机运动（PID速度控制）
- 📡 采集雷达数据（360°扫描）
- 🧭 读取姿态传感器（MPU6500）
- 📏 测量轮速编码器
- 📱 与Python主机通信（蓝牙串口）

**主要功能**：
```c
// 发送传感器数据给Python
- 雷达数据 → JSON格式
- 姿态数据 → CSV格式  
- 里程计 → CSV格式

// 接收Python控制命令
- NAV命令 → 导航控制
- SPD命令 → 速度控制
- MODE命令 → 模式切换
```

---

### 2. **xxq_host** - Python主机端（智能大脑）

**作用**：
- 🗺️ SLAM建图（占据栅格地图）
- 🔍 自主探索（Frontier算法）
- 🛣️ 路径规划（A*算法）
- 🚧 局部避障（DWA算法）
- 📊 实时可视化（地图显示）

**核心模块**：
```python
xxq_host/src/
├── slam/
│   ├── occupancy_map.py      # 占据栅格地图
│   └── frontier_detector.py  # 前沿探索
├── navigation/
│   ├── path_planner.py       # A*路径规划
│   ├── dwa.py                # 动态窗口避障
│   └── controller.py         # 主控制器
├── visualization/
│   └── map_visualizer.py     # 可视化
└── communication/
    ├── protocol.py           # 数据协议
    └── robot_comm.py         # 串口通信
```

---

## 🚀 快速开始

### 方式1：完整系统运行（推荐）

```bash
# 1️⃣ 固件端（STM32）
# - 烧录xxq固件到STM32
# - 上电后自动启动，等待Python连接

# 2️⃣ 主机端（Python）
cd xxq_host
pip install -r requirements.txt
python scripts/demo_exploration.py  # 运行完整探索演示
```

### 方式2：单独测试

**只测试固件**：
```bash
# 打开串口终端（115200波特率）
# 发送命令：
1    # PID前进
A    # 雷达扫描
M    # 读取MPU数据
?    # 查看帮助
```

**只测试Python代码**：
```bash
cd xxq_host
pytest tests/ -v              # 运行测试（无需硬件）
python scripts/demo_slam.py   # SLAM演示（无需硬件）
```

---

## 📡 通信流程

```
┌─────────────┐                    ┌──────────────┐
│  STM32固件   │   UART4蓝牙串口     │  Python主机  │
│             │◄──────────────────►│              │
│  - 雷达数据  │   115200 baud      │  - SLAM建图  │
│  - 姿态数据  │                    │  - 路径规划  │
│  - 里程计   │                    │  - 决策控制  │
└─────────────┘                    └──────────────┘
```

**数据格式**：
- **上行**（STM32→Python）：传感器数据（JSON/CSV）
- **下行**（Python→STM32）：控制命令（NAV/SPD/MODE）

---

## 💡 核心算法

### 1. SLAM建图
```python
# 占据栅格地图 + Log-odds贝叶斯更新
from slam.occupancy_map import OccupancyGridMap

map = OccupancyGridMap(config)
map.update_with_lidar(lidar_data, robot_pose)
map.save_map('result.npy')
```

### 2. Frontier探索
```python
# 前沿点检测 + DBSCAN聚类
from slam.frontier_detector import FrontierDetector

detector = FrontierDetector(map)
frontiers = detector.find_frontiers(robot_pose)
target = detector.select_best_frontier(frontiers, robot_pose)
```

### 3. 路径规划
```python
# A*全局规划 + 路径平滑
from navigation.path_planner import PathPlanner

planner = PathPlanner(map)
path = planner.plan_path(start, goal)
smooth_path = planner.smooth_path(path)
```

### 4. 局部避障
```python
# DWA动态窗口法
from navigation.dwa import DWA

dwa = DWA(map)
best_v, best_w = dwa.plan(robot_pose, current_v, current_w, goal)
```

---

## 📊 系统状态

| 模块 | 状态 | 质量 |
|------|------|------|
| **STM32固件** | ✅ 完成 | 协议完整 |
| **Python主机** | ✅ 完成 | A级(93.5分) |
| **通信对接** | ✅ 完成 | 100%匹配 |
| **测试验证** | ✅ 完成 | 53个测试全通过 |
| **文档体系** | ✅ 完成 | 97%完整度 |

**整体完成度**：✅ **100%** 

---

## 📚 文档导航

### 🔰 新手入门
1. **本文档** - 项目总览
2. `xxq/README.md` - 固件使用说明
3. `xxq_host/README.md` - 主机端快速开始
4. `xxq_host/doc/用户手册.md` - 详细使用教程

### 🔧 开发参考
1. `xxq_host/doc/API文档.md` - API接口文档
2. `xxq_host/doc/软件组工作总结.md` - 技术方案
3. `xxq_host/doc/固件端通信代码.md` - 通信协议实现

### 📊 质量报告
1. `xxq_host/doc/测试报告.md` - 测试结果
2. `xxq_host/doc/代码质量报告.md` - 质量分析
3. `xxq_host/项目交付报告.md` - 交付总结
4. `系统集成总结报告.md` - 集成状态

---

## 🎮 演示脚本

### Python端演示（无需硬件）

```bash
cd xxq_host/scripts/

# 1. Frontier探索演示
python demo_frontier.py

# 2. 路径规划演示
python demo_path_planning.py

# 3. 完整探索演示
python demo_exploration.py

# 4. 可视化演示
python demo_visualization.py
```

### 固件端调试命令

```bash
# 打开串口终端，发送：
?    # 查看所有命令
1    # 开启PID前进
A    # 雷达扫描
M    # 读取MPU数据
E    # 编码器测试
```

---

## 🔍 常见问题

### Q: 如何理解整个系统？
**A**: 系统分两部分：
- **STM32固件**：硬件驱动层，负责传感器和电机
- **Python主机**：算法决策层，负责建图和规划
- 通过蓝牙串口通信协同工作

### Q: 代码结构是什么？
**A**: 
```
固件端(C语言)：main.c实现所有功能
主机端(Python)：模块化设计
  ├── slam/         地图和探索
  ├── navigation/   规划和避障
  ├── visualization/ 可视化
  └── communication/ 通信
```

### Q: 如何快速上手？
**A**: 
1. 先运行Python演示脚本（无需硬件）
2. 阅读`xxq_host/doc/用户手册.md`
3. 查看各模块的测试代码学习用法

### Q: 如何调试？
**A**:
```bash
# Python端
pytest tests/test_slam.py -v    # 测试SLAM
pytest tests/ -v                # 全部测试

# 固件端
发送 ?                          # 查看命令
发送 E                          # 编码器测试
发送 M                          # MPU测试
```

---

## 🎯 核心特性

✅ **完整的SLAM系统** - 占据栅格地图 + 贝叶斯更新  
✅ **智能探索算法** - Frontier检测 + 3种选择策略  
✅ **高效路径规划** - A*全局 + DWA局部避障  
✅ **实时可视化** - 30-60fps地图显示  
✅ **模块化设计** - 低耦合、易扩展  
✅ **完善测试** - 53个测试、100%通过  

---

## 📞 技术支持

- 📖 详细文档：`xxq_host/doc/`
- 🧪 测试代码：`xxq_host/tests/`
- 🎬 演示脚本：`xxq_host/scripts/`
- 📊 完整报告：`系统集成总结报告.md`

---

**项目版本**：v1.0.0  
**完成日期**：2025-10-09  
**项目状态**：✅ 已完成交付  

**祝你使用愉快！** 🚀✨

