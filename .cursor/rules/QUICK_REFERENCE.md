# Cursor Rules 快速参考卡

## 🎯 规则速查表

| # | 规则名称 | 关键词触发 | 文件类型 | 用途 |
|---|---------|----------|---------|------|
| 1 | **project-structure** | - | 所有文件 | 项目架构导航 |
| 2 | **python-coding-style** | - | `*.py` | Python代码规范 |
| 3 | **stm32-firmware** | - | `*.c`, `*.h` | STM32固件规范 |
| 4 | **config-management** | 配置、参数同步 | `config.py`, `main.c` | 参数管理 |
| 5 | **communication-protocol** | 通信协议 | `communication/*.py` | 串口通信 |
| 6 | **testing-guidelines** | 测试 | `tests/*.py` | 测试规范 |
| 7 | **algorithms** | SLAM、DWA、路径规划 | 手动触发 | 核心算法 |
| 8 | **debugging** | 调试、故障、问题 | 手动触发 | 故障排查 |
| 9 | **visualization** | 可视化、地图显示 | `visualization/*.py` | 可视化开发 |

---

## 🔍 问题→规则映射

| 你的问题 | 应该查看的规则 |
|---------|--------------|
| 如何修改通信数据格式？ | `communication-protocol.mdc` |
| Python代码应该怎么写？ | `python-coding-style.mdc` |
| 如何编写单元测试？ | `testing-guidelines.mdc` |
| DWA权重如何调整？ | `algorithms.mdc` |
| 机器人不动怎么办？ | `debugging.mdc` |
| 如何同步固件和Python参数？ | `config-management.mdc` |
| 地图可视化很卡怎么办？ | `visualization.mdc` |
| STM32代码规范是什么？ | `stm32-firmware.mdc` |
| 项目文件结构是什么？ | `project-structure.mdc` |

---

## 📁 文件→自动应用规则

| 编辑的文件 | 自动应用的规则 |
|-----------|--------------|
| `src/slam/occupancy_map.py` | project-structure + python-coding-style |
| `src/navigation/dwa.py` | project-structure + python-coding-style |
| `src/communication/robot_comm.py` | project-structure + python-coding-style + communication-protocol |
| `src/visualization/slam_visualizer.py` | project-structure + python-coding-style + visualization |
| `xxq_host/config.py` | project-structure + python-coding-style + config-management |
| `tests/test_slam.py` | project-structure + python-coding-style + testing-guidelines |
| `xxq/Core/Src/main.c` | project-structure + stm32-firmware + config-management + communication-protocol |
| `xxq/Core/Src/hardware/motor.c` | project-structure + stm32-firmware |
| `templates/slam_viewer.html` | project-structure + visualization |

---

## 🚨 常见问题快速解决

### 问题1: 串口连接失败
**症状**: `PermissionError` 或 `could not open port`  
**查看**: `debugging.mdc` → 问题1  
**快速解决**:
```bash
python scripts/find_stm32_port.py
# 关闭占用端口的程序
```

### 问题2: 接收不到数据
**症状**: `read_sensor_data()` 返回 `None`  
**查看**: `debugging.mdc` → 问题2  
**快速解决**:
1. 检查波特率匹配（115200）
2. 检查固件是否运行
3. 发送 `?` 测试通信

### 问题3: 机器人运动异常
**症状**: 不动/偏离/转向错误  
**查看**: `debugging.mdc` → 问题3 + `config-management.mdc`  
**快速解决**:
1. 验证 `WHEEL_BASE` 和 `WHEEL_RADIUS`
2. 检查编码器反馈
3. 运行 PWM 标定

### 问题4: 位姿估计漂移
**症状**: 静止时位姿仍变化  
**查看**: `debugging.mdc` → 问题5  
**快速解决**:
```python
comm.reset_pose(0, 0, 0)
# 检查 ENCODER_PPR 是否正确
```

### 问题5: SLAM地图质量差
**症状**: 墙壁模糊/噪点多  
**查看**: `debugging.mdc` → 问题6 + `algorithms.mdc`  
**快速解决**:
```python
# config.py
MAP_PROB_OCCUPIED = 0.9  # 增大减少误判
MAP_RESOLUTION = 0.05    # 提高分辨率
```

---

## ⚙️ 关键配置参数速查

### 必须同步的参数 ⚠️
```python
# xxq_host/config.py
WHEEL_BASE = 0.20        # ← 必须与固件一致
WHEEL_RADIUS = 0.033     # ← 必须与固件一致
ENCODER_PPR = 1560       # ← 必须与固件一致
```

```c
// xxq/Core/Src/main.c
#define WHEEL_BASE 0.20f     // ← 必须与Python一致
#define WHEEL_RADIUS 0.033f  // ← 必须与Python一致
#define ENCODER_PPR_LEFT 1560  // ← 必须与Python一致
```

**参考**: `config-management.mdc`

### SLAM参数调优
```python
MAP_RESOLUTION = 0.1          # 栅格大小（米）
MAP_PROB_OCCUPIED = 0.9       # 障碍物概率
MAP_PROB_FREE = 0.3           # 空闲概率
FRONTIER_CLUSTER_DIST = 3     # 前沿聚类距离
MIN_FRONTIER_SIZE = 10        # 最小前沿大小
```

**参考**: `algorithms.mdc` → SLAM章节

### DWA参数调优
```python
DWA_MAX_SPEED = 0.5           # 最大速度（m/s）
DWA_MAX_YAW_RATE = 90.0       # 最大角速度（°/s）
DWA_WEIGHT_HEADING = 0.1      # 朝向权重
DWA_WEIGHT_CLEARANCE = 0.5    # 障碍物距离权重
DWA_WEIGHT_VELOCITY = 0.4     # 速度权重
```

**参考**: `algorithms.mdc` → DWA章节

---

## 🧪 测试命令速查

```bash
# 运行所有测试
pytest tests/ -v

# 运行特定模块测试
pytest tests/test_slam.py -v

# 生成覆盖率报告
pytest --cov=src tests/

# 快速硬件测试（5分钟）
python tests/test_system_comprehensive.py --port COM5 --quick

# 紧急停止
python scripts/emergency_stop.py
```

**参考**: `testing-guidelines.mdc`

---

## 📊 性能基准

| 操作 | 目标时间 | 超时警告 | 规则参考 |
|------|---------|---------|---------|
| 地图更新 | <50ms | >100ms | `algorithms.mdc` |
| 前沿检测 | <100ms | >200ms | `algorithms.mdc` |
| 路径规划 | <500ms | >1000ms | `algorithms.mdc` |
| DWA计算 | <100ms | >200ms | `algorithms.mdc` |
| 可视化 | 30fps | <15fps | `visualization.mdc` |

---

## 🔗 重要文档链接

| 文档 | 用途 | 规则引用 |
|-----|------|---------|
| [通信协议详细说明](../xxq_host/doc/硬件通信协议详细说明.md) | 通信格式 | communication-protocol |
| [DWA算法详解](../xxq_host/doc/description/DWA动态窗口避障算法详解.md) | DWA实现 | algorithms |
| [完整系统测试指南](../xxq_host/doc/完整系统测试指南.md) | 测试流程 | testing-guidelines |
| [参数同步检查表](../xxq/docs/参数同步检查表.md) | 参数验证 | config-management |
| [Web可视化使用指南](../xxq_host/doc/description/Web可视化使用指南.md) | Web界面 | visualization |

---

## 💡 编码最佳实践速记

### Python ✅
```python
# ✅ 好
from config import WHEEL_BASE
from typing import Tuple

def calculate(v: float) -> Tuple[float, float]:
    """计算速度"""
    return v * WHEEL_BASE, v

# ❌ 差
def calculate(v):
    WHEEL_BASE = 0.2  # 硬编码
    return v * WHEEL_BASE, v
```

**参考**: `python-coding-style.mdc`

### STM32 C ✅
```c
// ✅ 好
#define WHEEL_BASE 0.20f
float calculate_velocity(float left_rps, float right_rps);

// ❌ 差
float calculate_velocity(float a, float b) {
    float wb = 0.2;  // 魔法数字
    return a * wb;
}
```

**参考**: `stm32-firmware.mdc`

### 测试 ✅
```python
# ✅ 好
def test_map_update():
    """测试地图更新功能"""
    map = OccupancyGridMap(100, 100, 0.1)
    map.update_cell(50, 50, 0.9)
    assert map.get_cell(50, 50) == 0.9

# ❌ 差
def test1():
    map = OccupancyGridMap(100, 100, 0.1)
    assert map  # 测试内容不明确
```

**参考**: `testing-guidelines.mdc`

---

## 🎨 可视化速查

### Matplotlib基础
```python
import matplotlib.pyplot as plt

plt.ion()  # 交互模式
fig, ax = plt.subplots()
img = ax.imshow(map.grid, cmap='gray', animated=True)  # Blitting
plt.show()
```

### Web服务器启动
```bash
cd xxq_host
python -c "from src.visualization.web_viewer import start_web_server; start_web_server()"
# 访问 http://localhost:5000
```

**参考**: `visualization.mdc`

---

## 🔧 STM32调试命令

通过串口发送单字符命令：

| 命令 | 功能 | 说明 |
|-----|------|------|
| `?` | 帮助 | 显示所有命令 |
| `1` | 前进 | PID控制1.5 RPS |
| `0` | 停止 | 紧急停止 |
| `2` | 左转 | 50% PWM |
| `3` | 右转 | 50% PWM |
| `A` | 雷达扫描 | 360°扫描 |
| `M` | MPU数据 | 读取姿态 |
| `E` | 编码器测试 | 读取计数 |

**参考**: `stm32-firmware.mdc`

---

## 📞 获取更多帮助

### 查看完整规则
```bash
cat .cursor/rules/algorithms.mdc
cat .cursor/rules/debugging.mdc
```

### 搜索规则内容
```bash
grep -r "WHEEL_BASE" .cursor/rules/
grep -r "串口" .cursor/rules/
```

### 在线文档
- 完整README: `.cursor/rules/README.md`
- 规则总结: `.cursor/rules/RULES_SUMMARY.md`

---

**版本**: v1.0.0  
**最后更新**: 2025-10-15  
**打印建议**: A4纸，双面打印

