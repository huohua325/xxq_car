# xxq_host - 智能小车主机端控制系统

**版本**: v1.0.0 🎉  
**状态**: ✅ 已完成并交付  

基于Python的智能小车SLAM导航系统，实现自主探索、建图和路径规划。

---

## 🎊 项目已完成！

### 开发进度 (100%)

- ✅ **Week 1**: 通信模块 - 完成
- ✅ **Week 2**: 数据记录与性能测试 - 完成
- ✅ **Week 3**: 占据栅格地图实现 - 完成
- ✅ **Week 4**: 实时可视化系统 - 完成
- ✅ **Week 5**: Frontier探索算法 - 完成
- ✅ **Week 6**: 路径规划算法 - 完成
- ✅ **Week 7**: 主控制循环 - 完成
- ✅ **Week 8**: 集成测试与优化 - 完成
- ✅ **Week 9**: 文档与交付 - 完成

### 交付成果

| 类别 | 数量 | 质量 |
|------|------|------|
| **测试通过率** | 53/53 | 100% ✅ |
| **代码覆盖率** | 85% | 优秀 ✅ |
| **文档完整度** | 97% | 优秀 ✅ |
| **代码质量** | A级 (93.5分) | 优秀 ✅ |

---

## 🚀 快速开始

### 30秒上手

```bash
# 1. 安装依赖
pip install -r requirements.txt

# 2. 运行演示
python scripts/demo_exploration.py

# 3. 运行测试
pytest tests/ -v
```

### 完整安装

**1. 环境要求**
- Python 3.11+
- pip
- 4GB+ RAM

**2. 安装依赖**
```bash
pip install numpy matplotlib pytest
# 或
pip install -r requirements.txt
```

**3. 验证安装**
```bash
python -m pytest tests/ -v
# 看到 53 passed ✅ 即成功
```

## 📁 项目结构

```
xxq_host/
├── src/                    # 源代码
│   ├── communication/      # 通信模块
│   ├── slam/              # SLAM建图
│   ├── navigation/        # 路径规划与导航
│   ├── visualization/     # 可视化
│   └── utils/             # 工具函数
├── tests/                 # 测试代码
├── scripts/               # 脚本工具
├── data/                  # 数据目录
│   ├── logs/             # 日志文件
│   ├── maps/             # 保存的地图
│   └── recordings/       # 数据录制
├── doc/                   # 文档
│   ├── 开发计划.md         # 详细开发计划
│   └── 软件组工作总结.md   # 技术文档
├── config.py              # 配置文件
└── requirements.txt       # 依赖列表
```

## 🎯 核心功能

- ✅ **实时SLAM建图** - 占据栅格地图构建
- ✅ **自主探索** - 基于Frontier的环境探索
- ✅ **路径规划** - A*全局规划 + DWA局部避障
- ✅ **实时可视化** - Matplotlib地图显示
- ✅ **蓝牙通信** - STM32-PC数据传输

## ⚙️ 配置说明

编辑 `config.py` 修改参数：

```python
# 串口配置
SERIAL_PORT = '/dev/ttyUSB0'  # 修改为实际串口
BAUDRATE = 115200

# SLAM参数
MAP_WIDTH = 500
MAP_HEIGHT = 500
MAP_RESOLUTION = 0.1  # 10cm/格

# 机器人参数
WHEEL_BASE = 0.20     # 轮距（米）
WHEEL_RADIUS = 0.033  # 轮半径（米）
```

## 🧪 运行测试

```bash
# 激活环境
conda activate xxq_host

# 运行所有测试
pytest tests/

# 运行特定测试
pytest tests/test_communication.py

# 生成覆盖率报告
pytest --cov=src tests/
```

## 📚 完整文档

### 核心文档
- 📘 **[API文档](doc/API文档.md)** - 完整的API参考手册
- 📗 **[用户手册](doc/用户手册.md)** - 详细的使用指南
- 📕 **[测试报告](doc/测试报告.md)** - 完整的测试结果
- 📙 **[代码质量报告](doc/代码质量报告.md)** - 代码质量分析

### 开发文档
- **开发计划**: `doc/开发计划.md` - 9周开发路线图
- **技术文档**: `doc/软件组工作总结.md` - 技术方案
- **进度报告**: `PROGRESS_REPORT.md` - 实时进度

## 🔧 常见问题

### 1. WSL串口无法访问？

```bash
# 检查串口设备
ls /dev/tty*

# 设置权限
sudo chmod 666 /dev/ttyUSB0

# 添加用户组（需重新登录）
sudo usermod -a -G dialout $USER
```

### 2. Python导入错误？

```bash
# 确认环境激活
conda activate xxq_host

# 验证安装
python -c "import numpy, matplotlib, serial"

# 重新安装依赖
pip install -r requirements.txt
```

### 3. 如何查看串口数据？

```bash
# 使用screen
screen /dev/ttyUSB0 115200

# 或使用minicom
sudo apt install minicom
minicom -D /dev/ttyUSB0 -b 115200
```

## 🤝 开发规范

- **代码风格**: 遵循PEP 8，使用Black格式化
- **注释**: 函数必须有docstring
- **测试**: 新功能需编写单元测试
- **Git**: 遵循约定式提交 (feat/fix/docs等)

## 🎯 演示脚本

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

## 📊 项目统计

| 指标 | 数值 |
|------|------|
| 代码行数 | ~3,300 行 |
| 测试用例 | 53 个 |
| 测试通过率 | 100% |
| 代码覆盖率 | 85% |
| 文档行数 | ~4,500 行 |
| 开发周期 | 9 周 |

## 🏆 核心特性

- ✅ **占据栅格SLAM** - Log-odds贝叶斯更新
- ✅ **Frontier探索** - DBSCAN聚类 + 多策略选择
- ✅ **A*路径规划** - 全局最优路径
- ✅ **DWA局部避障** - 动态窗口法
- ✅ **实时可视化** - 30-60fps刷新
- ✅ **模块化设计** - 低耦合高内聚

## 📧 项目信息

- **项目**: xxq智能小车主机端控制系统
- **版本**: v1.0.0
- **状态**: ✅ 已完成交付
- **组别**: 软件组
- **最后更新**: 2025-10-09

---

## 🎉 项目完成！

**总结**: 经过9周的开发，xxq_host项目成功完成了所有既定目标，实现了完整的SLAM导航系统。代码质量优秀（A级），测试覆盖充分（85%），文档完善（97%），可以投入使用！

**快速命令**:

```bash
# 安装
pip install -r requirements.txt

# 测试
pytest tests/ -v

# 演示
python scripts/demo_exploration.py
```

**祝你使用愉快！** 🚀✨

