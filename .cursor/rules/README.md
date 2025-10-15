# Cursor Rules for xxq智能小车项目

本目录包含用于指导AI助手的Cursor规则。这些规则帮助AI理解项目结构、编码规范和核心算法。

## 📋 规则列表

### 1. **project-structure.mdc** ✅ 总是应用
- **用途**: 项目总体架构和文件组织
- **包含内容**:
  - STM32固件端结构
  - Python主机端结构
  - 通信协议概述
  - 关键文档导航
  - 开发原则
- **应用范围**: 所有文件（alwaysApply: true）

### 2. **python-coding-style.mdc** 🐍 Python文件
- **用途**: Python代码风格和最佳实践
- **包含内容**:
  - PEP 8规范
  - Docstring格式
  - 类型注解要求
  - 配置参数使用
  - 错误处理规范
  - 性能优化建议
- **应用范围**: `*.py` 文件

### 3. **stm32-firmware.mdc** 🔧 STM32固件
- **用途**: STM32 C语言开发规范
- **包含内容**:
  - 命名规范
  - 注释规范
  - 硬件驱动标准
  - 关键参数配置
  - 调试命令
  - 定时器和中断使用
- **应用范围**: `*.c`, `*.h` 文件

### 4. **communication-protocol.mdc** 📡 通信模块
- **用途**: STM32与Python通信协议
- **包含内容**:
  - 上行数据格式（MPU/ODO/POSE/LIDAR）
  - 下行命令格式（NAV/SPD/MODE/RESET）
  - 数据解析示例
  - 错误处理
  - 调试技巧
- **应用范围**: 
  - 手动应用：通信协议相关问题
  - 自动应用：`**/communication/*.py`, `**/main.c`

### 5. **testing-guidelines.mdc** 🧪 测试规范
- **用途**: 测试编写和运行指南
- **包含内容**:
  - 测试命名规范
  - 单元测试/集成测试/系统测试
  - Fixture使用
  - Mock技巧
  - 覆盖率目标
  - 性能测试
- **应用范围**: 
  - 手动应用：测试相关问题
  - 自动应用：`**/tests/*.py`, `**/scripts/test_*.py`

### 6. **algorithms.mdc** 🧮 核心算法
- **用途**: SLAM和导航算法实现细节
- **包含内容**:
  - 占据栅格SLAM（Log-odds贝叶斯）
  - Frontier前沿探索（DBSCAN聚类）
  - A*路径规划（带路径平滑）
  - DWA动态窗口避障
  - 差分驱动运动学
  - 位姿估计（卡尔曼滤波）
- **应用范围**: 手动应用（通过description触发）

### 7. **config-management.mdc** ⚙️ 配置管理 🆕
- **用途**: 参数同步和配置管理
- **包含内容**:
  - 必须同步的参数（WHEEL_BASE等）
  - 参数修改流程
  - 配置文件组织
  - 参数验证工具
  - 单位约定
  - 版本控制
- **应用范围**: 
  - 手动应用：配置相关问题
  - 自动应用：`**/config.py`, `**/main.c`

### 8. **debugging.mdc** 🔍 调试指南 🆕
- **用途**: 故障排查和问题诊断
- **包含内容**:
  - 快速诊断流程
  - 8大常见问题及解决方案
  - 日志调试技巧
  - 性能分析方法
  - 硬件调试技巧
  - 紧急停止方法
- **应用范围**: 手动应用（通过description触发）

### 9. **visualization.mdc** 🎨 可视化规范 🆕
- **用途**: 实时地图可视化和Web界面开发
- **包含内容**:
  - Matplotlib实时可视化（Blitting优化）
  - Web可视化（Flask + WebSocket）
  - 可视化元素（地图/路径/前沿/机器人）
  - 性能优化技巧
  - 多子图布局
  - 调试可视化
- **应用范围**: 
  - 手动应用：可视化相关问题
  - 自动应用：`**/visualization/*.py`, `**/templates/*.html`

## 🎯 使用方式

### 自动应用
某些规则会根据文件类型自动应用：

| 文件类型 | 自动应用的规则 |
|---------|--------------|
| 所有文件 | `project-structure.mdc` |
| `*.py` | `python-coding-style.mdc` |
| `*.c`, `*.h` | `stm32-firmware.mdc` |
| `config.py`, `main.c` | `config-management.mdc` |
| `communication/*.py`, `main.c` | `communication-protocol.mdc` |
| `tests/*.py` | `testing-guidelines.mdc` |
| `visualization/*.py`, `templates/*.html` | `visualization.mdc` |

### 手动应用
在与AI对话时提及以下主题会触发相关规则：

- **"通信协议"** → `communication-protocol.mdc`
- **"测试"、"单元测试"** → `testing-guidelines.mdc`
- **"SLAM算法"、"DWA"、"路径规划"** → `algorithms.mdc`
- **"配置"、"参数同步"** → `config-management.mdc`
- **"调试"、"故障"、"问题"** → `debugging.mdc`
- **"可视化"、"地图显示"** → `visualization.mdc`

### 示例对话

**✅ 好的提问方式**:
- "如何修改通信协议中的MPU数据格式？"
  - 触发：`communication-protocol.mdc`
- "我需要为新的导航功能编写测试"
  - 触发：`testing-guidelines.mdc`
- "DWA算法的评价函数权重如何调整？"
  - 触发：`algorithms.mdc`
- "如何同步固件端和Python端的WHEEL_BASE参数？"
  - 触发：`config-management.mdc`
- "机器人不动，如何调试？"
  - 触发：`debugging.mdc`
- "如何优化地图可视化性能？"
  - 触发：`visualization.mdc`

**✅ 编辑文件时**:
- 打开 `src/slam/occupancy_map.py`
  - 自动应用：`project-structure.mdc` + `python-coding-style.mdc`
- 打开 `xxq/Core/Src/main.c`
  - 自动应用：`project-structure.mdc` + `stm32-firmware.mdc` + `config-management.mdc` + `communication-protocol.mdc`
- 打开 `xxq_host/config.py`
  - 自动应用：`project-structure.mdc` + `python-coding-style.mdc` + `config-management.mdc`
- 打开 `src/visualization/slam_visualizer.py`
  - 自动应用：`project-structure.mdc` + `python-coding-style.mdc` + `visualization.mdc`

## 📝 规则维护

### 何时更新规则

| 情况 | 需要更新的规则 |
|-----|--------------|
| 修改通信协议 | `communication-protocol.mdc` |
| 修改代码规范 | `python-coding-style.mdc` 或 `stm32-firmware.mdc` |
| 添加新模块 | `project-structure.mdc` |
| 修改算法实现 | `algorithms.mdc` |
| 修改测试流程 | `testing-guidelines.mdc` |
| 添加新配置参数 | `config-management.mdc` |
| 发现新问题和解决方案 | `debugging.mdc` |
| 改进可视化功能 | `visualization.mdc` |

### 更新步骤
1. 编辑对应的 `.mdc` 文件
2. 更新本 README（如有必要）
3. 测试规则是否正常工作

## 🔗 参考链接

### 关键文档
- 项目总览: [../../README.md](../../README.md)
- 主机端说明: [../../xxq_host/README.md](../../xxq_host/README.md)
- 通信协议: [../../xxq_host/doc/硬件通信协议详细说明.md](../../xxq_host/doc/硬件通信协议详细说明.md)
- 开发计划: [../../xxq_host/doc/plan/开发计划.md](../../xxq_host/doc/plan/开发计划.md)

### Cursor规则语法
- Cursor Rules官方文档
- Markdown格式
- 特殊语法: `[filename](mdc:relative/path/to/file)`

## ✨ 规则效果

使用这些规则后，AI助手能够：
- ✅ 理解项目的双端结构（STM32 + Python）
- ✅ 遵循正确的代码风格
- ✅ 正确实现通信协议
- ✅ 编写符合规范的测试
- ✅ 理解并修改核心算法
- ✅ 维护参数同步（固件端 ↔ config.py）

## 🎉 总结

这套规则涵盖了xxq智能小车项目的所有关键方面：
- **9个规则文件**（新增3个：配置管理、调试指南、可视化规范）
- **覆盖STM32固件和Python主机端**
- **包含算法、协议、测试、代码规范、调试、可视化**
- **自动应用 + 手动触发**
- **全方位开发支持：从编码到调试到优化**

祝你开发顺利！🚀

