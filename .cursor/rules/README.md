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

## 🎯 使用方式

### 自动应用
某些规则会根据文件类型自动应用：

| 文件类型 | 自动应用的规则 |
|---------|--------------|
| 所有文件 | `project-structure.mdc` |
| `*.py` | `python-coding-style.mdc` |
| `*.c`, `*.h` | `stm32-firmware.mdc` |
| `communication/*.py` | `communication-protocol.mdc` |
| `tests/*.py` | `testing-guidelines.mdc` |

### 手动应用
在与AI对话时提及以下主题会触发相关规则：

- **"通信协议"** → `communication-protocol.mdc`
- **"测试"、"单元测试"** → `testing-guidelines.mdc`
- **"SLAM算法"、"DWA"、"路径规划"** → `algorithms.mdc`

### 示例对话

**✅ 好的提问方式**:
- "如何修改通信协议中的MPU数据格式？"
  - 触发：`communication-protocol.mdc`
- "我需要为新的导航功能编写测试"
  - 触发：`testing-guidelines.mdc`
- "DWA算法的评价函数权重如何调整？"
  - 触发：`algorithms.mdc`

**✅ 编辑文件时**:
- 打开 `src/slam/occupancy_map.py`
  - 自动应用：`project-structure.mdc` + `python-coding-style.mdc`
- 打开 `xxq/Core/Src/main.c`
  - 自动应用：`project-structure.mdc` + `stm32-firmware.mdc`

## 📝 规则维护

### 何时更新规则

| 情况 | 需要更新的规则 |
|-----|--------------|
| 修改通信协议 | `communication-protocol.mdc` |
| 修改代码规范 | `python-coding-style.mdc` 或 `stm32-firmware.mdc` |
| 添加新模块 | `project-structure.mdc` |
| 修改算法实现 | `algorithms.mdc` |
| 修改测试流程 | `testing-guidelines.mdc` |

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
- **6个规则文件**
- **覆盖STM32固件和Python主机端**
- **包含算法、协议、测试、代码规范**
- **自动应用 + 手动触发**

祝你开发顺利！🚀

