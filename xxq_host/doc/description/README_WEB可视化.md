# 🌐 Web可视化 - 快速上手

> 一分钟启动xxq机器人的现代化Web可视化界面

---

## 🚀 三步启动

### 1️⃣ 安装依赖

**Windows用户** - 双击运行：
```
install_web_viz.bat
```

**或手动安装**：
```bash
pip install Flask Flask-SocketIO Pillow
```

### 2️⃣ 启动程序

```bash
python main_exploration.py --web
```

### 3️⃣ 打开浏览器

访问：**http://localhost:5000**

🎉 **完成！** 您将看到专业的SLAM实时建图界面。

---

## 📱 多设备访问

### 同一WiFi下的手机/平板访问

1. 查看电脑IP地址：
   ```bash
   ipconfig  # Windows
   ifconfig  # Linux/Mac
   ```

2. 在移动设备浏览器访问：
   ```
   http://192.168.x.x:5000
   ```

---

## 🎮 命令参数

```bash
# Web可视化（推荐）
python main_exploration.py --web

# 传统matplotlib
python main_exploration.py

# 无可视化（仅命令行）
python main_exploration.py --no-viz

# 自定义端口
python main_exploration.py --web --port 8080
```

---

## 🎨 界面展示

Web界面包含：

- **主地图区域** - 实时SLAM建图
  - 障碍物（黑）、空闲区（白）、未知区（灰）
  - 机器人位置（蓝色圆点）
  - 朝向箭头（红色）
  - 雷达点云（红点）
  - 规划路径（绿线）
  
- **探索统计卡片** - 进度条和统计数据
- **位姿信息卡片** - 实时位置、速度
- **图例说明卡片** - 颜色含义

**特点**：
- ✨ 渐变紫色背景，科技感十足
- 📊 现代化Dashboard布局
- 📱 响应式设计，自适应屏幕
- ⚡ 实时更新（WebSocket）

---

## ❓ 常见问题

**Q: 浏览器打不开？**
- 检查防火墙
- 尝试更换端口：`--port 8080`

**Q: 地图不更新？**
- 刷新页面（F5）
- 检查右上角连接状态

**Q: 移动设备无法访问？**
- 确保在同一WiFi
- 使用IP地址而非localhost
- 关闭电脑防火墙测试

---

## 📚 详细文档

- [Web可视化使用指南](./doc/Web可视化使用指南.md)
- [WEB可视化升级说明](./WEB可视化升级说明.md)

---

## 💡 提示

- 首次使用推荐Web模式，体验现代化界面
- 开发调试可使用传统matplotlib模式
- 演示时可用移动设备访问，更灵活

**祝您使用愉快！** 🎉


