# 使用BLE连接电脑的完整指南

## ✅ 已解决

您已成功通过BLE连接HC-04BLE到Windows电脑！

## 测试结果

- ✅ 设备地址: C4:25:01:20:02:8E
- ✅ 连接成功
- ✅ 数据接收正常（15.40包/秒）
- ✅ POSE数据和ODO数据都能收到

## 如何使用

### 快速测试（推荐）

```bash
# 测试BLE连接和数据接收（20秒）
python connect_ble.py --address C4:25:01:20:02:8E --duration 20

# 测试更长时间（60秒）
python connect_ble.py --address C4:25:01:20:02:8E --duration 60
```

### 使用步骤

1. **确保HC-04BLE通电且未连接其他设备**
2. **运行BLE连接工具**：
   ```bash
   python connect_ble.py --address C4:25:01:20:02:8E --duration 30
   ```
3. **观察数据接收情况**

## 注意事项

### ⚠️ BLE的限制

1. **数据被分包**
   - BLE的MTU（最大传输单元）通常是20-250字节
   - 长数据会被切分成多个包
   - 需要处理数据重组（您在测试中看到数据被切分了）

2. **速度比COM口慢**
   - BLE: ~15包/秒
   - USB COM口: ~100包/秒

3. **只能连接一个设备**
   - 手机连着时，电脑连不上
   - 电脑连着时，手机连不上

### ✅ BLE的优势

- ✅ 无线连接
- ✅ 低功耗
- ✅ 现在可以用Python直接控制
- ✅ 不需要COM口

## 完整测试方案（进阶）

如果要在实际项目中使用BLE，需要修改`robot_comm.py`：

### 方案A：创建BLE版本的robot_comm（推荐）

创建一个新的`robot_comm_ble.py`，保留原来的`robot_comm.py`：

```python
# robot_comm_ble.py
import asyncio
from bleak import BleakClient

class RobotCommBLE:
    def __init__(self, address):
        self.address = address
        self.client = None
        self.data_buffer = ""
        
    async def connect(self):
        self.client = await BleakClient(self.address)
        await self.client.start_notify(
            "0000ffe1-0000-1000-8000-00805f9b34fb",
            self._notification_handler
        )
    
    def _notification_handler(self, sender, data):
        # 处理接收到的数据
        self.data_buffer += data.decode('utf-8', errors='ignore')
        
        # 解析完整行
        while '\n' in self.data_buffer:
            line, self.data_buffer = self.data_buffer.split('\n', 1)
            self._parse_line(line)
    
    def _parse_line(self, line):
        # 解析POSE, ODO等数据
        # ... 现有的解析逻辑
        pass
```

### 方案B：简单测试（不修改现有代码）

直接用`connect_ble.py`测试：

1. 观察数据是否正常
2. 检查位姿是否更新
3. 手动发送控制命令测试

## 当前最佳实践

### 调试阶段（现在）

**建议使用USB串口**：
- 稳定
- 快速
- 数据完整（不分包）
- 方便调试

### 演示/部署阶段

**可以使用BLE**：
- 无线展示
- 移动方便

### 使用USB的方法

如果您的STM32开发板有USB口：

1. 用USB线连接开发板到电脑
2. 查找COM口：
   ```bash
   python find_port.py
   ```
3. 使用找到的COM口（如COM8）：
   ```bash
   python tests/test_system_comprehensive.py --port COM8 --quick
   ```

## 总结

### 问题：
- ❌ HC-04BLE是BLE设备，Windows COM口不支持

### 解决：
- ✅ 使用Python的`bleak`库直接连接BLE
- ✅ 绕过COM口限制
- ✅ 成功接收数据

### 结果：
- ✅ 电脑可以通过BLE连接HC-04BLE
- ✅ 数据接收正常
- ✅ 可以进行基本测试

### 建议：
- 💡 调试时用USB串口（如果有）
- 💡 演示时用BLE无线连接
- 💡 如需完整集成BLE，需要修改通信模块

## 快速命令参考

```bash
# 扫描BLE设备
python scan_ble.py

# 连接HC-04BLE
python connect_ble.py --address C4:25:01:20:02:8E --duration 30

# 查找USB串口（推荐用于调试）
python find_port.py

# 使用USB串口测试（推荐）
python tests/test_system_comprehensive.py --port COM8 --quick
```

---

**恭喜！您已经成功解决了BLE连接问题！** 🎉

调试建议用USB，演示可以用BLE！

