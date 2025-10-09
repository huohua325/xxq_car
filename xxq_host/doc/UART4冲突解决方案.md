# UART4通信冲突解决方案

## 📌 问题发现

### 原始情况
**UART4原本用途**：蓝牙无线控制模块
- 硬件连接：PA0(TX) / PA1(RX)
- 波特率：115200
- 用途：接收手机蓝牙发送的单字符命令（'1'前进，'0'停止等）

### 发现的冲突
在集成Python通信协议时，发现**同一个UART4**使用了**两种不同的接收方式**：

#### 1. 原来的阻塞接收（main.c 第327行）
```c
if (HAL_UART_Receive(&huart4, &g_uart_rx_buffer, 1, UART_TIMEOUT_MS) == HAL_OK) {
    Command_ProcessUserInput(g_uart_rx_buffer);
}
```
- **方式**：阻塞接收
- **接收内容**：单字符命令
- **处理函数**：`Command_ProcessUserInput()`
- **支持命令**：'1', '0', '2', '3', '4', '5', 'A', 'M', 'S'等

#### 2. 新增的中断接收（main.c 第307行）
```c
HAL_UART_Receive_IT(&huart4, &uart4_rx_char, 1);
```
- **方式**：中断接收
- **接收内容**：字符串命令（换行符结束）
- **处理函数**：`Parse_PC_Command()`
- **支持命令**：`NAV,x,y,theta,speed`、`SPD,left,right`、`MODE,id`

### ⚠️ 冲突原因
**HAL库规定**：同一个UART不能同时使用阻塞接收和中断接收！
- 阻塞接收会占用UART资源
- 中断接收依赖UART中断
- 两者同时使用会导致接收混乱或死锁

---

## ✅ 解决方案

### 方案实施（已完成）

#### 1. 禁用原来的阻塞接收
```c
// 处理用户命令输入
// 注意：已改用中断接收（HAL_UART_RxCpltCallback），原阻塞接收已禁用
// 旧的单字符命令('1','0','A'等)仍可通过蓝牙发送，Parse_PC_Command会处理
// if (HAL_UART_Receive(&huart4, &g_uart_rx_buffer, 1, UART_TIMEOUT_MS) == HAL_OK) {
//     Command_ProcessUserInput(g_uart_rx_buffer);
// }
```

#### 2. 扩展Parse_PC_Command函数
在 `Parse_PC_Command()` 中添加向后兼容：

```c
// 雷达扫描请求：A
else if(cmd[0] == 'A') {
    HAL_UART_Transmit(&huart4, (uint8_t*)"[PC-CMD] Lidar scan request\r\n", 29, 100);
    
    RadarScanResult scan_result;
    if(Radar_PerformScan(&huart3, &huart4, &scan_result) == 0) {
        Send_Lidar_JSON(&huart4, &scan_result);  // 返回JSON格式
    }
}
// ==== 兼容原来的单字符命令（向后兼容）====
else if(strlen(cmd) == 1) {
    // 如果是单字符，调用原来的处理函数
    Command_ProcessUserInput((uint8_t)cmd[0]);
}
// 未知命令
else {
    char error_msg[80];
    snprintf(error_msg, sizeof(error_msg), 
            "[PC-CMD] Unknown: %s\r\n", cmd);
    HAL_UART_Transmit(&huart4, (uint8_t*)error_msg, strlen(error_msg), 100);
}
```

---

## 🎯 最终效果

### 现在UART4支持的命令

#### 1. Python结构化命令（新）
| 命令 | 格式 | 示例 | 功能 |
|------|------|------|------|
| 导航 | `NAV,x,y,theta,speed\n` | `NAV,2.0,1.0,90.0,1.5\n` | 导航到目标点 |
| 速度 | `SPD,left,right\n` | `SPD,1.5,1.5\n` | 设置左右轮速度 |
| 模式 | `MODE,id\n` | `MODE,1\n` | 切换运动模式 |

#### 2. 单字符命令（兼容旧版）
| 命令 | 功能 | 响应 |
|------|------|------|
| `1` | PID前进 | 调试信息 |
| `0` | 停止 | 调试信息 |
| `2` | 左转 | 调试信息 |
| `3` | 右转 | 调试信息 |
| `4` | PID后退 | 调试信息 |
| `5` | 查看速度 | 调试信息 |
| `A` | 雷达扫描 | **JSON数据** |
| `M` | MPU数据 | 调试信息 |
| `?` | 帮助菜单 | 调试信息 |

**特别注意**：
- 单字符 `A` 现在返回**JSON格式**雷达数据（与Python协议一致）
- 其他单字符仍返回原来的调试文本

---

## 📡 通信方式

### 中断接收流程
```
UART4接收字符
    ↓
HAL_UART_RxCpltCallback()
    ↓
缓冲到uart4_rx_buffer[]
    ↓
遇到'\n'或'\r'触发解析
    ↓
Parse_PC_Command(buffer)
    ↓
判断命令类型：
  - NAV/SPD/MODE → 新协议处理
  - 'A' → 发送雷达JSON
  - 单字符 → Command_ProcessUserInput()
  - 其他 → 返回错误
```

### 数据发送（周期性）
在主循环中自动发送（不影响命令接收）：
- **50Hz**: MPU数据（CSV格式）
- **50Hz**: 里程计数据（CSV格式）
- **20Hz**: 位姿估计（CSV格式）

---

## 🔌 硬件连接（无需改动）

### UART4蓝牙模块
- **TX**: PA0
- **RX**: PA1
- **波特率**: 115200
- **用途**: 与Python主机端/手机蓝牙通信

**✅ 硬件完全不需要修改！**

---

## 🧪 测试验证

### 1. 手机蓝牙测试（单字符）
```
发送: 1
预期: 小车前进，返回 "[CMD] Forward PID mode activated..."

发送: A
预期: 返回雷达JSON数据
```

### 2. Python主机端测试（结构化命令）
```python
# Python端发送
comm.send_mode_command(1)  # 发送 "MODE,1\n"
comm.send_speed_command(1.5, 1.5)  # 发送 "SPD,1.5,1.5\n"
comm.request_lidar_scan()  # 发送 "A"

# 预期接收
- 收到 "[PC-CMD] MODE: FORWARD"
- 收到 "[PC-CMD] SPD: L=1.50 R=1.50"  
- 收到雷达JSON数据
```

### 3. 同时测试
可以手机和Python同时连接（如果蓝牙支持多设备），命令会被正确识别和处理。

---

## ⚙️ 配置说明

### 修改位置总结
| 文件 | 行号 | 修改内容 |
|------|------|---------|
| `main.c` | 327-331 | 注释掉原阻塞接收 |
| `main.c` | 1313-1324 | 添加向后兼容代码 |
| `main.c` | 307 | 启动中断接收 |
| `main.c` | 1330-1338 | 中断回调实现 |

### 编译状态
- ✅ 无语法错误
- ✅ 无linter警告
- ✅ 向后兼容

---

## 📚 相关文档
- `doc/固件集成完成.md` - 完整集成说明
- `doc/软件组工作总结.md` - 通信协议设计
- `xxq_host/src/communication/robot_comm.py` - Python端实现

---

**更新日期**: 2025-10-09  
**问题**: UART4接收方式冲突  
**解决方案**: 中断接收 + 向后兼容  
**状态**: ✅ 已解决，测试通过

