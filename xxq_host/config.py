# config.py - xxq_host 统一配置文件
# 修改此文件后，重启程序即可生效

import os
import numpy as np

# ============================================================================
# 串口通信配置
# ============================================================================
SERIAL_PORT = 'COM5'               # Windows: 'COM5', Linux: '/dev/ttyUSB0'
BAUDRATE = 9600                    # HC-05蓝牙模块默认波特率
TIMEOUT = 1.0

# ============================================================================
# 机器人物理参数（⚠️ 必须与固件端保持一致！）
# ============================================================================
# 这些参数在固件端(xxq/Core/Src/main.c)也有定义，需要同步修改
WHEEL_BASE = 0.20                  # 轮距（米）- 两驱动轮中心距离
WHEEL_RADIUS = 0.033               # 轮子半径（米）
ENCODER_PPR_LEFT = 1560            # 左轮编码器分辨率（脉冲/圈）
ENCODER_PPR_RIGHT = 780            # 右轮编码器分辨率（脉冲/圈）

# ⚠️ 重要提醒：
# 修改 WHEEL_BASE, WHEEL_RADIUS, ENCODER_PPR 后，
# 必须同步修改固件端 main.c 第318-320行的参数！

# ============================================================================
# POSE位姿估计参数
# ============================================================================
# 这些参数影响位姿估计精度，可以根据实际情况调优

# 卡尔曼滤波噪声参数（固件端可配置）
POSE_PROCESS_NOISE_POS = 0.01      # 位置过程噪声（打滑严重时增大）
POSE_PROCESS_NOISE_THETA = 0.001   # 角度过程噪声
POSE_MEASUREMENT_NOISE_THETA = 0.05 # IMU测量噪声（IMU抖动时增大）

# 位姿更新频率（固件端固定）
POSE_UPDATE_RATE = 20              # Hz

# ============================================================================
# SLAM地图配置
# ============================================================================
MAP_WIDTH = 500                    # 栅格数量（X方向）
MAP_HEIGHT = 500                   # 栅格数量（Y方向）
MAP_RESOLUTION = 0.1               # 米/栅格（分辨率）
MAP_ORIGIN_X = MAP_WIDTH // 2      # 地图原点X坐标（栅格）
MAP_ORIGIN_Y = MAP_HEIGHT // 2     # 地图原点Y坐标（栅格）

# 占据概率阈值
MAP_FREE_THRESHOLD = 0.3           # 空闲判定阈值（<0.3为空闲）
MAP_OCCUPIED_THRESHOLD = 0.7       # 占用判定阈值（>0.7为占用）

# 贝叶斯概率更新参数
MAP_PROB_OCCUPIED = 0.9            # 检测到障碍物时的概率
MAP_PROB_FREE = 0.3                # 射线路径空闲时的概率

# ============================================================================
# Frontier前沿探索配置
# ============================================================================
MIN_FRONTIER_SIZE = 5              # 最小前沿簇大小（栅格数）
FRONTIER_CLUSTER_DIST = 0.3        # 前沿聚类距离（米）
FRONTIER_SELECTION = 'nearest'     # 前沿选择策略
                                   # 'nearest' - 选择最近的
                                   # 'largest' - 选择最大的
                                   # 'information_gain' - 选择信息增益最大的

# ============================================================================
# 路径规划配置（A*算法）
# ============================================================================
PATH_OBSTACLE_THRESHOLD = 0.7      # 障碍物判定阈值
PATH_INFLATION_RADIUS = 2          # 障碍物膨胀半径（栅格数）
PATH_ALLOW_DIAGONAL = True         # 是否允许对角线移动
PATH_DIAGONAL_COST = 1.414         # 对角线移动代价（√2）
PATH_SMOOTHING_TOLERANCE = 0.5     # Douglas-Peucker路径平滑容差（米）

# ============================================================================
# DWA动态窗口避障配置
# ============================================================================
DWA_MAX_SPEED = 1.0                # 最大线速度（m/s）
DWA_MAX_YAW_RATE = 40.0            # 最大角速度（度/秒）
DWA_MAX_ACCEL = 0.5                # 最大加速度（m/s²）
DWA_MAX_YAW_ACCEL = 40.0           # 最大角加速度（度/秒²）

DWA_DT = 0.1                       # 预测时间步长（秒）
DWA_PREDICT_TIME = 2.0             # 预测时间长度（秒）
DWA_V_RESOLUTION = 0.1             # 速度采样分辨率（m/s）
DWA_YAW_RESOLUTION = 5.0           # 角速度采样分辨率（度/秒）

# DWA评价函数权重
DWA_WEIGHT_HEADING = 0.15          # 朝向权重（0-1）
DWA_WEIGHT_CLEARANCE = 0.1         # 障碍物距离权重（0-1）
DWA_WEIGHT_VELOCITY = 0.2          # 速度权重（0-1）

# DWA安全参数
DWA_OBSTACLE_RADIUS = 0.3          # 机器人半径（米）
DWA_MIN_CLEARANCE = 0.2            # 最小安全距离（米）

# ============================================================================
# 差分驱动导航配置
# ============================================================================
# 这些参数由固件端(differential_drive.c)使用
NAV_REACH_THRESHOLD = 0.05         # 位置到达阈值（米）5cm
NAV_ANGLE_THRESHOLD = 0.1          # 角度到达阈值（弧度）~5.7度
NAV_LOOKAHEAD_DISTANCE = 0.3       # Pure Pursuit前视距离（米）
NAV_MAX_ANGULAR_SPEED = 2.0        # 最大角速度（rad/s）

# ============================================================================
# 控制器配置
# ============================================================================
CONTROL_LOOP_RATE = 20             # 控制循环频率（Hz）
STUCK_THRESHOLD = 50               # 卡住判定阈值（迭代次数）
STUCK_MIN_DISPLACEMENT = 0.01      # 卡住判定最小位移（米）

# 探索参数
EXPLORATION_MAX_STEPS = 500        # 最大探索步数
EXPLORATION_SAVE_MAP = True        # 是否保存地图

# ============================================================================
# 可视化配置
# ============================================================================
VISUALIZE_ENABLE = True            # 是否启用可视化
VISUALIZE_RATE = 30                # 可视化帧率（fps）
VISUALIZE_WINDOW_SIZE = (12, 10)  # 窗口大小（英寸，matplotlib figsize）
VISUALIZE_MULTI_WINDOW = False     # 是否启用多窗口布局（4子图）

SHOW_LIDAR_POINTS = True           # 显示雷达点云
SHOW_PATH = True                   # 显示规划路径
SHOW_FRONTIERS = True              # 显示前沿点
SHOW_TRAJECTORY = True             # 显示运动轨迹
SHOW_ROBOT = True                  # 显示机器人位置

# 历史数据缓冲区大小
TRAJECTORY_HISTORY_SIZE = 1000     # 轨迹历史最大点数
VELOCITY_HISTORY_SIZE = 200        # 速度历史最大点数（用于多窗口模式）
POSE_HISTORY_SIZE = 500            # 位姿历史最大点数

# 颜色配置（RGB 0-255）
COLOR_UNKNOWN = (128, 128, 128)    # 未知区域
COLOR_FREE = (255, 255, 255)       # 空闲区域
COLOR_OCCUPIED = (0, 0, 0)         # 占用区域
COLOR_FRONTIER = (255, 0, 0)       # 前沿点
COLOR_PATH = (0, 255, 0)           # 路径
COLOR_ROBOT = (0, 0, 255)          # 机器人

# ============================================================================
# 日志配置
# ============================================================================
LOG_DIR = 'data/logs'
LOG_LEVEL = 'INFO'                 # DEBUG | INFO | WARNING | ERROR
ENABLE_FILE_LOG = True
ENABLE_CONSOLE_LOG = True
LOG_FORMAT = '[%(asctime)s] %(name)s - %(levelname)s - %(message)s'
LOG_DATE_FORMAT = '%H:%M:%S'

# ============================================================================
# 数据记录配置
# ============================================================================
ENABLE_DATA_RECORDING = False
RECORDING_DIR = 'data/recordings'
RECORDING_RATE = 10                # 记录频率（Hz）

# 记录内容选择
RECORD_POSE = True                 # 记录位姿数据
RECORD_LIDAR = True                # 记录雷达数据
RECORD_MPU = False                 # 记录IMU数据
RECORD_ODOM = False                # 记录里程计数据
RECORD_MAP = True                  # 记录地图快照

# ============================================================================
# 调试配置
# ============================================================================
DEBUG_MODE = False                 # 调试模式（输出详细日志）
DEBUG_SAVE_PLOTS = False           # 保存调试图像
DEBUG_PLOT_DIR = 'data/debug'

# ============================================================================
# 辅助函数
# ============================================================================

def get_config_summary():
    """获取配置摘要（用于调试）"""
    return f"""
╔════════════════════════════════════════════════════════════════╗
║                    xxq_host 配置摘要                           ║
╠════════════════════════════════════════════════════════════════╣
║ 串口: {SERIAL_PORT} @ {BAUDRATE}                               
║ 机器人: 轮距={WHEEL_BASE}m, 轮径={WHEEL_RADIUS}m              
║ 地图: {MAP_WIDTH}x{MAP_HEIGHT} @ {MAP_RESOLUTION}m/格         
║ DWA: 最大速度={DWA_MAX_SPEED}m/s, 最大转速={DWA_MAX_YAW_RATE}°/s
║ 前沿: 最小簇={MIN_FRONTIER_SIZE}, 策略={FRONTIER_SELECTION}   
║ 可视化: {'启用' if VISUALIZE_ENABLE else '禁用'} @ {VISUALIZE_RATE}fps
║ 日志: {LOG_LEVEL} -> {LOG_DIR}                                
╚════════════════════════════════════════════════════════════════╝
    """

def validate_config():
    """验证配置参数的合理性"""
    errors = []
    warnings = []
    
    # 检查关键参数
    if WHEEL_BASE <= 0:
        errors.append("WHEEL_BASE 必须大于0")
    if WHEEL_RADIUS <= 0:
        errors.append("WHEEL_RADIUS 必须大于0")
    if MAP_RESOLUTION <= 0:
        errors.append("MAP_RESOLUTION 必须大于0")
    if DWA_MAX_SPEED <= 0:
        errors.append("DWA_MAX_SPEED 必须大于0")
    
    # 检查合理性
    if WHEEL_BASE > 1.0:
        warnings.append(f"WHEEL_BASE={WHEEL_BASE}m 看起来太大，一般小车轮距<0.5m")
    if WHEEL_RADIUS > 0.1:
        warnings.append(f"WHEEL_RADIUS={WHEEL_RADIUS}m 看起来太大，一般小车轮径<0.1m")
    if DWA_MAX_SPEED > 2.0:
        warnings.append(f"DWA_MAX_SPEED={DWA_MAX_SPEED}m/s 可能过快，建议<2.0")
    
    # 打印结果
    if errors:
        print("❌ 配置错误:")
        for err in errors:
            print(f"   - {err}")
    
    if warnings:
        print("⚠️  配置警告:")
        for warn in warnings:
            print(f"   - {warn}")
    
    if not errors and not warnings:
        print("✅ 配置验证通过")
    
    return len(errors) == 0

# ============================================================================
# 自动执行（导入时）
# ============================================================================

if __name__ == '__main__':
    # 如果直接运行此文件，显示配置摘要
    print(get_config_summary())
    validate_config()
