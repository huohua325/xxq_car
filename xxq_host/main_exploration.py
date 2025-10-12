"""
自主探索主程序
启动机器人自主探索未知环境

运行模式：
  python main_exploration.py          # 默认：matplotlib可视化
  python main_exploration.py --web    # Web可视化（推荐）
  python main_exploration.py --no-viz # 无可视化（仅命令行）
"""

import sys
import time
import signal
import argparse
from src.communication.robot_comm import RobotComm, setup_comm_logging
from src.navigation.controller import RobotController
import config


# 全局对象（用于信号处理）
comm = None
controller = None


def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    print("\n\n[系统] 接收到中断信号，正在安全退出...")
    if comm:
        comm.stop_robot()
        time.sleep(0.5)
        comm.stop()
    print("[系统] 已退出")
    sys.exit(0)


def main():
    """主函数"""
    global comm, controller
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='xxq机器人自主探索系统')
    parser.add_argument('--web', action='store_true', 
                       help='使用Web可视化（浏览器访问）')
    parser.add_argument('--no-viz', action='store_true', 
                       help='禁用可视化（仅命令行输出）')
    parser.add_argument('--port', type=int, default=5000,
                       help='Web服务器端口（默认5000）')
    args = parser.parse_args()
    
    # 配置日志
    setup_comm_logging()
    
    print("=" * 70)
    print(" xxq_host - 自主探索系统")
    print("=" * 70)
    print(f"\n配置信息:")
    print(f"  串口: {config.SERIAL_PORT}")
    print(f"  波特率: {config.BAUDRATE}")
    print(f"  地图大小: {config.MAP_WIDTH}×{config.MAP_HEIGHT}格")
    print(f"  地图分辨率: {config.MAP_RESOLUTION}m/格")
    print(f"  DWA最大速度: {config.DWA_MAX_SPEED}m/s")
    print(f"  前沿簇大小: {config.MIN_FRONTIER_SIZE}")
    print()
    
    # 注册信号处理
    signal.signal(signal.SIGINT, signal_handler)
    
    # 1. 创建通信对象
    print("[1/4] 初始化通信模块...")
    comm = RobotComm(port=config.SERIAL_PORT, baudrate=config.BAUDRATE)
    
    # 2. 启动通信
    print("[2/4] 连接机器人...")
    if not comm.start():
        print("[错误] 无法连接串口，请检查:")
        print(f"  1. 串口设备是否存在: {config.SERIAL_PORT}")
        print(f"  2. 是否有权限（Windows无需，Linux需要）")
        print(f"  3. STM32是否已连接并通电")
        sys.exit(1)
    
    print("[系统] ✅ 串口已连接")
    
    # 3. 等待接收初始数据
    print("[3/4] 等待初始数据...")
    time.sleep(1.0)  # 等待1秒接收位姿和雷达数据
    
    # 检查是否收到POSE数据
    if comm.latest_pose:
        print(f"[系统] ✅ 收到初始位姿: x={comm.latest_pose.x:.3f}, "
              f"y={comm.latest_pose.y:.3f}, θ={comm.latest_pose.theta:.1f}°")
    else:
        print("[警告] ⚠️ 未收到位姿数据，将使用默认值(0, 0, 0)")
    
    # 4. 创建控制器（会自动绑定回调）
    print("[4/4] 初始化控制器...")
    
    # 确定可视化模式
    if args.no_viz:
        viz_mode = 'none'
        viz_enabled = False
    elif args.web:
        viz_mode = 'web'
        viz_enabled = True
    else:
        viz_mode = 'matplotlib'
        viz_enabled = True
    
    print(f"[系统] 可视化模式: {viz_mode}")
    
    controller = RobotController(
        comm=comm,
        enable_visualization=viz_enabled,
        visualization_mode=viz_mode,
        web_port=args.port if args.web else 5000
    )
    
    print("\n" + "=" * 70)
    print(" 准备就绪，开始自主探索！")
    print("=" * 70)
    
    if args.web:
        print(f"\n🌐 请在浏览器中打开: http://localhost:{args.port}")
        print("   (支持手机、平板等设备远程访问)\n")
    
    print("按 Ctrl+C 可随时停止探索\n")
    
    time.sleep(1.0)
    
    try:
        # 运行探索（默认500步，约50秒）
        success = controller.run_exploration(
            max_steps=500,
            save_map=True
        )
        
        if success:
            print("\n🎉 探索成功完成！")
        else:
            print("\n⚠️ 探索未完成（达到最大步数）")
    
    except KeyboardInterrupt:
        print("\n[系统] 用户中断探索")
    
    except Exception as e:
        print(f"\n[错误] 探索过程中出现异常: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # 清理
        print("\n[系统] 正在关闭...")
        if comm:
            comm.stop_robot()
            time.sleep(0.5)
            comm.stop()
        print("[系统] 已安全退出")


if __name__ == "__main__":
    main()

