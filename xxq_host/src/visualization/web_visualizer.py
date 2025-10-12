"""
Web实时可视化模块
使用Flask + WebSocket实现浏览器端SLAM建图实时展示
"""

import numpy as np
from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO, emit
import base64
import io
from typing import Optional, Tuple, List
import threading
import time
from PIL import Image


class WebVisualizer:
    """Web可视化器
    
    使用Flask提供Web服务，通过WebSocket推送实时数据到浏览器
    
    特性:
    - 实时地图更新
    - 机器人位姿追踪
    - 路径和前沿点显示
    - 统计数据仪表盘
    - 响应式设计，支持移动端
    
    Example:
        >>> from src.slam.occupancy_map import OccupancyGridMap
        >>> map_obj = OccupancyGridMap()
        >>> visualizer = WebVisualizer(map_obj, port=5000)
        >>> visualizer.start()
        >>> # 在浏览器中打开 http://localhost:5000
    """
    
    def __init__(self, map_obj, host='0.0.0.0', port=5000):
        """初始化Web可视化器
        
        Args:
            map_obj: OccupancyGridMap对象
            host: 服务器地址，'0.0.0.0'表示允许外部访问
            port: 服务器端口
        """
        self.map_obj = map_obj
        self.host = host
        self.port = port
        
        # 创建Flask应用
        self.app = Flask(__name__, 
                        template_folder='../../templates',
                        static_folder='../../static')
        self.app.config['SECRET_KEY'] = 'xxq_slam_secret_2025'
        
        # 创建SocketIO
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", async_mode='threading')
        
        # 状态数据
        self.robot_pose = None
        self.lidar_points = []
        self.path = []
        self.frontiers = []
        self.velocity = None
        self.frame_count = 0
        self.is_running = False
        
        # 注册路由
        self._setup_routes()
        
        print(f"[WebViz] 初始化完成，访问地址: http://localhost:{port}")
    
    def _setup_routes(self):
        """设置Flask路由"""
        
        @self.app.route('/')
        def index():
            """主页"""
            return render_template('slam_viewer.html')
        
        @self.app.route('/api/config')
        def get_config():
            """获取地图配置"""
            return jsonify({
                'width': self.map_obj.config.width,
                'height': self.map_obj.config.height,
                'resolution': self.map_obj.config.resolution,
                'origin_x': self.map_obj.config.origin_x,
                'origin_y': self.map_obj.config.origin_y
            })
        
        @self.socketio.on('connect')
        def handle_connect():
            """客户端连接"""
            print(f"[WebViz] 客户端已连接")
            emit('connection_response', {'status': 'connected'})
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            """客户端断开"""
            print(f"[WebViz] 客户端已断开")
        
        @self.socketio.on('request_update')
        def handle_request_update():
            """客户端请求数据更新"""
            self._emit_update()
    
    def start(self, blocking=False):
        """启动Web服务器
        
        Args:
            blocking: 是否阻塞运行（默认False，在后台线程运行）
        """
        self.is_running = True
        
        if blocking:
            print(f"[WebViz] 启动服务器（阻塞模式）: http://{self.host}:{self.port}")
            self.socketio.run(self.app, host=self.host, port=self.port, 
                            debug=False, use_reloader=False)
        else:
            # 在后台线程启动
            self.server_thread = threading.Thread(
                target=lambda: self.socketio.run(
                    self.app, 
                    host=self.host, 
                    port=self.port,
                    debug=False,
                    use_reloader=False,
                    allow_unsafe_werkzeug=True
                ),
                daemon=True
            )
            self.server_thread.start()
            print(f"[WebViz] ✅ 服务器已在后台启动: http://{self.host}:{self.port}")
            time.sleep(0.5)  # 等待服务器启动
    
    def stop(self):
        """停止Web服务器"""
        self.is_running = False
        print("[WebViz] 服务器已停止")
    
    def update(self, 
               robot_pose: Optional[Tuple[float, float, float]] = None,
               lidar_points: Optional[List[Tuple[float, float]]] = None,
               path: Optional[List[Tuple[float, float]]] = None,
               frontiers: Optional[List[Tuple[float, float]]] = None,
               velocity: Optional[Tuple[float, float]] = None):
        """更新可视化数据
        
        Args:
            robot_pose: 机器人位姿 (x, y, theta)
            lidar_points: 雷达点云列表
            path: 规划路径
            frontiers: 前沿点列表
            velocity: 机器人速度 (v, omega)
        """
        # 更新内部状态
        if robot_pose is not None:
            self.robot_pose = robot_pose
        if lidar_points is not None:
            self.lidar_points = lidar_points
        if path is not None:
            self.path = path
        if frontiers is not None:
            self.frontiers = frontiers
        if velocity is not None:
            self.velocity = velocity
        
        self.frame_count += 1
        
        # 推送数据到浏览器
        self._emit_update()
    
    def _emit_update(self):
        """向所有连接的客户端推送更新"""
        if not self.is_running:
            return
        
        try:
            # 准备数据
            data = {
                'frame': self.frame_count,
                'map_image': self._get_map_image_base64(),
                'robot_pose': self.robot_pose,
                'lidar_points': self.lidar_points[:100],  # 限制点数，避免数据量过大
                'path': self.path,
                'frontiers': self.frontiers[:50],  # 限制前沿点数量
                'velocity': self.velocity,
                'statistics': self._get_statistics()
            }
            
            # 通过WebSocket发送
            self.socketio.emit('update', data, namespace='/')
            
        except Exception as e:
            print(f"[WebViz] 推送数据失败: {e}")
    
    def _get_map_image_base64(self) -> str:
        """将地图转换为base64编码的PNG图像
        
        Returns:
            base64编码的图像字符串
        """
        try:
            # 获取地图数据 (0=未知, 0.5=空闲, 1=占用)
            grid = self.map_obj.grid.copy()
            
            # 转换为8位灰度图像 (0=黑, 255=白)
            # 未知(0.5) -> 灰色(128), 空闲(0) -> 白色(255), 占用(1) -> 黑色(0)
            img_array = np.zeros_like(grid, dtype=np.uint8)
            
            # 占用区域 (概率 > 0.6) -> 黑色
            img_array[grid > 0.6] = 0
            
            # 空闲区域 (概率 < 0.4) -> 白色
            img_array[grid < 0.4] = 255
            
            # 未知区域 (0.4 <= 概率 <= 0.6) -> 灰色
            unknown_mask = (grid >= 0.4) & (grid <= 0.6)
            img_array[unknown_mask] = 128
            
            # 翻转Y轴（图像坐标系与世界坐标系Y轴相反）
            img_array = np.flipud(img_array)
            
            # 转换为PIL Image
            img = Image.fromarray(img_array, mode='L')
            
            # 编码为PNG
            buffer = io.BytesIO()
            img.save(buffer, format='PNG')
            buffer.seek(0)
            
            # 转换为base64
            img_base64 = base64.b64encode(buffer.read()).decode('utf-8')
            
            return f"data:image/png;base64,{img_base64}"
            
        except Exception as e:
            print(f"[WebViz] 地图图像生成失败: {e}")
            return ""
    
    def _get_statistics(self) -> dict:
        """获取统计数据
        
        Returns:
            统计信息字典
        """
        try:
            stats = self.map_obj.get_statistics()
            return {
                'explored_ratio': float(stats.get('explored_ratio', 0)),
                'occupied_cells': int(stats.get('occupied_cells', 0)),
                'free_cells': int(stats.get('free_cells', 0)),
                'unknown_cells': int(stats.get('unknown_cells', 0))
            }
        except Exception as e:
            print(f"[WebViz] 获取统计数据失败: {e}")
            return {
                'explored_ratio': 0,
                'occupied_cells': 0,
                'free_cells': 0,
                'unknown_cells': 0
            }


# 便捷函数
def create_web_visualizer(map_obj, port=5000, auto_start=True):
    """创建并启动Web可视化器
    
    Args:
        map_obj: OccupancyGridMap对象
        port: 服务器端口
        auto_start: 是否自动启动
    
    Returns:
        WebVisualizer实例
    """
    visualizer = WebVisualizer(map_obj, port=port)
    if auto_start:
        visualizer.start(blocking=False)
    return visualizer

