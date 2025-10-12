"""
通信模块
支持串口(COM)和BLE两种连接方式
"""

from .robot_comm import RobotComm
from .robot_comm_ble import RobotCommBLE
from .protocol import (
    LidarData, MPUData, OdometryData, PoseData,
    CommandType, RobotMode
)


def create_robot_comm(connection_type='serial', **kwargs):
    """
    工厂函数：创建机器人通信对象
    
    Args:
        connection_type: 连接类型，'serial'(串口) 或 'ble'(蓝牙)
        **kwargs: 连接参数
            - serial模式: port, baudrate
            - ble模式: address
    
    Returns:
        RobotComm或RobotCommBLE对象
    
    Example:
        >>> # 串口连接
        >>> comm = create_robot_comm('serial', port='COM5', baudrate=115200)
        
        >>> # BLE连接
        >>> comm = create_robot_comm('ble', address='C4:25:01:20:02:8E')
    """
    if connection_type.lower() in ['serial', 'com', 'uart']:
        port = kwargs.get('port', 'COM5')
        baudrate = kwargs.get('baudrate', 115200)
        return RobotComm(port=port, baudrate=baudrate)
    
    elif connection_type.lower() in ['ble', 'bluetooth']:
        address = kwargs.get('address', None)
        if not address:
            raise ValueError("BLE连接需要提供address参数")
        return RobotCommBLE(address=address)
    
    else:
        raise ValueError(f"不支持的连接类型: {connection_type}，请使用'serial'或'ble'")


__all__ = [
    'RobotComm',
    'RobotCommBLE',
    'create_robot_comm',
    'LidarData',
    'MPUData',
    'OdometryData',
    'PoseData',
    'CommandType',
    'RobotMode',
]

