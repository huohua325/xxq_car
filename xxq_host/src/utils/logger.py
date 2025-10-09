"""
日志系统模块
统一的日志配置和管理
"""

import logging
import sys
from pathlib import Path
from datetime import datetime
from logging.handlers import RotatingFileHandler


def setup_logger(
    name: str,
    log_file: str = None,
    level: int = logging.INFO,
    console: bool = True,
    max_bytes: int = 10*1024*1024,  # 10MB
    backup_count: int = 5
) -> logging.Logger:
    """配置日志记录器
    
    Args:
        name: 日志记录器名称
        log_file: 日志文件路径（None则只输出到控制台）
        level: 日志级别
        console: 是否输出到控制台
        max_bytes: 单个日志文件最大字节数
        backup_count: 保留的备份文件数量
    
    Returns:
        配置好的Logger对象
    
    Example:
        >>> comm_logger = setup_logger('communication', 'data/logs/comm.log')
        >>> comm_logger.info('连接成功')
        >>> comm_logger.error('连接失败', exc_info=True)
    """
    
    # 创建logger
    logger = logging.getLogger(name)
    logger.setLevel(level)
    
    # 避免重复添加handler
    if logger.handlers:
        return logger
    
    # 格式化器
    formatter = logging.Formatter(
        fmt='[%(asctime)s] %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # 文件处理器（带轮转）
    if log_file:
        log_path = Path(log_file)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        
        file_handler = RotatingFileHandler(
            log_path,
            maxBytes=max_bytes,
            backupCount=backup_count,
            encoding='utf-8'
        )
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    
    # 控制台处理器
    if console:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(level)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)
    
    return logger


def setup_all_loggers(base_dir: str = 'data/logs', level: int = logging.INFO):
    """配置所有模块的日志记录器
    
    Args:
        base_dir: 日志基础目录
        level: 日志级别
    
    Returns:
        dict: 所有logger的字典
    """
    
    base_path = Path(base_dir)
    base_path.mkdir(parents=True, exist_ok=True)
    
    # 创建时间戳后缀
    timestamp = datetime.now().strftime('%Y%m%d')
    
    loggers = {
        'main': setup_logger('main', base_path / f'main_{timestamp}.log', level),
        'communication': setup_logger('communication', base_path / f'comm_{timestamp}.log', level),
        'slam': setup_logger('slam', base_path / f'slam_{timestamp}.log', level),
        'navigation': setup_logger('navigation', base_path / f'nav_{timestamp}.log', level),
        'visualization': setup_logger('visualization', base_path / f'viz_{timestamp}.log', level),
    }
    
    return loggers


class PerformanceLogger:
    """性能日志记录器
    
    用于记录函数执行时间、频率等性能指标
    """
    
    def __init__(self, logger: logging.Logger):
        self.logger = logger
        self.timings = {}
        self.call_counts = {}
    
    def log_execution_time(self, func_name: str, duration: float):
        """记录函数执行时间
        
        Args:
            func_name: 函数名称
            duration: 执行时长（秒）
        """
        if func_name not in self.timings:
            self.timings[func_name] = []
            self.call_counts[func_name] = 0
        
        self.timings[func_name].append(duration)
        self.call_counts[func_name] += 1
        
        # 每100次调用输出统计
        if self.call_counts[func_name] % 100 == 0:
            avg_time = sum(self.timings[func_name]) / len(self.timings[func_name])
            self.logger.info(
                f"[性能] {func_name}: 调用{self.call_counts[func_name]}次, "
                f"平均{avg_time*1000:.2f}ms"
            )
    
    def get_statistics(self, func_name: str = None):
        """获取性能统计
        
        Args:
            func_name: 函数名（None=所有）
        
        Returns:
            统计信息字典
        """
        if func_name:
            if func_name in self.timings:
                timings = self.timings[func_name]
                return {
                    'count': self.call_counts[func_name],
                    'avg': sum(timings) / len(timings),
                    'min': min(timings),
                    'max': max(timings)
                }
            return None
        
        # 所有函数的统计
        stats = {}
        for name in self.timings:
            timings = self.timings[name]
            stats[name] = {
                'count': self.call_counts[name],
                'avg': sum(timings) / len(timings),
                'min': min(timings),
                'max': max(timings)
            }
        return stats


# 装饰器：自动记录函数执行时间
def log_performance(logger: logging.Logger):
    """装饰器：记录函数性能
    
    Example:
        >>> logger = setup_logger('test')
        >>> @log_performance(logger)
        >>> def my_func():
        >>>     time.sleep(0.1)
    """
    import functools
    import time
    
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            start = time.time()
            result = func(*args, **kwargs)
            duration = time.time() - start
            
            logger.debug(f"{func.__name__} 执行时间: {duration*1000:.2f}ms")
            return result
        return wrapper
    return decorator

