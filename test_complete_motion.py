#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
完整运动测试脚本
测试：前进、后退、左转、右转
目标：验证所有基本运动功能和ODO数据获取
"""

import sys
import time
sys.path.insert(0, '.')
from ble_robot_control import SimpleBLERobotComm

# 配置
BLE_ADDRESS = "C4:25:01:20:02:8E"
TEST_DURATION = 3.0  # 每个测试持续3秒

# 根据PWM转速标定数据分析.md设置目标1 RPS的转速
# 左转：PWM 0.75 → 平均约1.0 RPS（左轮后退1.16，右轮前进0.92）
LEFT_TURN_LEFT_PWM = 0.75
LEFT_TURN_RIGHT_PWM = 0.28

# 右转：提高左轮PWM避免撞墙（左轮前进需要更快）
# 根据标定数据：左轮PWM 0.55 → 2.01 RPS（前进）
#                右轮PWM 0.45 → 1.50 RPS（后退）
# 平均速度：(2.01 + 1.50) / 2 = 1.76 RPS（偏快，但左轮主导避免撞墙）
RIGHT_TURN_LEFT_PWM = 0.55   # 左轮前进（提速避免撞墙）
RIGHT_TURN_RIGHT_PWM = 0.45  # 右轮后退


def test_complete_motion():
    """完整运动测试"""
    
    print("\n" + "="*80)
    print("🤖 完整运动功能测试")
    print("="*80)
    print("\n测试内容：")
    print("  1. 前进（PID控制，1.5 RPS）")
    print("  2. 后退（PID控制，1.5 RPS）")
    print("  3. 左转（TURN命令，目标 ~1.0 RPS）")
    print("  4. 右转（TURN命令，目标 ~1.0 RPS）")
    print(f"\n每项测试时长：{TEST_DURATION} 秒")
    print("="*80 + "\n")
    
    robot = SimpleBLERobotComm(address=BLE_ADDRESS, verbose=False)
    
    # 用于收集ODO数据
    odo_samples = []
    test_results = {}
    
    def on_odo_update(data):
        """ODO数据回调"""
        # 过滤异常值（>10 RPS）
        if abs(data.left_speed) < 10 and abs(data.right_speed) < 10:
            odo_samples.append({
                'timestamp': data.timestamp,
                'left_speed': data.left_speed,
                'right_speed': data.right_speed,
                'left_count': data.left_count,
                'right_count': data.right_count
            })
            # 实时显示（覆盖同一行）
            print(f"\r  [ODO] 左轮={data.left_speed:+6.2f} RPS, 右轮={data.right_speed:+6.2f} RPS", 
                  end='', flush=True)
    
    robot.on_odom_update = on_odo_update
    
    try:
        # 连接BLE
        print("🔵 连接BLE...")
        if not robot.connect():
            print("❌ 连接失败")
            return
        print("✅ BLE已连接\n")
        time.sleep(1)
        
        # ========== 测试1: 前进 ==========
        print("\n" + "="*80)
        print("📍 测试1/4: 前进运动（PID控制）")
        print("="*80)
        print(f"  命令: MODE,1")
        print(f"  目标: 1.5 RPS (PID控制)")
        print(f"  时长: {TEST_DURATION}秒\n")
        
        odo_samples.clear()
        robot.send_command("MODE,1\n")
        
        print(f"  ⏳ 测试中...")
        time.sleep(TEST_DURATION)
        
        # 停止
        robot.send_command("MODE,0\n")
        time.sleep(0.5)
        print()  # 换行
        
        # 分析结果
        test_results['forward'] = analyze_motion(odo_samples, "前进")
        time.sleep(1)
        
        # ========== 测试2: 后退 ==========
        print("\n" + "="*80)
        print("📍 测试2/4: 后退运动（PID控制）")
        print("="*80)
        print(f"  命令: MODE,2")
        print(f"  目标: 1.5 RPS (PID控制)")
        print(f"  时长: {TEST_DURATION}秒\n")
        
        odo_samples.clear()
        robot.send_command("MODE,2\n")
        
        print(f"  ⏳ 测试中...")
        time.sleep(TEST_DURATION)
        
        # 停止
        robot.send_command("MODE,0\n")
        time.sleep(0.5)
        print()  # 换行
        
        # 分析结果
        test_results['backward'] = analyze_motion(odo_samples, "后退")
        time.sleep(1)
        
        # ========== 测试3: 左转 ==========
        print("\n" + "="*80)
        print("📍 测试3/4: 左转运动（TURN命令，目标 ~1.0 RPS）")
        print("="*80)
        print(f"  命令: TURN,0,{LEFT_TURN_LEFT_PWM:.2f},{LEFT_TURN_RIGHT_PWM:.2f}")
        print(f"  配置: 左轮后退PWM={LEFT_TURN_LEFT_PWM:.2f}, 右轮前进PWM={LEFT_TURN_RIGHT_PWM:.2f}")
        print(f"  预期: 左轮~1.16 RPS（后退）, 右轮~0.92 RPS（前进）")
        print(f"  时长: {TEST_DURATION}秒\n")
        
        odo_samples.clear()
        robot.send_command(f"TURN,0,{LEFT_TURN_LEFT_PWM:.2f},{LEFT_TURN_RIGHT_PWM:.2f}\n")
        
        print(f"  ⏳ 测试中...")
        time.sleep(TEST_DURATION)
        
        # 停止
        robot.send_command("MODE,0\n")
        time.sleep(0.5)
        print()  # 换行
        
        # 分析结果
        test_results['left_turn'] = analyze_motion(odo_samples, "左转")
        time.sleep(1)
        
        # ========== 测试4: 右转 ==========
        print("\n" + "="*80)
        print("📍 测试4/4: 右转运动（TURN命令，目标 ~1.0 RPS）")
        print("="*80)
        print(f"  命令: TURN,1,{RIGHT_TURN_LEFT_PWM:.2f},{RIGHT_TURN_RIGHT_PWM:.2f}")
        print(f"  配置: 左轮前进PWM={RIGHT_TURN_LEFT_PWM:.2f}, 右轮后退PWM={RIGHT_TURN_RIGHT_PWM:.2f}")
        print(f"  预期: 左轮~2.01 RPS（前进，加速避墙）, 右轮~1.50 RPS（后退）")
        print(f"  策略: 左轮主导转向，避免撞墙")
        print(f"  时长: {TEST_DURATION}秒\n")
        
        odo_samples.clear()
        robot.send_command(f"TURN,1,{RIGHT_TURN_LEFT_PWM:.2f},{RIGHT_TURN_RIGHT_PWM:.2f}\n")
        
        print(f"  ⏳ 测试中...")
        time.sleep(TEST_DURATION)
        
        # 停止
        robot.send_command("MODE,0\n")
        time.sleep(0.5)
        print()  # 换行
        
        # 分析结果
        test_results['right_turn'] = analyze_motion(odo_samples, "右转")
        time.sleep(1)
        
        # ========== 总结报告 ==========
        print("\n" + "="*80)
        print("📊 测试总结报告")
        print("="*80)
        
        print_summary_table(test_results)
        
        # 评估整体性能
        print("\n💡 性能评估:")
        evaluate_performance(test_results)
        
        print("\n" + "="*80)
        print("✅ 测试完成！")
        print("="*80 + "\n")
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 用户中断测试")
        robot.send_command("MODE,0\n")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        robot.send_command("MODE,0\n")
    finally:
        robot.disconnect()


def analyze_motion(samples, motion_name):
    """分析运动数据"""
    if not samples or len(samples) < 5:
        print(f"\n  ⚠️ {motion_name}: 数据不足（样本数: {len(samples)}）")
        return {
            'valid': False,
            'sample_count': len(samples),
            'avg_left': 0,
            'avg_right': 0,
            'avg_total': 0
        }
    
    # 去掉前后各20%的数据（启动和停止阶段）
    trim_count = max(1, len(samples) // 5)
    valid_samples = samples[trim_count:-trim_count] if len(samples) > 2*trim_count else samples
    
    if not valid_samples:
        print(f"\n  ⚠️ {motion_name}: 有效数据不足")
        return {
            'valid': False,
            'sample_count': 0,
            'avg_left': 0,
            'avg_right': 0,
            'avg_total': 0
        }
    
    # 计算平均速度
    avg_left = sum(s['left_speed'] for s in valid_samples) / len(valid_samples)
    avg_right = sum(s['right_speed'] for s in valid_samples) / len(valid_samples)
    avg_total = (abs(avg_left) + abs(avg_right)) / 2
    
    # 计算速度均衡度
    if avg_total > 0.1:
        balance = min(abs(avg_left), abs(avg_right)) / max(abs(avg_left), abs(avg_right)) * 100
    else:
        balance = 0
    
    print(f"\n  📊 {motion_name}结果:")
    print(f"     左轮平均速度: {avg_left:+6.2f} RPS")
    print(f"     右轮平均速度: {avg_right:+6.2f} RPS")
    print(f"     平均转速:     {avg_total:6.2f} RPS")
    print(f"     速度均衡度:   {balance:6.1f}%")
    print(f"     有效样本数:   {len(valid_samples)}/{len(samples)}")
    
    return {
        'valid': True,
        'sample_count': len(valid_samples),
        'avg_left': avg_left,
        'avg_right': avg_right,
        'avg_total': avg_total,
        'balance': balance
    }


def print_summary_table(results):
    """打印总结表格"""
    print("\n| 运动方式 | 左轮速度 | 右轮速度 | 平均转速 | 均衡度 | 样本数 | 状态 |")
    print("|---------|---------|---------|---------|--------|--------|------|")
    
    for name, label, target in [
        ('forward', '前进', 1.5),
        ('backward', '后退', 1.5),
        ('left_turn', '左转', 1.0),
        ('right_turn', '右转', 1.75)  # 提高目标（左轮主导避墙）
    ]:
        if name not in results or not results[name]['valid']:
            print(f"| {label:<6} | - | - | - | - | 0 | ❌ 无数据 |")
            continue
        
        r = results[name]
        deviation = abs(r['avg_total'] - target) / target * 100 if target > 0 else 0
        
        # 判断状态
        if deviation < 10:
            status = "✅ 优秀"
        elif deviation < 20:
            status = "⚠️ 可接受"
        else:
            status = "❌ 偏差大"
        
        print(f"| {label:<6} | {r['avg_left']:+6.2f} | {r['avg_right']:+6.2f} | "
              f"{r['avg_total']:6.2f} | {r['balance']:5.1f}% | "
              f"{r['sample_count']:4d} | {status} |")


def evaluate_performance(results):
    """评估整体性能"""
    issues = []
    
    # 检查前进
    if 'forward' in results and results['forward']['valid']:
        if abs(results['forward']['avg_total'] - 1.5) > 0.3:
            issues.append(f"  ⚠️ 前进速度偏差较大（目标1.5 RPS，实测{results['forward']['avg_total']:.2f} RPS）")
    
    # 检查后退
    if 'backward' in results and results['backward']['valid']:
        if abs(results['backward']['avg_total'] - 1.5) > 0.3:
            issues.append(f"  ⚠️ 后退速度偏差较大（目标1.5 RPS，实测{results['backward']['avg_total']:.2f} RPS）")
    
    # 检查左转
    if 'left_turn' in results and results['left_turn']['valid']:
        if abs(results['left_turn']['avg_total'] - 1.0) > 0.3:
            issues.append(f"  ⚠️ 左转速度偏差较大（目标1.0 RPS，实测{results['left_turn']['avg_total']:.2f} RPS）")
        if results['left_turn']['balance'] < 70:
            issues.append(f"  ⚠️ 左转速度不均衡（均衡度{results['left_turn']['balance']:.1f}%）")
    
    # 检查右转
    if 'right_turn' in results and results['right_turn']['valid']:
        if abs(results['right_turn']['avg_total'] - 1.75) > 0.4:
            issues.append(f"  ⚠️ 右转速度偏差较大（目标1.75 RPS，实测{results['right_turn']['avg_total']:.2f} RPS）")
        # 右转允许不均衡（左轮主导避墙）
        if results['right_turn']['balance'] < 60:
            issues.append(f"  ⚠️ 右转速度不均衡（均衡度{results['right_turn']['balance']:.1f}%，左轮应主导）")
    
    if not issues:
        print("  ✅ 所有运动功能正常！")
    else:
        print("  发现以下问题：")
        for issue in issues:
            print(issue)
    
    # 给出建议
    print("\n📝 调整建议:")
    
    if 'left_turn' in results and results['left_turn']['valid']:
        left_avg = results['left_turn']['avg_total']
        if left_avg < 0.8:
            print(f"  • 左转速度偏低（{left_avg:.2f} RPS）→ 建议提高PWM至0.80-0.85")
        elif left_avg > 1.2:
            print(f"  • 左转速度偏高（{left_avg:.2f} RPS）→ 建议降低PWM至0.65-0.70")
        else:
            print(f"  ✅ 左转速度合适（{left_avg:.2f} RPS）")
    
    if 'right_turn' in results and results['right_turn']['valid']:
        right_avg = results['right_turn']['avg_total']
        right_left = results['right_turn']['avg_left']
        right_right = results['right_turn']['avg_right']
        if right_avg < 1.3:
            print(f"  • 右转速度偏低（{right_avg:.2f} RPS）→ 建议提高左轮PWM")
        elif right_avg > 2.2:
            print(f"  • 右转速度偏高（{right_avg:.2f} RPS）→ 建议降低左轮PWM")
        else:
            print(f"  ✅ 右转速度合适（{right_avg:.2f} RPS，左轮主导避墙）")
            if abs(right_left) < abs(right_right):
                print(f"     ⚠️ 左轮速度({abs(right_left):.2f})应 > 右轮速度({abs(right_right):.2f})")


if __name__ == '__main__':
    test_complete_motion()

