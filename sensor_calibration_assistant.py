#!/usr/bin/env python3
"""
传感器校准助手脚本
根据Sensor_Calibration_Guide.md的要求，计算自定义旋转值
"""

import re
from typing import Dict, Tuple

def parse_sensor_data(input_text: str) -> Dict[str, any]:
    """
    解析从wbot_main_driver pd命令获得的姿态数据
    """
    data = {}
    
    # 解析姿态数据 - 支持多种格式
    # 匹配 "vehicle_attitude: Roll=xxx deg, Pitch=xxx deg, Yaw=xxx deg" 格式
    attitude_match = re.search(r'vehicle_attitude:\s*Roll=([+-]?\d+(?:\.\d+)?)\s*deg,\s*Pitch=([+-]?\d+(?:\.\d+)?)\s*deg,\s*Yaw=([+-]?\d+(?:\.\d+)?)\s*deg', input_text)
    
    # 如果上面的匹配失败，尝试匹配 "vehicle_attitude: Roll=xxxdeg, Pitch=xxxdeg, Yaw=xxxdeg" (没有空格) 格式
    if not attitude_match:
        attitude_match = re.search(r'vehicle_attitude:\s*Roll=([+-]?\d+(?:\.\d+)?)\s*deg\s*,\s*Pitch=([+-]?\d+(?:\.\d+)?)\s*deg\s*,\s*Yaw=([+-]?\d+(?:\.\d+)?)(?:\s*deg)?', input_text)
    
    # 如果上面的匹配失败，尝试更宽松的匹配
    if not attitude_match:
        attitude_match = re.search(r'Roll=([+-]?\d+(?:\.\d+)?)\s*deg', input_text)
        pitch_match = re.search(r'Pitch=([+-]?\d+(?:\.\d+)?)\s*deg', input_text)
        yaw_match = re.search(r'Yaw=([+-]?\d+(?:\.\d+)?)\s*deg', input_text)
        
        if attitude_match and pitch_match and yaw_match:
            data['attitude'] = {
                'roll': float(attitude_match.group(1)),
                'pitch': float(pitch_match.group(1)),
                'yaw': float(yaw_match.group(1))
            }
            return data
    
    if attitude_match:
        data['attitude'] = {
            'roll': float(attitude_match.group(1)),
            'pitch': float(attitude_match.group(2)),
            'yaw': float(attitude_match.group(3))
        }
    
    return data

def calculate_custom_rotation(roll_error: float, pitch_error: float) -> Tuple[int, int, int]:
    """
    根据姿态误差计算自定义旋转值，按照Sensor_Calibration_Guide.md的要求，
    使用误差值的相反数作为旋转值
    """
    # 计算误差的相反数
    custom_roll = round(-roll_error) % 360
    custom_pitch = round(-pitch_error) % 360
    custom_yaw = 0  # 通常不对yaw进行校准
    
    # 确保结果在有效范围内
    custom_roll = custom_roll if custom_roll >= 0 else custom_roll + 360
    custom_pitch = custom_pitch if custom_pitch >= 0 else custom_pitch + 360
    
    return (custom_roll, custom_pitch, custom_yaw)

def main():
    print("传感器校准助手 - 自定义旋转计算器")
    print("="*60)
    print("请将从 'wbot_main_driver pd' 命令获得的数据粘贴到此处，然后按 Ctrl+D (Linux/Mac) 或 Ctrl+Z (Windows) 结束输入:")
    
    # 读取用户输入
    input_text = []
    try:
        while True:
            line = input()
            input_text.append(line)
    except EOFError:
        pass
    
    input_text = "\n".join(input_text)
    
    # 解析数据
    data = parse_sensor_data(input_text)
    
    if not data or 'attitude' not in data:
        print("错误：未找到姿态数据或数据格式不正确")
        print("请确保输入的数据包含类似这样的格式：")
        print("  vehicle_attitude: Roll=180 deg, Pitch=-90 deg, Yaw=95.67 deg")
        print("  或者：Roll=180 deg, Pitch=-90 deg, Yaw=95.67 deg")
        return
    
    print(f"\n解析到的姿态数据:")
    print(f"  Roll={data['attitude']['roll']:.2f}°, Pitch={data['attitude']['pitch']:.2f}°, Yaw={data['attitude']['yaw']:.2f}°")
    
    # 按照Sensor_Calibration_Guide.md的指导，创建自定义旋转
    custom_roll, custom_pitch, custom_yaw = calculate_custom_rotation(
        data['attitude']['roll'], 
        data['attitude']['pitch']
    )
    
    print(f"\n根据 Sensor_Calibration_Guide.md 创建自定义旋转:")
    print(f"  自定义旋转: Roll={custom_roll}°, Pitch={custom_pitch}°, Yaw={custom_yaw}°")
    print(f"  计算方式: 使用测量误差的相反数作为旋转值")
    print(f"  例如: 如果测量到 Roll={data['attitude']['roll']:.2f}°, Pitch={data['attitude']['pitch']:.2f}°")
    print(f"       则添加: {{ {custom_roll:3d}, {custom_pitch:3d}, {custom_yaw:3d} }}  // 自定义校准旋转，约等于 -({data['attitude']['roll']:.2f}), -({data['attitude']['pitch']:.2f}), 0")


if __name__ == "__main__":
    main()