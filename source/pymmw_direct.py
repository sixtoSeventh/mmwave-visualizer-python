#!/usr/bin/env python
"""
pymmw 简化版启动器 - 直接启动可视化，无需等待 NRST 复位
适用于 IWR1843 + SDK 3.6
"""

import os
import sys
import serial
import threading
import json
import argparse
import time

# 添加当前目录到路径
script_path = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, script_path)

from lib.shell import load_config, make_config, show_config, print_log
from mss import x8_mmw as mss

# ------------------------------------------------

def send_config_direct(cli_port, cfg):
    """直接发送配置命令到雷达"""
    
    # 先停止并清空
    commands = ['sensorStop', 'flushCfg']
    
    # 生成配置命令
    config_str = make_config(cfg)
    config_lines = [line.strip() for line in config_str.split('\n') if line.strip()]
    commands.extend(config_lines)
    
    # 启动传感器
    commands.append('sensorStart')
    
    print("正在发送配置命令...")
    for cmd in commands:
        cli_port.write((cmd + '\n').encode())
        time.sleep(0.05)
        # 读取响应（可选）
        if cli_port.in_waiting > 0:
            response = cli_port.read(cli_port.in_waiting).decode('latin-1', errors='ignore')
            # print(response, end='')  # 取消注释以查看响应
    
    print("配置发送完成！")


def data_receiver(data_port, cfg, par):
    """接收并处理数据"""
    
    # 启动可视化应用
    mss._proc_(cfg, par)
    
    # 数据处理循环
    input_buf = {'buffer': b''}
    output = {}
    sync = False
    
    while True:
        try:
            data = data_port.read(32)
            if len(data) > 0:
                input_buf['buffer'] += data
                
                # 检查魔术序列
                if data[:8] == mss._meta_['seq']:
                    if len(output) > 0:
                        plain = json.dumps(output)
                        mss._pipe_(plain)
                    
                    input_buf['buffer'] = data
                    input_buf['blocks'] = -1
                    input_buf['address'] = 0
                    input_buf['values'] = 0
                    input_buf['other'] = {}
                    output = {}
                    sync = True
                
                if sync:
                    flen = 0
                    while flen < len(input_buf['buffer']):
                        flen = len(input_buf['buffer'])
                        mss.aux_buffer(input_buf, output)
                        
        except Exception as e:
            print_log(e, sys._getframe())
            break


def main():
    parser = argparse.ArgumentParser(
        description='pymmw 简化版 - 直接启动可视化',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('-c', '--control-port', default='COM10', help='控制端口（CLI）')
    parser.add_argument('-d', '--data-port', default='COM9', help='数据端口')
    parser.add_argument('--config', default='mss/x8_mmw-xWR18xx.cfg', help='配置文件路径')
    
    args = parser.parse_args()
    
    print(f"pymmw 简化版启动器")
    print(f"控制端口: {args.control_port}")
    print(f"数据端口: {args.data_port}")
    print("-" * 40)
    
    try:
        # 打开串口
        cli_port = serial.Serial(args.control_port, 115200, timeout=0.1)
        data_port = serial.Serial(args.data_port, 921600, timeout=0.01)
        
        # 加载配置
        config_path = os.path.join(script_path, args.config)
        print(f"加载配置: {config_path}")
        
        with open(config_path, 'r') as f:
            content = load_config(f)
            cfg = json.loads(content)
        
        # 处理配置
        cfg, par = mss._conf_(cfg)
        
        # 显示配置信息
        show_config(cfg)
        
        # 发送配置
        send_config_direct(cli_port, cfg)
        
        # 启动数据接收线程
        print("\n开始接收数据...")
        print("（可视化窗口应该已经弹出，如果没有请检查 matplotlib 是否正常）")
        
        data_thread = threading.Thread(target=data_receiver, args=(data_port, cfg, par))
        data_thread.daemon = True
        data_thread.start()
        
        # 保持主线程运行
        print("\n按 Ctrl+C 退出...")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n正在退出...")
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            cli_port.write(b'sensorStop\n')
            cli_port.close()
            data_port.close()
        except:
            pass


if __name__ == "__main__":
    main()
