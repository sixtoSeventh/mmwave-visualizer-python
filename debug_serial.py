"""
串口调试脚本 - 用于捕获 IWR1843 复位时的原始输出
"""
import serial
import time

# 请根据您的实际情况修改端口号
CONTROL_PORT = 'COM10'

print(f"正在打开串口 {CONTROL_PORT}...")
try:
    ser = serial.Serial(CONTROL_PORT, 115200, timeout=1)
    print("串口已打开。请按下雷达板上的 NRST 按钮...")
    print("-" * 50)
    
    start_time = time.time()
    while time.time() - start_time < 15:  # 监听 15 秒
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            try:
                text = data.decode('latin-1')
                print(text, end='', flush=True)
            except:
                print(f"[原始字节]: {data}")
        time.sleep(0.1)
    
    print("\n" + "-" * 50)
    print("监听结束。如果上方没有输出，请检查：")
    print("1. 雷达是否已正确连接并通电")
    print("2. COM 端口号是否正确")
    print("3. 雷达固件是否正常烧录")
    
    ser.close()
except Exception as e:
    print(f"错误: {e}")
