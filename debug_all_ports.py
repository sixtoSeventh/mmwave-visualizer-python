"""
多端口串口调试脚本 - 同时监听多个端口以找到正确的控制端口
"""
import serial
import serial.tools.list_ports
import time
import threading

def monitor_port(port_name, results):
    """监听单个端口"""
    try:
        ser = serial.Serial(port_name, 115200, timeout=0.5)
        print(f"  [已打开] {port_name}")
        
        start_time = time.time()
        while time.time() - start_time < 15:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                try:
                    text = data.decode('latin-1')
                    results[port_name] = text
                    print(f"\n>>> {port_name} 收到数据:")
                    print(text)
                    print("<<<")
                except:
                    results[port_name] = f"[原始字节]: {data}"
                    print(f"\n>>> {port_name} 收到原始字节: {data}")
            time.sleep(0.1)
        ser.close()
    except Exception as e:
        print(f"  [失败] {port_name}: {e}")

# 列出所有可用端口
print("正在扫描可用串口...")
ports = list(serial.tools.list_ports.comports())
print(f"发现 {len(ports)} 个端口:")
for p in ports:
    print(f"  - {p.device}: {p.description}")

print("\n正在同时监听所有端口（15秒）...")
print("请按下雷达板上的 NRST 按钮...")
print("-" * 50)

results = {}
threads = []

# 尝试打开所有端口
for p in ports:
    t = threading.Thread(target=monitor_port, args=(p.device, results))
    t.start()
    threads.append(t)

# 等待所有线程完成
for t in threads:
    t.join()

print("-" * 50)
print("\n监听结束。")
if results:
    print("收到数据的端口:")
    for port, data in results.items():
        print(f"  {port}: {data[:100]}...")
else:
    print("没有任何端口收到数据。")
    print("\n请检查：")
    print("1. 雷达是否已通电（绿色 LED 应亮起）")
    print("2. USB 线是否正确连接")
    print("3. 设备管理器中是否显示 XDS110 端口")
