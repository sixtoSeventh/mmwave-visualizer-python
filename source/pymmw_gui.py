#!/usr/bin/env python
"""
pymmw GUI - 毫米波雷达可视化界面
适用于 IWR1843 + SDK 3.5/3.6
"""

import os
import sys
import json
import serial
import serial.tools.list_ports
import threading
import queue
import time

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import scipy.interpolate as spi
from collections import deque
from scipy.spatial import cKDTree

# 添加当前目录到路径
script_path = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, script_path)

from lib.shell import load_config, make_config, show_config
from lib.helper import *
from mss import x8_mmw as mss

class RadarGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("pymmw - 毫米波雷达可视化")
        self.root.geometry("1400x900")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 状态变量
        self.is_connected = False
        self.is_running = False
        self.cli_port = None
        self.data_port = None
        self.data_thread = None
        self.data_queue = queue.Queue(maxsize=10)  # 限制队列大小防止积压
        self.cfg = None
        self.par = None
        
        # 时序稳定性过滤设置
        self.point_history = deque(maxlen=5)
        self.stability_radius = 0.15
        self.stability_min_hits = 2
        
        # 性能监控
        self.last_update_time = time.time()
        self.frame_counter = 0
        
        # 创建界面
        self.create_widgets()
        
        # 启动更新循环
        self.update_plots()
        
    def create_widgets(self):
        # 主框架
        main_frame = ttk.Frame(self.root, padding="5")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 左侧控制面板
        control_frame = ttk.LabelFrame(main_frame, text="控制面板", padding="10")
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        
        # 串口选择
        ttk.Label(control_frame, text="控制端口 (CLI):").pack(anchor=tk.W)
        self.cli_port_var = tk.StringVar()
        self.cli_port_combo = ttk.Combobox(control_frame, textvariable=self.cli_port_var, width=15)
        self.cli_port_combo.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(control_frame, text="数据端口:").pack(anchor=tk.W)
        self.data_port_var = tk.StringVar()
        self.data_port_combo = ttk.Combobox(control_frame, textvariable=self.data_port_var, width=15)
        self.data_port_combo.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(control_frame, text="刷新端口", command=self.refresh_ports).pack(fill=tk.X, pady=5)
        
        # 配置文件
        ttk.Label(control_frame, text="配置文件:").pack(anchor=tk.W, pady=(10, 0))
        self.config_var = tk.StringVar(value="mss/x8_mmw-xWR18xx.cfg")
        config_frame = ttk.Frame(control_frame)
        config_frame.pack(fill=tk.X, pady=(0, 10))
        ttk.Entry(config_frame, textvariable=self.config_var, width=20).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(config_frame, text="...", width=3, command=self.browse_config).pack(side=tk.RIGHT)
        
        # 连接按钮
        self.connect_btn = ttk.Button(control_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.pack(fill=tk.X, pady=5)
        
        # 开始/停止按钮
        self.start_btn = ttk.Button(control_frame, text="开始采集", command=self.toggle_acquisition, state=tk.DISABLED)
        self.start_btn.pack(fill=tk.X, pady=5)
        
        # 状态显示
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        ttk.Label(control_frame, text="状态信息", font=('', 10, 'bold')).pack(anchor=tk.W)
        
        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(control_frame, textvariable=self.status_var, foreground="gray").pack(anchor=tk.W)
        
        self.frame_count_var = tk.StringVar(value="帧数: 0")
        ttk.Label(control_frame, textvariable=self.frame_count_var).pack(anchor=tk.W)
        
        self.object_count_var = tk.StringVar(value="目标数: 0")
        ttk.Label(control_frame, textvariable=self.object_count_var).pack(anchor=tk.W)
        
        self.fps_var = tk.StringVar(value="FPS: 0.0")
        ttk.Label(control_frame, textvariable=self.fps_var).pack(anchor=tk.W)
        
        self.queue_var = tk.StringVar(value="队列: 0")
        ttk.Label(control_frame, textvariable=self.queue_var).pack(anchor=tk.W)
        
        # 可视化选项
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        ttk.Label(control_frame, text="可视化选项", font=('', 10, 'bold')).pack(anchor=tk.W)
        
        self.show_range_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(control_frame, text="距离剖面", variable=self.show_range_var).pack(anchor=tk.W)
        
        self.show_pointcloud_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(control_frame, text="3D点云", variable=self.show_pointcloud_var).pack(anchor=tk.W)
        
        self.show_azimuth_var = tk.BooleanVar(value=False)  # 默认禁用以提升性能
        ttk.Checkbutton(control_frame, text="方位角热图 (耗性能)", variable=self.show_azimuth_var).pack(anchor=tk.W)
        
        self.show_doppler_var = tk.BooleanVar(value=False)  # 默认禁用以提升性能
        ttk.Checkbutton(control_frame, text="多普勒热图 (耗性能)", variable=self.show_doppler_var).pack(anchor=tk.W)
        
        # 雷达控制
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        ttk.Label(control_frame, text="雷达控制", font=('', 10, 'bold')).pack(anchor=tk.W)
        
        self.clutter_removal_var = tk.BooleanVar(value=False)
        self.clutter_removal_cb = ttk.Checkbutton(
            control_frame, 
            text="杂波抑制 (Clutter Removal)", 
            variable=self.clutter_removal_var,
            command=self.toggle_clutter_removal
        )
        self.clutter_removal_cb.pack(anchor=tk.W)
        
        # 时序稳定性过滤
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        ttk.Label(control_frame, text="点云过滤", font=('', 10, 'bold')).pack(anchor=tk.W)
        
        self.use_stability_filter = tk.BooleanVar(value=False)
        ttk.Checkbutton(control_frame, text="时序稳定性过滤", variable=self.use_stability_filter).pack(anchor=tk.W)
        
        # 匹配半径（滑块 + 输入框）
        radius_frame = ttk.Frame(control_frame)
        radius_frame.pack(fill=tk.X, pady=5)
        ttk.Label(radius_frame, text="匹配半径 (m):").pack(side=tk.LEFT)
        self.radius_var = tk.DoubleVar(value=0.15)
        self.radius_entry = ttk.Entry(radius_frame, textvariable=self.radius_var, width=8)
        self.radius_entry.pack(side=tk.RIGHT)
        
        self.radius_scale = ttk.Scale(control_frame, from_=0.05, to=1.0, variable=self.radius_var, orient=tk.HORIZONTAL)
        self.radius_scale.pack(fill=tk.X)
        
        # 最小命中数（滑块 + 输入框）
        hits_frame = ttk.Frame(control_frame)
        hits_frame.pack(fill=tk.X, pady=5)
        ttk.Label(hits_frame, text="最小命中数:").pack(side=tk.LEFT)
        self.hits_var = tk.IntVar(value=2)
        self.hits_entry = ttk.Entry(hits_frame, textvariable=self.hits_var, width=8)
        self.hits_entry.pack(side=tk.RIGHT)
        
        ttk.Scale(control_frame, from_=1, to=5, variable=self.hits_var, orient=tk.HORIZONTAL).pack(fill=tk.X)
        
        # 点云大小
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        ttk.Label(control_frame, text="点云大小:").pack(anchor=tk.W, pady=(10, 0))
        self.point_size_var = tk.IntVar(value=8)
        ttk.Scale(control_frame, from_=2, to=20, variable=self.point_size_var, orient=tk.HORIZONTAL).pack(fill=tk.X)
        
        # 右侧可视化面板
        viz_frame = ttk.Frame(main_frame)
        viz_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 创建2x2网格的图表
        self.create_plots(viz_frame)
        
        # 初始化串口列表
        self.refresh_ports()
        
    def create_plots(self, parent):
        # 创建图表框架
        self.fig = Figure(figsize=(12, 9), dpi=100)
        self.fig.set_facecolor('#f0f0f0')
        
        # 2x2 子图布局
        # 1. 距离剖面 (左上) - 匹配 plot_range_profile.py
        self.ax_range = self.fig.add_subplot(2, 2, 1)
        self.ax_range.set_title('Range Profile', fontsize=10)
        self.ax_range.set_xlabel('Distance [m]')
        self.ax_range.set_ylabel('Relative power [dB]')
        self.ax_range.set_xlim([0, 12])
        self.ax_range.set_ylim([0, 100])
        self.ax_range.set_yticks(range(0, 101, 10))
        self.ax_range.grid(linestyle=':')
        self.range_line, = self.ax_range.plot([], [], 'b-', linewidth=1)
        self.noise_line, = self.ax_range.plot([], [], 'g-', linewidth=1, alpha=0.5)
        
        # 2. 3D点云 (右上) - 匹配 plot_detected_objects.py
        self.ax_3d = self.fig.add_subplot(2, 2, 2, projection='3d')
        self.ax_3d.set_title('CFAR Detection', fontsize=10)
        self.ax_3d.set_xlabel('x [m]')
        self.ax_3d.set_ylabel('y [m]')
        self.ax_3d.set_zlabel('z [m]')
        self.ax_3d.set_xlim([-5, 5])
        self.ax_3d.set_ylim([0, 10])
        self.ax_3d.set_zlim([-5, 5])
        self.ax_3d.view_init(azim=-45, elev=15)
        # 透明背景面板
        self.ax_3d.xaxis.pane.fill = False
        self.ax_3d.yaxis.pane.fill = False
        self.ax_3d.zaxis.pane.fill = False
        self.ax_3d.xaxis._axinfo['grid']['linestyle'] = ':'
        self.ax_3d.yaxis._axinfo['grid']['linestyle'] = ':'
        self.ax_3d.zaxis._axinfo['grid']['linestyle'] = ':'
        self.scatter_3d = None
        # 添加原点小立方体
        from itertools import product, combinations
        r = [-0.075, +0.075]
        for s, e in combinations(np.array(list(product(r,r,r))), 2):
            if np.sum(np.abs(s-e)) == r[1]-r[0]:
                self.ax_3d.plot3D(*zip(s,e), color="black", linewidth=0.5)
        
        # 3. 方位角热图 (左下) - 匹配 plot_range_azimuth_heat_map.py
        self.ax_azimuth = self.fig.add_subplot(2, 2, 3)
        self.ax_azimuth.set_title('Azimuth-Range FFT Heatmap', fontsize=10)
        self.ax_azimuth.set_xlabel('Lateral distance along [m]')
        self.ax_azimuth.set_ylabel('Longitudinal distance along [m]')
        self.azimuth_img = self.ax_azimuth.imshow(
            np.zeros((100, 100)), cmap='jet', aspect='auto',
            extent=[-5, 5, 0, 10], alpha=0.95, vmin=0, vmax=1000,
            origin='lower'
        )
        # 添加参考线
        self.ax_azimuth.plot([0, 0], [0, 10], color='white', linewidth=0.5, linestyle=':', zorder=1)
        self.ax_azimuth.plot([0, -5], [0, 5], color='white', linewidth=0.5, linestyle=':', zorder=1)
        self.ax_azimuth.plot([0, 5], [0, 5], color='white', linewidth=0.5, linestyle=':', zorder=1)
        # 添加圆弧参考线
        import matplotlib.patches as pat
        for i in range(1, 11):
            arc = pat.Arc((0, 0), width=i*2, height=i*2, angle=90, theta1=-90, theta2=90, 
                          color='white', linewidth=0.5, linestyle=':', zorder=1)
            self.ax_azimuth.add_patch(arc)
        
        # 4. 多普勒热图 (右下) - 匹配 plot_range_doppler_heat_map.py
        self.ax_doppler = self.fig.add_subplot(2, 2, 4)
        self.ax_doppler.set_title('Doppler-Range FFT Heatmap', fontsize=10)
        self.ax_doppler.set_xlabel('Longitudinal distance [m]')
        self.ax_doppler.set_ylabel('Radial velocity [m/s]')
        self.doppler_img = self.ax_doppler.imshow(
            np.zeros((15, 256)), cmap='jet', aspect='auto',
            extent=[0, 10, -1, 1], alpha=0.95,
            interpolation='quadric'
        )
        self.doppler_img.set_clim(0, 1024**2)  # 固定颜色范围，匹配原版 abs 模式
        self.ax_doppler.grid(color='white', linestyle=':', linewidth=0.5)
        self.ax_doppler.plot([0, 10], [0, 0], color='white', linestyle=':', linewidth=0.5, zorder=1)
        
        self.fig.tight_layout(pad=2)
        
        # 嵌入到Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        
    def refresh_ports(self):
        """刷新可用串口列表"""
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cli_port_combo['values'] = ports
        self.data_port_combo['values'] = ports
        
        # 自动选择
        if ports:
            # 尝试智能匹配
            for i, p in enumerate(ports):
                if 'COM10' in p or 'Enhanced' in p:
                    self.cli_port_var.set(p)
                elif 'COM9' in p or 'Standard' in p:
                    self.data_port_var.set(p)
            
            if not self.cli_port_var.get() and len(ports) >= 2:
                self.cli_port_var.set(ports[1])
                self.data_port_var.set(ports[0])
                
    def browse_config(self):
        """浏览配置文件"""
        filename = filedialog.askopenfilename(
            initialdir=os.path.join(script_path, 'mss'),
            filetypes=[("配置文件", "*.cfg"), ("所有文件", "*.*")]
        )
        if filename:
            self.config_var.set(os.path.relpath(filename, script_path))
            
    def toggle_connection(self):
        """切换连接状态"""
        if not self.is_connected:
            self.connect()
        else:
            self.disconnect()
            
    def connect(self):
        """连接雷达"""
        try:
            cli_port_name = self.cli_port_var.get()
            data_port_name = self.data_port_var.get()
            
            if not cli_port_name or not data_port_name:
                messagebox.showerror("错误", "请选择串口")
                return
                
            # 打开串口
            self.cli_port = serial.Serial(cli_port_name, 115200, timeout=0.1)
            self.data_port = serial.Serial(data_port_name, 921600, timeout=0.01)
            
            # 加载配置
            config_path = os.path.join(script_path, self.config_var.get())
            with open(config_path, 'r') as f:
                content = load_config(f)
                self.cfg = json.loads(content)
            
            self.cfg, self.par = mss._conf_(self.cfg)
            
            self.is_connected = True
            self.connect_btn.config(text="断开")
            self.start_btn.config(state=tk.NORMAL)
            self.status_var.set("已连接")
            
            # 更新图表范围
            range_max = range_maximum(self.cfg)
            self.ax_range.set_xlim([0, range_max])
            self.ax_3d.set_ylim([0, range_max])
            # 重新创建热图以更新范围
            self.azimuth_img.set_extent([-range_max/2, range_max/2, 0, range_max])
            self.doppler_img.set_extent([0, range_max, -doppler_maximum(self.cfg), doppler_maximum(self.cfg)])
            
        except Exception as e:
            messagebox.showerror("连接错误", str(e))
            self.disconnect()
            
    def disconnect(self):
        """断开连接"""
        self.stop_acquisition()
        
        if self.cli_port:
            try:
                self.cli_port.close()
            except:
                pass
            self.cli_port = None
            
        if self.data_port:
            try:
                self.data_port.close()
            except:
                pass
            self.data_port = None
            
        self.is_connected = False
        self.connect_btn.config(text="连接")
        self.start_btn.config(state=tk.DISABLED)
        self.status_var.set("未连接")
        
    def toggle_acquisition(self):
        """切换采集状态"""
        if not self.is_running:
            self.start_acquisition()
        else:
            self.stop_acquisition()
            
    def start_acquisition(self):
        """开始数据采集"""
        if not self.is_connected:
            return
            
        try:
            # 发送配置
            commands = ['sensorStop', 'flushCfg']
            config_str = make_config(self.cfg)
            config_lines = [line.strip() for line in config_str.split('\n') if line.strip()]
            commands.extend(config_lines)
            commands.append('sensorStart')
            
            for cmd in commands:
                self.cli_port.write((cmd + '\n').encode())
                time.sleep(0.05)
                
            self.is_running = True
            self.start_btn.config(text="停止采集")
            self.status_var.set("采集中...")
            
            # 启动数据接收线程
            self.data_thread = threading.Thread(target=self.data_receiver, daemon=True)
            self.data_thread.start()
            
        except Exception as e:
            messagebox.showerror("启动错误", str(e))
            
    def stop_acquisition(self):
        """停止数据采集"""
        self.is_running = False
        
        if self.cli_port and self.cli_port.is_open:
            try:
                self.cli_port.write(b'sensorStop\n')
            except:
                pass
                
        self.start_btn.config(text="开始采集")
        if self.is_connected:
            self.status_var.set("已停止")
            
    def toggle_clutter_removal(self):
        """切换杂波抑制功能"""
        if not self.is_connected or not self.cli_port or not self.cli_port.is_open:
            messagebox.showwarning("警告", "请先连接雷达")
            self.clutter_removal_var.set(False)
            return
        
        try:
            enabled = self.clutter_removal_var.get()
            val = 1 if enabled else 0
            cmd = f'clutterRemoval -1 {val}\n'
            self.cli_port.write(cmd.encode())
            time.sleep(0.05)
            
            status = "已启用" if enabled else "已禁用"
            self.status_var.set(f"杂波抑制: {status}")
            print(f"发送命令: clutterRemoval -1 {val}")
        except Exception as e:
            messagebox.showerror("命令错误", f"发送失败: {e}")
            # 还原复选框状态
            self.clutter_removal_var.set(not self.clutter_removal_var.get())
    
    def filter_stable_points(self, current_points):
        """
        时序稳定性过滤器：保留在多帧中持续出现的点
        使用 KD-Tree 进行高效的空间近邻搜索
        """
        if not self.use_stability_filter.get():
            self.point_history.append(current_points.copy())
            return current_points
            
        if len(current_points) == 0:
            return current_points
        
        # 将当前帧添加到历史
        self.point_history.append(current_points.copy())
        
        # 至少需要2帧历史进行对比
        if len(self.point_history) < 2:
            return current_points
        
        # 构建历史点云的 KD-Tree (排除当前帧)
        history_points = []
        for i in range(len(self.point_history) - 1):
            hist = self.point_history[i]
            if len(hist) > 0:
                history_points.append(hist[:, :3])
        
        if len(history_points) == 0:
            return current_points
            
        all_history = np.vstack(history_points)
        tree = cKDTree(all_history)
        
        # 检查当前帧的点在历史中是否有足够的邻居
        stable_mask = np.zeros(len(current_points), dtype=bool)
        radius = self.radius_var.get()
        min_hits = self.hits_var.get()
        
        for i, pt in enumerate(current_points[:, :3]):
            neighbors = tree.query_ball_point(pt, r=radius)
            if len(neighbors) >= min_hits:
                stable_mask[i] = True
        
        return current_points[stable_mask]
            
    def data_receiver(self):
        """数据接收线程"""
        input_buf = {'buffer': b'', 'blocks': -1, 'address': 0, 'values': 0, 'other': {}}
        output = {}
        sync = False
        
        while self.is_running:
            try:
                data = self.data_port.read(32)
                if len(data) > 0:
                    input_buf['buffer'] += data
                    
                    if data[:8] == mss._meta_['seq']:
                        if len(output) > 0:
                            self.data_queue.put(output.copy())
                        
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
                if self.is_running:
                    print(f"数据接收错误: {e}")
                break
                
    def update_plots(self):
        """更新图表（在主线程中）"""
        try:
            # 帧跳过优化：只处理队列中最新的帧，丢弃旧帧
            data = None
            skipped = 0
            while not self.data_queue.empty():
                try:
                    data = self.data_queue.get_nowait()
                    skipped += 1
                except queue.Empty:
                    break
            
            # 更新队列深度显示
            self.queue_var.set(f"队列: {self.data_queue.qsize()} (跳过: {max(0, skipped-1)})")
            
            if data is None:
                self.root.after(100, self.update_plots)  # 增加到100ms
                return
            
            # 性能计数
            self.frame_counter += 1
            
            # 更新帧计数
            if 'header' in data:
                self.frame_count_var.set(f"帧数: {data['header'].get('number', 0)}")
                self.object_count_var.set(f"目标数: {data['header'].get('objects', 0)}")
            
            # 更新距离剖面
            if self.show_range_var.get() and 'range_profile' in data:
                y = data['range_profile']
                x = np.linspace(0, self.ax_range.get_xlim()[1], len(y))
                self.range_line.set_data(x, y)
                
            if self.show_range_var.get() and 'noise_profile' in data:
                y = data['noise_profile']
                x = np.linspace(0, self.ax_range.get_xlim()[1], len(y))
                self.noise_line.set_data(x, y)
            
            # 更新3D点云（带稳定性过滤）
            if self.show_pointcloud_var.get() and 'detected_points' in data:
                points_dict = data['detected_points']
                if points_dict and len(points_dict) > 0:
                    # 转换为 numpy 数组 [x, y, z, v]
                    plist = []
                    for p in points_dict.values():
                        plist.append([p['x'], p['y'], p['z'], p.get('velocity', 0)])
                    current_points = np.array(plist)
                    
                    # 应用稳定性过滤
                    filtered_points = self.filter_stable_points(current_points)
                    
                    if len(filtered_points) > 0:
                        xs = filtered_points[:, 0]
                        ys = filtered_points[:, 1]
                        zs = filtered_points[:, 2]
                        
                        # 移除旧的散点图
                        if self.scatter_3d:
                            self.scatter_3d.remove()
                        
                        # 创建新的散点图（实心圆）
                        self.scatter_3d = self.ax_3d.scatter(
                            xs, ys, zs, 
                            c='blue',  # 实心蓝色
                            s=self.point_size_var.get()*10, 
                            alpha=0.8, marker='o'
                        )
                    else:
                        if self.scatter_3d:
                            self.scatter_3d.remove()
                            self.scatter_3d = None
                else:
                    # 即使为空也更新历史以保持同步
                    self.filter_stable_points(np.empty((0, 4)))
                    if self.scatter_3d:
                        self.scatter_3d.remove()
                        self.scatter_3d = None

            
            # 更新方位角热图
            if self.show_azimuth_var.get() and 'azimuth_static' in data:
                try:
                    a = data['azimuth_static']
                    range_bins = num_range_bin(self.cfg)
                    tx_az = num_tx_azim_antenna(self.cfg)
                    rx_ant = num_rx_antenna(self.cfg)
                    angle_bins = 64
                    
                    if len(a) == range_bins * tx_az * rx_ant * 2:
                        a = np.array([a[i] + 1j * a[i+1] for i in range(0, len(a), 2)])
                        a = np.reshape(a, (range_bins, tx_az * rx_ant))
                        a = np.fft.fft(a, angle_bins)
                        a = np.abs(a)
                        a = np.fft.fftshift(a, axes=(1,))
                        self.azimuth_img.set_array(a.T)
                        self.azimuth_img.autoscale()
                except:
                    pass
            
            # 更新多普勒热图
            if self.show_doppler_var.get() and 'range_doppler' in data:
                try:
                    a = np.array(data['range_doppler'])
                    range_bins = num_range_bin(self.cfg)
                    doppler_bins = num_doppler_bin(self.cfg)
                    
                    if len(a) == range_bins * doppler_bins:
                        # 应用线性压缩模式（匹配原版）
                        log_lin = 1/256
                        comp_lin = 1.0
                        a = comp_lin * np.power(2, a * log_lin)
                        
                        b = np.reshape(a, (range_bins, doppler_bins))
                        c = np.fft.fftshift(b, axes=(1,))
                        self.doppler_img.set_array(c[:,1:].T)
                        self.doppler_img.autoscale()
                except:
                    pass
            # 计算FPS
            current_time = time.time()
            elapsed = current_time - self.last_update_time
            if elapsed >= 1.0:  # 每秒更新一次FPS显示
                fps = self.frame_counter / elapsed
                self.fps_var.set(f"FPS: {fps:.1f}")
                self.last_update_time = current_time
                self.frame_counter = 0
            
            # 刷新canvas（使用draw_idle减少开销）
            self.canvas.draw_idle()
                
        except Exception as e:
            print(f"更新错误: {e}")
            import traceback
            traceback.print_exc()
            
        # 继续更新循环（降低刷新率到100ms）
        self.root.after(100, self.update_plots)
        
    def on_closing(self):
        """关闭窗口"""
        self.stop_acquisition()
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = RadarGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
