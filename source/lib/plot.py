#
# Copyright (c) 2018, Manfred Constapel
# This file is licensed under the terms of the MIT license.
#

#
# plot support
#

import sys
import time
import threading
import json
import queue

import numpy as np

import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import art3d

# ------------------------------------------------

class Line(art3d.Line3D):  # a line in 3D space

    def __init__(self, from_xyz=(0, 0, 0), to_xyz=(1, 1, 1), *args, **kwargs):
        xs, ys, zs = tuple(zip(from_xyz, to_xyz))
        art3d.Line3D.__init__(self, xs, ys, zs, *args, **kwargs)
        

    def location(self, from_, to_, *args):
        xs, ys, zs = tuple(zip(from_, to_))
        self.set_data_3d(xs, ys, zs)


class Point(Line):  # a point (a very short line) in 3D space

    def __init__(self, xyz=(0, 0, 0), color='black', marker='.', size=1, vanish=1.0, alpha=1.0):
        
        Line.__init__(self, xyz, xyz,
                      color=color, marker=marker, markersize=size,
                      markeredgewidth=1, linestyle='', fillstyle='none', alpha=alpha)
        
        self._vanish = vanish
        self._fade_delta = 0.1
        self._last_fade = time.time()

    def fade(self):
        """在主线程中调用的淡出逻辑"""
        if self._vanish is None: return True
        
        now = time.time()
        if now - self._last_fade < 0.1 * self._vanish:
            return True
        
        self._last_fade = now
        na = self.get_alpha() - self._fade_delta
        if not self.get_visible():
            try: self.remove()
            except: pass
            return False
        if na > 0:
            self.set_alpha(na)
            return True
        else:
            self.set_visible(False)
            try: self.remove()
            except: pass
            return False

    def location(self, at_, *args):
        Line.location(self, at_, at_)

# ------------------------------------------------

def set_aspect_equal_3d(ax):  # axis have to be equal 

    xlim = ax.get_xlim3d()
    ylim = ax.get_ylim3d()
    zlim = ax.get_zlim3d()

    xmean = np.mean(xlim)
    ymean = np.mean(ylim)
    zmean = np.mean(zlim)

    plot_radius = max([abs(lim - mean_) for lims, mean_ in ((xlim, xmean), (ylim, ymean), (zlim, zmean)) for lim in lims])

    ax.set_xlim3d([xmean - plot_radius, xmean + plot_radius])
    ax.set_ylim3d([ymean - plot_radius, ymean + plot_radius])
    ax.set_zlim3d([zmean - plot_radius, zmean + plot_radius])


def move_figure(fig, xy):
    backend = mpl.get_backend()
    if   backend.lower().startswith('tk'):
        try: fig.canvas.manager.window.wm_geometry("+%d+%d" % xy)
        except: pass
    elif backend.lower().startswith('wx'):
        try: fig.canvas.manager.window.SetPosition(xy)
        except: pass
    elif backend.lower().startswith('qt') or \
         backend.lower().startswith('gtk'):
        try: fig.canvas.manager.window.move(*xy)
        except: pass

# ------------------------------------------------

def update_data(q):
    """后台线程：只负责从标准输入读取数据并解析 JSON"""
    while q.alive:
        line = sys.stdin.readline()
        if not line:
            q.alive = False
            break
        try:
            temp = json.loads(line)
            q.put(temp)
        except:
            pass


def _on_timer(fig, q, func):
    """主线程：负责更新所有 GUI 元素"""
    if not q.alive:
        # plt.close(fig) # 不直接关闭，让窗口系统处理
        return
    
    cnt, clk = 0, 0
    updated = False
    
    # 1. 更新数据驱动的图形
    try:
        while not q.empty():
            item = q.get_nowait()
            if 'header' in item:
                clk, cnt = item['header']['time'], item['header']['number']
            func(item)
            updated = True
    except:
        pass

    # 2. 处理 3D 点云的自动淡出（如果在主线程中管理 artist）
    try:
        for artist in fig.gca().get_children():
            if isinstance(artist, Point):
                artist.fade()
    except:
        pass
        
    if updated:
        try:
            fig.canvas.draw_idle()
            fig.canvas.manager.set_window_title(
                'time: {} | count: {} | wait: {} | cycles: {:010} '.format(
                    '{:.3f}'.format(time.time())[-7:],
                    cnt,
                    q.qsize(),
                    clk))
        except:
            pass


def start_plot(fig, ax, func, fps):
    # 强制设置一个适合 Windows 的非阻塞后端（如果可能）
    # plt.ion() 
    
    q = queue.Queue()
    q.alive = True
    
    # 启动数据接收线程
    tu = threading.Thread(target=update_data, args=(q,))
    tu.daemon = True
    tu.start()

    # 使用 Matplotlib 的定时器机制在主线程中更新界面
    interval = int(1000 / fps)
    timer = fig.canvas.new_timer(interval=interval)
    timer.add_callback(_on_timer, fig, q, func)
    timer.start()

    try:
        plt.show(block=True)
    except KeyboardInterrupt:
        pass
    finally:
        q.alive = False
        timer.stop()
