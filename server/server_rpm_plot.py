import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import time
import numpy as np
import pandas as pd
from datetime import datetime
import os

TIME_INTERVAL = 200  # ms

class RPMPlotter:
    def __init__(self):
        self.rpm_data = [[] for _ in range(3)]  # Data storage for plotting RPM
        self.time_data = []
        self.start_time = time.time()
        self.last_update_time = 0
        self.buffer = []
        self.plot_window = None
        self.user_zoom = False  # Thêm biến để theo dõi người dùng đã zoom/pan chưa
        
        # Tạo thư mục logs nếu chưa tồn tại
        if not os.path.exists("logs"):
            os.makedirs("logs")
        
        # Tạo figure và axes
        self.fig, self.ax = plt.subplots(figsize=(10, 6))  # Tăng kích thước biểu đồ
        self.lines = [self.ax.plot([], [], label=f"Motor {i+1}", linewidth=2)[0] for i in range(3)]
        
        # Làm đẹp biểu đồ
        self.ax.set_xlim(0, 30)
        self.ax.set_ylim(-200, 200)
        self.ax.set_xlabel("Time (s)", fontsize=12)
        self.ax.set_ylabel("RPM", fontsize=12)
        self.ax.set_title("Motor RPM Real-time Monitoring", fontsize=14)
        self.ax.grid(True, linestyle='--', alpha=0.7)
        self.ax.legend(fontsize=10)
        
        # Thêm thanh công cụ
        plt.tight_layout()
        
        self.ani = None
        self.canvas = None
        self.toolbar = None

        # Thêm handler cho sự kiện zoom/pan
        self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_event)
        self.fig.canvas.mpl_connect('button_release_event', self.on_mouse_event)
        self.fig.canvas.mpl_connect('scroll_event', self.on_mouse_event)

    def show_plot(self):
        import tkinter as tk
        
        # If window exists but is hidden, show it again
        if self.plot_window is not None and self.plot_window.winfo_exists():
            self.plot_window.deiconify()  # Make window visible again
            self.plot_window.focus_force()
            return
            
        # Otherwise create a new window
        self.plot_window = tk.Toplevel()
        self.plot_window.title("RPM Monitoring")
        self.plot_window.geometry("800x700")
        
        # Tạo canvas cho matplotlib
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_window)
        self.canvas.draw()
        
        # Thêm thanh công cụ của matplotlib với các nút zoom, pan, save,...
        toolbar_frame = tk.Frame(self.plot_window)
        toolbar_frame.pack(side=tk.TOP, fill=tk.X)
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        self.toolbar.update()
        
        # Đặt canvas vào cửa sổ
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        # Bắt đầu animation
        if self.ani is None:
            self.ani = FuncAnimation(self.fig, self.update_plot, interval=TIME_INTERVAL, cache_frame_data=False)
        
        # Xử lý sự kiện đóng cửa sổ
        self.plot_window.protocol("WM_DELETE_WINDOW", self.on_close)

    def on_close(self):
        # Only hide the window, don't destroy animation
        if self.plot_window:
            self.plot_window.withdraw()

    def update_plot(self, frame):
        """Cập nhật dữ liệu trên biểu đồ."""
        current_time = time.time()
        if current_time - self.last_update_time < TIME_INTERVAL / 1000:
            return

        if not self.buffer:
            return

        for encoders in self.buffer:
            self._add_rpm_data(encoders)
        self.buffer.clear()

        for i in range(3):
            self.lines[i].set_data(self.time_data, self.rpm_data[i])

        # Chỉ tự động điều chỉnh trục nếu người dùng chưa zoom/pan hoặc 
        # dữ liệu đã vượt ra ngoài phạm vi hiện tại
        if not self.user_zoom:
            if len(self.time_data) > 1:
                self.ax.set_xlim(max(0, self.time_data[-1] - 30), self.time_data[-1] + 1)

            # Tự động điều chỉnh trục Y
            if any(rpm_list for rpm_list in self.rpm_data):
                all_values = [val for sublist in self.rpm_data for val in sublist if val is not None]
                if all_values:
                    min_val = min(all_values)
                    max_val = max(all_values)
                    padding = max(20, (max_val - min_val) * 0.1)
                    self.ax.set_ylim(min_val - padding, max_val + padding)
        # Kiểm tra xem dữ liệu mới đã vượt ra khỏi khung hiện tại chưa
        elif len(self.time_data) > 0 and self.time_data[-1] > self.ax.get_xlim()[1]:
            # Chỉ mở rộng phạm vi trục x nếu cần, giữ nguyên mức zoom
            current_span = self.ax.get_xlim()[1] - self.ax.get_xlim()[0]
            self.ax.set_xlim(self.time_data[-1] - current_span, self.time_data[-1])

        # Cập nhật canvas nếu đang hiển thị
        if self.canvas and self.plot_window and self.plot_window.winfo_exists():
            self.canvas.draw_idle()  # Chỉ vẽ lại khi cần thiết
            
        self.last_update_time = current_time

    def add_rpm_data(self, encoders):
        """Nhận dữ liệu và thêm vào buffer."""
        self.buffer.append(encoders)

    def _add_rpm_data(self, encoders):
        """Thêm dữ liệu từ buffer vào danh sách."""
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)

        for i in range(3):
            try:
                value = float(encoders[i]) if encoders[i] not in [None, ""] else 0
                self.rpm_data[i].append(value)
            except ValueError:
                self.rpm_data[i].append(0)

        # Đồng bộ hóa dữ liệu, tránh lỗi shape mismatch
        min_length = min(len(self.time_data), *(len(self.rpm_data[i]) for i in range(3)))

        self.time_data = self.time_data[-min_length:]
        for i in range(3):
            self.rpm_data[i] = self.rpm_data[i][-min_length:]
    
    def cleanup(self):
        """Dọn dẹp tài nguyên khi đóng ứng dụng"""
        if self.ani:
            self.ani.event_source.stop()
            self.ani = None
        
        if self.plot_window:
            self.plot_window.destroy()
            self.plot_window = None

    def on_mouse_event(self, event):
        # Đánh dấu rằng người dùng đã can thiệp vào đồ thị
        self.user_zoom = True

rpm_plotter = RPMPlotter()

def update_rpm_plot(encoders):
    rpm_plotter.add_rpm_data(encoders)