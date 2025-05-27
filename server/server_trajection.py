import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import threading
import time
import tkinter as tk
from tkinter import ttk
import math
import queue

class TrajectoryVisualizer:
    def __init__(self):
        self.trajectory_points = []
        self.trajectory_window = None
        self.canvas = None
        self.fig = None
        self.ax = None
        self.is_active = False
        self.lock = threading.Lock()
        self.update_queue = queue.Queue()
        
        # Giới hạn đồ thị và grid
        self.PLOT_X_MIN = -1.2
        self.PLOT_X_MAX = 5.4
        self.PLOT_Y_MIN = -1.2
        self.PLOT_Y_MAX = 5.4
        self.GRID_SIZE = 0.6  # m
        
        # Robot parameters
        self.wheel_radius = 0.03  # m
        self.robot_radius = 0.1543  # m
        
        # Vị trí ban đầu
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_update_time = None
        
        # Cài đặt thời gian cập nhật đồ thị (miliseconds)
        self.update_interval = 50  # ms - Giảm xuống để cập nhật nhanh hơn
        self.last_plot_update = 0

    def initialize_plot(self):
        """Initialize the trajectory visualization window"""
        if self.trajectory_window is not None:
            return
        
        self.trajectory_window = tk.Toplevel()
        self.trajectory_window.title("Robot Trajectory (from Encoders)")
        self.trajectory_window.geometry("800x600")
        self.trajectory_window.protocol("WM_DELETE_WINDOW", self.close)
        
        # Create matplotlib figure and canvas
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        
        # Thiết lập grid cố định
        grid_size = self.GRID_SIZE
        
        # Set fixed ticks at grid intervals
        x_ticks = np.arange(self.PLOT_X_MIN, self.PLOT_X_MAX + grid_size/2, grid_size)
        y_ticks = np.arange(self.PLOT_Y_MIN, self.PLOT_Y_MAX + grid_size/2, grid_size)
        self.ax.set_xticks(x_ticks)
        self.ax.set_yticks(y_ticks)
        
        # Highlight major grid lines
        self.ax.grid(True, linewidth=0.8)
        self.ax.set_facecolor('#f8f8f8')
        
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('Robot Trajectory (from Encoders)')
        
        # Set initial plot limits
        self.ax.set_xlim(self.PLOT_X_MIN, self.PLOT_X_MAX)
        self.ax.set_ylim(self.PLOT_Y_MIN, self.PLOT_Y_MAX)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.trajectory_window)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        
        # Thanh công cụ Matplotlib
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.trajectory_window)
        self.toolbar.update()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

        # Add control buttons
        button_frame = ttk.Frame(self.trajectory_window)
        button_frame.pack(side=tk.BOTTOM, fill=tk.X)
        
        ttk.Button(button_frame, text="Close", command=self.close).pack(side=tk.RIGHT, padx=5, pady=5)
        
        self.is_active = True
        self.last_update_time = time.time()
        self.last_plot_update = time.time() * 1000  # ms
        
        # Start the update checker
        self.check_update_queue()
        
    def check_update_queue(self):
        """Check for updates in the queue and apply them"""
        try:
            # Xử lý các cập nhật vị trí
            positions_updated = False
            while not self.update_queue.empty():
                data = self.update_queue.get_nowait()
                # Cập nhật vị trí mà không cập nhật đồ thị
                self._update_position(data)
                positions_updated = True
            
            # Chỉ cập nhật đồ thị sau khoảng thời gian đã đặt
            current_time = time.time() * 1000  # ms
            if positions_updated and (current_time - self.last_plot_update > self.update_interval):
                self._update_plot()
                self.last_plot_update = current_time
                
        except Exception as e:
            print(f"Error in update queue processing: {e}")
        
        # Schedule next check
        if self.trajectory_window:
            self.trajectory_window.after(50, self.check_update_queue)
    
    def update_trajectory(self, rpm_values, bno055_heading=None):
        """Update trajectory based on encoder readings and optional BNO055 heading"""
        self.update_queue.put((rpm_values, bno055_heading))

    def normalize_angle(self, angle):
        """Chuẩn hóa góc về khoảng -pi đến pi"""
        return ((angle + np.pi) % (2 * np.pi)) - np.pi

    def _update_position(self, data):
        """Chỉ cập nhật vị trí mà không vẽ đồ thị"""
        if not self.is_active:
            return
        
        rpm_values, bno055_heading = data
        
        current_time = time.time()
        
        # Skip if this is the first update
        if self.last_update_time is None:
            self.last_update_time = current_time
            return
            
        dt = 0.02
        
        # Convert RPM to rad/s for each wheel
        omega_wheels = [(rpm * 2 * np.pi / 60) for rpm in rpm_values]
        
        # Sử dụng động học thuận để tính toán vận tốc robot
        vx_global, vy_global, omega = self.forward_kinematics(
            omega_wheels[0], omega_wheels[1], omega_wheels[2], 
            self.wheel_radius, self.robot_radius, self.theta
        )
        
        # Lưu lại vận tốc để vẽ
        self.last_velocities = (vx_global, vy_global, omega)
        
        # Integrate velocity to get position
        self.x += vx_global * dt
        self.y += vy_global * dt
        self.theta += omega * dt
        
        # Chuẩn hóa theta về khoảng -pi đến pi
        self.theta = self.normalize_angle(self.theta)
        
        # Thêm điểm vào quỹ đạo
        with self.lock:
            self.trajectory_points.append((self.x, self.y))
        
        self.last_update_time = current_time

    def _update_plot(self):
        """Cập nhật đồ thị với dữ liệu hiện tại"""
        if not self.is_active or not self.ax:
            return
        
        with self.lock:
            # Get trajectory points
            x_points = [p[0] for p in self.trajectory_points]
            y_points = [p[1] for p in self.trajectory_points]
            
            # Clear and redraw
            self.ax.clear()
            self.ax.plot(x_points, y_points, 'b-')
            
            # Thiết lập grid cố định
            grid_size = self.GRID_SIZE
            
            # Đặt giới hạn cố định sử dụng các hằng số
            self.ax.set_xlim(self.PLOT_X_MIN, self.PLOT_X_MAX)
            self.ax.set_ylim(self.PLOT_Y_MIN, self.PLOT_Y_MAX)
            
            # Set fixed ticks at grid intervals
            x_ticks = np.arange(self.PLOT_X_MIN, self.PLOT_X_MAX + grid_size/2, grid_size)
            y_ticks = np.arange(self.PLOT_Y_MIN, self.PLOT_Y_MAX + grid_size/2, grid_size)
            self.ax.set_xticks(x_ticks)
            self.ax.set_yticks(y_ticks)
            
            # Highlight major grid lines
            self.ax.grid(True, linewidth=0.8)
            
            # Đảm bảo có màu nền để grid dễ nhìn hơn
            self.ax.set_facecolor('#f8f8f8')
            
            # Tính toán sin và cos của góc theta
            cos_theta = np.cos(self.theta)
            sin_theta = np.sin(self.theta)
            
            # Vẽ thân robot (hình tròn)
            robot_body = plt.Circle((self.x, self.y), self.robot_radius, fill=False, color='r', linewidth=1.5)
            self.ax.add_patch(robot_body)
            
            # Vẽ trục tọa độ body frame
            arrow_length = self.robot_radius * 1.2
            
            # Trục x của body frame - màu đỏ (hướng heading của robot)
            self.ax.arrow(self.x, self.y, arrow_length * cos_theta, arrow_length * sin_theta,
                         head_width=0.05, head_length=0.07, fc='r', ec='r', linewidth=1.5)
            
            # Trục y của body frame - màu xanh lá
            self.ax.arrow(self.x, self.y, -arrow_length * sin_theta, arrow_length * cos_theta,
                         head_width=0.05, head_length=0.07, fc='g', ec='g', linewidth=1.5)
            
            # Lấy vận tốc hiện tại
            if hasattr(self, 'last_velocities') and self.last_velocities:
                # Lấy giá trị vận tốc mới nhất đã tính
                vx_global, vy_global, omega = self.last_velocities
                
                # Vẽ vector vận tốc
                vel_mag = np.sqrt(vx_global**2 + vy_global**2)
                if (vel_mag > 0.01):  # Chỉ vẽ khi vận tốc đủ lớn
                    # Vẽ vector tốc độ - màu xanh dương đậm
                    vel_scale = 2.0  # Tỷ lệ để vector dài hơn và dễ thấy
                    self.ax.arrow(self.x, self.y, 
                                 vx_global * vel_scale, vy_global * vel_scale,
                                 head_width=0.06, head_length=0.08, 
                                 fc='blue', ec='blue', linewidth=2)
                    
                    # Hiển thị giá trị tốc độ và góc
                    vel_angle = np.degrees(np.arctan2(vy_global, vx_global))
                    self.ax.text(self.PLOT_X_MIN + 0.2, self.PLOT_Y_MIN + 0.2,
                                f"Velocity:\n{vel_mag:.2f} m/s\nDirection: {vel_angle:.1f}°\nOmega: {np.degrees(omega):.1f}°/s", 
                                fontsize=10, 
                                bbox=dict(facecolor='lightblue', alpha=0.9, boxstyle='round,pad=0.5',
                                         edgecolor='blue', linewidth=1))
            
            # Vẽ labels bánh xe
            wheel_angles = [0, 2*np.pi/3, 4*np.pi/3]  # 0°, 120°, 240°
            
            for i, angle in enumerate(wheel_angles):
                # Tính góc của bánh xe trong hệ toàn cục
                global_angle = self.theta + angle
                
                # Tính vị trí của bánh xe trên đường tròn thân robot
                wheel_x = self.x + self.robot_radius * np.cos(global_angle)
                wheel_y = self.y + self.robot_radius * np.sin(global_angle)
                
                # Đánh số bánh xe
                label_offset = 0.03
                label_x = wheel_x + label_offset * np.cos(global_angle)
                label_y = wheel_y + label_offset * np.sin(global_angle)
                self.ax.text(label_x, label_y, str(i+1), fontsize=8, 
                             ha='center', va='center', color='black', 
                             bbox=dict(facecolor='white', alpha=0.7, boxstyle='circle'))
            
            # Format plot
            self.ax.set_aspect('equal')
            self.ax.set_xlabel('X Position (m)')
            self.ax.set_ylabel('Y Position (m)')
            self.ax.set_title('Omnidirectional Robot Trajectory (from Encoders)')
            
            # Hiển thị vị trí và góc
            normalized_theta_deg = np.degrees(self.normalize_angle(self.theta))
            self.ax.text(self.PLOT_X_MAX - 1.0, self.PLOT_Y_MAX - 0.5,
                        f'Position: ({self.x:.2f}m, {self.y:.2f}m)\n'
                        f'Heading: {normalized_theta_deg:.1f}°', 
                        fontsize=10, fontweight='bold',
                        bbox=dict(facecolor='yellow', alpha=0.8, boxstyle='round,pad=0.5'))
            
            self.canvas.draw()
    
    def show(self):
        """Show the trajectory window"""
        if not self.is_active or not self.trajectory_window:
            self.initialize_plot()
        else:
            self.trajectory_window.deiconify()
            self.is_active = True
    
    def close(self):
        """Close the trajectory window"""
        if self.trajectory_window:
            self.trajectory_window.withdraw()
            #self.is_active = False

    def forward_kinematics(self, omega1, omega2, omega3, wheel_radius, robot_radius, theta):
        """Tính toán động học thuận cho robot ba bánh đa hướng"""
        H = np.array([
            [-2/3 * np.sin(theta), -2/3 * np.cos(theta + np.pi/6), 2/3 * np.sin(theta + np.pi/3)],
            [2/3 * np.cos(theta), -2/3 * np.sin(theta + np.pi/6), -2/3 * np.cos(theta + np.pi/3)],
            [1/(3*robot_radius), 1/(3*robot_radius), 1/(3*robot_radius)]
            ])

        omega_wheels = np.array([omega1, omega2, omega3]) * wheel_radius
        velocity_body = H @ omega_wheels
        vx_body, vy_body, omega = velocity_body
        
        return vx_body, vy_body, omega 

# Create a singleton instance
trajectory_visualizer = TrajectoryVisualizer()