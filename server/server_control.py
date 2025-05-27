import tkinter as tk
from tkinter import ttk, messagebox
import math
import time

ANGLE_ROTATION = 2 * math.pi  # (Rad/s)
TIME_ROTATION = 5  # 2s
RPM = 60
M_PER_ROUND = 0.06 * math.pi
 
T = 5 #s - Time for test 1m
class ControlGUI:
    def __init__(self, root):
        self.root = root
        self.server = None
        self.orientation = 0  # Hướng của robot (góc quay)
        # self.max_speed = RPM * M_PER_ROUND / 60
        self.max_speed = RPM * M_PER_ROUND / 60
        self.control_window = None
        self.recording = False
        self.trajectory = []
        self.running_trajectory = False
        self.trajectory_index = 0
        self.path_points = []  # Lưu lại quỹ đạo di chuyển
        
        # Thêm biến để kiểm soát nhấn phím một lần
        self.key_states = {}
        self.key_pressed = False

    def run(self):
        if self.control_window is not None and self.control_window.winfo_exists():
            self.control_window.focus_force()
            return
            
        self.control_window = tk.Toplevel(self.root)
        self.control_window.title("Robot Control Interface")
        self.control_window.geometry("400x300")
        self.control_window.resizable(True, True)
        
        # Đảm bảo cửa sổ điều khiển được tách ra khỏi cửa sổ chính
        self.control_window.transient(self.root)
        
        # Vùng hiển thị thông tin
        info_frame = ttk.LabelFrame(self.control_window, text="Robot Information", padding=10)
        info_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Hiển thị hướng dẫn sử dụng
        controls_text = """
        Control Keys:
        W: Move Forward
        S: Move Backward
        A: Move Left
        D: Move Right
        Q: Rotate Left
        E: Rotate Right
        R: Toggle Recording
        T: Run Trajectory
        C: Stop Movement
        ESC: Close Window
        """
        
        ttk.Label(info_frame, text=controls_text, justify="left").pack(anchor="w")
        
        # Hiển thị trạng thái
        self.status_frame = ttk.Frame(self.control_window, padding=5)
        self.status_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(self.status_frame, text="Status:").pack(side="left")
        self.status_label = ttk.Label(self.status_frame, text="Ready")
        self.status_label.pack(side="left", padx=5)
        
        # Hiển thị nút điều khiển
        control_buttons = ttk.Frame(self.control_window, padding=5)
        control_buttons.pack(fill="x", padx=10, pady=5)
        
        ttk.Button(control_buttons, text="Record Path", command=self.toggle_recording).pack(side="left", padx=5)
        ttk.Button(control_buttons, text="Run Path", command=self.run_trajectory).pack(side="left", padx=5)
        ttk.Button(control_buttons, text="Stop", command=self.stop_robot).pack(side="left", padx=5)
        ttk.Button(control_buttons, text="Close", command=self.close_control).pack(side="right", padx=5)
        
        # Đăng ký các sự kiện bàn phím
        self.control_window.bind("<Key>", self.on_key_press)
        self.control_window.bind("<KeyRelease>", self.on_key_release)
        
        # Đăng ký sự kiện đóng cửa sổ
        self.control_window.protocol("WM_DELETE_WINDOW", self.close_control)
        
        # Đặt focus để nhận sự kiện bàn phím
        self.control_window.focus_set()
        
    def on_key_press(self, event):
        key = event.keysym.lower()
        
        # Nếu phím đã được xử lý, bỏ qua
        if key in self.key_states and self.key_states[key]:
            return
            
        # Đánh dấu phím đã được nhấn
        self.key_states[key] = True
        
        if key == "escape":
            self.close_control()
            return
            
        # Xử lý các lệnh di chuyển
        dot_x, dot_y, dot_theta = 0, 0, 0
        
        if key == "w":  # Lên
            dot_y += self.max_speed
            self.update_status("Moving Forward")
        elif key == "s":  # Xuống
            dot_y -= self.max_speed
            self.update_status("Moving Backward")
        elif key == "a":  # Trái
            dot_x -= self.max_speed
            self.update_status("Moving Left")
        elif key == "d":  # Phải
            dot_x += self.max_speed
            self.update_status("Moving Right")
        elif key == "q":  # Quay trái
            dot_theta = -(ANGLE_ROTATION / TIME_ROTATION)
            self.orientation += dot_theta * 0.1
            self.update_status("Rotating Left")
        elif key == "e":  # Quay phải
            dot_theta = ANGLE_ROTATION / TIME_ROTATION
            self.orientation += dot_theta * 0.1
            self.update_status("Rotating Right")
        elif key == "r":  # Ghi quỹ đạo
            self.toggle_recording()
            return
        elif key == "t":  # Chạy quỹ đạo
            self.run_trajectory()
            return
        elif key == "c":  # Dừng
            self.stop_robot()
            return
            
        # Gửi lệnh điều khiển nếu có
        if dot_x != 0 or dot_y != 0 or dot_theta != 0:
            self.send_command(dot_x, dot_y, dot_theta)
            
            # Nếu đang ghi quỹ đạo, lưu lại
            if self.recording:
                self.trajectory.append((dot_x, dot_y, dot_theta))
    
    def on_key_release(self, event):
        key = event.keysym.lower()
        self.key_states[key] = False
    
    def update_status(self, message):
        if self.status_label:
            self.status_label.config(text=message)
    
    def close_control(self):
        if self.control_window:
            self.control_window.destroy()
            self.control_window = None
    
    def toggle_recording(self):
        self.recording = not self.recording
        if self.recording:
            self.update_status("Recording Path")
            self.trajectory = []
        else:
            self.update_status("Path Recorded")
            self.save_trajectory()
    
    def save_trajectory(self):
        if not self.trajectory:
            return
        
        try:
            with open("trajectory.txt", "w") as f:
                for cmd in self.trajectory:
                    f.write(f"{cmd[0]},{cmd[1]},{cmd[2]}\n")
            print("Trajectory saved to trajectory.txt")
        except Exception as e:
            print(f"Error saving trajectory: {e}")
    
    def load_trajectory(self):
        try:
            self.trajectory = []
            with open("trajectory.txt", "r") as f:
                for line in f:
                    values = line.strip().split(",")
                    if len(values) == 3:
                        self.trajectory.append((float(values[0]), float(values[1]), float(values[2])))
            print(f"Loaded {len(self.trajectory)} commands from trajectory")
            return True
        except Exception as e:
            print(f"Error loading trajectory: {e}")
            return False
    
    def run_trajectory(self):
        if self.running_trajectory:
            return
            
        if self.load_trajectory():
            self.running_trajectory = True
            self.trajectory_index = 0
            self.update_status("Running Path")
            self.execute_trajectory()
        else:
            self.update_status("No Path Available")
    
    def execute_trajectory(self):
        if not self.running_trajectory or self.trajectory_index >= len(self.trajectory):
            self.running_trajectory = False
            self.update_status("Path Completed")
            return
            
        if self.control_window is None or not self.control_window.winfo_exists():
            self.running_trajectory = False
            return
            
        cmd = self.trajectory[self.trajectory_index]
        dot_x, dot_y, dot_theta = cmd
        
        self.send_command(dot_x, dot_y, dot_theta)
        self.trajectory_index += 1
        
        # Lập lịch chạy lệnh tiếp theo sau một khoảng thời gian
        delay = 200  # ms
        self.control_window.after(delay, self.execute_trajectory)
    
    def stop_robot(self):
        self.send_command(0, 0, 0)
        self.running_trajectory = False
        self.update_status("Stopped")
    
    def send_command(self, dot_x, dot_y, dot_theta):
        if self.server:
            self.server.send_command(dot_x, dot_y, dot_theta)
        else:
            print("Server not connected!")
    
    def set_server(self, server):
        self.server = server