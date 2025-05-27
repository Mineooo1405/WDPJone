import tkinter as tk
from tkinter import filedialog, ttk, messagebox
import time
from server import Server
from server_control import ControlGUI

class ServerGUI:
    def __init__(self, root):
        self.root = root
        self.server = Server(self)
        self.encoder_labels = []
        self.speed_entries = []
        self.pid_entries = []
        self.control_gui = ControlGUI(root)
        self.control_gui.set_server(self.server)
        
        self.setup_gui()

    def setup_gui(self):
        self.root.title("Omni Robot Server Control")
        self.root.geometry("850x900")
        
        # Configure styles
        style = ttk.Style()
        style.configure("TButton", padding=5, relief="flat", background="#4CAF50")
        style.configure("Red.TButton", background="#F44336", foreground="white")
        style.configure("Green.TButton", background="#4CAF50", foreground="white")
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill="both", expand=True)
        
        # Status bar at top
        status_frame = ttk.Frame(main_frame, relief="sunken", padding="2")
        status_frame.pack(fill="x", pady=(0, 10))
        
        ttk.Label(status_frame, text="Status:").pack(side="left")
        self.status_label = ttk.Label(status_frame, text="Disconnected")
        self.status_label.pack(side="left", padx=(5, 0))

        # Create notebook for tabs
        notebook = ttk.Notebook(main_frame)
        notebook.pack(fill="both", expand=True)
        
        # Create tabs
        firmware_tab = ttk.Frame(notebook, padding=10)
        control_tab = ttk.Frame(notebook, padding=10)
        settings_tab = ttk.Frame(notebook, padding=10)
        
        notebook.add(firmware_tab, text="Firmware Update")
        notebook.add(control_tab, text="Robot Control")
        notebook.add(settings_tab, text="Settings")
        
        # Firmware Tab
        firmware_frame = ttk.LabelFrame(firmware_tab, text="Firmware Management", padding=10)
        firmware_frame.pack(fill="x", pady=5)
        
        ttk.Button(firmware_frame, text="Start Firmware Server", 
                  command=self.server.start_firmware_server).grid(row=0, column=0, padx=5, pady=5)
        
        self.choose_file_button = ttk.Button(firmware_frame, text="Choose Firmware", 
                                       command=self.choose_file, state="disabled")
        self.choose_file_button.grid(row=0, column=1, padx=5, pady=5)
        
        self.send_firmware_button = ttk.Button(firmware_frame, text="Send Firmware", 
                                        command=self.server.send_firmware, state="disabled")
        self.send_firmware_button.grid(row=0, column=2, padx=5, pady=5)
        
        self.switch_upgrade_button = ttk.Button(firmware_frame, text="Switch to Upgrade Mode", 
                                         command=self.server.send_upgrade_command,
                                         state="disabled")  # Bắt đầu với trạng thái disabled
        self.switch_upgrade_button.grid(row=0, column=3, padx=5, pady=5)
        
        # Progress bar (hidden initially)
        self.progress_var = tk.DoubleVar()
        self.progress_frame = ttk.Frame(firmware_tab)
        ttk.Label(self.progress_frame, text="Upload progress:").pack(side="left")
        self.progress_bar = ttk.Progressbar(self.progress_frame, 
                                           variable=self.progress_var,
                                           maximum=100, length=400)
        self.progress_bar.pack(side="left", padx=5)
        
        # Control Tab
        control_frame = ttk.LabelFrame(control_tab, text="Robot Control", padding=10)
        control_frame.pack(fill="x", pady=5)
        
        ttk.Button(control_frame, text="Start Control Server", 
                  command=self.server.start_control_server).grid(row=0, column=0, padx=5, pady=5)
        
        ttk.Button(control_frame, text="Stop Server", 
                  command=self.server.stop_control_server).grid(row=0, column=1, padx=5, pady=5)
                  
        self.manual_control_button = ttk.Button(control_frame, text="Manual Control", 
                                         command=self.manual_control, state="normal")
        self.manual_control_button.grid(row=0, column=2, padx=5, pady=5)
        

        # Create a dedicated frame for trajectory buttons
        trajectory_frame = ttk.LabelFrame(control_tab, text="Trajectory Visualization", padding=10)
        trajectory_frame.pack(fill="x", pady=5)

        # Add the two separate buttons 
        ttk.Button(trajectory_frame, text="Show Server Trajectory", 
                command=self.server.show_trajectory_plot).grid(row=0, column=0, padx=20, pady=5)
                
        ttk.Button(trajectory_frame, text="Show Robot Trajectory", 
                command=self.server.show_robot_position_plot).grid(row=0, column=1, padx=20, pady=5)

        # Add explanatory labels
        ttk.Label(trajectory_frame, text="(Calculated from encoders)", 
                font=("", 8)).grid(row=1, column=0, padx=5)
        ttk.Label(trajectory_frame, text="(Reported by robot)", 
                font=("", 8)).grid(row=1, column=1, padx=5)
        
        # Tạo khung chứa cho Motor Control và BNO055
        control_container = ttk.Frame(control_tab)
        control_container.pack(fill="x", pady=5)

        # Motor control frame - đặt bên trái
        motor_frame = ttk.LabelFrame(control_container, text="Motor Control", padding=10)
        motor_frame.grid(row=0, column=0, padx=(0,5), pady=5, sticky="nsew")

        # BNO055 frame - đặt bên phải
        bno055_frame = ttk.LabelFrame(control_container, text="BNO055 Sensor", padding=10)
        bno055_frame.grid(row=0, column=1, padx=(5,0), pady=5, sticky="nsew")

        # Đặt trọng số cho cột để chúng mở rộng đồng đều
        control_container.columnconfigure(0, weight=1)
        control_container.columnconfigure(1, weight=1)

        # Tạo chỉ báo trạng thái hiệu chuẩn trong khung BNO055
        calib_frame = ttk.Frame(bno055_frame)
        calib_frame.pack(fill="x", pady=5)

        ttk.Label(calib_frame, text="Calibration Status:").pack(side="left", padx=5)
        self.calib_indicator = tk.Label(calib_frame, text="Not Calibrated", 
                                     bg="#F44336", fg="white", 
                                     width=15, relief="flat")
        self.calib_indicator.pack(side="left", padx=10)

        movement_frame = ttk.Frame(bno055_frame)
        movement_frame.pack(fill="x", pady=5)

        ttk.Label(movement_frame, text="Movement Status:").pack(side="left", padx=5)
        self.movement_indicator = tk.Label(movement_frame, text="Stationary", 
                                        bg="#F44336", fg="white", 
                                        width=15, relief="flat")
        self.movement_indicator.pack(side="left", padx=10)

        # Tạo frame hiển thị vị trí
        position_frame = ttk.LabelFrame(bno055_frame, text="Position & Velocity", padding=5)
        position_frame.pack(fill="x", pady=5)

        # Vị trí
        pos_frame = ttk.Frame(position_frame)
        pos_frame.pack(fill="x", pady=2)
        ttk.Label(pos_frame, text="Position (m):").pack(side="left", padx=5)
        ttk.Label(pos_frame, text="X:").pack(side="left", padx=2)
        self.pos_x_label = ttk.Label(pos_frame, text="0.0000", width=8)
        self.pos_x_label.pack(side="left")
        ttk.Label(pos_frame, text="Y:").pack(side="left", padx=2)
        self.pos_y_label = ttk.Label(pos_frame, text="0.0000", width=8)
        self.pos_y_label.pack(side="left")

        # Vận tốc
        vel_frame = ttk.Frame(position_frame)
        vel_frame.pack(fill="x", pady=2)
        ttk.Label(vel_frame, text="Velocity (m/s):").pack(side="left", padx=5)
        ttk.Label(vel_frame, text="X:").pack(side="left", padx=2)
        self.vel_x_label = ttk.Label(vel_frame, text="0.0000", width=8)
        self.vel_x_label.pack(side="left")
        ttk.Label(vel_frame, text="Y:").pack(side="left", padx=2)
        self.vel_y_label = ttk.Label(vel_frame, text="0.0000", width=8)
        self.vel_y_label.pack(side="left")
        # Thêm nội dung khác cho BNO055 frame (hiện tại để trống)
        # ttk.Label(bno055_frame, text="IMU data will be shown here").pack(pady=20)

        # Các Labels cho motor_frame như cũ
        ttk.Label(motor_frame, text="Motor").grid(row=0, column=0, padx=5, pady=5)
        ttk.Label(motor_frame, text="Speed").grid(row=0, column=1, padx=5, pady=5)
        ttk.Label(motor_frame, text="Action").grid(row=0, column=2, padx=5, pady=5)
        ttk.Label(motor_frame, text="Current RPM").grid(row=0, column=3, padx=5, pady=5)

        self.speed_entries = []
        
        for i in range(3):
            ttk.Label(motor_frame, text=f"Motor {i+1}:").grid(row=i+1, column=0, padx=5, pady=5)
            
            speed_entry = ttk.Entry(motor_frame, width=7)
            speed_entry.insert(0, "0")
            speed_entry.grid(row=i+1, column=1, padx=5, pady=5)
            self.speed_entries.append(speed_entry)
            
            set_button = ttk.Button(motor_frame, text="Set", 
                                   command=lambda idx=i, e=speed_entry: self.set_motor_speed(idx, e))
            set_button.grid(row=i+1, column=2, padx=5, pady=5)
            
            encoder_label = ttk.Label(motor_frame, text=f"RPM: 0")
            self.encoder_labels.append(encoder_label)
            encoder_label.grid(row=i+1, column=3, padx=5, pady=5)
        

        pid_container = ttk.Frame(control_tab)
        pid_container.pack(fill="x", pady=5)

        # PID control frame - place on the left
        pid_frame = ttk.LabelFrame(pid_container, text="PID Control", padding=10)
        pid_frame.grid(row=0, column=0, padx=(0, 5), pady=5, sticky="nsew")

        # Robot operation frame - place on the right
        robot_op_frame = ttk.LabelFrame(pid_container, text="Robot Operation", padding=10)
        robot_op_frame.grid(row=0, column=1, padx=(5, 0), pady=5, sticky="nsew")

        # Configure weights to distribute space
        pid_container.columnconfigure(0, weight=3)  # PID frame gets more space
        pid_container.columnconfigure(1, weight=1)  # Robot operation frame gets less space
        
        tk.Button(robot_op_frame, text="Start Robot   ", 
                command=self.server.send_start_robot, 
                bg="#2196F3", fg="white",
                padx=5, pady=2).grid(row=0, column=0, padx=10, pady=5)

        # Add the Start Position button to the new frame
        tk.Button(robot_op_frame, text="Start Position", 
                command=self.server.send_start_position_command,
                bg="#2196F3", fg="white",
                padx=5, pady=2).grid(row=1, column=0, padx=10, pady=5)

        ttk.Button(pid_frame, text="Show RPM Plot", 
                command=self.server.show_rpm_plot).grid(row=0, column=0, padx=5, pady=5)
                
        ttk.Button(pid_frame, text="Save PID Config", 
                command=self.server.save_pid_config).grid(row=0, column=2, padx=5, pady=5)
                
        ttk.Button(pid_frame, text="Load PID Config", 
                command=self.server.load_pid_config).grid(row=0, column=3, padx=5, pady=5)
        
        # Labels for PID columns
        ttk.Label(pid_frame, text="Motor").grid(row=1, column=0, padx=5, pady=5)
        ttk.Label(pid_frame, text="Kp").grid(row=1, column=1, padx=5, pady=5)
        ttk.Label(pid_frame, text="Ki").grid(row=1, column=2, padx=5, pady=5)
        ttk.Label(pid_frame, text="Kd").grid(row=1, column=3, padx=5, pady=5)
        ttk.Label(pid_frame, text="Action").grid(row=1, column=4, padx=5, pady=5)
        
        self.pid_entries = []
        
        for i in range(3):
            ttk.Label(pid_frame, text=f"Motor {i+1}:").grid(row=i+2, column=0, padx=5, pady=5)
            
            p_entry = ttk.Entry(pid_frame, width=7)
            i_entry = ttk.Entry(pid_frame, width=7)
            d_entry = ttk.Entry(pid_frame, width=7)
            p_entry.insert(0, "0")
            i_entry.insert(0, "0")
            d_entry.insert(0, "0")
            p_entry.grid(row=i+2, column=1, padx=5, pady=5)
            i_entry.grid(row=i+2, column=2, padx=5, pady=5)
            d_entry.grid(row=i+2, column=3, padx=5, pady=5)
            self.pid_entries.append((p_entry, i_entry, d_entry))
            
            set_button = ttk.Button(pid_frame, text="Set", 
                                  command=lambda idx=i: self.set_pid(idx))
            set_button.grid(row=i+2, column=4, padx=5, pady=5)
            
        # Settings Tab
        log_frame = ttk.LabelFrame(settings_tab, text="Logging Settings", padding=10)
        log_frame.pack(fill="x", pady=5)
        
        self.log_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(log_frame, text="Enable data logging", 
                       variable=self.log_var, 
                       command=self.toggle_logging).pack(anchor="w")
                       
        # Monitor Section (put at bottom of the window)
        monitor_frame = ttk.LabelFrame(main_frame, text="Status Monitor", padding=5)
        monitor_frame.pack(fill="both", expand=True, pady=10)
        
        # Create a frame for the monitor text and scrollbar
        text_frame = ttk.Frame(monitor_frame)
        text_frame.pack(fill="both", expand=True)
        
        self.monitor_text = tk.Text(text_frame, height=10, width=80, wrap="word")
        self.monitor_text.pack(side="left", fill="both", expand=True)
        
        scrollbar = ttk.Scrollbar(text_frame, command=self.monitor_text.yview)
        scrollbar.pack(side="right", fill="y")
        self.monitor_text.config(yscrollcommand=scrollbar.set)
        
        # Button to clear monitor
        ttk.Button(monitor_frame, text="Clear Monitor", 
                  command=self.clear_monitor).pack(anchor="e", pady=(5, 0))
                  
        # Initialize with a welcome message
        self.update_monitor("Server interface initialized. Ready to start.")
        self.server.set_position_visualizer_callback()

    def toggle_logging(self):
        self.server.log_data = self.log_var.get()
        self.update_monitor(f"Data logging {'enabled' if self.server.log_data else 'disabled'}")

    def clear_monitor(self):
        self.monitor_text.delete(1.0, tk.END)
        self.update_monitor("Monitor cleared")

    def setup_progress_bar(self, total_size):
        """Setup and show progress bar for file upload"""
        self.progress_frame.pack(fill="x", pady=10)
        self.progress_var.set(0)
        self.progress_bar.configure(maximum=total_size)

    def update_progress(self, value):
        """Update progress bar value"""
        self.progress_var.set(value)
        self.root.update_idletasks()

    def hide_progress_bar(self):
        """Hide progress bar when not needed"""
        self.progress_frame.pack_forget()

    def update_status(self, status):
        """Update connection status"""
        self.status_label.config(text=status)

    def set_pid(self, motor_index):
        try:
            p = float(self.pid_entries[motor_index][0].get())
            i = float(self.pid_entries[motor_index][1].get())
            d = float(self.pid_entries[motor_index][2].get())
            self.server.set_pid_values(motor_index, p, i, d)
        except ValueError:
            self.update_monitor(f"Invalid PID values for Motor {motor_index + 1}.")
            messagebox.showerror("Error", f"Invalid PID values for Motor {motor_index + 1}")

    def update_pid_entries(self, motor_index, p, i, d):
        """Update PID entry fields with loaded values"""
        self.pid_entries[motor_index][0].delete(0, tk.END)
        self.pid_entries[motor_index][0].insert(0, str(p))
        self.pid_entries[motor_index][1].delete(0, tk.END)
        self.pid_entries[motor_index][1].insert(0, str(i))
        self.pid_entries[motor_index][2].delete(0, tk.END)
        self.pid_entries[motor_index][2].insert(0, str(d))

    def choose_file(self):
        """Open file dialog to select firmware file"""
        file_path = filedialog.askopenfilename(
            title="Select Firmware File",
            filetypes=[("Binary files", "*.bin"), ("All files", "*.*")]
        )
        if file_path:
            self.server.file_path = file_path
            self.update_monitor(f"Selected firmware file: {file_path}")
            self.send_firmware_button.config(state="normal")

    def enable_file_selection(self):
        """Enable firmware file selection buttons"""
        self.choose_file_button.config(state="normal")

    def enable_control_buttons(self):
        """Enable control buttons when connected"""
        self.manual_control_button.config(state="normal")
        # Bật nút Switch Upgrade Mode khi client kết nối vào control server
        self.switch_upgrade_button.config(state="normal")

    def disable_buttons(self):
        """Disable all buttons when disconnected"""
        self.choose_file_button.config(state="disabled")
        self.send_firmware_button.config(state="disabled")
        self.switch_upgrade_button.config(state="disabled")

    def disable_control_buttons(self):
        """Disable control buttons when disconnected"""
        # The manual control button can remain enabled since it opens a separate window
        # Tắt nút Switch Upgrade Mode khi client ngắt kết nối
        self.switch_upgrade_button.config(state="disabled")
        self.calib_indicator.config(bg="#F44336", text="Not Calibrated")
    
    def update_encoders(self, encoders):
        """Update the encoder labels with new values"""
        for i, value in enumerate(encoders):
            self.encoder_labels[i].config(text=f"RPM: {value:.1f}")

    def update_monitor(self, message):
        """Add message to the monitor with timestamp"""
        timestamp = time.strftime("%H:%M:%S")
        self.monitor_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.monitor_text.see(tk.END)  # Scroll to the end

    def update_speed_entry(self, motor_index, speed):
        """Update the motor speed entry field"""
        self.speed_entries[motor_index].delete(0, tk.END)
        self.speed_entries[motor_index].insert(0, str(speed))

    def set_motor_speed(self, motor_index, entry):
        """Set motor speed from UI entry field"""
        try:
            speed = float(entry.get())
            self.server.set_speed(motor_index, speed)
        except ValueError:
            self.update_monitor(f"Invalid speed value for Motor {motor_index + 1}")
            messagebox.showerror("Error", f"Invalid speed value for Motor {motor_index + 1}")

    def manual_control(self):
        """Open manual control window for robot navigation"""
        self.control_gui.run()
        self.update_monitor("Manual control window opened")

    def update_calibration_status(self, is_calibrated):
        """Cập nhật trạng thái hiệu chuẩn của BNO055"""
        if is_calibrated:
            self.calib_indicator.config(bg="#4CAF50", text="Calibrated")
        else:
            self.calib_indicator.config(bg="#F44336", text="Not Calibrated")
    
    def update_movement_status(self, is_moving):
        """Cập nhật trạng thái di chuyển của robot"""
        if is_moving:
            self.movement_indicator.config(bg="#4CAF50", text="Moving")
        else:
            self.movement_indicator.config(bg="#F44336", text="Stationary")
    def update_position_velocity(self, pos_x, pos_y, vel_x, vel_y):
        """Cập nhật thông tin vị trí và vận tốc từ BNO055"""
        self.pos_x_label.config(text=f"{pos_x:.4f}")
        self.pos_y_label.config(text=f"{pos_y:.4f}")
        self.vel_x_label.config(text=f"{vel_x:.4f}")
        self.vel_y_label.config(text=f"{vel_y:.4f}")