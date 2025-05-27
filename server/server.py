import os
import json
import socket
import threading
import re
import time
import math
from tkinter import messagebox

from server_rpm_plot import update_rpm_plot
from server_rpm_plot import rpm_plotter
from server_position import robot_position_visualizer
from server_trajection import trajectory_visualizer

from ekf_position import ekf  # Thêm dòng này để import module EKF
import numpy as np

RPM = 60
M_PER_ROUND = 0.06 * math.pi

class Server:
    def __init__(self, gui):
        self.gui = gui
        
        self.control_active = False
        self.firmware_active = False
        self.file_path = None
        self.speed = [0, 0, 0]  # Speed for three motors
        self.encoders = [0, 0, 0]  # Encoder values for three motors
        self.bno055_heading = 0.0
        self.pid_values = [[0.0, 0.0, 0.0] for _ in range(3)]  # PID values as floats
        self.bno055_calibrated = False

        self.server_socket = None
        self.client_socket = None
        self.sending_firmware = False
        self.client_connected = False
        self.connection_status = "Disconnected"
        self.log_data = True  # Enable data logging by default
                # Khởi tạo dictionary để quản lý file log
        self.log_files = {}
        self.start_times = {}
        self.supported_types = ["encoder", "bno055", "log", "position"]
        
        # Tạo file log mới cho loại dữ liệu
    def setup_log_file(self, data_type):
        if not self.log_data:
            return
            
        # If this is our first log file, initialize the common start time
        if not hasattr(self, 'common_start_time') or not self.log_files:
            self.common_start_time = time.time()
            self.gui.update_monitor(f"Starting new logging session at {time.strftime('%H:%M:%S')}")
        
        # Skip if this type is already being logged
        if data_type in self.log_files:
            return
                
        log_dir = "logs"
        os.makedirs(log_dir, exist_ok=True)
        session_id = time.strftime('%Y%m%d_%H%M%S', time.localtime(self.common_start_time))
        log_filename = f"{log_dir}/{data_type}_log_{session_id}.csv"
        
        self.log_files[data_type] = open(log_filename, "w", newline='')
        
        # Sử dụng csv writer cho tất cả các loại dữ liệu
        import csv
        self.log_writers = getattr(self, 'log_writers', {})
        self.log_writers[data_type] = csv.writer(self.log_files[data_type])
        
        # Create header based on data type
        if data_type == "encoder":
            self.log_writers[data_type].writerow(["Time", "RPM1", "RPM2", "RPM3"])
        elif data_type == "bno055":
            # Thứ tự mới: Time, PosX, PosY, VelX, VelY, Heading, Pitch, Roll, AccelX, AccelY, AccelZ, IsMoving
            self.log_writers[data_type].writerow([
                "Time", "PosX", "PosY", "VelX", "VelY",
                "Heading", "Pitch", "Roll", 
                "AccelX", "AccelY", "AccelZ",
                "IsMoving"
            ])
        elif data_type == "log":
            self.log_writers[data_type].writerow(["Time", "Message"])
        elif data_type == "position":
            self.log_writers[data_type].writerow([
                "Time", 
                "Raw_X", "Raw_Y", "Raw_Theta", 
                "Filtered_X", "Filtered_Y", "Filtered_Theta",
                "Raw_VelX", "Raw_VelY", 
                "Filtered_VelX", "Filtered_VelY",
                "Vel_Control_X", "Vel_Control_Y"
            ])
            
        self.gui.update_monitor(f"Started logging {data_type} data to {log_filename}")
    
    # Đóng tất cả file log
    def close_all_logs(self):
        for log_type, log_file in self.log_files.items():
            log_file.close()
            self.gui.update_monitor(f"{log_type.capitalize()} data log closed")
        self.log_files = {}
        self.start_times = {}

    def start_firmware_server(self):
        if self.firmware_active:
            self.gui.update_monitor("Firmware server already running.")
            return
        
        self.firmware_active = True
        threading.Thread(target=self.firmware_server_thread, daemon=True).start()

    def firmware_server_thread(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(("0.0.0.0", 12345))
            self.server_socket.listen(1)
            self.gui.update_monitor("Firmware server started, waiting for client...")
            self.gui.update_status("Firmware server listening")
            
            self.client_socket, addr = self.server_socket.accept()
            self.client_connected = True
            self.connection_status = f"Connected to {addr[0]}:{addr[1]}"
            self.gui.update_monitor(f"Client connected for firmware: {addr}")
            self.gui.update_status(self.connection_status)
            self.gui.enable_file_selection()

            threading.Thread(target=self.receive_upgrade, args=(self.client_socket,), daemon=True).start()
            
        except Exception as e:
            self.firmware_active = False
            self.gui.update_monitor(f"Error starting firmware server: {e}")
            self.gui.update_status("Server error")

    def send_firmware(self):
        if not self.client_connected:
            self.gui.update_monitor("No client connected.")
            messagebox.showerror("Error", "No client connected")
            return
            
        if not self.file_path:
            self.gui.update_monitor("No firmware file selected.")
            messagebox.showerror("Error", "Please select a firmware file first")
            return
            
        try:
            self.sending_firmware = True
            self.gui.update_status("Sending firmware...")
            file_size = 0
            sent_bytes = 0
            
            with open(self.file_path, "rb") as f:
                # Get file size
                f.seek(0, 2)
                file_size = f.tell()
                f.seek(0)
                
                # Send in chunks
                chunk_size = 1024
                self.gui.setup_progress_bar(file_size)
                
                while True:
                    chunk = f.read(chunk_size)
                    if not chunk:
                        break
                    self.client_socket.sendall(chunk)
                    sent_bytes += len(chunk)
                    self.gui.update_progress(sent_bytes)
                    
            self.gui.update_monitor(f"Firmware sent successfully: {sent_bytes} bytes")
            messagebox.showinfo("Success", "Firmware sent successfully")
        except Exception as e:
            self.gui.update_monitor(f"Error sending firmware: {e}")
            messagebox.showerror("Error", f"Failed to send firmware: {e}")
        finally: 
            self.sending_firmware = False
            self.gui.hide_progress_bar()
            self.gui.update_status(self.connection_status)
            self.client_socket.shutdown(socket.SHUT_WR)
    
    def receive_upgrade(self, sock):
        try:
            while True:
                if self.sending_firmware:
                    time.sleep(0.1)
                    continue

                data = sock.recv(1024).decode()
                if not data:
                    self.gui.update_monitor("Firmware client disconnected")
                    break
                self.gui.update_monitor(f"Upgrade status: {data}")
        except Exception as e:
            self.gui.update_monitor(f"Error receiving upgrade status: {e}")
        finally:
            # Đóng socket khi client ngắt kết nối
            try:
                if sock:
                    sock.close()
            except:
                pass
            if self.client_socket == sock:
                self.client_socket = None
            self.client_connected = False
            self.connection_status = "Disconnected"
            self.gui.update_status("Disconnected")
            self.gui.disable_buttons()

    def send_upgrade_command(self):
        if not self.client_connected:
            messagebox.showerror("Error", "No client connected")
            return
            
        try:
            self.client_socket.sendall(b"Upgrade")
            self.gui.update_monitor("Sent 'Upgrade' command to the client.")
        except Exception as e:
            self.gui.update_monitor(f"Failed to send 'Upgrade' command: {e}")

    def start_control_server(self):
        if self.control_active:
            self.gui.update_monitor("Control server already running.")
            return
            
        self.control_active = True
        threading.Thread(target=self.control_server_thread, daemon=True).start()

    def stop_control_server(self):
        if not self.control_active:
            return
            
        self.control_active = False
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
            
        self.gui.update_monitor("Control server stopped")
        self.gui.update_status("Server stopped")

    def control_server_thread(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(("0.0.0.0", 12346))
            self.server_socket.listen(1)
            self.control_active = True
            self.gui.update_status("Waiting for connection...")
            
            while self.control_active:
                try:
                    self.client_socket, addr = self.server_socket.accept()
                    self.client_socket.settimeout(0.5)
                    self.client_connected = True
                    self.gui.update_status(f"Connected to {addr[0]}")
                    self.gui.enable_control_buttons()
                    
                    # Gọi hàm mới thay vì hàm cũ
                    self.receive_client_data(self.client_socket)
                    
                except socket.timeout:
                    continue
                except Exception as e:
                    self.gui.update_monitor(f"Server error: {e}")
                    break
                    
        except Exception as e:
            self.gui.update_monitor(f"Server init error: {e}")
        finally:
            if self.server_socket:
                self.server_socket.close()
            self.control_active = False
            self.gui.update_status("Server stopped")
            self.gui.disable_buttons()

        # Hàm xử lý dữ liệu encoder
    def process_encoder_data(self, encoder_data):
        if not isinstance(encoder_data, list) or len(encoder_data) < 3:
            self.gui.update_monitor(f"Invalid encoder data format: {encoder_data}")
            return
            
        # Cập nhật giá trị encoder
        self.encoders[0] = float(encoder_data[0])
        self.encoders[1] = float(encoder_data[1])
        self.encoders[2] = float(encoder_data[2])
        
        # Cập nhật UI
        self.gui.update_encoders(self.encoders)
        update_rpm_plot(self.encoders)
        
        trajectory_visualizer.update_trajectory(self.encoders, self.bno055_heading)

        # Ghi log nếu được bật
        self.setup_log_file("encoder")
        if self.log_data and "encoder" in self.log_files:
            timestamp = time.time() - self.common_start_time
            self.log_writers["encoder"].writerow([f"{timestamp:.3f}", str(self.encoders[0]), 
                                                 str(self.encoders[1]), str(self.encoders[2])])
            self.log_files["encoder"].flush()

    def process_bno055_data(self, bno_data):
        """
        Xử lý dữ liệu BNO055 với định dạng mới:
        {
        "id": "device_id",
        "type": "bno055",
        "data": {
            "position": [x, y],
            "velocity": [vx, vy],
            "euler": [heading, pitch, roll],
            "accel": [ax, ay, az],
            "is_moving": true/false
        }
        }
        """
        try:
            # Kiểm tra xem bno_data có phải dict không
            if not isinstance(bno_data, dict):
                self.gui.update_monitor(f"Invalid BNO055 data format, expected dictionary")
                return

            # Lấy timestamp
            timestamp = time.time() - self.common_start_time
            
            # Trích xuất position (default = [0,0])
            pos_x, pos_y = 0.0, 0.0
            position = bno_data.get("position", None)
            if position and len(position) >= 2:
                pos_x, pos_y = position[0], position[1]

            # Trích xuất velocity (default = [0,0])
            vel_x, vel_y = 0.0, 0.0
            velocity = bno_data.get("velocity", None)
            if velocity and len(velocity) >= 2:
                vel_x, vel_y = velocity[0], velocity[1]
                
            # Trích xuất euler (default = [0,0,0])
            heading, pitch, roll = 0.0, 0.0, 0.0
            euler_array = bno_data.get("euler", None)
            if euler_array and len(euler_array) >= 3:
                heading, pitch, roll = euler_array[0], euler_array[1], euler_array[2]
                self.bno055_heading = heading  # Cập nhật heading toàn cục

            # Trích xuất accel (default = [0,0,0])
            accel_x, accel_y, accel_z = 0.0, 0.0, 0.0
            accel_array = bno_data.get("accel", None)
            if accel_array and len(accel_array) >= 3:
                accel_x, accel_y, accel_z = accel_array[0], accel_array[1], accel_array[2]
                
            # Trích xuất is_moving (default = False)
            is_moving = bno_data.get("is_moving", False)
            
            # Cập nhật UI
            self.gui.update_movement_status(is_moving)
            
            self.gui.update_position_velocity(pos_x, pos_y, vel_x, vel_y)
            # Ghi log theo thứ tự mới: Time, PosX, PosY, VelX, VelY, Heading, Pitch, Roll, AccelX, AccelY, AccelZ, IsMoving
            self.setup_log_file("bno055")
            if self.log_data and "bno055" in self.log_files:
                self.log_writers["bno055"].writerow([
                    f"{timestamp:.3f}",
                    f"{pos_x:.4f}", f"{pos_y:.4f}",
                    f"{vel_x:.4f}", f"{vel_y:.4f}",
                    f"{heading:.2f}", f"{pitch:.2f}", f"{roll:.2f}",
                    f"{accel_x:.2f}", f"{accel_y:.2f}", f"{accel_z:.2f}",
                    "1" if is_moving else "0"
                ])
                self.log_files["bno055"].flush()
                        
        except Exception as e:
            self.gui.update_monitor(f"Error processing BNO055 data: {e}")
            import traceback
            traceback.print_exc()

    def process_position_data(self, position_data):
        """
        Process position data sent directly from the robot
        Using EKF with measurement vector [x, y, theta, vx, vy]
        """
        try:
            # Extract position and velocity from the message
            if isinstance(position_data, dict):
                if "position" in position_data and len(position_data["position"]) >= 3:
                    x = float(position_data["position"][0])
                    y = float(position_data["position"][1])
                    theta = float(position_data["position"][2]) 
                else:
                    self.gui.update_monitor(f"Invalid position data format: {position_data}")
                    return
                    
                # Extract velocity if available
                vel_x, vel_y = 0.0, 0.0
                if "velocity" in position_data and len(position_data["velocity"]) >= 2:
                    vel_x = float(position_data["velocity"][0])
                    vel_y = float(position_data["velocity"][1])
                
                # Extract angular velocity and acceleration if available 
                omega = 0.0
                accel_x, accel_y = 0.0, 0.0
                if "angular_velocity" in position_data:
                    omega = float(position_data["angular_velocity"])
                if "acceleration" in position_data and len(position_data["acceleration"]) >= 2:
                    accel_x = float(position_data["acceleration"][0])
                    accel_y = float(position_data["acceleration"][1])
                
                # Extract velocity control if available (for logging)
                vel_control_x, vel_control_y = 0.0, 0.0
                if "vel_control" in position_data and len(position_data["vel_control"]) >= 2:
                    vel_control_x = float(position_data["vel_control"][0])
                    vel_control_y = float(position_data["vel_control"][1])
            else:
                self.gui.update_monitor(f"Invalid message format: {position_data}")
                return
            
            # Current time for EKF time step calculation
            current_time = time.time()
            
            # Initialize EKF if not yet initialized
            if not ekf.initialized:
                ekf.initialize(x, y, theta)
                ekf.last_time = current_time
                
                # First reading - use raw data
                filtered_x, filtered_y, filtered_theta = x, y, theta
                filtered_vx, filtered_vy = vel_x, vel_y
            else:
                # Calculate time delta
                dt = current_time - ekf.last_time
                ekf.last_time = current_time
                
                # Create control vector [omega, ax, ay]
                control = [accel_x, accel_y]
                
                # Prediction step with control input
                ekf.predict(control)
                
                # Update with full measurement [x, y, theta, vx, vy]
                # ekf.update(x, y, theta, vel_x, vel_y)
                ekf.update(theta, vel_x, vel_y)
                # Get filtered state
                state = ekf.get_state()
                filtered_x = state['x']
                filtered_y = state['y'] 
                filtered_theta = state['theta']
                filtered_vx = state['v_x']
                filtered_vy = state['v_y']
                
            # Convert theta back to degrees for visualization
            filtered_theta_deg = filtered_theta * 180.0 / np.pi
                
            # Update robot position visualizer with filtered data
            robot_position_visualizer.update_robot_position(x, y, theta)
            
            # Log both raw and filtered data
            self.setup_log_file("position")
            if self.log_data and "position" in self.log_files:
                timestamp = time.time() - self.common_start_time
                self.log_writers["position"].writerow([
                    f"{timestamp:.3f}", 
                    f"{x:.4f}", f"{y:.4f}", f"{theta*180/np.pi:.4f}",
                    f"{filtered_x:.4f}", f"{filtered_y:.4f}", f"{filtered_theta_deg:.4f}",
                    f"{vel_x:.4f}", f"{vel_y:.4f}", 
                    f"{filtered_vx:.4f}", f"{filtered_vy:.4f}",
                    f"{vel_control_x:.4f}", f"{vel_control_y:.4f}"
                ])
                self.log_files["position"].flush()
                
        except Exception as e:
            self.gui.update_monitor(f"Error processing position data: {e}")
            import traceback
            traceback.print_exc()

    # Hàm xử lý thông điệp log từ thiết bị
    def process_log_message(self, log_message):
        # Hiển thị log lên UI
        self.gui.update_monitor(f"Device log: {log_message}")
        
        # Ghi log nếu được bật
        self.setup_log_file("log")
        if self.log_data and "log" in self.log_files:
            timestamp = time.time() - self.common_start_time
            self.log_writers["log"].writerow([f"{timestamp:.3f}", log_message])
            self.log_files["log"].flush()

    def receive_client_data(self, sock):
        buffer = ""  # Lưu dữ liệu bị phân mảnh
        
        try:
            while self.control_active:
                try:
                    data = sock.recv(1024).decode()
                    if not data:
                        self.gui.update_monitor("Control client disconnected")
                        break

                    buffer += data  # Thêm dữ liệu mới vào buffer
                    
                    while "\n" in buffer:  # Kiểm tra nếu có dòng kết thúc
                        line, buffer = buffer.split("\n", 1)  # Lấy một dòng hoàn chỉnh
                        line = line.strip()  # Loại bỏ ký tự trắng thừa

                        # Phân tích dữ liệu JSON
                        try:
                            json_data = json.loads(line)
                            # Kiểm tra trường type
                            if "type" in json_data:
                                message_type = json_data["type"]

                                # Phân phối dữ liệu dựa vào loại
                                if message_type == "encoder" and "data" in json_data:
                                    self.process_encoder_data(json_data["data"])
                                    
                                elif message_type == "bno055" and "data" in json_data:
                                    self.process_bno055_data(json_data["data"])

                                elif message_type == "position" and "data" in json_data:
                                    self.process_position_data(json_data["data"])   

                                elif message_type == "log" and "message" in json_data:
                                    self.process_log_message(json_data["message"])
                                elif message_type == "registration" and "robot_id" in json_data:
                                    robot_id = json_data["robot_id"]
                                    self.gui.update_monitor(f"Robot {robot_id} registered")
                                    # Send registration response immediately
                                    try:
                                        sock.sendall("registration_response".encode())
                                        self.gui.update_monitor(f"Sent registration confirmation to robot {robot_id}")
                                    except Exception as e:
                                        self.gui.update_monitor(f"Error sending registration response: {e}")
                                else:
                                    self.gui.update_monitor(f"Unknown message type or missing data: {json_data}")
                            
                            else:
                                self.gui.update_monitor(f"Missing type field in JSON: {json_data}")
                                
                        except json.JSONDecodeError:
                            # Dữ liệu không đúng định dạng JSON
                            self.gui.update_monitor(f"Invalid JSON format: {line}")
                                
                except socket.timeout:
                    continue
                    
        except Exception as e:
            self.gui.update_monitor(f"Data reception error: {e}")
            print(f"Buffer at error: {buffer}")
        finally:
            # Đóng socket khi client ngắt kết nối
            try:
                if sock:
                    sock.close()
            except:
                pass
            if self.client_socket == sock:
                self.client_socket = None
            self.client_connected = False
            self.connection_status = "Disconnected"
            self.gui.update_status("Disconnected")
            self.gui.disable_control_buttons()
            self.close_all_logs()

    def send_command(self, dot_x, dot_y, dot_theta):
        # Gửi lệnh điều khiển đến client
        stop_time = 20;
        if not self.client_connected:
            print("Not connected - can't send command")
            return
            
        command = f"dot_x:{dot_x:.4f} dot_y:{dot_y:.4f} dot_theta:{dot_theta:.4f} stop_time:{stop_time}"
        
        try:
            self.client_socket.sendall(command.encode())
            # Thêm log chi tiết nếu cần
            if abs(dot_x) > 0.01 or abs(dot_y) > 0.01 or abs(dot_theta) > 0.01:
                print(f"Sent: {command}")
        except Exception as e:
            print(f"Send command error: {e}")
            self.gui.update_monitor(f"Command send error: {e}")
            
    def send_position_goal(self, x, y, theta=0.0):
        """Send a position goal to the robot
        x, y, theta: coordinates and orientation in meters/radians
        """
        if not self.client_connected:
            print("Not connected - can't send position goal")
            return
        
        speed = RPM * M_PER_ROUND / 60
        # Format the command for a position goal
        # Using different prefix to distinguish from velocity commands
        command = f"x:{x:.2f} y:{y:.2f} vel:{speed:.4f}"
        
        try:
            self.client_socket.sendall(command.encode())
            self.gui.update_monitor(f"Sent position goal: x={x:.2f}m, y={y:2f}m")
        except Exception as e:
            print(f"Send position goal error: {e}")
            self.gui.update_monitor(f"Position goal send error: {e}")    
            
    def set_speed(self, motor_index, speed):
        if not self.client_connected:
            self.gui.update_monitor("Not connected - can't set speed")
            return
            
        self.speed[motor_index] = speed
        try:
            command = f"MOTOR_{motor_index + 1}_SPEED:{speed}"
            self.client_socket.sendall(command.encode())
            self.gui.update_monitor(f"Motor {motor_index + 1} speed updated to {speed}")
        except Exception as e:
            self.gui.update_monitor(f"Error setting speed: {e}")

    def emergency_stop(self):
        """Send emergency stop command to all motors"""
        if not self.client_connected:
            return
            
        try:
            self.client_socket.sendall(b"EMERGENCY_STOP")
            self.gui.update_monitor("EMERGENCY STOP sent")
            
            # # Also reset all local speed values
            # for i in range(3):
            #     self.speed[i] = 0
            #     self.gui.update_speed_entry(i, 0)
        except Exception as e:
            self.gui.update_monitor(f"Error sending emergency stop: {e}")

    def send_start_robot(self):
        if not self.client_connected:
            messagebox.showerror("Error", "No client connected")
            return
            
        try:
            self.client_socket.sendall(b"Start Robot")
            self.gui.update_monitor("Sent 'Start Robot' command to the client.")
        except Exception as e:
            self.gui.update_monitor(f"Failed to send 'Start Robot' command: {e}")

    def send_start_position_command(self):
        """Send start position command to the client"""
        if not self.client_connected:
            print("Not connected - can't send start position command")
            self.gui.update_monitor("Not connected - can't send start position command")
            return
            
        try:
            self.client_socket.sendall(b"Start Position")
            self.gui.update_monitor("Sent 'Start Position' command to the client")
        except Exception as e:
            print(f"Send start position command error: {e}")
            self.gui.update_monitor(f"Start position command send error: {e}")

    def show_trajectory_plot(self):
        """Display the trajectory visualization window"""
        trajectory_visualizer.show()
        self.gui.update_monitor("Trajectory visualization displayed")

    def show_robot_position_plot(self):
        """Display the robot position visualization window"""
        robot_position_visualizer.show()
        self.gui.update_monitor("Robot position visualization displayed")

    def show_rpm_plot(self):
        """Display the RPM plot window"""
        rpm_plotter.show_plot()
        self.gui.update_monitor("RPM monitoring plot displayed")

    def set_pid_values(self, motor_index, p, i, d):
        if not self.client_connected:
            self.gui.update_monitor("Not connected - can't set PID values")
            return
            
        self.pid_values[motor_index] = [p, i, d]
        try:
            pid_command = f"MOTOR:{motor_index + 1} Kp:{p} Ki:{i} Kd:{d}"
            self.client_socket.sendall(pid_command.encode())
            self.gui.update_monitor(f"PID values set: MOTOR:{motor_index + 1} Kp:{p} Ki:{i} Kd:{d}")
        except Exception as e:
            self.gui.update_monitor(f"Error setting PID values: {e}")

    def save_pid_config(self):
        """Save current PID configuration to file"""
        try:
            with open("pid_config.txt", "w") as f:
                for idx, (p, i, d) in enumerate(self.pid_values):
                    # Đảm bảo giá trị là số thực
                    p_float = float(p)
                    i_float = float(i)
                    d_float = float(d)
                    f.write(f"Motor{idx+1}:{p_float},{i_float},{d_float}\n")
            self.gui.update_monitor("PID configuration saved successfully")
        except Exception as e:
            self.gui.update_monitor(f"Error saving PID config: {e}")

    def load_pid_config(self):
        """Load PID configuration from file"""
        try:
            with open("pid_config.txt", "r") as f:
                for line in f:
                    if not line.strip():
                        continue
                    parts = line.strip().split(":")
                    if len(parts) != 2:
                        continue
                    motor_name, values = parts
                    # Xử lý đúng định dạng - chuyển sang float rồi sang int
                    motor_str = motor_name.replace("Motor", "")
                    try:
                        motor_index = int(float(motor_str)) - 1
                    except ValueError:
                        self.gui.update_monitor(f"Invalid motor number: {motor_str}")
                        continue
                    
                    p, i, d = map(float, values.split(","))
                    self.pid_values[motor_index] = [p, i, d]
                    self.gui.update_pid_entries(motor_index, p, i, d)
            self.gui.update_monitor("PID configuration loaded")
        except FileNotFoundError:
            self.gui.update_monitor("PID config file not found")
        except Exception as e:
            self.gui.update_monitor(f"Error loading PID config: {e}")

    def set_position_visualizer_callback(self):
        """Set the callback for the position visualizer to send position goals"""
        robot_position_visualizer.set_destination_callback(self.send_position_goal)
        self.gui.update_monitor("Position visualizer destination callback configured")