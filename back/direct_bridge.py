import sys
import os
import asyncio
import websockets
import json
import logging
from connection_manager import ConnectionManager 
import time
import argparse
import math
from dotenv import load_dotenv
import base64

# Load environment variables from .env file
load_dotenv()

# --- Configuration from Environment Variables with Fallbacks ---
TCP_PORT_DEFAULT = 12346
WS_PORT_DEFAULT = 9003
OTA_PORT_DEFAULT = 12345
LOG_LEVEL_DEFAULT = "INFO"
LOG_DIRECTORY_DEFAULT = "logs/bridge_logs"
MAX_TRAJECTORY_POINTS_DEFAULT = 1000
WHEEL_RADIUS_DEFAULT = 0.0325
ROBOT_BASE_DISTANCE_L_DEFAULT = 0.1
PID_CONFIG_FILE_DEFAULT = "pid_config.txt"
TEMP_FIRMWARE_DIR_DEFAULT = "temp_firmware"
TCP_CLIENT_TIMEOUT_DEFAULT = 60.0

# --- Robot Alias Management ---
robot_alias_manager = {
    "ip_port_to_alias": {},  # "192.168.1.100:12346" -> "robot1"
    "alias_to_ip_port": {},  # "robot1" -> "192.168.1.100:12346"
    "ip_to_alias": {},       # "192.168.1.100" -> "robot1" (maps IP to the *first* alias assigned to that IP)
    "alias_to_ip": {},       # "robot1" -> "192.168.1.100"
    "next_robot_number": 1,
    "lock": asyncio.Lock()
}

# --- Global set for UI WebSocket clients ---
ui_websockets = set()

# --- Helper function to broadcast to all UI clients ---
async def broadcast_to_all_ui(message_payload):
    if ui_websockets: # Check if there are any UI clients
        message_json = json.dumps(message_payload)
        # Create a list of tasks to send messages concurrently
        tasks = [ws.send(message_json) for ws in ui_websockets]
        results = await asyncio.gather(*tasks, return_exceptions=True)
        for i, result in enumerate(results):
            if isinstance(result, Exception):
                ws_to_remove = list(ui_websockets)[i] # This indexing might be fragile if set changes during await
                logger.error(f"Error sending to UI websocket {ws_to_remove.remote_address}: {result}. Removing.")
                
# --- TrajectoryCalculator class ---
class TrajectoryCalculator: 
    def __init__(self):
        self.robot_data = {}  # Keyed by unique_robot_key (ip:port or alias/IP)
        self.CONTROL_SUPPRESS_WINDOW = 0.5  # seconds: if encoder seen recently, skip control integration

    def _ensure_robot_data(self, unique_robot_key):
        if unique_robot_key not in self.robot_data:
            self.robot_data[unique_robot_key] = {
                "x": 0.0, "y": 0.0, "theta": 0.0,
                "last_timestamp_encoder": None,
                "last_timestamp_control": None,
                "last_imu_path_time": 0.0,
                "path_history": [],
                "latest_imu_data": None,
                "imu_missing_warned": False
            }

    def update_imu_data(self, unique_robot_key, imu_data):
        self._ensure_robot_data(unique_robot_key)
        # Accepts either {"yaw": ...} or {"euler": [roll, pitch, yaw]}
        yaw = None
        if "yaw" in imu_data:
            yaw = imu_data["yaw"]
        elif "euler" in imu_data and len(imu_data["euler"]) == 3:
            yaw = imu_data["euler"][2]
        robot_state = self.robot_data[unique_robot_key]
        if yaw is not None:
            robot_state["theta"] = yaw
        robot_state["latest_imu_data"] = imu_data
        # Reset warning flag when IMU arrives
        robot_state["imu_missing_warned"] = False

        # Optionally append a path point on IMU-only updates to allow UI drawing (x,y unchanged)
        now_t = time.time()
        if now_t - robot_state.get("last_imu_path_time", 0.0) >= 0.2:  # throttle to 5 Hz
            new_point = {"x": robot_state["x"], "y": robot_state["y"], "theta": robot_state["theta"]}
            robot_state["path_history"].append(new_point)
            if len(robot_state["path_history"]) > MAX_TRAJECTORY_POINTS_DEFAULT:
                robot_state["path_history"] = robot_state["path_history"][-MAX_TRAJECTORY_POINTS_DEFAULT:]
            robot_state["last_imu_path_time"] = now_t
        # Return current snapshot for convenience
        current_pose = {"x": robot_state["x"], "y": robot_state["y"], "theta": robot_state["theta"]}
        return {"position": current_pose, "path": list(robot_state["path_history"]) }

    def _rpm_to_omega(self, rpm):
        return rpm * (2 * math.pi) / 60.0

    def update_position_packet(self, unique_robot_key, position_payload):
        """
        Authoritatively set the robot pose from firmware 'position' packet.
        position_payload: dict with keys x, y, theta (radians).
        Resets control integration timer and appends to path history.
        """
        self._ensure_robot_data(unique_robot_key)
        robot_state = self.robot_data[unique_robot_key]

        try:
            x = float(position_payload.get("x", robot_state["x"]))
            y = float(position_payload.get("y", robot_state["y"]))
            theta = float(position_payload.get("theta", robot_state["theta"]))
        except Exception:
            # If payload malformed, return current snapshot
            current_pose = {"x": robot_state["x"], "y": robot_state["y"], "theta": robot_state["theta"]}
            return {"position": current_pose, "path": list(robot_state["path_history"]) }

        robot_state["x"] = x
        robot_state["y"] = y
        robot_state["theta"] = theta
        # Firmware pose is authoritative; reset integration timers
        robot_state["last_timestamp_control"] = None

        new_point = {"x": x, "y": y, "theta": theta}
        robot_state["path_history"].append(new_point)
        if len(robot_state["path_history"]) > MAX_TRAJECTORY_POINTS_DEFAULT:
            robot_state["path_history"] = robot_state["path_history"][-MAX_TRAJECTORY_POINTS_DEFAULT:]

        return {"position": new_point, "path": list(robot_state["path_history"]) }

    def update_encoder_data(self, unique_robot_key, encoder_data):
        self._ensure_robot_data(unique_robot_key)
        robot_state = self.robot_data[unique_robot_key]
        imu_data = robot_state["latest_imu_data"]
        # If IMU is missing, proceed using current theta but warn only once
        if not imu_data and not robot_state["imu_missing_warned"]:
            logger.warning(f"IMU data not available for {unique_robot_key}. Falling back to odometry-only heading.")
            robot_state["imu_missing_warned"] = True

        timestamp_encoder = encoder_data.get("timestamp", time.time())
        # The 'data' key in encoder_data from transform_robot_message contains the RPM list
        rpms = encoder_data.get("data", []) # Use "data" as per transform_robot_message
        
        if not (isinstance(rpms, list) and len(rpms) >= 3): # Check for at least 3 for safety
            logger.warning(f"Invalid RPM data for {unique_robot_key}: {rpms}. Expected list of 3 numbers in 'data' field. Returning current state.")
            current_pose_dict = {"x": robot_state["x"], "y": robot_state["y"], "theta": robot_state["theta"]}
            # Return the structured dictionary
            return {"position": current_pose_dict, "path": list(robot_state["path_history"])}
        
        current_heading_rad = robot_state["theta"] # Default to current state theta
        if imu_data:
            if "yaw" in imu_data: # This check is fine
                current_heading_rad = imu_data["yaw"]
            elif "euler" in imu_data and isinstance(imu_data.get("euler"), list) and len(imu_data["euler"]) == 3:
                current_heading_rad = imu_data["euler"][2]
        if robot_state["last_timestamp_encoder"] is None:
            robot_state["last_timestamp_encoder"] = timestamp_encoder
            return None  # No movement yet
        dt = timestamp_encoder - robot_state["last_timestamp_encoder"]
        if dt <= 0:
            return None
        robot_state["last_timestamp_encoder"] = timestamp_encoder
        omega_m1, omega_m2, omega_m3 = [self._rpm_to_omega(rpm) for rpm in rpms]
        # Simple kinematics: vx = avg(omegas) * wheel_radius
        vx_robot = WHEEL_RADIUS_DEFAULT * (omega_m1 + omega_m2 + omega_m3) / 3.0
        vy_robot = 0  # For 3-wheel omni, you may want to use a more complex model

        # Transform robot-frame velocities to world-frame using standard rotation
        theta = robot_state["theta"]
        cos_h = math.cos(theta)
        sin_h = math.sin(theta)
        vx_world = vx_robot * cos_h - vy_robot * sin_h
        vy_world = vx_robot * sin_h + vy_robot * cos_h
        robot_state["x"] += vx_world * dt
        robot_state["y"] += vy_world * dt
        robot_state["theta"] = current_heading_rad
        # Since we used reliable encoder data, reset control integration timer to avoid stale dt accumulation
        robot_state["last_timestamp_control"] = None
        # At the end of successful calculation:
        new_point = {"x": robot_state["x"], "y": robot_state["y"], "theta": robot_state["theta"]}
        robot_state["path_history"].append(new_point)
        if len(robot_state["path_history"]) > MAX_TRAJECTORY_POINTS_DEFAULT:
            robot_state["path_history"] = robot_state["path_history"][-MAX_TRAJECTORY_POINTS_DEFAULT:]
        # This return is already correct
        return {"position": new_point, "path": list(robot_state["path_history"]) }

    def update_control_command(self, unique_robot_key, vx_robot, vy_robot, omega, timestamp=None, override=False):
        """
        Integrate control commands when encoder data is absent. vx/vy are robot-frame linear velocities (m/s),
        omega is angular velocity (rad/s). Uses current heading to convert to world frame.
        Suppresses integration if encoder updated very recently unless override=True.
        """
        self._ensure_robot_data(unique_robot_key)
        robot_state = self.robot_data[unique_robot_key]

        # If encoder has been seen very recently, skip control-based integration to avoid double-counting
        last_enc_t = robot_state.get("last_timestamp_encoder")
        now_t = timestamp if timestamp is not None else time.time()
        if last_enc_t is not None and (now_t - last_enc_t) <= self.CONTROL_SUPPRESS_WINDOW and not override:
            return {"position": {"x": robot_state["x"], "y": robot_state["y"], "theta": robot_state["theta"]},
                    "path": list(robot_state["path_history"]) }

        # If commanded stop, reset control timer and append a point for UI, then return
        if abs(vx_robot) < 1e-6 and abs(vy_robot) < 1e-6 and abs(omega) < 1e-6:
            robot_state["last_timestamp_control"] = None
            new_point = {"x": robot_state["x"], "y": robot_state["y"], "theta": robot_state["theta"]}
            robot_state["path_history"].append(new_point)
            if len(robot_state["path_history"]) > MAX_TRAJECTORY_POINTS_DEFAULT:
                robot_state["path_history"] = robot_state["path_history"][-MAX_TRAJECTORY_POINTS_DEFAULT:]
            return {"position": new_point, "path": list(robot_state["path_history"]) }

        # Initialize timing for control on first non-zero command
        if robot_state.get("last_timestamp_control") is None:
            robot_state["last_timestamp_control"] = now_t
            # Append a point so UI can draw even before motion accumulates
            new_point = {"x": robot_state["x"], "y": robot_state["y"], "theta": robot_state["theta"]}
            robot_state["path_history"].append(new_point)
            if len(robot_state["path_history"]) > MAX_TRAJECTORY_POINTS_DEFAULT:
                robot_state["path_history"] = robot_state["path_history"][-MAX_TRAJECTORY_POINTS_DEFAULT:]
            return {"position": new_point, "path": list(robot_state["path_history"]) }

        dt = now_t - robot_state["last_timestamp_control"]
        if dt <= 0:
            return {"position": {"x": robot_state["x"], "y": robot_state["y"], "theta": robot_state["theta"]},
                    "path": list(robot_state["path_history"]) }
        # Clamp dt to prevent large jumps after idle
        if dt > 0.25:
            dt = 0.25
        robot_state["last_timestamp_control"] = now_t

        # Transform to world frame using current heading (no axis swap; +X body is forward/arrow)
        theta = robot_state["theta"]
        cos_h = math.cos(theta)
        sin_h = math.sin(theta)
        vx_world = vx_robot * cos_h - vy_robot * sin_h
        vy_world = vx_robot * sin_h + vy_robot * cos_h

        robot_state["x"] += vx_world * dt
        robot_state["y"] += vy_world * dt
        robot_state["theta"] += omega * dt  # Will be overridden by IMU when available

        new_point = {"x": robot_state["x"], "y": robot_state["y"], "theta": robot_state["theta"]}
        robot_state["path_history"].append(new_point)
        if len(robot_state["path_history"]) > MAX_TRAJECTORY_POINTS_DEFAULT:
            robot_state["path_history"] = robot_state["path_history"][-MAX_TRAJECTORY_POINTS_DEFAULT:]
        return {"position": new_point, "path": list(robot_state["path_history"]) }

    def clear_path_history(self, unique_robot_key):
        self._ensure_robot_data(unique_robot_key)
        robot_state = self.robot_data[unique_robot_key]
        robot_state["path_history"].clear()
        logger.info(f"Path history cleared for robot {unique_robot_key}")
        
        # Return the current pose and an empty path to allow broadcasting an update
        current_pose_dict = {"x": robot_state["x"], "y": robot_state["y"], "theta": robot_state["theta"]}
        return {"position": current_pose_dict, "path": []}

    def get_snapshot(self, unique_robot_key):
        """Return current pose and full path for a robot, if known."""
        if unique_robot_key not in self.robot_data:
            return None
        robot_state = self.robot_data[unique_robot_key]
        current_pose_dict = {"x": robot_state["x"], "y": robot_state["y"], "theta": robot_state["theta"]}
        return {"position": current_pose_dict, "path": list(robot_state["path_history"]) }

class DataLogger:
    def __init__(self, log_directory=None):
        self.log_directory = log_directory or os.environ.get("LOG_DIRECTORY", LOG_DIRECTORY_DEFAULT)
        os.makedirs(self.log_directory, exist_ok=True)
        self.log_files = {} 
        self.session_start_time = time.strftime('%Y%m%d_%H%M%S')

    def get_log_file(self, unique_robot_key, data_type): # Changed robot_id to unique_robot_key
        if unique_robot_key not in self.log_files:
            self.log_files[unique_robot_key] = {}

        if data_type not in self.log_files[unique_robot_key]:
            safe_robot_key = unique_robot_key.replace(":", "_").replace(".","_") # Make it more filename friendly
            log_filename = os.path.join(self.log_directory, f"{data_type}_{safe_robot_key}_{self.session_start_time}.txt")
            
            try:
                file_handle = open(log_filename, "a") 
                self.log_files[unique_robot_key][data_type] = file_handle
                logger.info(f"Logging {data_type} for {unique_robot_key} to {log_filename}")
                if os.path.getsize(log_filename) == 0:
                    if data_type == "encoder":
                        file_handle.write("Time RPM1 RPM2 RPM3\n")
                    elif data_type == "bno055" or data_type == "imu":
                        file_handle.write("Time Heading Pitch Roll W X Y Z AccelX AccelY AccelZ GravityX GravityY GravityZ\n") 
                    elif data_type == "log" or data_type == "log_data":
                        file_handle.write("Time Message\n")
                    elif data_type == "position_update":
                        file_handle.write("Time X Y Theta\n")
                    file_handle.flush()
            except Exception as e:
                logger.error(f"Failed to open log file for {unique_robot_key} {data_type}: {e}")
                return None
        return self.log_files[unique_robot_key].get(data_type)

    def log_data(self, unique_robot_key, data_type, message_dict): # Changed robot_id to unique_robot_key
        file_handle = self.get_log_file(unique_robot_key, data_type)
        if not file_handle:
            return

        try:
            log_timestamp = message_dict.get("timestamp", time.time())
            
            if data_type == "encoder" or data_type == "encoder_data":
                log_line = f"{log_timestamp:.3f} {message_dict.get('rpm_1',0)} {message_dict.get('rpm_2',0)} {message_dict.get('rpm_3',0)}\n"
            elif data_type == "bno055" or data_type == "imu" or data_type == "imu_data":
                heading = message_dict.get("heading", 0.0)
                pitch = message_dict.get("pitch", 0.0)
                roll = message_dict.get("roll", 0.0)
                w = message_dict.get("quat_w", 1.0) 
                x = message_dict.get("quat_x", 0.0)
                y = message_dict.get("quat_y", 0.0)
                z = message_dict.get("quat_z", 0.0)
                ax = message_dict.get("lin_accel_x", 0.0)
                ay = message_dict.get("lin_accel_y", 0.0)
                az = message_dict.get("lin_accel_z", 0.0)
                gx = message_dict.get("gravity_x", 0.0)
                gy = message_dict.get("gravity_y", 0.0)
                gz = message_dict.get("gravity_z", 0.0)
                
                log_line = f"{log_timestamp:.3f} {heading:.2f} {pitch:.2f} {roll:.2f} {w:.4f} {x:.4f} {y:.4f} {z:.4f} {ax:.2f} {ay:.2f} {az:.2f} {gx:.2f} {gy:.2f} {gz:.2f}\n"
            elif data_type == "log" or data_type == "log_data": 
                log_line = f"{log_timestamp:.3f} {message_dict.get('message', '')}\n"
            elif data_type == "position_update":
                pos = message_dict.get("position", {})
                log_line = f"{log_timestamp:.3f} {pos.get('x',0):.3f} {pos.get('y',0):.3f} {pos.get('theta',0):.3f}\n"
            else:
                log_line = f"{log_timestamp:.3f} {json.dumps(message_dict)}\n"
            
            file_handle.write(log_line)
            file_handle.flush() 
        except Exception as e:
            logger.error(f"Error writing to log for {unique_robot_key} {data_type}: {e}")

    def close_logs(self, unique_robot_key=None): # Changed robot_id to unique_robot_key
        if unique_robot_key:
            if unique_robot_key in self.log_files:
                for data_type, file_handle in self.log_files[unique_robot_key].items():
                    try:
                        file_handle.close()
                    except Exception as e:
                        logger.error(f"Error closing log file for {unique_robot_key} {data_type}: {e}")
                del self.log_files[unique_robot_key]
                logger.info(f"Closed log files for {unique_robot_key}")
        else: 
            for r_id in list(self.log_files.keys()):
                self.close_logs(r_id)
            logger.info("Closed all log files.")

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("DirectBridge")
log_tcp = logger.getChild("TCP")
log_ws = logger.getChild("WS")
log_ota = logger.getChild("OTA")
log_fw = logger.getChild("FWUP")
log_traj = logger.getChild("TRAJ")
# Initialize DataLogger with None, so it picks up from env or default
data_logger = DataLogger()


# --- OTAConnection class (Handles OTA Server Functionality) ---
class OTAConnection: 
    def __init__(self):
        # For OTA Server functionality
        # Support multiple robots: map robot_ip -> prepared firmware path
        self._firmware_map = {}  # { robot_ip: firmware_path }
        self.ota_server_instance = None # To hold the asyncio.Server object
        self._ws_broadcast = broadcast_to_all_ui

    def set_prepared_firmware_for_robot(self, target_robot_ip: str, file_path: str):
        """Register a prepared firmware file for a specific robot IP."""
        if not target_robot_ip or not os.path.exists(file_path):
            logger.error(f"Invalid firmware mapping: ip={target_robot_ip}, path={file_path}")
            return False
        self._firmware_map[target_robot_ip] = file_path
        logger.info(f"Prepared firmware for {target_robot_ip}: {file_path}")
        return True

    async def prepare_firmware_for_send(self, file_path, target_robot_ip): # Changed target_robot_id to target_robot_ip
        if not os.path.exists(file_path):
            logger.error(f"Firmware file not found: {file_path}")
            return False
        return self.set_prepared_firmware_for_robot(target_robot_ip, file_path)

    async def handle_ota_robot_connection(self, reader, writer):
        addr = writer.get_extra_info('peername')
        robot_actual_ip = addr[0]
        robot_ip_port_str = f"{robot_actual_ip}:12345"
        # Use self.ota_port_arg_val if available and consistent, or directly log the port it's bound to.
        # Assuming self.ota_port_arg_val holds the port the persistent server was started on.
        log_ota.info(f"Client connected from {robot_ip_port_str} (listening port {getattr(self, 'ota_port_arg_val', 'UNKNOWN')})")

        firmware_was_sent = False
        current_firmware_path_for_this_connection = self._firmware_map.get(robot_actual_ip)
        if current_firmware_path_for_this_connection:
            log_ota.info(f"Target robot {robot_actual_ip} connected. Firmware {os.path.basename(current_firmware_path_for_this_connection)} is designated.")
        else:
            log_ota.warning(f"Client {robot_actual_ip} connected, but no firmware prepared/available for this IP.")
        
        # Notify UI that a client connected to OTA server
        try:
            await self._ws_broadcast({
                "type": "ota_status",
                "robot_ip": robot_actual_ip,
                "status": "client_connected",
                "message": f"Robot {robot_ip_port_str} connected to OTA server",
                "ota_port": getattr(self, 'ota_port_arg_val', None),
                "timestamp": time.time()
            })
        except Exception:
            pass

        if current_firmware_path_for_this_connection:
            try:
                log_ota.info(f"Start sending firmware {os.path.basename(current_firmware_path_for_this_connection)} to {robot_ip_port_str}")
                with open(current_firmware_path_for_this_connection, "rb") as f:
                    chunk_size = 1024
                    total_size = os.path.getsize(current_firmware_path_for_this_connection)
                    sent_bytes = 0
                    # Optional handshake: wait briefly for any byte from client (some firmwares send a kickoff byte)
                    try:
                        await asyncio.wait_for(reader.read(1), timeout=0.5)
                        log_ota.debug(f"Received kickoff byte from {robot_ip_port_str} - starting stream")
                    except asyncio.TimeoutError:
                        # No kickoff received; proceed anyway
                        await asyncio.sleep(0.1)
                    # Notify start
                    try:
                        await self._ws_broadcast({
                            "type": "ota_status",
                            "robot_ip": robot_actual_ip,
                            "status": "sending",
                            "message": "Sending firmware...",
                            "progress": 0,
                            "total_bytes": total_size,
                            "sent_bytes": 0,
                            "timestamp": time.time()
                        })
                    except Exception:
                        pass
                    while True: # This loop is now correctly indented
                        chunk = f.read(chunk_size)
                        if not chunk:
                            break
                        writer.write(chunk)
                        await writer.drain()
                        sent_bytes += len(chunk)
                        # Throttle progress events
                        if total_size > 0 and (sent_bytes == total_size or sent_bytes % (chunk_size*16) == 0):
                            try:
                                await self._ws_broadcast({
                                    "type": "ota_status",
                                    "robot_ip": robot_actual_ip,
                                    "status": "sending",
                                    "message": "Sending firmware...",
                                    "progress": int(sent_bytes * 100 / total_size),
                                    "total_bytes": total_size,
                                    "sent_bytes": sent_bytes,
                                    "timestamp": time.time()
                                })
                            except Exception:
                                pass
                log_ota.info(f"Firmware sent successfully to {robot_ip_port_str}")
                firmware_was_sent = True
                # Notify complete
                try:
                    await self._ws_broadcast({
                        "type": "ota_status",
                        "robot_ip": robot_actual_ip,
                        "status": "ota_complete",
                        "message": "Firmware sent successfully",
                        "progress": 100,
                        "timestamp": time.time()
                    })
                except Exception:
                    pass
                
                # Firmware sent successfully, so consume mapping for this robot
                if robot_actual_ip in self._firmware_map and self._firmware_map[robot_actual_ip] == current_firmware_path_for_this_connection:
                    del self._firmware_map[robot_actual_ip]
                log_ota.info(f"Consumed firmware {os.path.basename(current_firmware_path_for_this_connection)} after sending to {robot_actual_ip}.")
                
                # Optionally, delete the temp firmware file if desired
                try:
                    if os.path.exists(current_firmware_path_for_this_connection): # Check before deleting
                        os.remove(current_firmware_path_for_this_connection)
                        log_ota.info(f"Deleted temporary firmware file: {current_firmware_path_for_this_connection}")
                except OSError as e_del:
                    logger.error(f"Error deleting temporary firmware file {current_firmware_path_for_this_connection}: {e_del}")
            except Exception as e:
                log_ota.error(f"Error sending firmware to {robot_ip_port_str}: {e}")
                try:
                    await self._ws_broadcast({
                        "type": "ota_status",
                        "robot_ip": robot_actual_ip,
                        "status": "ota_failed",
                        "message": str(e),
                        "timestamp": time.time()
                    })
                except Exception:
                    pass
                # If send failed for the intended target, still consume the firmware mapping for safety
                if robot_actual_ip in self._firmware_map and self._firmware_map[robot_actual_ip] == current_firmware_path_for_this_connection:
                    del self._firmware_map[robot_actual_ip]
                    log_ota.info(f"Cleared firmware path for {robot_actual_ip} after send error for {current_firmware_path_for_this_connection}.")
            else:
                log_ota.info(f"No firmware sent to {robot_ip_port_str} (not prepared or wrong target).")
        
        writer.close()
        try:
            await writer.wait_closed()
        except Exception as e_close:
            logger.error(f"Error during writer.wait_closed() for OTA client {robot_ip_port_str}: {e_close}")
        
        log_ota.info(f"Client {robot_ip_port_str} disconnected. OTA server remains listening.")
        # The main self.ota_server_instance is NOT closed here.

    async def start_ota_server_once(self, ota_port=12345):
        # This method will be replaced by start_persistent_ota_server and is no longer called directly
        # for starting the server if it's always on. Retained for reference or if a different OTA mode is needed.
        # self.ota_port_arg_val = ota_port 
        if self.ota_server_instance:
            logger.info("OTA server is already running or preparing.")
            return True
        # Legacy method retained; use first prepared mapping if exists
        if not self._firmware_map:
            logger.error("Cannot start OTA server (once): No firmware prepared or no target robot IP set.")
            return False
        try:
            # This server instance would be temporary if using "start_once" logic.
            # For always-on, the server instance is managed differently.
            temp_server = await asyncio.start_server(
                self.handle_ota_robot_connection, '0.0.0.0', ota_port
            )
            logger.info(f"Temporary OTA Server started on 0.0.0.0:{ota_port}. It will close after one connection.")
            # Note: temp_server is not stored; this method is legacy and generally unused.
            return True
        except Exception as e:
            logger.error(f"Failed to start temporary OTA server on port {ota_port}: {e}")
            return False

    async def start_persistent_ota_server(self, ota_port):
        if self.ota_server_instance and self.ota_server_instance.is_serving():
            logger.info(f"Persistent OTA server is already running on port {ota_port}.")
            return True
        try:
            self.ota_server_instance = await asyncio.start_server(
                self.handle_ota_robot_connection, '0.0.0.0', ota_port
            )
            self.ota_port_arg_val = ota_port # Store for reference, though port is fixed once started
            logger.info(f"Persistent OTA Server started successfully on 0.0.0.0:{ota_port}.")
            return True
        except Exception as e:
            logger.error(f"Failed to start persistent OTA server on port {ota_port}: {e}")
            self.ota_server_instance = None
            return False

    async def stop_ota_server(self):
        if self.ota_server_instance:
            self.ota_server_instance.close()
            await self.ota_server_instance.wait_closed()
            self.ota_server_instance = None
            logger.info("OTA Server stopped by request.")
        self._firmware_map.clear()

# ==================== FirmwareUploadManager ====================
class FirmwareUploadManager:
    """
    Gom từng chunk b64 rồi ghi ra file .bin trong TEMP_FIRMWARE_DIR.
    Key theo robot_ip để có thể song song nhiều robot.
    """
    def __init__(self, temp_dir):
        self.temp_dir = temp_dir
        os.makedirs(self.temp_dir, exist_ok=True)
        self._uploads = {}      # robot_ip -> dict(info)

    def start(self, robot_ip, filename, filesize):
        path = os.path.join(self.temp_dir, f"{robot_ip}_{int(time.time())}_{filename}")
        f = open(path, "wb")
        self._uploads[robot_ip] = {
            "file": f,
            "path": path,
            "filesize": filesize,
            "received": 0
        }
        logger.info(f"[FW-UP] Start upload {filename} ({filesize} bytes) for {robot_ip} → {path}")

    def add_chunk(self, robot_ip, b64_chunk):
        if robot_ip not in self._uploads:
            logger.warning(f"[FW-UP] Received chunk for {robot_ip} but upload not started")
            return 0
        raw = base64.b64decode(b64_chunk)
        inf = self._uploads[robot_ip]
        inf["file"].write(raw)
        inf["received"] += len(raw)
        return inf["received"]

    def finish(self, robot_ip):
        if robot_ip not in self._uploads:
            logger.warning(f"[FW-UP] finish called for {robot_ip} but not found")
            return None
        inf = self._uploads.pop(robot_ip)
        inf["file"].close()
        if inf["received"] != inf["filesize"]:
            logger.error(f"[FW-UP] Size mismatch for {robot_ip}: {inf['received']} / {inf['filesize']}")
            return None
        logger.info(f"[FW-UP] Completed upload for {robot_ip}. File saved: {inf['path']}")
        return inf["path"]

    def get_received_bytes(self, robot_ip):
        """
        Trả về số byte đã nhận cho robot_ip (dùng cho progress bar).
        """
        if robot_ip in self._uploads:
            return self._uploads[robot_ip]["received"]
        return 0
# ===============================================================


class DirectBridge:
    GLOBAL_SUBSCRIPTION_KEY = "__GLOBAL__" # Định nghĩa hằng số ở đây

    def __init__(self, tcp_port, ws_port, pid_config_file_path=None): # Added pid_config_file_path
        self.manager = ConnectionManager() # Removed robot_alias_manager argument
        self.tcp_port = tcp_port 
        self.ws_port = ws_port
        self.ota_connection = OTAConnection()
        self.trajectory_calculator = TrajectoryCalculator()
        self.pid_config_file = pid_config_file_path if pid_config_file_path else os.environ.get("PID_CONFIG_FILE", PID_CONFIG_FILE_DEFAULT)
        self.pid_config_cache = {}  # Initialize PID config cache
        self.temp_firmware_dir = os.environ.get("TEMP_FIRMWARE_DIR", TEMP_FIRMWARE_DIR_DEFAULT)
        os.makedirs(self.temp_firmware_dir, exist_ok=True)
        self.ota_port_arg = None # Will be set from main_bridge_runner
        self.data_logger = data_logger # Use the global instance
        self.websocket_subscriptions = {} # Stores subscriptions per websocket client
        self.subscribers_lock = asyncio.Lock() # Added lock for subscribers dictionary
        self._latest_encoder_data = {} # Initialize latest encoder data
        self._latest_imu_data = {} # Initialize latest IMU data
        self.fw_upload_mgr = FirmwareUploadManager(self.temp_firmware_dir)

    def build_robot_status_snapshot(self, unique_robot_key: str, robot_alias: str, robot_ip: str):
        """Assemble a robot_status snapshot from latest cached data."""
        latest_enc = self._latest_encoder_data.get(unique_robot_key)
        rpm_list = []
        if isinstance(latest_enc, dict):
            data_field = latest_enc.get("data")
            if isinstance(data_field, list):
                rpm_list = data_field[:3]
        if len(rpm_list) < 3:
            rpm_list = [0, 0, 0]

        pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        snap = self.trajectory_calculator.get_snapshot(unique_robot_key)
        if isinstance(snap, dict) and isinstance(snap.get("position"), dict):
            pos = snap["position"]
            pose["x"] = float(pos.get("x", 0.0))
            pose["y"] = float(pos.get("y", 0.0))
            pose["theta"] = float(pos.get("theta", 0.0))

        payload = {
            "type": "robot_status",
            "robot_ip": robot_ip,
            "robot_alias": robot_alias,
            "timestamp": time.time(),
            "encoders": {"rpm": rpm_list},
            "position": pose,
        }
        return payload

    async def get_robot_key_from_alias(self, robot_alias: str) -> str:
        """Resolve alias (now using robot IP) -> unique robot key (ip:port)."""
        if not robot_alias:
            return None
        # If alias already in ip:port form, return it directly
        if ":" in robot_alias and robot_alias.count(":") == 1:
            return robot_alias
        # Otherwise treat alias as IP and resolve via ConnectionManager
        return ConnectionManager.get_robot_id_by_ip(robot_alias)

    async def send_text_command_by_alias(self, robot_alias: str, command_text: str) -> bool:
        """Send a plaintext command (ensures trailing newline) to robot by alias."""
        unique_key = await self.get_robot_key_from_alias(robot_alias)
        if not unique_key:
            return False
        conn_tuple = ConnectionManager.get_tcp_client(unique_key)
        if not conn_tuple:
            return False
        reader, writer = conn_tuple
        try:
            # Ensure newline termination for robust parsing on device
            if not command_text.endswith("\n"):
                command_text = command_text + "\n"
            writer.write(command_text.encode("utf-8"))
            await writer.drain()
            return True
        except Exception:
            return False

    async def send_upgrade_command_by_alias(self, robot_alias: str, mode_override: str = None) -> bool:
        """Try multiple upgrade command variants to maximize compatibility with firmware.
        mode_override may be one of: 'raw' | 'newline' | 'both' to override env/default.
        """
        unique_key = await self.get_robot_key_from_alias(robot_alias)
        if not unique_key:
            return False
        conn_tuple = ConnectionManager.get_tcp_client(unique_key)
        if not conn_tuple:
            return False
        reader, writer = conn_tuple
        try:
            # Choose how to send the Upgrade command based on override/env (default: raw)
            # BRIDGE_UPGRADE_MODE: 'newline' | 'raw' | 'both'
            mode = (mode_override or os.environ.get("BRIDGE_UPGRADE_MODE", "raw")).lower()
            if mode == "raw":
                writer.write(b"Upgrade")
                await writer.drain()
                log_tcp.info(f"Sent Upgrade (raw) to {robot_alias} ({unique_key})")
            elif mode == "both":
                writer.write(b"Upgrade")
                await writer.drain()
                await asyncio.sleep(0.05)
                writer.write(b"Upgrade\n")
                await writer.drain()
                log_tcp.info(f"Sent Upgrade (raw + newline) to {robot_alias} ({unique_key})")
            else:  # 'newline'
                writer.write(b"Upgrade\n")
                await writer.drain()
                log_tcp.info(f"Sent Upgrade (newline) to {robot_alias} ({unique_key})")
            return True
        except Exception as e:
            log_tcp.error(f"Failed to send Upgrade command to {robot_alias}: {e}")
            return False

    async def get_connected_robots_list(self):
        """Return connected robots using IP as alias."""
        robots_list = []
        for item in ConnectionManager.get_all_robots_with_ip():
            if item.get("connected"):
                robots_list.append({
                    "alias": item.get("ip"),
                    "ip": item.get("ip"),
                    "unique_key": item.get("robot_id"),
                    "status": "connected"
                })
        return robots_list

    async def send_connected_robots_list_to_client(self, websocket_client):
        """
        Sends the current list of connected robots to a specific WebSocket client.
        """
        try:
            connected_robots = await self.get_connected_robots_list()
            message_payload = {
                "type": "available_robots_initial_list", # Or a more descriptive type
                "robots": connected_robots,
                "timestamp": time.time()
            }
            await websocket_client.send(json.dumps(message_payload))
            logger.info(f"Sent initial list of {len(connected_robots)} connected robots to WS client {websocket_client.remote_address}")
        except websockets.exceptions.ConnectionClosed:
            logger.warning(f"Failed to send initial robot list to {websocket_client.remote_address}: Connection closed.")
        except Exception as e:
            logger.error(f"Error sending initial robot list to {websocket_client.remote_address}: {e}", exc_info=True)

    def get_websocket_cors_headers(self, path: str, request_headers):
        # request_headers is of type websockets.datastructures.Headers
        # Default frontend origin for Vite dev environment
        frontend_origin = os.environ.get("FRONTEND_ORIGIN", "http://localhost:5173") 
        
        cors_headers_list = [
            ('Access-Control-Allow-Origin', frontend_origin)
        ]
        
        # If not allowing all origins, and a specific origin is set, 
        # we can also allow credentials.
        if frontend_origin != "*":
            cors_headers_list.append(('Access-Control-Allow-Credentials', 'true'))
            # You might also want to specify allowed methods and headers if your JS client sends them
            # during the upgrade request, though often not strictly needed for WS itself.
            cors_headers_list.append(('Access-Control-Allow-Methods', 'GET, OPTIONS'))
            cors_headers_list.append(('Access-Control-Allow-Headers', 'Content-Type, Authorization'))

        # Log the origin from the request headers for debugging purposes
        # origin_from_request = request_headers.get("Origin")
        # logger.debug(f"WebSocket handshake request from origin: {origin_from_request}. Allowed origin: {frontend_origin}. Path: {path}")
        logger.debug(f"Adding CORS headers for WebSocket handshake to {frontend_origin}: {cors_headers_list}")
        return cors_headers_list

    async def load_pid_config_from_file(self, target_robot_ip=None):
        pid_data_per_motor = {}
        try:
            with open(self.pid_config_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    parts = line.split(',')
                    if len(parts) == 4:  # motor_id,kp,ki,kd
                        try:
                            # Assuming motor_id in file can be "Motor1" or "1"
                            motor_id_str = parts[0].replace("Motor", "").strip()
                            motor_id = int(motor_id_str)
                            kp, ki, kd = map(float, parts[1:])
                            pid_data_per_motor[motor_id] = {"kp": kp, "ki": ki, "kd": kd}
                        except ValueError as e:
                            logger.warning(f"Skipping malformed PID entry in '{self.pid_config_file}': {line} - {e}")
                    else:
                        logger.warning(f"Skipping malformed line in PID config '{self.pid_config_file}': {line}")
        except FileNotFoundError:
            logger.warning(f"PID config file '{self.pid_config_file}' not found.")
            return None
        except Exception as e:
            logger.error(f"Error reading PID config file '{self.pid_config_file}': {e}")
            return None

        if not pid_data_per_motor:
            logger.warning(f"No valid PID data loaded from '{self.pid_config_file}'.")
            return None

        if target_robot_ip is not None:  # Explicitly check for None
            writer_to_use = None
            robot_alias_for_log = target_robot_ip
            unique_key_target = None

            # Find writer for the target_robot_ip directly from ConnectionManager
            conn = ConnectionManager.get_tcp_client_by_addr(target_robot_ip)
            if conn:
                try:
                    _, writer_to_use = conn
                except Exception:
                    writer_to_use = None
                robot_alias_for_log = target_robot_ip
            
            if writer_to_use:
                try:
                    logger.info(f"Sending PID configuration from '{self.pid_config_file}' to robot {robot_alias_for_log} ({target_robot_ip}).")
                    for motor_id, p_values in pid_data_per_motor.items():
                        pid_command_str = f"MOTOR:{motor_id} Kp:{p_values['kp']} Ki:{p_values['ki']} Kd:{p_values['kd']}\n"
                        writer_to_use.write(pid_command_str.encode('utf-8'))
                        await writer_to_use.drain()
                        logger.debug(f"Sent to {target_robot_ip}: {pid_command_str}")
                        await asyncio.sleep(0.05) # Small delay
                    logger.info(f"PID configuration from '{self.pid_config_file}' sent to robot {robot_alias_for_log} ({target_robot_ip}).")
                except Exception as e:
                    logger.error(f"Error sending PID config to robot {robot_alias_for_log} ({target_robot_ip}): {e}")
            else:
                logger.warning(f"Cannot send loaded PID to {target_robot_ip}: Robot not found or not connected.")
            return pid_data_per_motor 
        else:
            logger.info(f"PID configuration loaded from '{self.pid_config_file}' for caching: {pid_data_per_motor}")
            return pid_data_per_motor

    async def save_pid_config_to_file(self, pid_data_per_motor=None):
        # pid_data_per_motor: {1: {"kp": val, "ki": val, "kd": val}, 2: {...}}
        # Nếu không có pid_data_per_motor, sẽ lưu giá trị mặc định hoặc hiện tại (nếu cơ chế lưu trữ)
        try:
            with open(self.pid_config_file, "w") as f:
                if pid_data_per_motor:
                    for motor_num, pids in pid_data_per_motor.items():
                        f.write(f"Motor{motor_num}:{pids['kp']},{pids['ki']},{pids['kd']}\n")
                else: # Lưu giá trị mặc định nếu không có dữ liệu
                    for i in range(1, 4): # Giả sử 3 motor
                        f.write(f"Motor{i}:0.0,0.0,0.0\n")
            logger.info(f"PID configuration saved to {self.pid_config_file}")
            return True
        except Exception as e:
            logger.error(f"Error saving PID config: {e}")
            return False
    
    async def start(self):
        # Start the persistent OTA server
        if self.ota_port_arg is not None:
            asyncio.create_task(self.ota_connection.start_persistent_ota_server(self.ota_port_arg))
        else:
            logger.warning("OTA port not specified, OTA server will not start.")

        # Load PID configurations from file and cache them
        loaded_pids = await self.load_pid_config_from_file() # target_robot_ip is None, loads for caching
        if loaded_pids:
            self.pid_config_cache = loaded_pids
            # Log message is now part of load_pid_config_from_file when target_robot_ip is None
        else:
            logger.warning(f"Could not load and cache PID configuration from '{self.pid_config_file}'.")

        self.tcp_server = await asyncio.start_server(
            self.handle_tcp_client, '0.0.0.0', self.tcp_port
        )
        log_tcp.info(f"Control server started on 0.0.0.0:{self.tcp_port}")
        
        self.ws_server = await websockets.serve(
            self.handle_ws_client, 
            '0.0.0.0', 
            self.ws_port,
            #additional_headers=self.get_websocket_cors_headers
        )
        log_ws.info(f"WebSocket server started on 0.0.0.0:{self.ws_port}")
    
    async def handle_tcp_client(self, reader, writer):
        peername = writer.get_extra_info('peername')
        robot_ip_address = peername[0]
        robot_port = peername[1]
        unique_robot_key = f"{robot_ip_address}:{robot_port}"

        # Use robot IP as alias/identifier
        current_alias = robot_ip_address
        async with robot_alias_manager["lock"]:
            robot_alias_manager["ip_port_to_alias"][unique_robot_key] = current_alias
            robot_alias_manager["alias_to_ip_port"][current_alias] = unique_robot_key
            robot_alias_manager["ip_to_alias"][robot_ip_address] = current_alias
            robot_alias_manager["alias_to_ip"][current_alias] = robot_ip_address
        log_tcp.info(f"Connection from {robot_ip_address}:{robot_port} registered with alias(IP): {current_alias}")

        if not current_alias: # Should not happen if logic above is correct
            logger.error(f"Failed to assign or retrieve alias for {unique_robot_key}. Closing connection.")
            writer.close()
            await writer.wait_closed()
            return

        # Đăng ký TCP client với ConnectionManager
        ConnectionManager.set_tcp_client(
            robot_id=unique_robot_key,  # Sử dụng unique_robot_key làm ID trong ConnectionManager
            tcp_client=(reader, writer), 
            client_addr=(robot_ip_address, robot_port)
        )
        log_tcp.info(f"Client {current_alias} ({unique_robot_key}) registered with ConnectionManager.")

        log_tcp.info(f"Processing started for {current_alias} ({unique_robot_key}).")
        
        # Không gửi JSON ACK qua TCP cho robot. Firmware chỉ mong đợi lệnh plaintext
        # hoặc 'registration_response' sau khi gửi gói 'registration'.

        # Send cached PID config if available
        if self.pid_config_cache:
            log_tcp.info(f"Attempting to send cached PID config to newly connected robot {current_alias} ({robot_ip_address}).")
            try:
                if self.pid_config_cache: # Ensure cache is not empty
                    for motor_id, params in self.pid_config_cache.items():
                        pid_command_str = f"MOTOR:{motor_id} Kp:{params['kp']} Ki:{params['ki']} Kd:{params['kd']}\n"
                        writer.write(pid_command_str.encode('utf-8'))
                        await writer.drain()
                        log_tcp.debug(f"Sent cached PID to {current_alias}: {pid_command_str.strip()}")
                        await asyncio.sleep(0.05) # Small delay
                    log_tcp.info(f"Sent cached PID config to {current_alias} ({robot_ip_address}).")
                else:
                    log_tcp.info(f"PID cache for {current_alias} is empty, not sending.")
            except Exception as e:
                logger.error(f"Error sending cached PID config to {current_alias} ({robot_ip_address}): {e}")
        else:
            logger.info(f"No cached PID configuration to send to {current_alias} ({robot_ip_address}).")
            
        robot_announced_to_ui = False
        # Announce new/re-established robot to UI clients
        robot_announcement_payload = {
            "type": "available_robot_update",
            "action": "add", # or "update" if re-established
            "robot": {
                "ip": robot_ip_address,
                "alias": current_alias,
                "unique_key": unique_robot_key,
                "status": "connected" 
            },
            "timestamp": time.time()
        }
        await broadcast_to_all_ui(robot_announcement_payload)
        robot_announced_to_ui = True

        try:
            # Giai đoạn 1: Gửi xác nhận kết nối cho Robot (đã làm ở trên)
            # No longer specifically reading/logging an initial registration packet here.
            # The main loop will handle the first data packet received.
            
            # Giai đoạn 2: Vòng lặp xử lý dữ liệu chính
            # self.running is not defined in this class, assuming it's meant to be a global or instance variable for graceful shutdown.
            # For now, let's assume the loop runs until disconnection.
            while True: # Replace with self.running if defined elsewhere for graceful shutdown
                current_line_bytes = None
                try:
                    current_line_bytes = await asyncio.wait_for(reader.readline(), timeout=TCP_CLIENT_TIMEOUT_DEFAULT) # Use defined constant
                except asyncio.TimeoutError:
                    # Không đóng kết nối chỉ vì không có dữ liệu trong khoảng thời gian chờ.
                    # Giữ kết nối mở để robot có thể tiếp tục gửi khi có dữ liệu.
                    log_tcp.debug(f"No data from {current_alias} for {TCP_CLIENT_TIMEOUT_DEFAULT}s; keeping connection open.")
                    continue
                    
                if not current_line_bytes: 
                    log_tcp.info(f"Disconnected or read error: {current_alias} ({unique_robot_key}).")
                    break 
                
                raw_data_str = current_line_bytes.decode().strip()
                logger.debug(f"Data from {current_alias} ({robot_ip_address}) (Control): {raw_data_str}")

                try:
                    message_from_robot = json.loads(raw_data_str)
                    
                    # DEBUG LOGGING to see the raw parsed message before transformation
                    #logger.info(f"[PRE-TRANSFORM] Robot: {current_alias}, Type: '{message_from_robot.get('type')}', Keys: '{list(message_from_robot.keys())}', DataPreview: {str(message_from_robot.get('data'))[:100] if message_from_robot.get('data') else 'N/A'}")

                    transformed_message = transform_robot_message(message_from_robot)
                    
                    # If robot just sent registration, optionally send ACK based on env
                    if message_from_robot.get("type") == "registration":
                        ack_mode = os.environ.get("BRIDGE_REG_ACK_MODE", "none").lower()
                        if ack_mode == "plain":
                            try:
                                writer.write(b"registration_response\n")
                                await writer.drain()
                                logger.info(f"Sent plain 'registration_response' to {current_alias} as registration ACK (mode=plain).")
                            except Exception as e:
                                logger.error(f"Failed sending registration_response to {current_alias}: {e}")
                        else:
                            logger.info(f"Registration received from {current_alias}; ACK suppressed (mode={ack_mode}).")

                    # Populate/overwrite with correct IP and alias from the connection
                    transformed_message["robot_ip"] = robot_ip_address 
                    transformed_message["robot_alias"] = current_alias
                    
                    # Ensure the type from transformation is used for logging and broadcast
                    data_type_for_log_and_broadcast = transformed_message.get("type", "unknown_data")
                    
                    # Log the (potentially) transformed data
                    self.data_logger.log_data(unique_robot_key, data_type_for_log_and_broadcast, transformed_message)

                    # Broadcast the transformed message
                    await self.broadcast_to_subscribers(current_alias, transformed_message)

                    # Trajectory calculation logic (ensure _latest_encoder_data and _latest_imu_data are initialized in __init__)
                    # This part now relies on the transformed_message structure
                    msg_type = transformed_message.get("type")
                    if msg_type == "encoder_data":
                        # transformed_message for encoder_data is:
                        # {"type": "encoder_data", "robot_ip": ..., "robot_alias": ..., "timestamp": ..., "data": [rpm1, rpm2, rpm3]}
                        
                        encoder_rpms_list = transformed_message.get("data")
                        message_timestamp = transformed_message.get("timestamp")

                        if encoder_rpms_list is not None and message_timestamp is not None and isinstance(encoder_rpms_list, list) and len(encoder_rpms_list) >= 3:
                            # Cache latest encoder snapshot for robot_status
                            self._latest_encoder_data[unique_robot_key] = transformed_message
                            # Prepare payload for TrajectoryCalculator
                            # The TrajectoryCalculator now expects the 'data' field to contain the RPMs directly
                            # and 'timestamp' at the top level of the dict passed to it.
                            # message_from_robot already contains this structure if it's an encoder message.
                            # However, transform_robot_message puts rpms into transformed_message["data"]
                            # and timestamp into transformed_message["timestamp"]
                            
                            # We should pass the transformed_message directly if its 'data' field has the RPMs
                            # and it has a 'timestamp' field.
                            # The TrajectoryCalculator's update_encoder_data expects a dict like:
                            # {"timestamp": ..., "data": [rpm1, rpm2, rpm3]} (based on its internal parsing)

                            trajectory_update_result = self.trajectory_calculator.update_encoder_data(
                                unique_robot_key,
                                transformed_message # Pass the transformed_message which has "data": [rpms] and "timestamp"
                            )
                            
                            # logger.info(f"DEBUG: trajectory_update_result from calculator: {trajectory_update_result}")

                            if trajectory_update_result and isinstance(trajectory_update_result.get("position"), dict) and isinstance(trajectory_update_result.get("path"), list):
                                current_pose_data = trajectory_update_result["position"]
                                path_history_data = trajectory_update_result["path"]
                                trajectory_message_for_ws = {
                                    "type": "realtime_trajectory",
                                    "robot_ip": robot_ip_address ,
                                    "robot_alias": current_alias,
                                    "timestamp": time.time(), 
                                    "position": current_pose_data,
                                    "path": path_history_data
                                }
                                await self.broadcast_to_subscribers(current_alias, trajectory_message_for_ws)
                                robot_status_payload = self.build_robot_status_snapshot(unique_robot_key, current_alias, robot_ip_address)
                                await self.broadcast_to_subscribers(current_alias, robot_status_payload)
                            else:
                                # First encoder after connect initializes timestamp and returns None.
                                # Send a snapshot to let UI render immediately.
                                snap = self.trajectory_calculator.get_snapshot(unique_robot_key)
                                if isinstance(snap, dict) and isinstance(snap.get("position"), dict) and isinstance(snap.get("path"), list):
                                    fallback_payload = {
                                        "type": "realtime_trajectory",
                                        "robot_ip": robot_ip_address,
                                        "robot_alias": current_alias,
                                        "timestamp": time.time(),
                                        "position": snap["position"],
                                        "path": snap["path"]
                                    }
                                    await self.broadcast_to_subscribers(current_alias, fallback_payload)
                                    # Also provide status
                                    robot_status_payload = self.build_robot_status_snapshot(unique_robot_key, current_alias, robot_ip_address)
                                    await self.broadcast_to_subscribers(current_alias, robot_status_payload)
                                else:
                                    logger.warning(f"Skipping trajectory broadcast for {current_alias} due to invalid result from TrajectoryCalculator: {trajectory_update_result}")
                    
                    elif msg_type == "imu_data": 
                        # transformed_message for imu_data is:
                        # {"type": "imu_data", "robot_ip": ..., "robot_alias": ..., "timestamp": ..., "data": {"time":..., "euler":..., "quaternion":...}}
                        self._latest_imu_data[unique_robot_key] = transformed_message # Store the whole transformed payload
                        
                        # Update IMU data in the trajectory calculator immediately
                        imu_result = self.trajectory_calculator.update_imu_data(unique_robot_key, transformed_message.get("data", {}))
                        # Broadcast trajectory snapshot based on IMU to allow drawing even with IMU-only
                        if isinstance(imu_result, dict) and isinstance(imu_result.get("position"), dict) and isinstance(imu_result.get("path"), list):
                            imu_traj_payload = {
                                "type": "realtime_trajectory",
                                "robot_ip": robot_ip_address,
                                "robot_alias": current_alias,
                                "timestamp": time.time(),
                                "position": imu_result["position"],
                                "path": imu_result["path"],
                            }
                            await self.broadcast_to_subscribers(current_alias, imu_traj_payload)
                        # Always push robot_status on IMU update
                        robot_status_payload = self.build_robot_status_snapshot(unique_robot_key, current_alias, robot_ip_address)
                        await self.broadcast_to_subscribers(current_alias, robot_status_payload)

                    elif msg_type == "position_update":
                        # transformed_message for position_update is:
                        # {"type": "position_update", "data": {"x": float(m), "y": float(m), "theta": float(rad)}}
                        pos_data = transformed_message.get("data", {})
                        traj_res = self.trajectory_calculator.update_position_packet(unique_robot_key, pos_data)
                        if isinstance(traj_res, dict) and isinstance(traj_res.get("position"), dict) and isinstance(traj_res.get("path"), list):
                            traj_payload = {
                                "type": "realtime_trajectory",
                                "robot_ip": robot_ip_address,
                                "robot_alias": current_alias,
                                "timestamp": time.time(),
                                "position": traj_res["position"],
                                "path": traj_res["path"],
                            }
                            await self.broadcast_to_subscribers(current_alias, traj_payload)
                            robot_status_payload = self.build_robot_status_snapshot(unique_robot_key, current_alias, robot_ip_address)
                            await self.broadcast_to_subscribers(current_alias, robot_status_payload)

                except json.JSONDecodeError:
                    log_tcp.error(f"Invalid JSON from {current_alias} ({robot_ip_address}) (Control): {raw_data_str}")
                except Exception as e_proc_loop:
                    log_tcp.error(f"Error processing data from {current_alias} (Control): {e_proc_loop}", exc_info=True)
        
        except ConnectionResetError:
            log_tcp.warning(f"Connection reset by {current_alias} ({unique_robot_key}).")
        except Exception as e_main_handler:
            log_tcp.error(f"Unhandled error in TCP connection with {current_alias} ({unique_robot_key}): {str(e_main_handler)}", exc_info=True)
        
        finally:
            # ConnectionManager.remove_tcp_client(unique_robot_key) # Assuming manager handles this
            ConnectionManager.remove_tcp_client(unique_robot_key) # Corrected call
            data_logger.close_logs(unique_robot_key) 
            if unique_robot_key in self._latest_encoder_data: del self._latest_encoder_data[unique_robot_key]
            if unique_robot_key in self._latest_imu_data: del self._latest_imu_data[unique_robot_key]
            
            async with robot_alias_manager["lock"]:
                if unique_robot_key in robot_alias_manager["ip_port_to_alias"]:
                    alias_being_removed = robot_alias_manager["ip_port_to_alias"][unique_robot_key]
                    del robot_alias_manager["ip_port_to_alias"][unique_robot_key]
                    if robot_alias_manager.get("alias_to_ip_port", {}).get(alias_being_removed) == unique_robot_key:
                         del robot_alias_manager["alias_to_ip_port"][alias_being_removed]
                    
                    if robot_alias_manager.get("ip_to_alias", {}).get(robot_ip_address) == alias_being_removed:
                        if robot_ip_address in robot_alias_manager["ip_to_alias"]:
                            del robot_alias_manager["ip_to_alias"][robot_ip_address]
                        if alias_being_removed in robot_alias_manager["alias_to_ip"]:
                            del robot_alias_manager["alias_to_ip"][alias_being_removed]
                    
                    log_tcp.info(f"Cleaned up alias mappings for {alias_being_removed} ({unique_robot_key})")
                    current_alias = alias_being_removed 
                else:
                    log_tcp.warning(f"Attempted to clean up alias for {unique_robot_key} but it was not found in ip_port_to_alias. Current alias var: {current_alias}")

            if robot_announced_to_ui:
                robot_disconnect_payload = {
                    "type": "available_robot_update",
                    "action": "remove",
                    "robot": {
                        "ip": robot_ip_address,
                        "alias": current_alias,
                        "unique_key": unique_robot_key
                    },
                    "timestamp": time.time()
                }
                await broadcast_to_all_ui(robot_disconnect_payload)

            if writer and not writer.is_closing():
                writer.close()
                try:
                    await writer.wait_closed()
                except Exception as e:
                    logger.error(f"Error closing writer for {current_alias} ({robot_ip_address}) (Key: {unique_robot_key}): {str(e)}")
            log_tcp.info(f"Connection closed for {current_alias} ({robot_ip_address}) (Key: {unique_robot_key})")

    async def broadcast_to_subscribers(self, robot_alias_source, payload):
        if not isinstance(payload, dict) or "type" not in payload or payload.get("robot_alias") != robot_alias_source:
            logger.error(f"Invalid payload for broadcast. 'type' or 'robot_alias' mismatch/missing. Expected robot_alias: {robot_alias_source}, Payload: {payload}")
            return

        data_type_to_send = payload["type"]
        message_json = json.dumps(payload)
        
        active_websockets_map = {ws.remote_address: ws for ws in ui_websockets}
        clients_failed_to_send = set() # Store websocket objects that failed

        async with self.subscribers_lock:
            for client_addr, client_specific_subs in list(self.websocket_subscriptions.items()):
                ws_client = active_websockets_map.get(client_addr)
                if not ws_client:
                    # logger.debug(f"Client address {client_addr} in subscriptions but not in active ui_websockets map. Will be cleaned up if ws disconnected.")
                    continue

                sent_to_this_client_for_payload = False

                # 1. Check for specific robot subscription
                if robot_alias_source in client_specific_subs and \
                   data_type_to_send in client_specific_subs[robot_alias_source]:
                    try:
                        await ws_client.send(message_json)
                        sent_to_this_client_for_payload = True
                        # logger.debug(f"Sent {data_type_to_send} for {robot_alias_source} to WS client {client_addr}")
                    except websockets.exceptions.ConnectionClosed:
                        logger.warning(f"WS ConnectionClosed for client {client_addr} (specific sub) during broadcast. Marking for removal.")
                        clients_failed_to_send.add(ws_client)
                    except Exception as e:
                        logger.error(f"Error sending {data_type_to_send} (specific sub) to WS client {client_addr}: {e}")
                        clients_failed_to_send.add(ws_client)
                
                # 2. Check for GLOBAL subscription, if not already sent for this payload to this client
                if not sent_to_this_client_for_payload and \
                   self.GLOBAL_SUBSCRIPTION_KEY in client_specific_subs and \
                   data_type_to_send in client_specific_subs[self.GLOBAL_SUBSCRIPTION_KEY]:
                    try:
                        await ws_client.send(message_json)
                        # logger.debug(f"Sent {data_type_to_send} (GLOBAL sub) for {robot_alias_source} to WS client {client_addr}")
                    except websockets.exceptions.ConnectionClosed:
                        logger.warning(f"WS ConnectionClosed for client {client_addr} (global sub) during broadcast. Marking for removal.")
                        clients_failed_to_send.add(ws_client)
                    except Exception as e:
                        logger.error(f"Error sending {data_type_to_send} (global sub) to WS client {client_addr}: {e}")
                        clients_failed_to_send.add(ws_client)

        # Cleanup failed clients
        if clients_failed_to_send:
            async with self.subscribers_lock: # Lock for modifying self.websocket_subscriptions
                for ws_to_remove in clients_failed_to_send:
                    failed_client_addr = ws_to_remove.remote_address # Get address before potential errors
                    if ws_to_remove in ui_websockets:
                        ui_websockets.remove(ws_to_remove)
                    if failed_client_addr and failed_client_addr in self.websocket_subscriptions:
                        del self.websocket_subscriptions[failed_client_addr]
                    logger.info(f"Cleaned up WS client {failed_client_addr} from subscriptions and ui_websockets due to send failure.")

    async def handle_ws_client(self, websocket):
        path = "/"
        if hasattr(websocket, 'resource_name'):
            path = websocket.resource_name
        elif hasattr(websocket, 'path'):
            path = websocket.path
        elif hasattr(websocket, 'request_uri'):
            path = websocket.request_uri

        client_addr = websocket.remote_address
        ws_identifier = f"{client_addr[0]}:{client_addr[1]}" if client_addr else "UnknownWSClient"
        logger.info(f"WebSocket client connected: {ws_identifier} on path: {path}")
        
        ui_websockets.add(websocket)
        await self.send_connected_robots_list_to_client(websocket) # This line will now work

        try:
            async for message_str in websocket:
                try:
                    data = json.loads(message_str)
                    command = data.get("command")
                    msg_type = data.get("type") # For subscriptions or alternative command key
                    # Map common actions sent via 'type' to 'command' for compatibility with frontend
                    if not command and isinstance(msg_type, str) and msg_type in (
                        "upload_firmware_start",
                        "firmware_data_chunk",
                        "upload_firmware_end",
                        "get_firmware_version",
                        "upgrade_signal",
                    ):
                        command = msg_type
                    robot_alias = data.get("robot_alias")
                    if not robot_alias:
                        robot_alias = data.get("robot_ip")  # Allow using IP as alias for all commands

                    if command == "subscribe":
                        data_type_to_sub = msg_type
                        target_alias = robot_alias

                        if not data_type_to_sub:
                            logger.warning(f"WS ({ws_identifier}): 'subscribe' command missing 'type'. Payload: {data}")
                            await websocket.send(json.dumps({"type": "error", "command": command, "message": "Missing 'type' (data_type) for subscription"}))
                            continue
                        if not target_alias:
                            logger.warning(f"WS ({ws_identifier}): 'subscribe' command missing 'robot_alias'. Payload: {data}")
                            await websocket.send(json.dumps({"type": "error", "command": command, "message": "Missing 'robot_alias' for subscription"}))
                            continue
                        
                        # For 'subscribe', the entity key is the robot_alias
                        actual_subscription_entity_key = target_alias
                        display_target = target_alias
                        
                        # Verify alias exists
                        async with robot_alias_manager["lock"]:
                            if target_alias not in robot_alias_manager["alias_to_ip_port"]:
                                logger.warning(f"WS ({ws_identifier}): 'subscribe' command for unknown alias '{target_alias}'. Payload: {data}")
                                await websocket.send(json.dumps({"type": "error", "command": command, "message": f"Unknown robot_alias '{target_alias}' for subscription."}))
                                continue

                        async with self.subscribers_lock:
                            if client_addr not in self.websocket_subscriptions:
                                self.websocket_subscriptions[client_addr] = {}
                            if actual_subscription_entity_key not in self.websocket_subscriptions[client_addr]:
                                self.websocket_subscriptions[client_addr][actual_subscription_entity_key] = set()
                            self.websocket_subscriptions[client_addr][actual_subscription_entity_key].add(data_type_to_sub)
                        
                        logger.info(f"{ws_identifier} subscribed to '{data_type_to_sub}' for '{display_target}' (Key: {actual_subscription_entity_key}) using 'subscribe' command.")
                        await websocket.send(json.dumps({"type": "ack", "command": command, "status": "success", "data_type": data_type_to_sub, "subscribed_key": actual_subscription_entity_key, "robot_alias": target_alias}))

                        # Immediately send snapshot for realtime_trajectory if requested
                        if data_type_to_sub == "realtime_trajectory":
                            # Resolve unique key from alias (IP)
                            unique_robot_key = await self.get_robot_key_from_alias(target_alias)
                            try:
                                # Try to get current snapshot if robot was seen before
                                snap = None
                                if unique_robot_key:
                                    snap = self.trajectory_calculator.get_snapshot(unique_robot_key)

                                # If no snapshot exists yet, send a default empty snapshot
                                if not snap:
                                    snap = {
                                        "position": {"x": 0.0, "y": 0.0, "theta": 0.0},
                                        "path": []
                                    }

                                snapshot_payload = {
                                    "type": "realtime_trajectory",
                                    "robot_ip": target_alias,
                                    "robot_alias": target_alias,
                                    "timestamp": time.time(),
                                    "position": snap["position"],
                                    "path": snap["path"],
                                }
                                await websocket.send(json.dumps(snapshot_payload))
                                logger.info(f"Sent initial trajectory snapshot to {ws_identifier} for {target_alias} with {len(snap['path'])} points.")
                            except Exception as e:
                                logger.error(f"Error sending trajectory snapshot to {ws_identifier}: {e}")

                    elif command == "unsubscribe":
                        data_type_to_unsub = msg_type
                        target_alias = robot_alias

                        if not data_type_to_unsub:
                            logger.warning(f"WS ({ws_identifier}): 'unsubscribe' command missing 'type'. Payload: {data}")
                            await websocket.send(json.dumps({"type": "error", "command": command, "message": "Missing 'type' (data_type) for unsubscription"}))
                            continue
                        if not target_alias:
                            logger.warning(f"WS ({ws_identifier}): 'unsubscribe' command missing 'robot_alias'. Payload: {data}")
                            await websocket.send(json.dumps({"type": "error", "command": command, "message": "Missing 'robot_alias' for unsubscription"}))
                            continue
                        
                        unsubscription_entity_key = target_alias
                        display_target = target_alias

                        async with self.subscribers_lock:
                            if client_addr in self.websocket_subscriptions and \
                               unsubscription_entity_key in self.websocket_subscriptions[client_addr]:
                                self.websocket_subscriptions[client_addr][unsubscription_entity_key].discard(data_type_to_unsub)
                                if not self.websocket_subscriptions[client_addr][unsubscription_entity_key]: # If set is empty
                                    del self.websocket_subscriptions[client_addr][unsubscription_entity_key]
                                if not self.websocket_subscriptions[client_addr]: # If dict for client_addr is empty
                                    del self.websocket_subscriptions[client_addr]
                                logger.info(f"{ws_identifier} unsubscribed from '{data_type_to_unsub}' for '{display_target}' (Key: {unsubscription_entity_key}) using 'unsubscribe' command.")
                                await websocket.send(json.dumps({"type": "ack", "command": command, "status": "success", "data_type": data_type_to_unsub, "unsubscribed_key": unsubscription_entity_key, "robot_alias": target_alias}))
                            else:
                                logger.info(f"{ws_identifier} attempted to unsubscribe from '{data_type_to_unsub}' for '{display_target}' but no active subscription found.")
                                await websocket.send(json.dumps({"type": "ack", "command": command, "status": "not_subscribed", "data_type": data_type_to_unsub, "robot_alias": target_alias}))
                    
                    elif command == "clear_trajectory": # Handle new command
                        if robot_alias:
                            unique_robot_key = await self.get_robot_key_from_alias(robot_alias)
                            if unique_robot_key and self.trajectory_calculator:
                                logger.info(f"Received 'clear_trajectory' command for {robot_alias} (key: {unique_robot_key}) from {ws_identifier}")
                                trajectory_update_result = self.trajectory_calculator.clear_path_history(unique_robot_key)
                                
                                # Broadcast the cleared state immediately to all subscribers of this robot
                                if trajectory_update_result:
                                    # Determine robot_ip for the message (you might need a helper for this)
                                    robot_ip_for_message = "N/A"
                                    async with robot_alias_manager["lock"]:
                                        ip_tmp = robot_alias_manager["alias_to_ip"].get(robot_alias)
                                        if ip_tmp:
                                            robot_ip_for_message = ip_tmp
                                    
                                    cleared_trajectory_message = {
                                        "type": "realtime_trajectory",
                                        "robot_ip": robot_ip_for_message,
                                        "robot_alias": robot_alias,
                                        "timestamp": time.time(),
                                        "position": trajectory_update_result["position"],
                                        "path": trajectory_update_result["path"] # This will be an empty list
                                    }
                                    await self.broadcast_to_subscribers(robot_alias, cleared_trajectory_message)
                                    logger.info(f"Broadcasted cleared trajectory for {robot_alias}")
                            else:
                                logger.warning(f"Could not clear trajectory for {robot_alias}: unknown alias or no trajectory calculator.")
                        else:
                            logger.warning(f"Received 'clear_trajectory' command without robot_alias from {ws_identifier}")
                    
                    # === Plaintext control commands mapping ===
                    elif command == "start_robot":
                        if robot_alias:
                            ok = await self.send_text_command_by_alias(robot_alias, "Start Robot")
                            await websocket.send(json.dumps({"type":"ack","command":command,"status":"success" if ok else "failed","robot_alias": robot_alias}))
                        else:
                            await websocket.send(json.dumps({"type":"error","command":command,"message":"Missing robot_alias"}))
                    elif command == "start_position":
                        if robot_alias:
                            ok = await self.send_text_command_by_alias(robot_alias, "Start Position")
                            await websocket.send(json.dumps({"type":"ack","command":command,"status":"success" if ok else "failed","robot_alias": robot_alias}))
                        else:
                            await websocket.send(json.dumps({"type":"error","command":command,"message":"Missing robot_alias"}))
                    elif command == "emergency_stop":
                        if robot_alias:
                            ok = await self.send_text_command_by_alias(robot_alias, "EMERGENCY_STOP")
                            await websocket.send(json.dumps({"type":"ack","command":command,"status":"success" if ok else "failed","robot_alias": robot_alias}))
                        else:
                            await websocket.send(json.dumps({"type":"error","command":command,"message":"Missing robot_alias"}))
                    elif command == "vector_control":
                        # expects: dot_x, dot_y, dot_theta, stop_time
                        try:
                            dot_x = float(data.get("dot_x", 0))
                            dot_y = float(data.get("dot_y", 0))
                            dot_theta = float(data.get("dot_theta", 0))
                            stop_time = int(data.get("stop_time", 0))
                            if not robot_alias:
                                raise ValueError("Missing robot_alias")
                            cmd_str = f"dot_x:{dot_x} dot_y:{dot_y} dot_theta:{dot_theta} stop_time:{stop_time}"
                            ok = await self.send_text_command_by_alias(robot_alias, cmd_str)
                            # Update trajectory from control command too (in case no encoder yet)
                            unique_key_for_ctrl = await self.get_robot_key_from_alias(robot_alias)
                            if unique_key_for_ctrl:
                                now_t = time.time()
                                # First tick: registers/start integration window
                                ctrl_result = self.trajectory_calculator.update_control_command(
                                    unique_key_for_ctrl,
                                    vx_robot=dot_x,
                                    vy_robot=dot_y,
                                    omega=dot_theta,
                                    timestamp=now_t,
                                    override=True,
                                )
                                last_result = ctrl_result
                                # If a duration is provided for a one-shot step, integrate once more at now+dt
                                if stop_time and (abs(dot_x) > 1e-6 or abs(dot_y) > 1e-6 or abs(dot_theta) > 1e-6):
                                    dt = min(max(stop_time / 1000.0, 0.05), 0.35)  # clamp between 50ms and 350ms
                                    last_result = self.trajectory_calculator.update_control_command(
                                        unique_key_for_ctrl,
                                        vx_robot=dot_x,
                                        vy_robot=dot_y,
                                        omega=dot_theta,
                                        timestamp=now_t + dt,
                                        override=True,
                                    )
                                if isinstance(last_result, dict) and isinstance(last_result.get("position"), dict) and isinstance(last_result.get("path"), list):
                                    ctrl_traj_payload = {
                                        "type": "realtime_trajectory",
                                        "robot_ip": robot_alias,
                                        "robot_alias": robot_alias,
                                        "timestamp": time.time(),
                                        "position": last_result["position"],
                                        "path": last_result["path"],
                                    }
                                    await self.broadcast_to_subscribers(robot_alias, ctrl_traj_payload)
                                    # And status
                                    robot_status_payload = self.build_robot_status_snapshot(unique_key_for_ctrl, robot_alias, robot_alias)
                                    await self.broadcast_to_subscribers(robot_alias, robot_status_payload)
                            await websocket.send(json.dumps({"type":"ack","command":command,"status":"success" if ok else "failed","robot_alias": robot_alias}))
                        except Exception as e:
                            await websocket.send(json.dumps({"type":"error","command":command,"message": str(e)}))
                    elif command == "position_control":
                        # expects: x, y, vel
                        try:
                            pos_x = float(data.get("x"))
                            pos_y = float(data.get("y"))
                            vel = float(data.get("vel"))
                            if not robot_alias:
                                raise ValueError("Missing robot_alias")
                            cmd_str = f"x:{pos_x} y:{pos_y} vel:{vel}"
                            ok = await self.send_text_command_by_alias(robot_alias, cmd_str)
                            await websocket.send(json.dumps({"type":"ack","command":command,"status":"success" if ok else "failed","robot_alias": robot_alias}))
                        except Exception as e:
                            await websocket.send(json.dumps({"type":"error","command":command,"message": str(e)}))
                    elif command == "motor_speed":
                        # expects: motor_id (1..3), speed (int)
                        try:
                            motor_id = int(data.get("motor_id"))
                            motor_speed = int(data.get("speed"))
                            if not robot_alias:
                                raise ValueError("Missing robot_alias")
                            cmd_str = f"MOTOR_{motor_id}_SPEED:{motor_speed};"
                            ok = await self.send_text_command_by_alias(robot_alias, cmd_str)
                            await websocket.send(json.dumps({"type":"ack","command":command,"status":"success" if ok else "failed","robot_alias": robot_alias}))
                        except Exception as e:
                            await websocket.send(json.dumps({"type":"error","command":command,"message": str(e)}))
                    elif command == "set_pid":
                        # expects: motor_id, kp, ki, kd
                        try:
                            motor_id = int(data.get("motor_id"))
                            kp = float(data.get("kp"))
                            ki = float(data.get("ki"))
                            kd = float(data.get("kd"))
                            if not robot_alias:
                                raise ValueError("Missing robot_alias")
                            cmd_str = f"MOTOR:{motor_id} Kp:{kp} Ki:{ki} Kd:{kd}"
                            ok = await self.send_text_command_by_alias(robot_alias, cmd_str)
                            await websocket.send(json.dumps({"type":"ack","command":command,"status":"success" if ok else "failed","robot_alias": robot_alias}))
                        except Exception as e:
                            await websocket.send(json.dumps({"type":"error","command":command,"message": str(e)}))
                    elif command == "upload_firmware_start":
                        try:
                            target_ip = data.get("robot_ip")
                            filename = data.get("filename")
                            filesize = int(data.get("filesize", 0))
                            if not target_ip or not filename or filesize <= 0:
                                raise ValueError("Missing robot_ip, filename or invalid filesize")
                            self.fw_upload_mgr.start(target_ip, filename, filesize)
                            await websocket.send(json.dumps({
                                "type": "firmware_status",
                                "status": "ok",
                                "robot_ip": target_ip,
                                "message": "Upload initialized"
                            }))
                        except Exception as e:
                            await websocket.send(json.dumps({
                                "type": "firmware_status",
                                "status": "error",
                                "message": str(e)
                            }))
                    elif command == "firmware_data_chunk":
                        try:
                            target_ip = data.get("robot_ip")
                            b64_chunk = data.get("data")
                            chunk_index = int(data.get("chunk_index", 0))
                            total_chunks = int(data.get("total_chunks", 0))
                            if not target_ip or not b64_chunk:
                                raise ValueError("Missing robot_ip or data chunk")
                            received = self.fw_upload_mgr.add_chunk(target_ip, b64_chunk)
                            await websocket.send(json.dumps({
                                "type": "firmware_chunk_ack",
                                "robot_ip": target_ip,
                                "chunk_index": chunk_index,
                                "total_chunks": total_chunks,
                                "received": received
                            }))
                        except Exception as e:
                            await websocket.send(json.dumps({
                                "type": "firmware_status",
                                "status": "error",
                                "message": str(e)
                            }))
                    elif command == "upload_firmware_end":
                        try:
                            target_ip = data.get("robot_ip")
                            if not target_ip:
                                raise ValueError("Missing robot_ip")
                            saved_path = self.fw_upload_mgr.finish(target_ip)
                            if not saved_path:
                                raise RuntimeError("Firmware aggregation failed or size mismatch")
                            # Prepare OTA server to send to this IP on next OTA connection (multi-robot aware)
                            self.ota_connection.set_prepared_firmware_for_robot(target_ip, saved_path)
                            await websocket.send(json.dumps({
                                "type": "firmware_prepared_for_ota",
                                "robot_ip": target_ip,
                                "ota_port": self.ota_port_arg,
                                "firmware_size": os.path.getsize(saved_path)
                            }))
                        except Exception as e:
                            await websocket.send(json.dumps({
                                "type": "firmware_status",
                                "status": "error",
                                "message": str(e)
                            }))
                    elif command == "load_pid_config":
                        # expects: robot_ip (preferred). Fall back to robot_alias if provided
                        try:
                            target_ip = data.get("robot_ip") or data.get("robot_alias")
                            if not target_ip:
                                raise ValueError("Missing robot_ip")
                            loaded = await self.load_pid_config_from_file(target_ip)
                            if loaded:
                                await websocket.send(json.dumps({
                                    "type": "pid_config_response",
                                    "status": "loaded",
                                    "pids": loaded,
                                    "robot_ip": target_ip
                                }))
                            else:
                                await websocket.send(json.dumps({
                                    "type": "pid_config_response",
                                    "status": "error",
                                    "message": f"Could not load PID config from '{self.pid_config_file}'",
                                    "robot_ip": target_ip
                                }))
                        except Exception as e:
                            await websocket.send(json.dumps({
                                "type": "pid_config_response",
                                "status": "error",
                                "message": str(e)
                            }))
                    elif command in ("upgrade", "upgrade_signal"):
                        # Trigger OTA switch on device to connect to OTA server
                        if robot_alias:
                            mode_override = data.get("mode")
                            ok = await self.send_upgrade_command_by_alias(robot_alias, mode_override)
                            await websocket.send(json.dumps({
                                "type":"ack",
                                "command":command,
                                "status":"success" if ok else "failed",
                                "robot_alias": robot_alias
                            }))
                            await broadcast_to_all_ui({
                                "type": "firmware_status",
                                "status": "sent" if ok else "failed",
                                "robot_ip": robot_alias,
                                "message": (f"Upgrade command sent (mode={(mode_override or os.environ.get('BRIDGE_UPGRADE_MODE', 'raw')).lower()})" if ok else "Upgrade command failed")
                            })
                        else:
                            await websocket.send(json.dumps({"type":"error","command":command,"message":"Missing robot_alias"}))
                    elif command == "get_robot_status":
                        try:
                            target_ip = data.get("robot_ip") or data.get("robot_alias")
                            if not target_ip:
                                raise ValueError("Missing robot_ip")
                            unique_key = await self.get_robot_key_from_alias(target_ip)
                            if not unique_key:
                                raise ValueError("Robot not connected")
                            payload = self.build_robot_status_snapshot(unique_key, target_ip, target_ip)
                            await websocket.send(json.dumps(payload))
                        except Exception as e:
                            await websocket.send(json.dumps({"type": "error", "command": command, "message": str(e)}))
                    
                    # ... (other potential commands like 'get_pid', 'set_pid', etc.) ...

                except json.JSONDecodeError:
                    logger.error(f"Failed to parse JSON from {ws_identifier}: {message_str}")
                except Exception as e:
                    logger.error(f"Error processing message from {ws_identifier} ('{message_str}'): {e}", exc_info=True)
        finally:
            logger.info(f"Cleaning up WebSocket client: {ws_identifier}")
            ui_websockets.discard(websocket) # Remove from global set of active UI websockets
            
            # Clean up this client's subscriptions from self.websocket_subscriptions
            async with self.subscribers_lock:
                if client_addr in self.websocket_subscriptions:
                    del self.websocket_subscriptions[client_addr]
            
            # Removed: Old cleanup logic for websocket.robot_data_subscriptions and global `subscribers`
            # async with self.subscribers_lock:
            #     for sub_key, subscribed_types in list(websocket.robot_data_subscriptions.items()): # Iterate over a copy
            #         for data_type in list(subscribed_types): # Iterate over a copy
            #             if data_type in subscribers and sub_key in subscribers[data_type]:
            #                 subscribers[data_type][sub_key].discard(websocket)
            #                 if not subscribers[data_type][sub_key]: # If set is empty
            #                     del subscribers[data_type][sub_key]
            #                 if not subscribers[data_type]: # If dict for data_type is empty
            #                     del subscribers[data_type]
            logger.info(f"WebSocket client {ws_identifier} removed from all subscriptions in DirectBridge.")

def transform_robot_message(message_dict: dict) -> dict:
    """
    Transforms incoming robot messages (already parsed as dict) to a standardized format
    for the frontend.
    The robot_ip and robot_alias fields will be correctly populated by the caller (handle_tcp_client).
    """
    transformed_payload = {
        "robot_ip": "PENDING_IP", 
        "robot_alias": "PENDING_ALIAS", 
        "timestamp": message_dict.get("timestamp", time.time()) 
    }

    original_type = message_dict.get("type")
    robot_reported_id = message_dict.get("id")

    # 1. Identify IMU data (ESP32 sends type: "bno055")
    if original_type == "bno055" and "data" in message_dict and isinstance(message_dict["data"], dict):
        transformed_payload["type"] = "imu_data"
        imu_data_content = message_dict["data"]
        transformed_payload["data"] = {
            "time": imu_data_content.get("time"), 
            "euler": imu_data_content.get("euler"),
            "quaternion": imu_data_content.get("quaternion"),
        }
        if robot_reported_id:
             transformed_payload["data"]["robot_reported_id"] = robot_reported_id

    # 2. Identify Encoder data (ESP32 sends type: "encoder") - MOVED UP FOR PRIORITY
    elif original_type == "encoder" and "data" in message_dict and isinstance(message_dict.get("data"), list):
        transformed_payload["type"] = "encoder_data"
        transformed_payload["data"] = message_dict["data"] 
        if robot_reported_id:
             transformed_payload["robot_reported_id"] = robot_reported_id

    # 3. Identify Position data (ESP32 sends type: "position")
    elif original_type == "position" and "data" in message_dict and isinstance(message_dict["data"], dict):
        # Expect firmware shape:
        # {
        #   "id":"<ID>",
        #   "type":"position",
        #   "data":{
        #     "position":[x,y,thetaDeg],
        #     "velocity":[vx,vy],
        #     "acceleration":[ax,ay,az],
        #     "vel_control":[vcx,vcy]
        #   }
        # }
        transformed_payload["type"] = "position_update"
        pos_arr = message_dict["data"].get("position")
        theta_rad = 0.0
        x = 0.0
        y = 0.0
        if isinstance(pos_arr, list) and len(pos_arr) >= 3:
            try:
                x = float(pos_arr[0])
                y = float(pos_arr[1])
                theta_deg = float(pos_arr[2])
                theta_rad = theta_deg * math.pi / 180.0
            except Exception:
                pass
        transformed_payload["data"] = {"x": x, "y": y, "theta": theta_rad}
        if robot_reported_id:
            transformed_payload["robot_reported_id"] = robot_reported_id

    # 4. Identify Log data (ESP32 / test.py sends type: "log_data")
    elif original_type == "log": # MODIFIED from "log" to "log_data"
        transformed_payload["type"] = "log" # Keep the type as log_data for frontend
        transformed_payload["message"] = message_dict.get("message")
        transformed_payload["level"] = message_dict.get("level", "debug") # Default to "info" if not present
        if robot_reported_id:
             transformed_payload["robot_reported_id"] = robot_reported_id
    
    # 5. Identify Registration message (Đã có từ trước, giữ nguyên)
    elif original_type == "registration" and "capabilities" in message_dict:
        transformed_payload["type"] = "registration"
        transformed_payload["data"] = {
            "capabilities": message_dict.get("capabilities"),
            "robot_reported_id": message_dict.get("robot_id") or robot_reported_id,
        }
        if "robot_id" in message_dict: 
            transformed_payload["robot_reported_id_explicit"] = message_dict["robot_id"]
        elif robot_reported_id and "robot_id" not in message_dict.get("data", {}): 
             transformed_payload["robot_reported_id_explicit"] = robot_reported_id
            
    # 6. Handle other messages that have a 'type' field (generic passthrough) - THIS SHOULD BE CHECKED LATER
    elif original_type: # This might catch specific types if not handled above explicitly
        transformed_payload["type"] = f"generic_{original_type}"
        transformed_payload["data"] = message_dict.copy() 
    
    # 7. Fallback for completely unknown structures
    else:
        transformed_payload["type"] = "unknown_json_data"
        transformed_payload["data"] = message_dict.copy()
        logger.warning(f"Unknown JSON structure received (no type field), classifying as 'unknown_json_data': {message_dict}")

    return transformed_payload

async def main_bridge_runner(): 
    parser = argparse.ArgumentParser(description="DirectBridge TCP-WebSocket bridge")
    parser.add_argument("--tcp-port", type=int, default=int(os.environ.get("TCP_PORT", TCP_PORT_DEFAULT)), help=f"TCP server port (default: {TCP_PORT_DEFAULT})")
    parser.add_argument("--ws-port", type=int, default=int(os.environ.get("WS_BRIDGE_PORT", WS_PORT_DEFAULT)), help=f"WebSocket server port (default: {WS_PORT_DEFAULT})")
    parser.add_argument("--ota-port", type=int, default=int(os.environ.get("OTA_PORT", OTA_PORT_DEFAULT)), help=f"TCP server port for OTA (default: {OTA_PORT_DEFAULT})")
    parser.add_argument("--log-level", type=str, default=os.environ.get("LOG_LEVEL", LOG_LEVEL_DEFAULT).upper(), help=f"Logging level (default: {LOG_LEVEL_DEFAULT})")
    parser.add_argument("--pid-config", type=str, default=os.environ.get("PID_CONFIG_FILE", PID_CONFIG_FILE_DEFAULT), help=f"PID config file (default: {PID_CONFIG_FILE_DEFAULT})")
    args = parser.parse_args()
    
    logging.getLogger().setLevel(getattr(logging, args.log_level.upper(), logging.INFO))
    logger.info(f"Logger set to level: {args.log_level.upper()}")

    bridge = DirectBridge(tcp_port=args.tcp_port, ws_port=args.ws_port, pid_config_file_path=args.pid_config)
    bridge.ota_port_arg = args.ota_port
    
    await bridge.start()
    logger.info(f"DirectBridge running. Control TCP on {args.tcp_port}, WebSocket on {args.ws_port}, OTA on {args.ota_port}.")
    try:
        while True:
            await asyncio.sleep(3600)
    except KeyboardInterrupt:
        logger.info("DirectBridge stopping...")
    finally:
        data_logger.close_logs()
        if hasattr(bridge, 'ota_connection') and bridge.ota_connection.ota_server_instance: # Check attribute before calling
            await bridge.ota_connection.stop_ota_server()
        logger.info("DirectBridge stopped.")

if __name__ == "__main__":
    try:
        asyncio.run(main_bridge_runner())
    except KeyboardInterrupt:
        logger.info("Application terminated by user.")