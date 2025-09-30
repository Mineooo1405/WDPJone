import sys
import os
import asyncio
import websockets
import json
import logging
try:
    # Prefer package import when used as a package
    from back.connection_manager import ConnectionManager
except Exception:
    # Fallback to top-level module import for direct script execution
    from connection_manager import ConnectionManager
import time
import argparse
import math
from dotenv import load_dotenv
import base64
import io
# Optional heavy dependencies (used for map/occupancy features). We guard-import to avoid editor import errors if
# the active interpreter isn't set yet. At runtime, map features will raise a clear error if these are missing.
try:
    import numpy as np  # type: ignore
except Exception:  # ImportError or environment issues
    np = None  # type: ignore
try:
    from PIL import Image  # type: ignore
except Exception:
    Image = None  # type: ignore
try:
    # Local lightweight DB (SQLite by default)
    from database import Database  # type: ignore
except Exception:
    Database = None  # type: ignore

# Load environment variables from .env file and optional env_config.txt
load_dotenv()  # .env in current working dir
try:
    # Also support env_config.txt located alongside this script
    this_dir = os.path.dirname(os.path.abspath(__file__))
    env_cfg_path = os.path.join(this_dir, 'env_config.txt')
    if os.path.exists(env_cfg_path):
        load_dotenv(env_cfg_path, override=True)
except Exception:
    # Non-fatal if unable to load extra env config
    pass

# Centralized backend settings (fills os.environ defaults too)
try:
    # Prefer package-style import
    from back.config import settings  # type: ignore
except Exception:
    try:
        # Fallback for direct execution
        from config import settings  # type: ignore
    except Exception:
        settings = None  # Optional; module will continue using os.environ fallbacks

# --- Configuration from Environment Variables with Fallbacks ---
TCP_PORT_DEFAULT = 12346
WS_PORT_DEFAULT = 9003
OTA_PORT_DEFAULT = 12345  # Deprecated in RasPi-only mode
LOG_LEVEL_DEFAULT = "INFO"
LOG_DIRECTORY_DEFAULT = "logs/bridge_logs"
MAX_TRAJECTORY_POINTS_DEFAULT = 1000
WHEEL_RADIUS_DEFAULT = 0.0325
ROBOT_BASE_DISTANCE_L_DEFAULT = 0.1
# For mecanum, you can override via env:
#  - WHEEL_RADIUS (meters)
#  - MECANUM_LX (half width, meters)
#  - MECANUM_LY (half length, meters)
MECANUM_LX_DEFAULT = 0.1  # half-width (m) from center to wheel along x
MECANUM_LY_DEFAULT = 0.1  # half-length (m) from center to wheel along y
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
    if ui_websockets:  # Check if there are any UI clients
        message_json = json.dumps(message_payload)
        # Snapshot to avoid concurrent set mutation and index-based lookups
        recipients = list(ui_websockets)
        results = await asyncio.gather(*[ws.send(message_json) for ws in recipients], return_exceptions=True)
        for ws, result in zip(recipients, results):
            if isinstance(result, Exception):
                logger.error(f"Error sending to UI websocket {getattr(ws, 'remote_address', None)}: {result}. Removing.")
                try:
                    ui_websockets.discard(ws)
                except Exception:
                    pass
                
# --- TrajectoryCalculator class ---
class TrajectoryCalculator: 
    def __init__(self):
        self.robot_data = {}  # Keyed by unique_robot_key (ip:port or alias/IP)
        self.CONTROL_SUPPRESS_WINDOW = 0.5  # seconds: if encoder seen recently, skip control integration
        # Per-robot type: 'omni' (3-wheel) | 'mecanum' (4-wheel). Default 'omni'
        self.robot_types = {}

    def set_robot_type(self, unique_robot_key: str, robot_type: str):
        if robot_type not in ("omni", "mecanum"):
            return
        self.robot_types[unique_robot_key] = robot_type

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
        elif "euler" in imu_data and isinstance(imu_data.get("euler"), list) and len(imu_data["euler"]) == 3:
            # Many firmwares (e.g., BNO055) send Euler as [heading(yaw), pitch, roll]
            # Make this configurable via IMU_EULER_YAW_INDEX (default 0)
            try:
                yaw_index = int(os.environ.get("IMU_EULER_YAW_INDEX", "0"))
            except Exception:
                yaw_index = 0
            yaw_index = 0 if yaw_index not in (0, 1, 2) else yaw_index
            yaw = imu_data["euler"][yaw_index]
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
            # Reduced noise: do not warn repeatedly when IMU absent
            # logger.warning(f"IMU data not available for {unique_robot_key}. Falling back to odometry-only heading.")
            robot_state["imu_missing_warned"] = True

        timestamp_encoder = encoder_data.get("timestamp", time.time())
        # The 'data' key in encoder_data from transform_robot_message contains the RPM list
        rpms = encoder_data.get("data", []) # Use "data" as per transform_robot_message

        if not (isinstance(rpms, list) and len(rpms) >= 3):  # minimal sanity check
            logger.warning(f"Invalid RPM data for {unique_robot_key}: {rpms}. Expected list of >=3 numbers in 'data' field. Returning current state.")
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
        # Determine kinematic model by robot type
        robot_type = self.robot_types.get(unique_robot_key, "omni")
        # Heuristic: if we receive 4+ wheel RPMs but type not set to mecanum, assume mecanum
        # to avoid mis-integration when UI hasn't sent set_robot_type yet.
        if robot_type != "mecanum" and isinstance(rpms, list) and len(rpms) >= 4:
            robot_type = "mecanum"
        vx_robot = 0.0
        vy_robot = 0.0
        omega_body = 0.0

        if robot_type == "mecanum" and len(rpms) >= 4:
            # Convert to rad/s
            w1, w2, w3, w4 = [self._rpm_to_omega(rpm) for rpm in rpms[:4]]
            r = float(os.environ.get("WHEEL_RADIUS", WHEEL_RADIUS_DEFAULT))
            # Determine geometry for omega calculation (Lsum = lx + ly)
            Lsum_env = os.environ.get("MECANUM_LSUM")
            if Lsum_env is not None:
                try:
                    Lsum = float(Lsum_env)
                except Exception:
                    Lsum = MECANUM_LX_DEFAULT + MECANUM_LY_DEFAULT
            else:
                try:
                    lx = float(os.environ.get("MECANUM_LX", MECANUM_LX_DEFAULT))
                    ly = float(os.environ.get("MECANUM_LY", MECANUM_LY_DEFAULT))
                    Lsum = lx + ly
                except Exception:
                    Lsum = MECANUM_LX_DEFAULT + MECANUM_LY_DEFAULT
                # If ROBOT_RADIUS provided and lx/ly not explicitly set, prefer Lsum = 2*ROBOT_RADIUS
                try:
                    if os.environ.get("MECANUM_LX") is None and os.environ.get("MECANUM_LY") is None:
                        robot_radius = float(os.environ.get("ROBOT_RADIUS", "nan"))
                        if robot_radius == robot_radius:  # not NaN
                            Lsum = 2.0 * robot_radius
                except Exception:
                    pass
            if Lsum == 0:
                Lsum = 1e-6

            # Standard mecanum forward kinematics (wheel order: FL, FR, RL, RR)
            vx_robot = (r / 4.0) * (w1 + w2 + w3 + w4)
            vy_robot = (r / 4.0) * (-w1 + w2 + w3 - w4)
            omega_body = (r / (4.0 * Lsum)) * (-w1 + w2 - w3 + w4)

            # Optional sign adjustments from env to match roller orientation/wiring
            try:
                vy_sign = float(os.environ.get("MECANUM_VY_SIGN", "1.0"))
            except Exception:
                vy_sign = 1.0
            try:
                omega_sign = float(os.environ.get("MECANUM_OMEGA_SIGN", "1.0"))
            except Exception:
                omega_sign = 1.0
            vy_robot *= vy_sign
            omega_body *= omega_sign
        else:
            # 3-wheel omni (legacy): simple average forward velocity, no lateral velocity modeled
            omega_m1, omega_m2, omega_m3 = [self._rpm_to_omega(rpm) for rpm in rpms[:3]]
            vx_robot = WHEEL_RADIUS_DEFAULT * (omega_m1 + omega_m2 + omega_m3) / 3.0
            vy_robot = 0.0

        # Transform robot-frame velocities to world-frame using standard rotation
        theta = robot_state["theta"]
        cos_h = math.cos(theta)
        sin_h = math.sin(theta)
        vx_world = vx_robot * cos_h - vy_robot * sin_h
        vy_world = vx_robot * sin_h + vy_robot * cos_h
        robot_state["x"] += vx_world * dt
        robot_state["y"] += vy_world * dt
        # Heading from IMU is authoritative when present; otherwise integrate omega
        if imu_data is not None:
            robot_state["theta"] = current_heading_rad
        else:
            robot_state["theta"] += omega_body * dt
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
        self.disabled = os.environ.get("DISABLE_FILE_LOG", "0").strip().lower() in ("1","true","yes")
        self.log_directory = log_directory or os.environ.get("LOG_DIRECTORY", LOG_DIRECTORY_DEFAULT)
        if not self.disabled:
            os.makedirs(self.log_directory, exist_ok=True)
        self.log_files = {} 
        self.session_start_time = time.strftime('%Y%m%d_%H%M%S')

    def get_log_file(self, unique_robot_key, data_type):
        if self.disabled:
            return
        if unique_robot_key not in self.log_files:
            self.log_files[unique_robot_key] = {}

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

    def log_data(self, unique_robot_key, data_type, message_dict):
        file_handle = self.get_log_file(unique_robot_key, data_type)
        if not file_handle:
            return

        try:
            log_timestamp = message_dict.get("timestamp", time.time())

            # Encoder: support message_dict['data'] = [r1,r2,r3,...] or rpm_1,rpm_2,...
            if data_type in ("encoder", "encoder_data"):
                if isinstance(message_dict.get("data"), list):
                    rpms = message_dict.get("data")
                    r1 = rpms[0] if len(rpms) > 0 else 0
                    r2 = rpms[1] if len(rpms) > 1 else 0
                    r3 = rpms[2] if len(rpms) > 2 else 0
                else:
                    r1 = message_dict.get('rpm_1', 0)
                    r2 = message_dict.get('rpm_2', 0)
                    r3 = message_dict.get('rpm_3', 0)
                log_line = f"{log_timestamp:.3f} {r1} {r2} {r3}\n"

            # IMU/BNO055: support nested 'data' with 'euler' and 'quaternion' or flattened keys
            elif data_type in ("bno055", "imu", "imu_data"):
                src = message_dict.get("data") if isinstance(message_dict.get("data"), dict) else message_dict
                heading = 0.0
                pitch = 0.0
                roll = 0.0
                if isinstance(src.get("euler"), (list, tuple)) and len(src.get("euler")) >= 3:
                    heading, pitch, roll = src.get("euler")[0], src.get("euler")[1], src.get("euler")[2]
                else:
                    heading = src.get("heading", heading)
                    pitch = src.get("pitch", pitch)
                    roll = src.get("roll", roll)

                if isinstance(src.get("quaternion"), (list, tuple)) and len(src.get("quaternion")) >= 4:
                    quat_w, quat_x, quat_y, quat_z = src.get("quaternion")[:4]
                else:
                    quat_w = src.get("quat_w", 1.0)
                    quat_x = src.get("quat_x", 0.0)
                    quat_y = src.get("quat_y", 0.0)
                    quat_z = src.get("quat_z", 0.0)

                # Prefer new array fields when present
                accel_arr = src.get("lin_accel") if isinstance(src.get("lin_accel"), (list, tuple)) else src.get("accel")
                if isinstance(accel_arr, (list, tuple)):
                    ax, ay, az = (list(accel_arr) + [0.0,0.0,0.0])[:3]
                else:
                    ax = src.get("lin_accel_x", 0.0)
                    ay = src.get("lin_accel_y", 0.0)
                    az = src.get("lin_accel_z", 0.0)

                grav_arr = src.get("gravity") if isinstance(src.get("gravity"), (list, tuple)) else None
                if isinstance(grav_arr, (list, tuple)):
                    gx, gy, gz = (list(grav_arr) + [0.0,0.0,0.0])[:3]
                else:
                    gx = src.get("gravity_x", 0.0)
                    gy = src.get("gravity_y", 0.0)
                    gz = src.get("gravity_z", 0.0)

                log_line = f"{log_timestamp:.3f} {heading:.2f} {pitch:.2f} {roll:.2f} {quat_w:.4f} {quat_x:.4f} {quat_y:.4f} {quat_z:.4f} {ax:.2f} {ay:.2f} {az:.2f} {gx:.2f} {gy:.2f} {gz:.2f}\n"

            elif data_type in ("log", "log_data"):
                log_line = f"{log_timestamp:.3f} {message_dict.get('message', '')}\n"

            elif data_type == "position_update":
                pos = message_dict.get("position") if message_dict.get("position") else message_dict.get("data", {})
                if isinstance(pos, dict):
                    px = float(pos.get('x', 0.0))
                    py = float(pos.get('y', 0.0))
                    pth = float(pos.get('theta', 0.0))
                else:
                    px = py = pth = 0.0
                log_line = f"{log_timestamp:.3f} {px:.3f} {py:.3f} {pth:.3f}\n"

            else:
                log_line = f"{log_timestamp:.3f} {json.dumps(message_dict)}\n"

            file_handle.write(log_line)
            file_handle.flush()
        except Exception as e:
            logger.error(f"Error writing to log for {unique_robot_key} {data_type}: {e}")

    def close_logs(self, unique_robot_key=None):
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

# --- Reduce noise: only keep connection events and BE<->Robot payload directions ---
for lg in (logger, log_tcp, log_ws, log_ota):
    lg.handlers = []  # let root/basicConfig handle
    lg.propagate = True
    lg.setLevel(logging.INFO)
log_fw = logger.getChild("FWUP")
log_traj = logger.getChild("TRAJ")
# Initialize DataLogger (can be disabled by DISABLE_FILE_LOG=1)
data_logger = DataLogger()


# Ensure DirectBridge has safe defaults for websocket subscription bookkeeping even if constructed elsewhere
try:
    from types import SimpleNamespace
except Exception:
    SimpleNamespace = None


# --- OTAConnection class (Handles OTA Server Functionality) ---
class OTAConnection: 
    def __init__(self):
        # For OTA Server functionality
        # Support multiple robots: map robot_ip -> prepared firmware path
        self._firmware_map = {}  # { robot_ip: firmware_path }  # Deprecated
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
            # OTA server disabled in RasPi-only mode
            return False
            logger.info(f"Temporary OTA Server started on 0.0.0.0:{ota_port}. It will close after one connection.")
            # Note: temp_server is not stored; this method is legacy and generally unused.
            return True
        except Exception as e:
            logger.error(f"Failed to start temporary OTA server on port {ota_port}: {e}")
            return False

    async def start_persistent_ota_server(self, ota_port):
        # Disabled in RasPi-only mode
        logger.info("OTA server disabled (RasPi-only mode)")
        return False

    async def stop_ota_server(self):
        # No-op in RasPi-only mode
        self._firmware_map.clear()

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

class DirectBridge:
    GLOBAL_SUBSCRIPTION_KEY = "__GLOBAL__" # Định nghĩa hằng số ở đây

    def __init__(self, tcp_port, ws_port, pid_config_file_path=None): # Added pid_config_file_path
        self.manager = ConnectionManager()  # Removed robot_alias_manager argument
        self.tcp_port = tcp_port
        self.ws_port = ws_port
        # OTA removed for RasPi-only mode
        self.ota_connection = None
        self.trajectory_calculator = TrajectoryCalculator()
        self.pid_config_file = pid_config_file_path if pid_config_file_path else os.environ.get("PID_CONFIG_FILE", PID_CONFIG_FILE_DEFAULT)
        self.pid_config_cache = {}  # Initialize PID config cache
        self.temp_firmware_dir = os.environ.get("TEMP_FIRMWARE_DIR", TEMP_FIRMWARE_DIR_DEFAULT)
        os.makedirs(self.temp_firmware_dir, exist_ok=True)
        self.ota_port_arg = None  # Will be set from main_bridge_runner
        self.data_logger = data_logger  # Use the global instance
        self.websocket_subscriptions = {}  # Stores subscriptions per websocket client
        self.subscribers_lock = asyncio.Lock()  # Added lock for subscribers dictionary
        self._latest_encoder_data = {}  # Initialize latest encoder data
        self._latest_imu_data = {}  # Initialize latest IMU data
        self._last_broadcast_time_by_robot = {}  # rate-limit global broadcasts per robot
        # Track robot type per unique robot key (e.g., "ip:port").
        # Values: "omni" or "mecanum". Default will be set on first connection.
        self.robot_type_by_key = {}
        # Database disabled per request
        self.db = None
        # RasPi-only firmware upload manager
        self.fw_upload_mgr = FirmwareUploadManager(self.temp_firmware_dir)
        # Map and navigation
        self.grid_map = None  # {"occ": np.ndarray(bool), "resolution": float, "origin": (ox, oy), "size": (w,h)}
        self._nav_tasks = {}

        # RasPi gateway mode (backend acts as client to RasPi laptop server)
        try:
            from back.config import settings as _settings  # type: ignore
        except Exception:
            try:
                from config import settings as _settings  # type: ignore
            except Exception:
                _settings = None  # type: ignore

        self.raspi_host = (os.environ.get("RASPI_HOST") or (_settings.raspi_host if _settings else None))
        try:
            self.raspi_port = int(os.environ.get("RASPI_PORT", str(_settings.raspi_port if _settings else 2004)))
        except Exception:
            self.raspi_port = 2004
        self._raspi_reader = None
        self._raspi_writer = None
        self._raspi_seen_robot_ids = set()
        self._raspi_ready_event = asyncio.Event()
        self._raspi_last_firmware_path = None
        # Last-seen timestamps per data type to throttle logs and health-check
        self._last_seen_by_type = {}
        # Start background watchdog to warn if no encoder/imu for 5s per robot
        self._watchdog_task = asyncio.create_task(self._rx_watchdog())

    async def _handle_raspi_line(self, raw_line: str):
        """Process one newline-delimited JSON coming from RasPi laptop port.
        Expected payloads mirror firmware's: encoder/bno055/position/log.
        """
        raw_line = (raw_line or "").strip()
        if not raw_line:
            return
        try:
            message_from_robot = json.loads(raw_line)
        except Exception:
            log_tcp.error(f"Invalid JSON from RasPi: {raw_line}")
            return

        # Determine robot identity
        current_alias = None
        try:
            current_alias = str(message_from_robot.get("id") or "unknown")
        except Exception:
            current_alias = "unknown"
        unique_robot_key = current_alias  # Use id as unique key in RasPi mode
        robot_ip_address = "raspi"

        # First sight of this robot id → register alias mapping and announce to UI
        if current_alias and current_alias not in self._raspi_seen_robot_ids:
            self._raspi_seen_robot_ids.add(current_alias)
            try:
                async with robot_alias_manager["lock"]:
                    robot_alias_manager["alias_to_ip_port"][current_alias] = unique_robot_key
                    robot_alias_manager["ip_to_alias"][current_alias] = current_alias
                    robot_alias_manager["ip_port_to_alias"][unique_robot_key] = current_alias
            except Exception:
                pass
            try:
                await broadcast_to_all_ui({
                    "type": "available_robot_update",
                    "action": "add",
                    "robot": {
                        "ip": robot_ip_address,
                        "alias": current_alias,
                        "unique_key": unique_robot_key,
                        "status": "connected",
                        "robot_type": self.robot_type_by_key.get(unique_robot_key, "omni"),
                    },
                    "timestamp": time.time()
                })
                log_tcp.info(f"RasPi robot announced to UI: {current_alias}")
            except Exception:
                pass
            # DB disabled

        try:
            transformed_message = transform_robot_message(message_from_robot)
            # Attach identity
            transformed_message["robot_ip"] = robot_ip_address
            transformed_message["robot_alias"] = current_alias

            # Persist (disabled) / broadcast using the same logic as TCP path
            msg_type = transformed_message.get("type")
            self.data_logger.log_data(unique_robot_key, msg_type or "unknown_data", transformed_message)
            await self.broadcast_to_subscribers(current_alias, transformed_message)

            # Health/logging: record last-seen time for encoder/imu and emit concise logs
            now_t = time.time()
            if msg_type in ("encoder_data", "imu_data"):
                key = (current_alias, msg_type)
                last = self._last_seen_by_type.get(key)
                # Log only once per 5s at most for "received" message
                if not last or (now_t - last) >= 5.0:
                    log_tcp.info(f"RasPi RX ok: {msg_type} from {current_alias}")
                self._last_seen_by_type[key] = now_t

            if msg_type == "encoder_data":
                encoder_rpms_list = transformed_message.get("data")
                message_timestamp = transformed_message.get("timestamp")
                if encoder_rpms_list is not None and message_timestamp is not None and isinstance(encoder_rpms_list, list):
                    if len(encoder_rpms_list) >= 4 and self.robot_type_by_key.get(unique_robot_key) != "mecanum":
                        self.robot_type_by_key[unique_robot_key] = "mecanum"
                        # DB disabled
                        if getattr(self, "db", None):
                            try:
                                self.db.update_robot_type(unique_robot_key, "mecanum")
                            except Exception:
                                pass
                    if getattr(self, "db", None):
                        try:
                            self.db.insert_encoder(unique_robot_key, encoder_rpms_list, message_timestamp)
                        except Exception as e:
                            logger.error(f"DB insert_encoder failed for {unique_robot_key}: {e}")
                    self._latest_encoder_data[unique_robot_key] = transformed_message
                    self.trajectory_calculator.update_encoder_data(unique_robot_key, transformed_message)

            elif msg_type == "imu_data":
                self._latest_imu_data[unique_robot_key] = transformed_message
                # DB disabled
                if getattr(self, "db", None):
                    try:
                        self.db.insert_imu(unique_robot_key, transformed_message.get("data", {}), transformed_message.get("timestamp"))
                    except Exception as e:
                        logger.error(f"DB insert_imu failed for {unique_robot_key}: {e}")

            elif msg_type == "position_update":
                pos_data = transformed_message.get("data", {})
                self.trajectory_calculator.update_position_packet(unique_robot_key, pos_data)
                # DB disabled
                if getattr(self, "db", None):
                    try:
                        self.db.insert_position(unique_robot_key, float(pos_data.get("x", 0.0)), float(pos_data.get("y", 0.0)), float(pos_data.get("theta", 0.0)), transformed_message.get("timestamp"))
                    except Exception as e:
                        logger.error(f"DB insert_position failed for {unique_robot_key}: {e}")

            elif msg_type == "log":
                # DB disabled
                if getattr(self, "db", None):
                    try:
                        message_txt = transformed_message.get("message") or transformed_message.get("data") or ""
                        self.db.insert_log(unique_robot_key, str(message_txt), level=str(transformed_message.get("level", "INFO")), timestamp=transformed_message.get("timestamp"))
                    except Exception as e:
                        logger.error(f"DB insert_log failed for {unique_robot_key}: {e}")

        except Exception as e:
            log_tcp.error(f"Error processing RasPi packet: {e}")

    async def _raspi_client_loop(self):
        """Maintain a TCP client connection to RasPi laptop server and forward data."""
        host = self.raspi_host
        port = self.raspi_port
        if not host:
            return
        log_tcp.info(f"RasPi gateway mode enabled. Connecting to {host}:{port} ...")
        backoff = 1.0
        while True:
            try:
                reader, writer = await asyncio.open_connection(host, port)
                self._raspi_reader, self._raspi_writer = reader, writer
                log_tcp.info(f"Connected to RasPi at {host}:{port}")
                # Read newline-delimited JSON
                while True:
                    try:
                        line = await asyncio.wait_for(reader.readline(), timeout=60.0)
                    except asyncio.TimeoutError:
                        continue
                    if not line:
                        log_tcp.warning("RasPi connection closed by remote.")
                        break
                    text = line.decode("utf-8", errors="ignore").strip()
                    # Handle non-JSON handshake tokens from RasPi firmware flow
                    if text.lower().find("ready") != -1:
                        try:
                            self._raspi_ready_event.set()
                            log_tcp.info("RasPi signaled 'ready' for firmware update.")
                        except Exception:
                            pass
                        continue
                    await self._handle_raspi_line(text)
            except Exception as e:
                log_tcp.error(f"RasPi connect/error: {e}")
            finally:
                try:
                    if self._raspi_writer and (not self._raspi_writer.is_closing()):
                        self._raspi_writer.close()
                        await self._raspi_writer.wait_closed()
                except Exception:
                    pass
                self._raspi_reader, self._raspi_writer = None, None
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2.0, 10.0)

    # ===== Map helpers =====
    def _load_occupancy_from_image(self, b64_png: str, threshold: int = 127):
        """Decode base64 image to occupancy (True=occupied). Returns (occ, w, h)."""
        if Image is None or np is None:
            raise RuntimeError("Map features require numpy and Pillow. Please install 'numpy' and 'Pillow' in the Python environment running the backend.")
        img_bytes = base64.b64decode(b64_png)
        with Image.open(io.BytesIO(img_bytes)) as im:
            im = im.convert("L")
            arr = np.array(im)
        h, w = arr.shape
        occ = arr < max(0, min(255, threshold))
        return occ, w, h

    def _world_to_grid(self, x: float, y: float):
        if not self.grid_map:
            return 0, 0
        ox, oy = self.grid_map["origin"]
        res = self.grid_map["resolution"]
        gx = int((x - ox) / res)
        gy = int((y - oy) / res)
        return gx, gy

    def _grid_to_world(self, gx: int, gy: int):
        ox, oy = self.grid_map["origin"]
        res = self.grid_map["resolution"]
        return ox + gx * res, oy + gy * res

    def _neighbors(self, x: int, y: int, w: int, h: int):
        for dx, dy, cost in ((1,0,1),(-1,0,1),(0,1,1),(0,-1,1),(1,1,math.sqrt(2)),(1,-1,math.sqrt(2)),(-1,1,math.sqrt(2)),(-1,-1,math.sqrt(2))):
            nx, ny = x+dx, y+dy
            if 0 <= nx < w and 0 <= ny < h:
                yield nx, ny, cost

    def _astar(self, start, goal):
        if not self.grid_map:
            return None
        occ = self.grid_map["occ"]
        h, w = occ.shape
        sx, sy = start
        gx, gy = goal
        if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
            return None
        if occ[sy, sx] or occ[gy, gx]:
            return None
        import heapq
        open_heap = []
        heapq.heappush(open_heap, (0.0, (sx, sy)))
        g_cost = {(sx,sy): 0.0}
        parent = {}
        def hfun(nx, ny):
            return math.hypot(nx-gx, ny-gy)
        visited = set()
        while open_heap:
            _, (cx, cy) = heapq.heappop(open_heap)
            if (cx, cy) in visited:
                continue
            visited.add((cx, cy))
            if (cx, cy) == (gx, gy):
                path = [(cx, cy)]
                while (cx, cy) in parent:
                    cx, cy = parent[(cx, cy)]
                    path.append((cx, cy))
                path.reverse()
                return path
            for nx, ny, step in self._neighbors(cx, cy, w, h):
                if occ[ny, nx]:
                    continue
                tentative = g_cost[(cx, cy)] + step
                if tentative < g_cost.get((nx, ny), 1e18):
                    g_cost[(nx, ny)] = tentative
                    parent[(nx, ny)] = (cx, cy)
                    heapq.heappush(open_heap, (tentative + hfun(nx, ny), (nx, ny)))
        return None

    def _simplify_path(self, grid_path, max_keep: int = 500):
        if not grid_path:
            return []
        simplified = [grid_path[0]]
        def collinear(a,b,c):
            return (b[0]-a[0])*(c[1]-a[1]) == (b[1]-a[1])*(c[0]-a[0])
        for p in grid_path[1:-1]:
            if len(simplified) < 2 or not collinear(simplified[-2], simplified[-1], p):
                simplified.append(p)
        simplified.append(grid_path[-1])
        if len(simplified) > max_keep:
            step = max(1, len(simplified)//max_keep)
            simplified = simplified[::step]
            if simplified[-1] != grid_path[-1]:
                simplified.append(grid_path[-1])
        return simplified

    async def _follow_path(self, robot_alias: str, world_path, speed: float = 0.2, reach_tol: float = 0.08):
        try:
            await self.broadcast_to_subscribers(robot_alias, {
                "type": "navigation_status",
                "robot_ip": robot_alias,
                "robot_alias": robot_alias,
                "timestamp": time.time(),
                "status": "started"
            })
            for (wx, wy) in world_path:
                await self.handle_internal_command({"command": "position_control", "robot_alias": robot_alias, "x": wx, "y": wy, "vel": speed})
                deadline = time.time() + 8.0
                while time.time() < deadline:
                    unique_key = await self.get_robot_key_from_alias(robot_alias)
                    if not unique_key:
                        break
                    snap = self.trajectory_calculator.get_snapshot(unique_key)
                    if isinstance(snap, dict) and isinstance(snap.get("position"), dict):
                        pos = snap["position"]
                        if math.hypot(pos.get("x",0)-wx, pos.get("y",0)-wy) <= reach_tol:
                            break
                    await asyncio.sleep(0.1)
            await self.broadcast_to_subscribers(robot_alias, {
                "type": "navigation_status",
                "robot_ip": robot_alias,
                "robot_alias": robot_alias,
                "timestamp": time.time(),
                "status": "completed"
            })
        except asyncio.CancelledError:
            await self.broadcast_to_subscribers(robot_alias, {
                "type": "navigation_status",
                "robot_ip": robot_alias,
                "robot_alias": robot_alias,
                "timestamp": time.time(),
                "status": "cancelled"
            })
            raise

    async def handle_internal_command(self, data: dict):
        command = data.get("command")
        robot_alias = data.get("robot_alias")
        if command == "position_control":
            try:
                pos_x = float(data.get("x"))
                pos_y = float(data.get("y"))
                vel = float(data.get("vel", 0.2))
                if not robot_alias:
                    return False
                cmd_str = f"x:{pos_x} y:{pos_y} vel:{vel}"
                ok = await self.send_text_command_by_alias(robot_alias, cmd_str)
                return ok
            except Exception:
                return False
        return False

    def build_robot_status_snapshot(self, unique_robot_key: str, robot_alias: str, robot_ip: str):
        """Assemble a robot_status snapshot from latest cached data."""
        latest_enc = self._latest_encoder_data.get(unique_robot_key)
        rpm_list = []
        if isinstance(latest_enc, dict):
            data_field = latest_enc.get("data")
            if isinstance(data_field, list):
                rpm_list = data_field[:]  # keep all provided encoder channels
        # Normalize to at least 3 values for legacy consumers
        if len(rpm_list) < 3:
            rpm_list = (rpm_list + [0, 0, 0])[:3]

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
            "robot_type": getattr(self, 'robot_type_by_key', {}).get(unique_robot_key, "omni"),
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
        """Send a plaintext command to robot. If RasPi mode is enabled, forward via RasPi socket.
        Otherwise, send to the direct TCP client mapped by alias.
        """
        # RasPi forwarding path
        if getattr(self, '_raspi_writer', None) is not None:
            try:
                data = command_text if command_text.endswith("\n") else (command_text + "\n")
                self._raspi_writer.write(data.encode('utf-8'))
                await self._raspi_writer.drain()
                return True
            except Exception as e:
                log_tcp.error(f"Failed sending command via RasPi: {e}")
                # fall through to try direct path

        unique_key = await self.get_robot_key_from_alias(robot_alias)
        if not unique_key:
            return False
        conn_tuple = ConnectionManager.get_tcp_client(unique_key)
        if not conn_tuple:
            return False
        reader, writer = conn_tuple
        try:
            data = command_text if command_text.endswith("\n") else (command_text + "\n")
            writer.write(data.encode("utf-8"))
            await writer.drain()
            return True
        except Exception:
            return False

    async def send_upgrade_command_by_alias(self, robot_alias: str, mode_override: str = None) -> bool:
        """Send Upgrade command. If RasPi is configured, forward via RasPi writer (expects RasPi firmware flow)."""
        mode = (mode_override or os.environ.get("BRIDGE_UPGRADE_MODE", "raw")).lower()
        payload = b"Upgrade\n" if mode == "newline" else (b"Upgrade" if mode == "raw" else b"Upgrade\n")

        # RasPi path first if available
        if getattr(self, '_raspi_writer', None) is not None:
            try:
                self._raspi_writer.write(payload)
                await self._raspi_writer.drain()
                log_tcp.info("Sent Upgrade via RasPi")
                return True
            except Exception as e:
                log_tcp.error(f"Failed Upgrade via RasPi: {e}")

        # Fallback to direct connection
        unique_key = await self.get_robot_key_from_alias(robot_alias)
        if not unique_key:
            return False
        conn_tuple = ConnectionManager.get_tcp_client(unique_key)
        if not conn_tuple:
            return False
        reader, writer = conn_tuple
        try:
            writer.write(payload)
            await writer.drain()
            log_tcp.info(f"Sent Upgrade directly to {robot_alias} ({unique_key})")
            return True
        except Exception as e:
            log_tcp.error(f"Failed to send Upgrade to {robot_alias}: {e}")
            return False

    async def get_connected_robots_list(self):
        """Return connected robots using IP as alias."""
        robots_list = []
        for item in ConnectionManager.get_all_robots_with_ip():
            if item.get("connected"):
                ip = item.get("ip")
                port = item.get("port")
                alias_unique = f"{ip}:{port}" if ip and port is not None else ip
                ukey = item.get("robot_id")
                robots_list.append({
                    "alias": alias_unique,
                    "ip": ip,
                    "unique_key": ukey,
                    "status": "connected",
                    "robot_type": getattr(self, 'robot_type_by_key', {}).get(ukey, "omni")
                })

        # Include robots seen via RasPi gateway (no direct TCP client to backend)
        try:
            seen = getattr(self, '_raspi_seen_robot_ids', set()) or set()
            existing_aliases = {r.get('alias') for r in robots_list}
            for alias in seen:
                if alias not in existing_aliases:
                    robots_list.append({
                        "alias": alias,
                        "ip": alias,
                        "unique_key": alias,
                        "status": "connected",
                        "robot_type": getattr(self, 'robot_type_by_key', {}).get(alias, "omni")
                    })
        except Exception:
            pass

        return robots_list

    async def send_connected_robots_list_to_client(self, websocket):
        """Helper to send the current connected robots list to a newly connected websocket client.
        Keeps a consistent initial payload format expected by the frontend.
        """
        try:
            robots = await self.get_connected_robots_list()
            # Bổ sung các robot đã thấy qua RasPi gateway (nếu có) để client mới nhìn thấy ngay
            try:
                seen = getattr(self, '_raspi_seen_robot_ids', set()) or set()
                existing_aliases = {r.get('alias') for r in robots}
                for alias in seen:
                    if alias not in existing_aliases:
                        robots.append({
                            "alias": alias,
                            "ip": alias,  # sử dụng alias như ip hiển thị
                            "unique_key": alias,
                            "status": "connected",
                            "robot_type": getattr(self, 'robot_type_by_key', {}).get(alias, "omni")
                        })
            except Exception:
                pass
            payload = {
                "type": "available_robots_initial_list",
                "robots": robots,
                "timestamp": time.time()
            }
            await websocket.send(json.dumps(payload))
            log_ws.info(f"WS init list -> {getattr(websocket, 'remote_address', None)}: {len(robots)} robot(s)")
        except Exception as e:
            logger.error(f"Failed to send connected robots list to WS client {getattr(websocket, 'remote_address', None)}: {e}")
    def get_websocket_cors_headers(self, path: str, request_headers):
        # request_headers is of type websockets.datastructures.Headers
        # Default frontend origin for Vite dev environment
        frontend_origin = os.environ.get("FRONTEND_ORIGIN", "http://localhost:5173") 
        
    async def _rx_watchdog(self):
        """Periodically check if encoder/imu have stopped for any robot (>=5s)."""
        try:
            while True:
                await asyncio.sleep(1.0)
                now_t = time.time()
                # Build set of robots seen via raspi
                try:
                    seen_aliases = list(self._raspi_seen_robot_ids)
                except Exception:
                    seen_aliases = []
                for alias in seen_aliases:
                    for typ in ("encoder_data", "imu_data"):
                        last = self._last_seen_by_type.get((alias, typ))
                        if last and (now_t - last) > 5.0:
                            log_tcp.warning(f"No {typ} from {alias} for >5s")
                            # avoid spamming every second
                            self._last_seen_by_type[(alias, typ)] = now_t  # reset so next warn comes after 5s again
        except asyncio.CancelledError:
            return
        except Exception:
            # watchdog must not crash the bridge
            pass
        # Removed stray CORS helper block (not used). If needed, re-enable get_websocket_cors_headers.

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
        # OTA server disabled

        # Load PID configurations from file and cache them
        loaded_pids = await self.load_pid_config_from_file() # target_robot_ip is None, loads for caching
        if loaded_pids:
            self.pid_config_cache = loaded_pids
            # Log message is now part of load_pid_config_from_file when target_robot_ip is None
        else:
            logger.warning(f"Could not load and cache PID configuration from '{self.pid_config_file}'.")

        # In RasPi mode, we still keep TCP server for direct robot connections (optional),
        # and additionally start RasPi client loop if configured.
        # Remove direct ESP server: operate only via RasPi gateway
        if getattr(self, 'raspi_host', None):
            asyncio.create_task(self._raspi_client_loop())
            log_tcp.info("Direct TCP server disabled. Using RasPi gateway only.")
        else:
            log_tcp.warning("RASPI_HOST is not configured. No robot data source is active.")
        
        # Cho phép giá trị float ("20.0") hoặc int cho ping interval/timeout
        def _as_float_env(name: str, default: float) -> float:
            raw = os.environ.get(name)
            if raw is None or raw.strip() == "":
                return default
            try:
                return float(raw)
            except ValueError:
                try:
                    return float(int(raw))
                except Exception:
                    return default
        def _as_int_env(name: str, default: int) -> int:
            raw = os.environ.get(name)
            if raw is None or raw.strip() == "":
                return default
            try:
                return int(float(raw))
            except Exception:
                return default

        ws_ping_interval = _as_float_env("WS_PING_INTERVAL", 30.0)
        ws_ping_timeout = _as_float_env("WS_PING_TIMEOUT", 20.0)
        ws_max_size = _as_int_env("WS_MAX_SIZE", 1*1024*1024)
        self.ws_server = await websockets.serve(
            self.handle_ws_client,
            '0.0.0.0',
            self.ws_port,
            ping_interval=ws_ping_interval,
            ping_timeout=ws_ping_timeout,
            max_size=ws_max_size,
            #additional_headers=self.get_websocket_cors_headers
        )
        log_ws.info(f"WebSocket server started on 0.0.0.0:{self.ws_port}")
    
    async def handle_tcp_client(self, reader, writer):
        peername = writer.get_extra_info('peername')
        robot_ip_address = peername[0]
        robot_port = peername[1]
        unique_robot_key = f"{robot_ip_address}:{robot_port}"

        # Use robot IP as alias/identifier; if duplicate IP already mapped, fall back to ip:port to avoid collision
        desired_alias = robot_ip_address
        async with robot_alias_manager["lock"]:
            existing_for_ip = robot_alias_manager["alias_to_ip_port"].get(desired_alias)
            if existing_for_ip and existing_for_ip != unique_robot_key:
                # Another client already using the plain IP alias; disambiguate with port
                desired_alias = f"{robot_ip_address}:{robot_port}"
                log_tcp.warning(f"Alias collision on IP {robot_ip_address}; using unique alias '{desired_alias}' for {unique_robot_key}")

            current_alias = desired_alias
            robot_alias_manager["ip_port_to_alias"][unique_robot_key] = current_alias
            robot_alias_manager["alias_to_ip_port"][current_alias] = unique_robot_key
            # Only set ip_to_alias if not already set, to keep a stable primary alias for the IP
            if robot_ip_address not in robot_alias_manager["ip_to_alias"]:
                robot_alias_manager["ip_to_alias"][robot_ip_address] = current_alias
            robot_alias_manager["alias_to_ip"][current_alias] = robot_ip_address
        log_tcp.info(f"Connection from {robot_ip_address}:{robot_port} registered with alias: {current_alias}")

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

        # Initialize robot type default on first sight
        if unique_robot_key not in getattr(self, 'robot_type_by_key', {}):
            self.robot_type_by_key[unique_robot_key] = "omni"
            # Also mirror into calculator for kinematics decisions
            try:
                self.trajectory_calculator.set_robot_type(unique_robot_key, "omni")
            except Exception:
                pass

        log_tcp.info(f"Processing started for {current_alias} ({unique_robot_key}).")
        
        # Direct ESP path deprecated in RasPi-only mode

        # Send cached PID config if available. Optional via BRIDGE_SEND_PID_ON_CONNECT env var
        send_pid_on_connect = os.environ.get("BRIDGE_SEND_PID_ON_CONNECT", "1").strip().lower() not in ("0", "false", "no")
        if self.pid_config_cache and send_pid_on_connect:
            log_tcp.info(f"Attempting to send cached PID config to newly connected robot {current_alias} ({robot_ip_address}).")
            try:
                for motor_id, params in self.pid_config_cache.items():
                    pid_command_str = f"MOTOR:{motor_id} Kp:{params['kp']} Ki:{params['ki']} Kd:{params['kd']}\n"
                    try:
                        writer.write(pid_command_str.encode('utf-8'))
                        await writer.drain()
                        log_tcp.debug(f"Sent cached PID to {current_alias}: {pid_command_str.strip()}")
                    except Exception as e:
                        logger.error(f"Failed to send PID line to {current_alias}: {e}")
                        break
                log_tcp.info(f"Completed PID config send to {current_alias} ({robot_ip_address}).")
            except Exception as e:
                logger.error(f"Error sending cached PID config to {current_alias} ({robot_ip_address}): {e}")
        else:
            if not send_pid_on_connect:
                log_tcp.info(f"Skipping sending cached PID to {current_alias} due to BRIDGE_SEND_PID_ON_CONNECT=0")
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
                "status": "connected",
                "robot_type": self.robot_type_by_key.get(unique_robot_key, "omni"),
            },
            "timestamp": time.time()
        }
        await broadcast_to_all_ui(robot_announcement_payload)
        robot_announced_to_ui = True
        # DB disabled

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
                    
                    # Minimal logging only when needed elsewhere

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
                            # Auto-detect mecanum on first 4+ channel encoder message and propagate to UI
                            try:
                                if len(encoder_rpms_list) >= 4:
                                    prev_type = getattr(self, 'robot_type_by_key', {}).get(unique_robot_key, "omni")
                                    if prev_type != "mecanum":
                                        # Update backend state
                                        self.robot_type_by_key[unique_robot_key] = "mecanum"
                                        try:
                                            self.trajectory_calculator.set_robot_type(unique_robot_key, "mecanum")
                                        except Exception:
                                            pass
                                        # Update DB robot_type
                                        if getattr(self, "db", None):
                                            try:
                                                self.db.update_robot_type(unique_robot_key, "mecanum")
                                            except Exception as e:
                                                logger.error(f"DB update_robot_type failed for {unique_robot_key}: {e}")
                                        # Notify UI list item so TrajectoryWidget can switch icon
                                        await broadcast_to_all_ui({
                                            "type": "available_robot_update",
                                            "action": "update",
                                            "robot": {
                                                "ip": current_alias,
                                                "alias": current_alias,
                                                "unique_key": unique_robot_key,
                                                "status": "connected",
                                                "robot_type": "mecanum",
                                            },
                                            "timestamp": time.time(),
                                        })
                            except Exception:
                                pass
                            # Persist encoders
                            if getattr(self, "db", None):
                                try:
                                    self.db.insert_encoder(unique_robot_key, encoder_rpms_list, message_timestamp)
                                except Exception as e:
                                    logger.error(f"DB insert_encoder failed for {unique_robot_key}: {e}")
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
                                # Optionally broadcast compact robot_status to all connected UI clients so
                                # unselected robots can still be visualized in summaries or lists.
                                if os.environ.get("BRIDGE_BROADCAST_ALL_STATUS", "0").strip() in ("1", "true", "yes"):
                                    try:
                                        await broadcast_to_all_ui(robot_status_payload)
                                    except Exception:
                                        pass
                            # Optionally broadcast raw encoder data to all UI clients (throttled)
                            if os.environ.get("BRIDGE_BROADCAST_ALL_TYPES", "0").strip() in ("1", "true", "yes"):
                                try:
                                    now_t = time.time()
                                    last_t = self._last_broadcast_time_by_robot.get(unique_robot_key, 0.0)
                                    if (now_t - last_t) >= float(os.environ.get("BRIDGE_BROADCAST_MIN_INTERVAL", "0.2")):
                                        await broadcast_to_all_ui(transformed_message)
                                        self._last_broadcast_time_by_robot[unique_robot_key] = now_t
                                except Exception:
                                    pass
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
                                    if os.environ.get("BRIDGE_BROADCAST_ALL_STATUS", "0").strip() in ("1", "true", "yes"):
                                        try:
                                            await broadcast_to_all_ui(robot_status_payload)
                                        except Exception:
                                            pass
                                else:
                                    logger.warning(f"Skipping trajectory broadcast for {current_alias} due to invalid result from TrajectoryCalculator: {trajectory_update_result}")
                    
                    elif msg_type == "imu_data": 
                        # transformed_message for imu_data is:
                        # {"type": "imu_data", "robot_ip": ..., "robot_alias": ..., "timestamp": ..., "data": {"time":..., "euler":..., "quaternion":...}}
                        self._latest_imu_data[unique_robot_key] = transformed_message # Store the whole transformed payload
                        
                        # Persist IMU
                        if getattr(self, "db", None):
                            try:
                                self.db.insert_imu(unique_robot_key, transformed_message.get("data", {}), transformed_message.get("timestamp"))
                            except Exception as e:
                                logger.error(f"DB insert_imu failed for {unique_robot_key}: {e}")
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
                        if os.environ.get("BRIDGE_BROADCAST_ALL_STATUS", "0").strip() in ("1", "true", "yes"):
                            try:
                                await broadcast_to_all_ui(robot_status_payload)
                            except Exception:
                                pass
                        # Optionally broadcast IMU packets to all UI clients (throttled)
                        if os.environ.get("BRIDGE_BROADCAST_ALL_TYPES", "0").strip() in ("1", "true", "yes"):
                            try:
                                now_t = time.time()
                                last_t = self._last_broadcast_time_by_robot.get(unique_robot_key, 0.0)
                                if (now_t - last_t) >= float(os.environ.get("BRIDGE_BROADCAST_MIN_INTERVAL", "0.2")):
                                    await broadcast_to_all_ui(transformed_message)
                                    self._last_broadcast_time_by_robot[unique_robot_key] = now_t
                            except Exception:
                                pass

                    elif msg_type == "position_update":
                        # transformed_message for position_update is:
                        # {"type": "position_update", "data": {"x": float(m), "y": float(m), "theta": float(rad)}}
                        pos_data = transformed_message.get("data", {})
                        traj_res = self.trajectory_calculator.update_position_packet(unique_robot_key, pos_data)
                        # Persist position
                        if getattr(self, "db", None):
                            try:
                                self.db.insert_position(unique_robot_key, float(pos_data.get("x", 0.0)), float(pos_data.get("y", 0.0)), float(pos_data.get("theta", 0.0)), transformed_message.get("timestamp"))
                            except Exception as e:
                                logger.error(f"DB insert_position failed for {unique_robot_key}: {e}")
                        if isinstance(traj_res, dict) and isinstance(traj_res.get("position"), dict):
                            # transformed_message was already broadcast above; just update and broadcast status
                            robot_status_payload = self.build_robot_status_snapshot(unique_robot_key, current_alias, robot_ip_address)
                            await self.broadcast_to_subscribers(current_alias, robot_status_payload)
                            if os.environ.get("BRIDGE_BROADCAST_ALL_STATUS", "0").strip() in ("1", "true", "yes"):
                                try:
                                    await broadcast_to_all_ui(robot_status_payload)
                                except Exception:
                                    pass
                            # Optionally broadcast position packets to all UI clients (throttled)
                            if os.environ.get("BRIDGE_BROADCAST_ALL_TYPES", "0").strip() in ("1", "true", "yes"):
                                try:
                                    now_t = time.time()
                                    last_t = self._last_broadcast_time_by_robot.get(unique_robot_key, 0.0)
                                    if (now_t - last_t) >= float(os.environ.get("BRIDGE_BROADCAST_MIN_INTERVAL", "0.2")):
                                        await broadcast_to_all_ui(transformed_message)
                                        self._last_broadcast_time_by_robot[unique_robot_key] = now_t
                                except Exception:
                                    pass
                    elif msg_type == "log":
                        # Persist logs (if robot sends textual logs)
                        if getattr(self, "db", None):
                            try:
                                message_txt = transformed_message.get("message") or transformed_message.get("data") or ""
                                self.db.insert_log(unique_robot_key, str(message_txt), level=str(transformed_message.get("level", "INFO")), timestamp=transformed_message.get("timestamp"))
                            except Exception as e:
                                logger.error(f"DB insert_log failed for {unique_robot_key}: {e}")

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
                    # Remove alias_to_ip mapping for this alias regardless of whether it's the primary IP alias
                    if alias_being_removed in robot_alias_manager.get("alias_to_ip", {}):
                        del robot_alias_manager["alias_to_ip"][alias_being_removed]
                    # If the primary ip_to_alias points to this alias, clear it
                    if robot_alias_manager.get("ip_to_alias", {}).get(robot_ip_address) == alias_being_removed:
                        if robot_ip_address in robot_alias_manager["ip_to_alias"]:
                            del robot_alias_manager["ip_to_alias"][robot_ip_address]
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
            # Mark robot disconnected in DB
            if getattr(self, "db", None):
                try:
                    self.db.set_robot_status(unique_robot_key, "disconnected")
                except Exception as e:
                    logger.error(f"DB set_robot_status(disconnected) failed for {unique_robot_key}: {e}")

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
        # Throttle nhẹ theo (robot,type) bằng BRIDGE_BROADCAST_MIN_INTERVAL
        try:
            key = (robot_alias_source, data_type_to_send)
            now_t = time.time()
            min_dt = float(os.environ.get("BRIDGE_BROADCAST_MIN_INTERVAL", "0.05"))
            last_t = self._last_type_tick.get(key, 0.0)
            if (now_t - last_t) < min_dt:
                return
            self._last_type_tick[key] = now_t
        except Exception:
            pass
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
        log_ws.info(f"WS connected: {ws_identifier} path={path}")
        
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
                        
                        # Reduce subscription chatter
                        await websocket.send(json.dumps({"type": "ack", "command": command, "status": "success", "data_type": data_type_to_sub, "subscribed_key": actual_subscription_entity_key, "robot_alias": target_alias}))

                        # Immediately send snapshot for realtime_trajectory or position_update if requested
                        if data_type_to_sub in ("realtime_trajectory", "position_update"):
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

                                # For position_update subscription, send only position
                                if data_type_to_sub == "position_update":
                                    snapshot_payload = {
                                        "type": "position_update",
                                        "robot_ip": target_alias,
                                        "robot_alias": target_alias,
                                        "timestamp": time.time(),
                                        "position": snap["position"],
                                    }
                                else:
                                    snapshot_payload = {
                                        "type": "realtime_trajectory",
                                        "robot_ip": target_alias,
                                        "robot_alias": target_alias,
                                        "timestamp": time.time(),
                                        "position": snap["position"],
                                        "path": snap["path"],
                                    }
                                await websocket.send(json.dumps(snapshot_payload))
                                logger.info(f"Sent initial {'position' if data_type_to_sub=='position_update' else 'trajectory'} snapshot to {ws_identifier} for {target_alias}.")
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
                                # Reduce unsubscription chatter
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
                            if ok:
                                log_tcp.info(f"BE->Robot ({robot_alias}): EMERGENCY_STOP")
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
                            if ok:
                                log_tcp.info(f"BE->Robot ({robot_alias}): vector_control dot_x:{dot_x} dot_y:{dot_y} dot_theta:{dot_theta} stop_time:{stop_time}")
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
                    # Firmware upload/OTA commands removed in RasPi-only mode
                    elif command == "upload_map":
                        try:
                            b64_data = data.get("data")
                            resolution = float(data.get("resolution", 0.02))
                            origin_x = float(data.get("origin_x", 0.0))
                            origin_y = float(data.get("origin_y", 0.0))
                            threshold = int(data.get("threshold", 127))
                            if not b64_data:
                                raise ValueError("Missing base64 'data'")
                            occ, w, h = self._load_occupancy_from_image(b64_data, threshold)
                            self.grid_map = {"occ": occ, "resolution": resolution, "origin": (origin_x, origin_y), "size": (w, h)}
                            await websocket.send(json.dumps({"type": "map_loaded", "width": w, "height": h, "resolution": resolution, "origin_x": origin_x, "origin_y": origin_y}))
                        except Exception as e:
                            await websocket.send(json.dumps({"type":"error","command":command,"message": str(e)}))
                    elif command == "navigate_to":
                        try:
                            robot_alias = data.get("robot_alias") or data.get("robot_ip")
                            goal_x = float(data.get("x"))
                            goal_y = float(data.get("y"))
                            speed = float(data.get("speed", 0.2))
                            if not robot_alias:
                                raise ValueError("Missing robot_alias")
                            unique_key = await self.get_robot_key_from_alias(robot_alias)
                            if not unique_key:
                                raise ValueError("Robot not connected")
                            snap = self.trajectory_calculator.get_snapshot(unique_key)
                            if not (isinstance(snap, dict) and isinstance(snap.get("position"), dict)):
                                raise ValueError("No pose available")
                            start_pos = snap["position"]
                            world_path = []
                            if self.grid_map:
                                s_gx, s_gy = self._world_to_grid(start_pos.get("x",0), start_pos.get("y",0))
                                g_gx, g_gy = self._world_to_grid(goal_x, goal_y)
                                grid_path = self._astar((s_gx, s_gy), (g_gx, g_gy))
                                if not grid_path:
                                    raise ValueError("No path found")
                                grid_path = self._simplify_path(grid_path)
                                world_path = [self._grid_to_world(cx, cy) for (cx, cy) in grid_path]
                            else:
                                steps = 10
                                sx = float(start_pos.get("x", 0.0))
                                sy = float(start_pos.get("y", 0.0))
                                world_path = [
                                    (sx + (goal_x - sx) * i / (steps - 1), sy + (goal_y - sy) * i / (steps - 1))
                                    for i in range(steps)
                                ]
                            await self.broadcast_to_subscribers(robot_alias, {"type": "planned_path", "robot_ip": robot_alias, "robot_alias": robot_alias, "timestamp": time.time(), "points": [{"x": x, "y": y} for (x,y) in world_path]})
                            if robot_alias in self._nav_tasks and not self._nav_tasks[robot_alias].done():
                                self._nav_tasks[robot_alias].cancel()
                            self._nav_tasks[robot_alias] = asyncio.create_task(self._follow_path(robot_alias, world_path, speed))
                            await websocket.send(json.dumps({"type":"ack","command":command,"status":"started","robot_alias": robot_alias}))
                        except Exception as e:
                            await websocket.send(json.dumps({"type":"error","command":command,"message": str(e)}))
                    elif command == "cancel_navigation":
                        try:
                            robot_alias = data.get("robot_alias") or data.get("robot_ip")
                            if robot_alias in self._nav_tasks and not self._nav_tasks[robot_alias].done():
                                self._nav_tasks[robot_alias].cancel()
                            await websocket.send(json.dumps({"type":"ack","command":command,"status":"cancelled","robot_alias": robot_alias}))
                        except Exception as e:
                            await websocket.send(json.dumps({"type":"error","command":command,"message": str(e)}))
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
                    elif command == "set_robot_type":
                        try:
                            target_alias = data.get("robot_alias") or data.get("robot_ip")
                            rtype = str(data.get("robot_type", "")).lower()
                            if rtype not in ("omni", "mecanum"):
                                raise ValueError("robot_type must be 'omni' or 'mecanum'")
                            if not target_alias:
                                raise ValueError("Missing robot_alias")
                            unique_key = await self.get_robot_key_from_alias(target_alias)
                            if not unique_key:
                                raise ValueError("Robot not connected")
                            # store
                            self.robot_type_by_key[unique_key] = rtype
                            # update calculator as well
                            try:
                                self.trajectory_calculator.set_robot_type(unique_key, rtype)
                            except Exception:
                                pass
                            await websocket.send(json.dumps({"type": "ack", "command": command, "status": "success", "robot_alias": target_alias, "robot_type": rtype}))
                            # notify UI list item
                            await broadcast_to_all_ui({
                                "type": "available_robot_update",
                                "action": "update",
                                "robot": {
                                    "ip": target_alias,
                                    "alias": target_alias,
                                    "unique_key": unique_key,
                                    "status": "connected",
                                    "robot_type": rtype,
                                },
                                "timestamp": time.time(),
                            })
                        except Exception as e:
                            await websocket.send(json.dumps({"type":"error","command":command,"message": str(e)}))
                    elif command == "upload_firmware_start":
                        # FE bắt đầu upload firmware (RasPi mode)
                        try:
                            robot_ip = data.get("robot_ip") or data.get("robot_alias")
                            filename = data.get("filename","firmware.bin")
                            filesize = int(data.get("filesize", 0))
                            if not robot_ip or filesize <= 0:
                                raise ValueError("Missing robot_ip or invalid filesize")
                            self.fw_upload_mgr.start(robot_ip, filename, filesize)
                            await websocket.send(json.dumps({"type":"firmware_upload_ack","status":"ready","robot_ip":robot_ip}))
                        except Exception as e:
                            await websocket.send(json.dumps({"type":"firmware_upload_ack","status":"error","message":str(e)}))
                    elif command == "firmware_data_chunk":
                        try:
                            robot_ip = data.get("robot_ip") or data.get("robot_alias")
                            chunk_b64 = data.get("data")
                            if not robot_ip or not chunk_b64:
                                raise ValueError("Missing robot_ip or chunk data")
                            received = self.fw_upload_mgr.add_chunk(robot_ip, chunk_b64)
                            await websocket.send(json.dumps({"type":"firmware_progress","robot_ip":robot_ip,"received_bytes":received}))
                        except Exception as e:
                            await websocket.send(json.dumps({"type":"firmware_upload_ack","status":"error","message":str(e)}))
                    elif command == "upload_firmware_end":
                        try:
                            robot_ip = data.get("robot_ip") or data.get("robot_alias")
                            if not robot_ip:
                                raise ValueError("Missing robot_ip")
                            fw_path = self.fw_upload_mgr.finish(robot_ip)
                            if not fw_path:
                                raise ValueError("Firmware upload incomplete or size mismatch")
                            self._raspi_last_firmware_path = fw_path
                            log_tcp.info(f"Firmware uploaded for {robot_ip}: {fw_path}")
                            await websocket.send(json.dumps({"type":"firmware_upload_ack","status":"complete","robot_ip":robot_ip,"firmware_path":fw_path}))
                        except Exception as e:
                            await websocket.send(json.dumps({"type":"firmware_upload_ack","status":"error","message":str(e)}))
                    elif command in ("upgrade", "upgrade_signal"):
                        # RasPi firmware flow: send Upgrade, wait 'ready', then send OK + stream firmware + COMPLETE token
                        if robot_alias:
                            mode_override = data.get("mode")
                            if getattr(self, '_raspi_writer', None) is not None:
                                try:
                                    # --- 1) Send Upgrade ---
                                    # Support modes: 'raw' -> b"Upgrade", 'newline' -> b"Upgrade\n", 'both' -> b"Upgrade\n" (line-friendly)
                                    upgrade_mode = (mode_override or os.environ.get("BRIDGE_UPGRADE_MODE", "raw")).lower()
                                    if upgrade_mode not in ("raw", "newline", "both"):
                                        upgrade_mode = "raw"
                                    if upgrade_mode == "raw":
                                        payload = b"Upgrade"
                                    else:
                                        # For 'newline' and 'both', prefer newline-terminated to satisfy line-based parsers
                                        payload = b"Upgrade\n"
                                    self._raspi_ready_event.clear()
                                    self._raspi_writer.write(payload)
                                    await self._raspi_writer.drain()
                                    log_tcp.info(f"Sent Upgrade to RasPi (mode={upgrade_mode!r}, bytes={payload!r}), waiting for 'ready'...")
                                    # 2) Wait for 'ready' (with timeout)
                                    try:
                                        await asyncio.wait_for(self._raspi_ready_event.wait(), timeout=10.0)
                                    except asyncio.TimeoutError:
                                        raise Exception("Timeout waiting for RasPi 'ready' signal")
                                    log_tcp.info("RasPi ready. Sending 'OK' and firmware stream...")
                                    # --- 3) Send OK token (configurable, default 'okay\n') ---
                                    ok_token_str = os.environ.get("BRIDGE_OK_TOKEN", "okay\n")
                                    ok_bytes = ok_token_str.encode('utf-8', errors='ignore')
                                    self._raspi_writer.write(ok_bytes)
                                    await self._raspi_writer.drain()
                                    # 4) Stream firmware
                                    fw_path = getattr(self, '_raspi_last_firmware_path', None)
                                    if not fw_path or not os.path.exists(fw_path):
                                        raise Exception("No firmware uploaded or file missing")
                                    total = os.path.getsize(fw_path)
                                    sent = 0
                                    with open(fw_path, 'rb') as f:
                                        while True:
                                            chunk = f.read(1024)
                                            if not chunk:
                                                break
                                            self._raspi_writer.write(chunk)
                                            await self._raspi_writer.drain()
                                            sent += len(chunk)
                                    log_tcp.info(f"Firmware streamed: {sent}/{total} bytes")
                                    # --- 5) Send COMPLETE token (configurable; default 'complete\n') ---
                                    complete_token_str = os.environ.get("BRIDGE_UPGRADE_COMPLETE_TOKEN", "complete\n")
                                    complete_bytes = complete_token_str.encode('utf-8', errors='ignore')
                                    self._raspi_writer.write(complete_bytes)
                                    await self._raspi_writer.drain()
                                    log_tcp.info(f"Sent completion token to RasPi: {complete_bytes!r}. Firmware update sequence completed.")
                                    await websocket.send(json.dumps({"type":"ack","command":command,"status":"success","robot_alias": robot_alias}))
                                except Exception as e:
                                    log_tcp.error(f"Upgrade flow error: {e}")
                                    await websocket.send(json.dumps({"type":"ack","command":command,"status":"failed","robot_alias": robot_alias, "message": str(e)}))
                            else:
                                ok = await self.send_upgrade_command_by_alias(robot_alias, mode_override)
                                await websocket.send(json.dumps({
                                    "type":"ack",
                                    "command":command,
                                    "status":"success" if ok else "failed",
                                    "robot_alias": robot_alias
                                }))
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
                    elif command == "get_available_robots":
                        try:
                            robots = await self.get_connected_robots_list()
                            await websocket.send(json.dumps({
                                "type": "available_robots_initial_list",
                                "robots": robots,
                                "timestamp": time.time(),
                            }))
                        except Exception as e:
                            await websocket.send(json.dumps({"type":"error","command":command,"message": str(e)}))
                    
                    # ... (other potential commands like 'get_pid', 'set_pid', etc.) ...

                except json.JSONDecodeError:
                    logger.error(f"Failed to parse JSON from {ws_identifier}: {message_str}")
                except Exception as e:
                    logger.error(f"Error processing message from {ws_identifier} ('{message_str}'): {e}", exc_info=True)
        finally:
            log_ws.info(f"WS disconnected: {ws_identifier}")
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
            # Keep logs minimal

def transform_robot_message(message_dict: dict) -> dict:
    """
    Transform incoming robot JSON to a safe, consistent format for the frontend.
    Goals:
    - Never raise on missing/empty fields.
    - Coerce values to expected shapes with sensible defaults.
    - Preserve original payload under generic_* types when unknown.
    The robot_ip and robot_alias fields are filled by the caller after transform.
    """
    # --- Normalization Helpers ---
    # Extend NA token set to catch more firmware variants
    NA_TOKENS = {"na", "n/a", "null", "none", "nan", "n.a", "--", "missing", "invalid"}

    def _is_na(v):
        if v is None:
            return True
        if isinstance(v, str):
            vs = v.strip().lower()
            if vs == "":
                return True
            if vs in NA_TOKENS:
                return True
        return False

    def _num(v, default=0.0):
        """Coerce a value to float with NA handling; returns default on failure."""
        if _is_na(v):
            return float(default)
        try:
            # Strings like '12,3' (comma decimal) -> replace comma
            if isinstance(v, str):
                vv = v.strip().replace(',', '.')
                return float(vv)
            return float(v)
        except Exception:
            return float(default)

    def _list_num(seq, n, default=0.0):
        if not isinstance(seq, (list, tuple)):
            return [float(default) for _ in range(n)]
        out = []
        for i in range(n):
            out.append(_num(seq[i] if i < len(seq) else default, default))
        return out

    transformed_payload = {
        "robot_ip": "PENDING_IP",
        "robot_alias": "PENDING_ALIAS",
        "timestamp": message_dict.get("timestamp", time.time()),
    }

    original_type = message_dict.get("type")
    robot_reported_id = message_dict.get("id")

    # 1) IMU data (ESP32 sends type: "bno055"). Be tolerant of missing/empty fields.
    if original_type == "bno055":
        transformed_payload["type"] = "imu_data"
        src = message_dict.get("data")
        if not isinstance(src, dict):
            src = {}
        # Normalize euler and quaternion to fixed-length lists
        euler = src.get("euler")
        if isinstance(euler, (list, tuple)):
            euler = _list_num(euler, 3, 0.0)
        else:
            # Accept alternative keys or fill defaults
            euler = [
                _num(src.get("heading", src.get("yaw", 0.0)), 0.0),
                _num(src.get("pitch", 0.0), 0.0),
                _num(src.get("roll", 0.0), 0.0)
            ]
        quat = src.get("quaternion")
        if isinstance(quat, (list, tuple)):
            # Preserve identity quaternion default for w=1.0, others 0.0
            w = _num(quat[0] if len(quat) > 0 else 1.0, 1.0)
            x = _num(quat[1] if len(quat) > 1 else 0.0, 0.0)
            y = _num(quat[2] if len(quat) > 2 else 0.0, 0.0)
            z = _num(quat[3] if len(quat) > 3 else 0.0, 0.0)
            quat = [w, x, y, z]
        else:
            quat = [_num(src.get("quat_w", 1.0), 1.0), _num(src.get("quat_x", 0.0)), _num(src.get("quat_y", 0.0)), _num(src.get("quat_z", 0.0))]
        # Additional optional vectors: linear acceleration, gravity, gyro raw; plus status
        def _vec3(obj, key_list, default=(0.0,0.0,0.0)):
            # Try array key first (e.g., lin_accel), then fall back to individual axis keys
            for k in key_list:
                if isinstance(obj.get(k), (list, tuple)):
                    return _list_num(obj.get(k), 3, 0.0)
            # axis-key fallback maps
            axis_map = {
                "lin_accel": ("lin_accel_x","lin_accel_y","lin_accel_z"),
                "gravity": ("gravity_x","gravity_y","gravity_z"),
                "gyro_raw": ("gyro_x","gyro_y","gyro_z"),
            }
            # Use the first name as canonical for axis lookup
            canonical = key_list[0] if key_list else None
            if canonical and canonical in axis_map:
                ax, ay, az = axis_map[canonical]
                return [
                    _num(obj.get(ax, default[0])),
                    _num(obj.get(ay, default[1])),
                    _num(obj.get(az, default[2]))
                ]
            return list(default)

        lin_accel = _vec3(src, ["lin_accel"])
        gravity = _vec3(src, ["gravity"])
        gyro_raw = _vec3(src, ["gyro_raw"])
        status_raw = src.get("status")
        status = None
        if isinstance(status_raw, str):
            s = status_raw.strip()
            status = s if s else None

        transformed_payload["data"] = {
            "time": src.get("time", transformed_payload["timestamp"]),
            "euler": euler,
            "quaternion": quat,
            # Extended fields
            "lin_accel": lin_accel,
            "gravity": gravity,
            "gyro_raw": gyro_raw,
        }
        # Backward-compat for logger expecting 'accel'
        transformed_payload["data"]["accel"] = lin_accel
        if status is not None:
            transformed_payload["data"]["status"] = status
        if robot_reported_id:
            transformed_payload["data"]["robot_reported_id"] = robot_reported_id

    # 2) Encoder data (ESP32 sends type: "encoder"). Accept missing/empty list.
    elif original_type == "encoder":
        transformed_payload["type"] = "encoder_data"
        rpms = message_dict.get("data")
        if not isinstance(rpms, list):
            # Fallback: look for legacy keys rpm_1..rpm_4
            rpms = [message_dict.get("rpm_1"), message_dict.get("rpm_2"), message_dict.get("rpm_3"), message_dict.get("rpm_4")]
            rpms = [v for v in rpms if v is not None]
        # Coerce numbers and keep as-is length (can be 0..N); treat NA tokens as default 0.0
        rpms_coerced = []
        had_na = False
        if isinstance(rpms, list):
            for v in rpms:
                if _is_na(v):
                    had_na = True
                rpms_coerced.append(_num(v, 0.0))
        transformed_payload["data"] = rpms_coerced
        if had_na:
            transformed_payload["encoder_note"] = "one_or_more_values_missing_or_na"
        if robot_reported_id:
            transformed_payload["robot_reported_id"] = robot_reported_id

    # 3) Position data (Robot sends type: "position"). 
    # New format: {"id":"robot1", "type":"position", "source":"ekf", "data":{"position":[x, y, vx, vy, theta]}}
    elif original_type == "position":
        transformed_payload["type"] = "position_update"
        src = message_dict.get("data")
        if not isinstance(src, dict):
            src = {}
        
        # Store source info (ekf, bno055, localization, etc.)
        source = message_dict.get("source", "unknown")
        transformed_payload["source"] = source
        
        pos_arr = src.get("position")
        x, y, vx, vy, theta_rad = 0.0, 0.0, 0.0, 0.0, 0.0
        
        # New format: position array with 5 elements [x, y, vx, vy, theta]
        if isinstance(pos_arr, (list, tuple)) and len(pos_arr) >= 5:
            x = _num(pos_arr[0], 0.0)
            y = _num(pos_arr[1], 0.0)
            vx = _num(pos_arr[2], 0.0)
            vy = _num(pos_arr[3], 0.0)
            theta_rad = _num(pos_arr[4], 0.0)  # Assume already in radians
            
        # Legacy fallback: position array with 3 elements [x, y, theta_deg]
        elif isinstance(pos_arr, (list, tuple)) and len(pos_arr) >= 3:
            x = _num(pos_arr[0], 0.0)
            y = _num(pos_arr[1], 0.0)
            theta_deg = _num(pos_arr[2], 0.0)
            theta_rad = theta_deg * math.pi / 180.0
            # vx, vy remain 0.0
            
        # Legacy fallback: flat dict keys {x, y, theta}
        else:
            if not isinstance(pos_arr, (list, tuple)):
                x = _num(src.get("x", 0.0))
                y = _num(src.get("y", 0.0))
                vx = _num(src.get("vx", 0.0))
                vy = _num(src.get("vy", 0.0))
                # theta may be in degrees or radians; we assume degrees when magnitude > 2π*2
                theta_val = _num(src.get("theta", src.get("heading", 0.0)), 0.0)
                if abs(theta_val) > 6.28318 * 2:  # very likely degrees
                    theta_rad = theta_val * math.pi / 180.0
                else:
                    theta_rad = theta_val
        
        transformed_payload["data"] = {
            "x": x, 
            "y": y, 
            "theta": theta_rad,
            "vx": vx,
            "vy": vy
        }
        if any(_is_na(v) for v in [src.get("x"), src.get("y"), src.get("theta"), src.get("heading")]):
            transformed_payload["position_note"] = "one_or_more_fields_missing_or_na"
        if robot_reported_id:
            transformed_payload["robot_reported_id"] = robot_reported_id

    # 4) Log data (string message), tolerate missing message/level
    elif original_type == "log":
        transformed_payload["type"] = "log"
        raw_msg = message_dict.get("message", "")
        if _is_na(raw_msg):
            raw_msg = ""
        transformed_payload["message"] = raw_msg
        level_raw = message_dict.get("level", "debug")
        if _is_na(level_raw):
            level_raw = "debug"
        transformed_payload["level"] = str(level_raw).lower()
        if robot_reported_id:
            transformed_payload["robot_reported_id"] = robot_reported_id

    # 5) Registration handshake
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

    # 6) Generic passthrough if there's a type but not matched above
    elif original_type:
        transformed_payload["type"] = f"generic_{original_type}"
        # Ensure we pass a shallow copy to avoid accidental mutation
        try:
            transformed_payload["data"] = message_dict.copy()
        except Exception:
            transformed_payload["data"] = {"raw": str(message_dict)}

    # 7) Unknown without a type: classify and include raw
    else:
        transformed_payload["type"] = "unknown_json_data"
        try:
            transformed_payload["data"] = message_dict.copy()
        except Exception:
            transformed_payload["data"] = {"raw": str(message_dict)}
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
    logger.info(f"DirectBridge running. Control TCP on {args.tcp_port}, WebSocket on {args.ws_port}.")
    try:
        while True:
            await asyncio.sleep(3600)
    except KeyboardInterrupt:
        logger.info("DirectBridge stopping...")
    finally:
        data_logger.close_logs()
        # OTA disabled in RasPi-only mode: no server to stop
        logger.info("DirectBridge stopped.")

if __name__ == "__main__":
    try:
        asyncio.run(main_bridge_runner())
    except KeyboardInterrupt:
        logger.info("Application terminated by user.")