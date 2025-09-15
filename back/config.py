"""
Config tập trung cho Backend (DirectBridge, Start GUI, v.v.)

Mục tiêu:
- Gom tất cả biến cấu hình (port, đường dẫn, tham số cơ học, tuỳ chọn bridge, ...) vào một nơi.
- Cho phép ghi đè qua file .env hoặc env_config.txt (đặt cạnh thư mục back/), hoặc qua biến môi trường hệ thống.
- Cung cấp đối tượng settings đã ép kiểu để các module khác dùng trực tiếp.

Ưu tiên nguồn cấu hình (từ thấp đến cao):
  1) Giá trị mặc định trong mã nguồn
  2) env_config.txt (đặt tại back/env_config.txt)
  3) .env (đặt tại project root hoặc back/)
  4) Biến môi trường hệ thống (os.environ)

Cách dùng nhanh:
  from back.config import settings
  print(settings.ws_bridge_port)
  print(settings.imu_euler_yaw_index)

Khi import, module này sẽ nạp env_config.txt và .env rồi tạo đối tượng `settings`.
"""

from __future__ import annotations
import os
from dataclasses import dataclass
from typing import Optional

try:
    from dotenv import load_dotenv  # type: ignore
except Exception:
    def load_dotenv(*args, **kwargs):  # fallback no-op
        return False

# --- Nạp cấu hình từ file ---
def _load_env_files() -> None:
    """Nạp .env và env_config.txt (nếu có). Không lỗi nếu thiếu."""
    # 1) .env (project root hoặc back/)
    try:
        # Thử nạp .env ở CWD
        load_dotenv()
    except Exception:
        pass
    # 2) env_config.txt đặt cạnh file này (thư mục back/)
    try:
        this_dir = os.path.dirname(os.path.abspath(__file__))
        env_cfg_path = os.path.join(this_dir, 'env_config.txt')
        if os.path.exists(env_cfg_path):
            load_dotenv(env_cfg_path, override=True)
    except Exception:
        pass


def _get_str(name: str, default: str) -> str:
    val = os.environ.get(name)
    return val if val is not None and val != '' else default

def _get_int(name: str, default: int) -> int:
    try:
        return int(_get_str(name, str(default)))
    except Exception:
        return default

def _get_float(name: str, default: float) -> float:
    try:
        return float(_get_str(name, str(default)))
    except Exception:
        return default

def _get_bool(name: str, default: bool) -> bool:
    raw = _get_str(name, str(default)).strip().lower()
    return raw not in ('0', 'false', 'no', 'off', '')


@dataclass(frozen=True)
class Settings:
    # Cổng/địa chỉ
    tcp_port: int = 12346              # TCP server cho điều khiển robot
    ws_bridge_port: int = 9003         # WebSocket Bridge cho UI
    ota_port: int = 12345              # TCP cho OTA
    log_level: str = 'INFO'
    log_directory: str = 'logs/bridge_logs'

    # File cấu hình PID, thư mục firmware tạm
    pid_config_file: str = 'pid_config.txt'
    temp_firmware_dir: str = 'temp_firmware'

    # Tuỳ chọn WebSocket cho UI (nếu dùng)
    ws_ping_interval: float = 20.0
    ws_ping_timeout: float = 20.0
    ws_max_size: int = 2_000_000

    # Kinematics / cảm biến
    wheel_radius: float = 0.0325
    mecanum_lx: float = 0.1           # nửa bề rộng (m)
    mecanum_ly: float = 0.1           # nửa chiều dài (m)
    mecanum_lsum: Optional[float] = None  # nếu đặt, ưu tiên dùng (lx+ly)
    robot_radius: Optional[float] = None  # nếu đặt và lx/ly không đặt, dùng 2*robot_radius
    mecanum_vy_sign: float = 1.0
    mecanum_omega_sign: float = 1.0
    imu_euler_yaw_index: int = 0      # chỉ số yaw trong euler [0..2]

    # Broadcast/raw status
    broadcast_all_status: bool = True
    broadcast_all_types: bool = False
    broadcast_min_interval: float = 0.05


def _build_settings() -> Settings:
    _load_env_files()

    s = Settings(
        tcp_port=_get_int('TCP_PORT', 12346),
        ws_bridge_port=_get_int('WS_BRIDGE_PORT', 9003),
        ota_port=_get_int('OTA_PORT', 12345),
        log_level=_get_str('LOG_LEVEL', 'INFO').upper(),
        log_directory=_get_str('LOG_DIRECTORY', 'logs/bridge_logs'),
        pid_config_file=_get_str('PID_CONFIG_FILE', 'pid_config.txt'),
        temp_firmware_dir=_get_str('TEMP_FIRMWARE_DIR', 'temp_firmware'),
        ws_ping_interval=_get_float('WS_PING_INTERVAL', 20.0),
        ws_ping_timeout=_get_float('WS_PING_TIMEOUT', 20.0),
        ws_max_size=_get_int('WS_MAX_SIZE', 2_000_000),
        wheel_radius=_get_float('WHEEL_RADIUS', 0.0325),
        mecanum_lx=_get_float('MECANUM_LX', 0.1),
        mecanum_ly=_get_float('MECANUM_LY', 0.1),
        mecanum_lsum=None if _get_str('MECANUM_LSUM', '') == '' else _get_float('MECANUM_LSUM', 0.0),
        robot_radius=None if _get_str('ROBOT_RADIUS', '') == '' else _get_float('ROBOT_RADIUS', 0.0),
        mecanum_vy_sign=_get_float('MECANUM_VY_SIGN', 1.0),
        mecanum_omega_sign=_get_float('MECANUM_OMEGA_SIGN', 1.0),
        imu_euler_yaw_index=max(0, min(2, _get_int('IMU_EULER_YAW_INDEX', 0))),
        broadcast_all_status=_get_bool('BRIDGE_BROADCAST_ALL_STATUS', True),
        broadcast_all_types=_get_bool('BRIDGE_BROADCAST_ALL_TYPES', False),
        broadcast_min_interval=_get_float('BRIDGE_BROADCAST_MIN_INTERVAL', 0.05),
    )

    # Đồng bộ ngược sang os.environ để các module đang dùng os.environ.get vẫn nhận đúng giá trị
    os.environ.setdefault('TCP_PORT', str(s.tcp_port))
    os.environ.setdefault('WS_BRIDGE_PORT', str(s.ws_bridge_port))
    os.environ.setdefault('OTA_PORT', str(s.ota_port))
    os.environ.setdefault('LOG_LEVEL', s.log_level)
    os.environ.setdefault('LOG_DIRECTORY', s.log_directory)
    os.environ.setdefault('PID_CONFIG_FILE', s.pid_config_file)
    os.environ.setdefault('TEMP_FIRMWARE_DIR', s.temp_firmware_dir)
    os.environ.setdefault('WS_PING_INTERVAL', str(s.ws_ping_interval))
    os.environ.setdefault('WS_PING_TIMEOUT', str(s.ws_ping_timeout))
    os.environ.setdefault('WS_MAX_SIZE', str(s.ws_max_size))
    os.environ.setdefault('WHEEL_RADIUS', str(s.wheel_radius))
    os.environ.setdefault('MECANUM_LX', str(s.mecanum_lx))
    os.environ.setdefault('MECANUM_LY', str(s.mecanum_ly))
    if s.mecanum_lsum is not None:
        os.environ.setdefault('MECANUM_LSUM', str(s.mecanum_lsum))
    if s.robot_radius is not None:
        os.environ.setdefault('ROBOT_RADIUS', str(s.robot_radius))
    os.environ.setdefault('MECANUM_VY_SIGN', str(s.mecanum_vy_sign))
    os.environ.setdefault('MECANUM_OMEGA_SIGN', str(s.mecanum_omega_sign))
    os.environ.setdefault('IMU_EULER_YAW_INDEX', str(s.imu_euler_yaw_index))
    os.environ.setdefault('BRIDGE_BROADCAST_ALL_STATUS', '1' if s.broadcast_all_status else '0')
    os.environ.setdefault('BRIDGE_BROADCAST_ALL_TYPES', '1' if s.broadcast_all_types else '0')
    os.environ.setdefault('BRIDGE_BROADCAST_MIN_INTERVAL', str(s.broadcast_min_interval))

    return s


# Đối tượng cấu hình dùng chung
settings: Settings = _build_settings()

# Cho phép nạp lại trong runtime nếu cần
def refresh_settings() -> Settings:
    global settings
    settings = _build_settings()
    return settings
