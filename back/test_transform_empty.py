import json
import time
import sys

try:
    # If running from repo root
    from back.direct_bridge import transform_robot_message
except Exception:
    # Fallback when executed from back/ dir
    from direct_bridge import transform_robot_message


def run_case(name, payload):
    try:
        out = transform_robot_message(payload)
        print(f"CASE {name}: OK | type={out.get('type')} | data_keys={list(out.get('data', {})) if isinstance(out.get('data'), dict) else 'list'} | len={len(out.get('data', [])) if isinstance(out.get('data'), list) else '-'}")
    except Exception as e:
        print(f"CASE {name}: ERROR -> {e}")


def main():
    cases = {
        "imu_empty": {"type": "bno055", "data": {}},
        "imu_partial_euler": {"type": "bno055", "data": {"euler": [None, "", 45]}},
        "imu_no_data": {"type": "bno055"},
        "encoder_empty": {"type": "encoder", "data": []},
        "encoder_missing": {"type": "encoder"},
        "encoder_legacy": {"type": "encoder", "rpm_1": 10, "rpm_2": None, "rpm_3": "", "rpm_4": 40},
        "position_empty": {"type": "position", "data": {}},
        "position_partial": {"type": "position", "data": {"position": [1.2]}},
        "unknown": {"foo": "bar"},
        "log_missing_fields": {"type": "log"},
    }
    for name, payload in cases.items():
        run_case(name, payload)


if __name__ == "__main__":
    main()
