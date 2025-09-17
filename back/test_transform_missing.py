import time
from direct_bridge import transform_robot_message

def test_missing_imu_fields():
    raw = {"type": "bno055", "data": {"euler": ["N/A", 10, None], "quaternion": [1, "N/A", None, 0.5]}}
    out = transform_robot_message(raw)
    assert out["type"] == "imu_data"
    assert out["data"]["euler"] == [0.0, 10.0, 0.0]
    assert out["data"]["quaternion"][0] == 1.0
    assert out["data"]["quaternion"][1] == 0.0

def test_missing_encoder_list():
    raw = {"type": "encoder", "rpm_1": "N/A", "rpm_2": 50, "rpm_3": None}
    out = transform_robot_message(raw)
    assert out["type"] == "encoder_data"
    assert out["data"] == [0.0, 50.0]  # rpm_3 None removed, rpm_1 N/A -> 0.0

def test_position_flat_keys_and_degrees():
    raw = {"type": "position", "data": {"x": "1.5", "y": "N/A", "theta": 180}}
    out = transform_robot_message(raw)
    assert out["type"] == "position_update"
    # y becomes 0.0, theta 180 deg -> pi rad
    assert abs(out["data"]["x"] - 1.5) < 1e-6
    assert abs(out["data"]["y"] - 0.0) < 1e-6
    assert abs(out["data"]["theta"] - 3.14159265) < 1e-3

def test_log_na_values():
    raw = {"type": "log", "message": "N/A", "level": "N/A"}
    out = transform_robot_message(raw)
    assert out["type"] == "log"
    assert out["message"] == ""
    assert out["level"] == "debug"

if __name__ == "__main__":
    # rudimentary runner
    for fn in [test_missing_imu_fields, test_missing_encoder_list, test_position_flat_keys_and_degrees, test_log_na_values]:
        fn()
    print("All transform missing-field tests passed")