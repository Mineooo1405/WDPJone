import math
import time
from direct_bridge import transform_robot_message

# Comprehensive NA token coverage based on backend NA_TOKENS set
NA_VARIANTS = ["N/A","na","NaN","none","Missing","INVALID","--","n.a","null",""]


def test_encoder_na_tokens_and_note():
    raw = {"type": "encoder", "data": [10, "N/A", "--", 25, "invalid", 0]}
    out = transform_robot_message(raw)
    assert out["type"] == "encoder_data"
    # NA entries coerced to 0.0
    assert out["data"][0] == 10.0
    assert out["data"][1] == 0.0
    assert out["data"][2] == 0.0
    assert out["data"][3] == 25.0
    assert out["data"][4] == 0.0
    assert out["data"][5] == 0.0
    assert out.get("encoder_note") == "one_or_more_values_missing_or_na"


def test_position_na_and_note():
    raw = {"type": "position", "data": {"x": "N/A", "y": 1.2, "theta": "--"}}
    out = transform_robot_message(raw)
    assert out["type"] == "position_update"
    assert out["data"]["x"] == 0.0  # NA -> 0
    assert out["data"]["y"] == 1.2
    # theta "--" -> 0.0 rad
    assert out["data"]["theta"] == 0.0
    assert out.get("position_note") == "one_or_more_fields_missing_or_na"


def test_imu_na_arrays():
    raw = {"type": "bno055", "data": {"euler": ["N/A","--", 30], "quaternion": ["N/A", 0.1, None, "nan"]}}
    out = transform_robot_message(raw)
    assert out["type"] == "imu_data"
    # Euler sanitized: first two NA -> 0, last numeric -> 30
    assert out["data"]["euler"] == [0.0, 0.0, 30.0]
    # Quaternion: first NA -> 1.0 default for w, 0.1 kept, None -> 0.0, 'nan' -> 0.0
    q = out["data"]["quaternion"]
    assert q[0] == 1.0
    assert abs(q[1] - 0.1) < 1e-6
    assert q[2] == 0.0
    assert q[3] == 0.0


def test_mixed_case_na_tokens():
    # Mix multiple NA variants in encoder
    raw = {"type": "encoder", "data": ["Na", "missing", "INVALID", "n.a", "null"]}
    out = transform_robot_message(raw)
    assert out["type"] == "encoder_data"
    assert all(v == 0.0 for v in out["data"])  # all NA -> zeros
    assert out.get("encoder_note") == "one_or_more_values_missing_or_na"


if __name__ == "__main__":
    tests = [
        test_encoder_na_tokens_and_note,
        test_position_na_and_note,
        test_imu_na_arrays,
        test_mixed_case_na_tokens,
    ]
    for t in tests:
        t()
    print("All NA handling tests passed")
