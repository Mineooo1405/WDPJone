"""Small test runner for back/database.py
Creates a Database instance, upserts a robot, inserts sample encoder/imu/position/log rows,
and prints row counts to confirm tables and inserts work.

Run: python back/test_db_runner.py
"""
import time
from back import database as dbmod


def main():
    print("Initializing Database (default sqlite:///telemetry.db)...")
    db = dbmod.Database()  # uses DB_URL env or sqlite:///telemetry.db

    unique = f"test_robot:{int(time.time())}"
    print(f"Upserting robot {unique}...")
    robot = db.upsert_robot(unique, alias="testbot", ip="127.0.0.1", robot_type="omni", status="connected")
    print("Robot id:", robot.id, "unique_key:", robot.unique_key)

    print("Inserting encoder sample...")
    enc_id = db.insert_encoder(unique, [10.0, 11.0, 12.0], timestamp=time.time())
    print("Encoder row id:", enc_id)

    print("Inserting IMU sample...")
    imu_id = db.insert_imu(unique, {"euler": [0.1, 0.2, 0.3], "quaternion": [1,0,0,0]}, timestamp=time.time())
    print("IMU row id:", imu_id)

    print("Inserting position...")
    pos_id = db.insert_position(unique, 1.23, 4.56, 0.78, timestamp=time.time())
    print("Position row id:", pos_id)

    print("Inserting log...")
    log_id = db.insert_log(unique, "Test log message", level="DEBUG", timestamp=time.time())
    print("Log row id:", log_id)

    # Query counts
    with db._get_session() as s:
        robot_count = s.query(dbmod.Robot).count()
        enc_count = s.query(dbmod.EncoderValues).filter_by(robot_id=robot.id).count()
        imu_count = s.query(dbmod.IMUBNO055).filter_by(robot_id=robot.id).count()
        pos_count = s.query(dbmod.PositionUpdate).filter_by(robot_id=robot.id).count()
        log_count = s.query(dbmod.RobotLog).filter_by(robot_id=robot.id).count()

    print("Counts (for this robot): enc=", enc_count, "imu=", imu_count, "pos=", pos_count, "log=", log_count)
    print("Total robots in DB:", robot_count)
    print("Test completed.")


if __name__ == '__main__':
    main()
