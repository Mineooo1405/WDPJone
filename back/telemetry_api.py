from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from typing import List
import os
import datetime as dt

from back.database import Database, Robot, PositionUpdate, EncoderValues, IMUBNO055, RobotLog

app = FastAPI(title="Telemetry API")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize DB. Uses DB_URL env or sqlite:///telemetry.db by default
DB_URL = os.environ.get("DB_URL")
_db = Database(DB_URL)


def _dt_to_iso(v: dt.datetime) -> str:
    if v is None:
        return None
    if isinstance(v, str):
        return v
    return v.isoformat() + "Z"


@app.get("/health")
def health():
    return {"status": "ok"}


@app.get("/robots")
def list_robots():
    with _db._get_session() as s:
        rows = s.query(Robot).all()
        out = []
        for r in rows:
            out.append({
                "id": r.id,
                "unique_key": r.unique_key,
                "alias": r.alias,
                "ip": r.ip,
                "robot_type": r.robot_type,
                "status": r.status,
                "first_seen": _dt_to_iso(r.first_seen),
                "last_seen": _dt_to_iso(r.last_seen),
            })
        return out


@app.get("/robots/{unique_key}/latest")
def get_latest_position(unique_key: str):
    with _db._get_session() as s:
        r = s.query(Robot).filter_by(unique_key=unique_key).first()
        if not r:
            raise HTTPException(status_code=404, detail="robot not found")
        row = (
            s.query(PositionUpdate)
            .filter_by(robot_id=r.id)
            .order_by(PositionUpdate.created_at.desc())
            .limit(1)
            .first()
        )
        if not row:
            return {"position": None}
        return {
            "position": {"x": row.x, "y": row.y, "theta": row.theta, "timestamp": row.timestamp},
            "created_at": _dt_to_iso(row.created_at),
        }


@app.get("/robots/{unique_key}/positions")
def get_positions(unique_key: str, limit: int = 100):
    with _db._get_session() as s:
        r = s.query(Robot).filter_by(unique_key=unique_key).first()
        if not r:
            raise HTTPException(status_code=404, detail="robot not found")
        rows = (
            s.query(PositionUpdate)
            .filter_by(robot_id=r.id)
            .order_by(PositionUpdate.created_at.desc())
            .limit(limit)
            .all()
        )
        return [
            {"x": row.x, "y": row.y, "theta": row.theta, "timestamp": row.timestamp, "created_at": _dt_to_iso(row.created_at)}
            for row in rows
        ]


@app.get("/robots/{unique_key}/encoders")
def get_encoders(unique_key: str, limit: int = 200):
    with _db._get_session() as s:
        r = s.query(Robot).filter_by(unique_key=unique_key).first()
        if not r:
            raise HTTPException(status_code=404, detail="robot not found")
        rows = (
            s.query(EncoderValues)
            .filter_by(robot_id=r.id)
            .order_by(EncoderValues.created_at.desc())
            .limit(limit)
            .all()
        )
        return [
            {"data": [row.data_value1, row.data_value2, row.data_value3, row.data_value4], "timestamp": row.timestamp, "created_at": _dt_to_iso(row.created_at)}
            for row in rows
        ]


@app.get("/robots/{unique_key}/imu")
def get_imus(unique_key: str, limit: int = 200):
    with _db._get_session() as s:
        r = s.query(Robot).filter_by(unique_key=unique_key).first()
        if not r:
            raise HTTPException(status_code=404, detail="robot not found")
        rows = (
            s.query(IMUBNO055)
            .filter_by(robot_id=r.id)
            .order_by(IMUBNO055.created_at.desc())
            .limit(limit)
            .all()
        )
        return [
            {"euler": [row.euler_roll, row.euler_pitch, row.euler_yaw], "quaternion": [row.quaternion_w, row.quaternion_x, row.quaternion_y, row.quaternion_z], "timestamp": row.timestamp, "created_at": _dt_to_iso(row.created_at)}
            for row in rows
        ]


@app.get("/robots/{unique_key}/logs")
def get_logs(unique_key: str, limit: int = 200):
    with _db._get_session() as s:
        r = s.query(Robot).filter_by(unique_key=unique_key).first()
        if not r:
            raise HTTPException(status_code=404, detail="robot not found")
        rows = (
            s.query(RobotLog)
            .filter_by(robot_id=r.id)
            .order_by(RobotLog.created_at.desc())
            .limit(limit)
            .all()
        )
        return [
            {"message": row.message, "level": row.level, "timestamp": row.timestamp, "created_at": _dt_to_iso(row.created_at)}
            for row in rows
        ]
