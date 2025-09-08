import os
import datetime as dt
from typing import List, Optional

from sqlalchemy import (
    create_engine,
    Integer,
    Float,
    String,
    DateTime,
    Text,
    ForeignKey,
)
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column, relationship, sessionmaker


class Base(DeclarativeBase):
    pass


class Robot(Base):
    __tablename__ = "robots"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    unique_key: Mapped[str] = mapped_column(String(128), unique=True, nullable=False)
    alias: Mapped[Optional[str]] = mapped_column(String(128))
    ip: Mapped[Optional[str]] = mapped_column(String(64))
    robot_type: Mapped[str] = mapped_column(String(32), default="omni")
    status: Mapped[str] = mapped_column(String(32), default="disconnected")
    first_seen: Mapped[dt.datetime] = mapped_column(DateTime, default=lambda: dt.datetime.utcnow())
    last_seen: Mapped[dt.datetime] = mapped_column(DateTime, default=lambda: dt.datetime.utcnow(), onupdate=lambda: dt.datetime.utcnow())

    encoders: Mapped[List["EncoderValues"]] = relationship(back_populates="robot", cascade="all, delete-orphan")
    imus: Mapped[List["IMUBNO055"]] = relationship(back_populates="robot", cascade="all, delete-orphan")
    positions: Mapped[List["PositionUpdate"]] = relationship(back_populates="robot", cascade="all, delete-orphan")
    logs: Mapped[List["RobotLog"]] = relationship(back_populates="robot", cascade="all, delete-orphan")


class EncoderValues(Base):
    __tablename__ = "encoder_values"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    robot_id: Mapped[int] = mapped_column(ForeignKey("robots.id"), index=True)
    # Align with frontend Adapters expectations (data_value1..3/4)
    data_value1: Mapped[float] = mapped_column(Float, default=0.0)
    data_value2: Mapped[float] = mapped_column(Float, default=0.0)
    data_value3: Mapped[float] = mapped_column(Float, default=0.0)
    data_value4: Mapped[Optional[float]] = mapped_column(Float, nullable=True)
    timestamp: Mapped[Optional[float]] = mapped_column(Float, nullable=True)
    created_at: Mapped[dt.datetime] = mapped_column(DateTime, default=lambda: dt.datetime.utcnow())

    robot: Mapped[Robot] = relationship(back_populates="encoders")


class IMUBNO055(Base):
    __tablename__ = "imu_bno055"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    robot_id: Mapped[int] = mapped_column(ForeignKey("robots.id"), index=True)
    euler_roll: Mapped[float] = mapped_column(Float, default=0.0)
    euler_pitch: Mapped[float] = mapped_column(Float, default=0.0)
    euler_yaw: Mapped[float] = mapped_column(Float, default=0.0)
    quaternion_w: Mapped[float] = mapped_column(Float, default=1.0)
    quaternion_x: Mapped[float] = mapped_column(Float, default=0.0)
    quaternion_y: Mapped[float] = mapped_column(Float, default=0.0)
    quaternion_z: Mapped[float] = mapped_column(Float, default=0.0)
    sensor_time: Mapped[Optional[float]] = mapped_column(Float, nullable=True)
    timestamp: Mapped[Optional[float]] = mapped_column(Float, nullable=True)
    created_at: Mapped[dt.datetime] = mapped_column(DateTime, default=lambda: dt.datetime.utcnow())

    robot: Mapped[Robot] = relationship(back_populates="imus")


class PositionUpdate(Base):
    __tablename__ = "position_updates"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    robot_id: Mapped[int] = mapped_column(ForeignKey("robots.id"), index=True)
    x: Mapped[float] = mapped_column(Float, default=0.0)
    y: Mapped[float] = mapped_column(Float, default=0.0)
    theta: Mapped[float] = mapped_column(Float, default=0.0)
    timestamp: Mapped[Optional[float]] = mapped_column(Float, nullable=True)
    created_at: Mapped[dt.datetime] = mapped_column(DateTime, default=lambda: dt.datetime.utcnow())

    robot: Mapped[Robot] = relationship(back_populates="positions")


class RobotLog(Base):
    __tablename__ = "robot_logs"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    robot_id: Mapped[int] = mapped_column(ForeignKey("robots.id"), index=True)
    message: Mapped[str] = mapped_column(Text)
    level: Mapped[Optional[str]] = mapped_column(String(32), default="INFO")
    timestamp: Mapped[Optional[float]] = mapped_column(Float, nullable=True)
    created_at: Mapped[dt.datetime] = mapped_column(DateTime, default=lambda: dt.datetime.utcnow())

    robot: Mapped[Robot] = relationship(back_populates="logs")


class Database:
    """Lightweight DB wrapper for SQLite (or any SQLAlchemy URL) with helper upsert/insert methods."""

    def __init__(self, url: Optional[str] = None):
        db_url = url or os.environ.get("DB_URL") or "sqlite:///telemetry.db"
        connect_args = {"check_same_thread": False} if db_url.startswith("sqlite") else {}
        self.engine = create_engine(db_url, echo=False, future=True, connect_args=connect_args)
        self.SessionLocal = sessionmaker(bind=self.engine, expire_on_commit=False, autoflush=False, autocommit=False)
        Base.metadata.create_all(self.engine)

    # --- Robot helpers ---
    def _get_session(self):
        return self.SessionLocal()

    def get_or_create_robot(self, unique_key: str, alias: Optional[str] = None, ip: Optional[str] = None, robot_type: Optional[str] = None):
        with self._get_session() as s:
            r = s.query(Robot).filter_by(unique_key=unique_key).first()
            if r is None:
                r = Robot(unique_key=unique_key)
                s.add(r)
            # Update metadata
            if alias:
                r.alias = alias
            if ip:
                r.ip = ip
            if robot_type:
                r.robot_type = robot_type
            r.last_seen = dt.datetime.utcnow()
            if r.first_seen is None:
                r.first_seen = dt.datetime.utcnow()
            s.commit()
            s.refresh(r)
            return r

    def upsert_robot(self, unique_key: str, alias: Optional[str], ip: Optional[str], robot_type: Optional[str], status: Optional[str]):
        with self._get_session() as s:
            r = s.query(Robot).filter_by(unique_key=unique_key).first()
            if r is None:
                r = Robot(unique_key=unique_key)
                s.add(r)
            if alias is not None:
                r.alias = alias
            if ip is not None:
                r.ip = ip
            if robot_type is not None:
                r.robot_type = robot_type
            if status is not None:
                r.status = status
            now = dt.datetime.utcnow()
            if r.first_seen is None:
                r.first_seen = now
            r.last_seen = now
            s.commit()
            return r

    def set_robot_status(self, unique_key: str, status: str):
        with self._get_session() as s:
            r = s.query(Robot).filter_by(unique_key=unique_key).first()
            if r is None:
                return
            r.status = status
            r.last_seen = dt.datetime.utcnow()
            s.commit()

    def update_robot_type(self, unique_key: str, robot_type: str):
        with self._get_session() as s:
            r = s.query(Robot).filter_by(unique_key=unique_key).first()
            if r is None:
                return
            r.robot_type = robot_type
            r.last_seen = dt.datetime.utcnow()
            s.commit()

    # --- Insert helpers ---
    def _robot_id(self, s, unique_key: str) -> Optional[int]:
        r = s.query(Robot).filter_by(unique_key=unique_key).first()
        return r.id if r else None

    def insert_encoder(self, unique_key: str, values: List[float], timestamp: Optional[float] = None):
        with self._get_session() as s:
            rid = self._robot_id(s, unique_key)
            if rid is None:
                # Create robot placeholder
                r = Robot(unique_key=unique_key)
                s.add(r)
                s.flush()
                rid = r.id
            v1 = values[0] if len(values) > 0 else 0.0
            v2 = values[1] if len(values) > 1 else 0.0
            v3 = values[2] if len(values) > 2 else 0.0
            v4 = values[3] if len(values) > 3 else None
            row = EncoderValues(robot_id=rid, data_value1=v1, data_value2=v2, data_value3=v3, data_value4=v4, timestamp=timestamp)
            s.add(row)
            s.commit()
            return row.id

    def insert_imu(self, unique_key: str, data: dict, timestamp: Optional[float] = None):
        with self._get_session() as s:
            rid = self._robot_id(s, unique_key)
            if rid is None:
                r = Robot(unique_key=unique_key)
                s.add(r)
                s.flush()
                rid = r.id
            # Accept either keys yaw/pitch/roll or euler list and quaternion list
            roll = 0.0; pitch = 0.0; yaw = 0.0
            if isinstance(data.get("euler"), list) and len(data["euler"]) == 3:
                roll, pitch, yaw = data["euler"]
            else:
                roll = float(data.get("roll", 0.0)) if "roll" in data else 0.0
                pitch = float(data.get("pitch", 0.0)) if "pitch" in data else 0.0
                yaw = float(data.get("yaw", 0.0)) if "yaw" in data else 0.0
            quat = data.get("quaternion") or data.get("quat") or [1.0, 0.0, 0.0, 0.0]
            if not isinstance(quat, list) or len(quat) != 4:
                quat = [1.0, 0.0, 0.0, 0.0]
            sensor_time = data.get("time") or data.get("sensor_time")
            row = IMUBNO055(
                robot_id=rid,
                euler_roll=float(roll),
                euler_pitch=float(pitch),
                euler_yaw=float(yaw),
                quaternion_w=float(quat[0]),
                quaternion_x=float(quat[1]),
                quaternion_y=float(quat[2]),
                quaternion_z=float(quat[3]),
                sensor_time=float(sensor_time) if sensor_time is not None else None,
                timestamp=timestamp,
            )
            s.add(row)
            s.commit()
            return row.id

    def insert_position(self, unique_key: str, x: float, y: float, theta: float, timestamp: Optional[float] = None):
        with self._get_session() as s:
            rid = self._robot_id(s, unique_key)
            if rid is None:
                r = Robot(unique_key=unique_key)
                s.add(r)
                s.flush()
                rid = r.id
            row = PositionUpdate(robot_id=rid, x=float(x), y=float(y), theta=float(theta), timestamp=timestamp)
            s.add(row)
            s.commit()
            return row.id

    def insert_log(self, unique_key: str, message: str, level: str = "INFO", timestamp: Optional[float] = None):
        with self._get_session() as s:
            rid = self._robot_id(s, unique_key)
            if rid is None:
                r = Robot(unique_key=unique_key)
                s.add(r)
                s.flush()
                rid = r.id
            row = RobotLog(robot_id=rid, message=message, level=level, timestamp=timestamp)
            s.add(row)
            s.commit()
            return row.id
