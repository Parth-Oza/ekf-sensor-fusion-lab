"""Replay standard ROS 2 IMU and odometry messages through the EKF."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
from rosbags.highlevel import AnyReader, AnyReaderError
from rosbags.rosbag2 import ReaderError
from rosbags.serde.errors import SerdeError
from rosbags.typesys import Stores, get_typestore

from .ekf import ConstantVelocityEKF

IMU_MESSAGE_TYPE = "sensor_msgs/msg/Imu"
ODOMETRY_MESSAGE_TYPE = "nav_msgs/msg/Odometry"


class BagReplayError(ValueError):
    """Raised when a bag cannot satisfy the documented replay contract."""


@dataclass(frozen=True, slots=True)
class ReplayConfig:
    imu_topic: str = "/imu/data"
    odometry_topic: str = "/odom"
    acceleration_variance: float = 0.35
    position_variance: float = 0.25
    velocity_variance: float = 0.04

    def __post_init__(self) -> None:
        for name in ("acceleration_variance", "position_variance", "velocity_variance"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True, slots=True)
class TrajectoryPoint:
    timestamp_ns: int
    estimate_x: float
    estimate_y: float
    estimate_vx: float
    estimate_vy: float
    odometry_x: float
    odometry_y: float
    odometry_vx: float
    odometry_vy: float
    position_residual_m: float


@dataclass(frozen=True, slots=True)
class ReplayResult:
    trajectory: tuple[TrajectoryPoint, ...]
    metrics: dict[str, float | int | str]


def _validate_connections(reader: AnyReader, config: ReplayConfig) -> list[Any]:
    expected = {
        config.imu_topic: IMU_MESSAGE_TYPE,
        config.odometry_topic: ODOMETRY_MESSAGE_TYPE,
    }
    selected = [
        connection for connection in reader.connections if connection.topic in expected
    ]
    present_topics = {connection.topic for connection in selected}
    missing = [topic for topic in expected if topic not in present_topics]
    if missing:
        raise BagReplayError(f"Bag is missing required topic(s): {', '.join(missing)}")
    for connection in selected:
        message_type = expected[connection.topic]
        if connection.msgtype != message_type:
            raise BagReplayError(
                f"Topic {connection.topic!r} has type {connection.msgtype!r}; "
                f"expected {message_type!r}"
            )
    return selected


def replay_bag(path: str | Path, config: ReplayConfig | None = None) -> ReplayResult:
    """Replay a rosbag2 directory or storage file and return trajectory data."""

    replay_config = config or ReplayConfig()
    filter_ = ConstantVelocityEKF(
        acceleration_variance=replay_config.acceleration_variance
    )
    acceleration = np.zeros(2, dtype=float)
    trajectory: list[TrajectoryPoint] = []
    imu_messages = 0
    odometry_messages = 0
    first_timestamp: int | None = None
    previous_timestamp: int | None = None
    typestore = get_typestore(Stores.ROS2_JAZZY)

    try:
        with AnyReader([Path(path)], default_typestore=typestore) as reader:
            connections = _validate_connections(reader, replay_config)
            for connection, timestamp, rawdata in reader.messages(
                connections=connections
            ):
                if previous_timestamp is not None:
                    if timestamp < previous_timestamp:
                        raise BagReplayError("Bag messages are not in timestamp order")
                    elapsed = (timestamp - previous_timestamp) / 1_000_000_000
                    if elapsed > 0:
                        filter_.predict(elapsed, acceleration=acceleration)
                else:
                    first_timestamp = timestamp
                previous_timestamp = timestamp
                message = reader.deserialize(rawdata, connection.msgtype)

                if connection.topic == replay_config.imu_topic:
                    acceleration = np.array(
                        [message.linear_acceleration.x, message.linear_acceleration.y],
                        dtype=float,
                    )
                    if not np.all(np.isfinite(acceleration)):
                        raise BagReplayError(
                            "IMU acceleration contains a non-finite value"
                        )
                    imu_messages += 1
                    continue

                odometry_messages += 1
                position = np.array(
                    [message.pose.pose.position.x, message.pose.pose.position.y],
                    dtype=float,
                )
                velocity = np.array(
                    [message.twist.twist.linear.x, message.twist.twist.linear.y],
                    dtype=float,
                )
                if not np.all(np.isfinite(position)) or not np.all(
                    np.isfinite(velocity)
                ):
                    raise BagReplayError("Odometry contains a non-finite value")
                filter_.update_position(
                    position, variance=replay_config.position_variance
                )
                filter_.update_velocity(
                    velocity, variance=replay_config.velocity_variance
                )
                residual = float(np.linalg.norm(filter_.state[:2] - position))
                trajectory.append(
                    TrajectoryPoint(
                        timestamp_ns=timestamp,
                        estimate_x=float(filter_.state[0]),
                        estimate_y=float(filter_.state[1]),
                        estimate_vx=float(filter_.state[2]),
                        estimate_vy=float(filter_.state[3]),
                        odometry_x=float(position[0]),
                        odometry_y=float(position[1]),
                        odometry_vx=float(velocity[0]),
                        odometry_vy=float(velocity[1]),
                        position_residual_m=residual,
                    )
                )
    except BagReplayError:
        raise
    except (
        AnyReaderError,
        OSError,
        ReaderError,
        RuntimeError,
        SerdeError,
        ValueError,
    ) as exc:
        raise BagReplayError(f"Could not replay bag {path}: {exc}") from exc

    if not trajectory:
        raise BagReplayError("Bag contains no odometry messages")
    residuals = np.array([point.position_residual_m for point in trajectory])
    duration = (
        (previous_timestamp - first_timestamp) / 1_000_000_000
        if first_timestamp is not None and previous_timestamp is not None
        else 0.0
    )
    metrics: dict[str, float | int | str] = {
        "reference": "odometry_measurement",
        "imu_messages": imu_messages,
        "odometry_messages": odometry_messages,
        "duration_seconds": round(duration, 6),
        "position_residual_rmse_m": round(
            float(np.sqrt(np.mean(np.square(residuals)))), 6
        ),
        "position_residual_max_m": round(float(np.max(residuals)), 6),
    }
    return ReplayResult(tuple(trajectory), metrics)


def write_replay_outputs(
    result: ReplayResult, output_directory: str | Path
) -> tuple[Path, Path]:
    """Write the replay trajectory CSV and metrics JSON."""

    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    trajectory_path = destination / "trajectory.csv"
    metrics_path = destination / "metrics.json"
    fieldnames = list(asdict(result.trajectory[0]))
    with trajectory_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(asdict(point) for point in result.trajectory)
    metrics_path.write_text(
        json.dumps(result.metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return trajectory_path, metrics_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Replay a ROS 2 bag through the 2D EKF"
    )
    parser.add_argument("bag", help="rosbag2 directory or SQLite/MCAP storage file")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--odometry-topic", default="/odom")
    parser.add_argument("--output-dir", default="replay-output")
    parser.add_argument("--acceleration-variance", type=float, default=0.35)
    parser.add_argument("--position-variance", type=float, default=0.25)
    parser.add_argument("--velocity-variance", type=float, default=0.04)
    args = parser.parse_args()
    config = ReplayConfig(
        imu_topic=args.imu_topic,
        odometry_topic=args.odometry_topic,
        acceleration_variance=args.acceleration_variance,
        position_variance=args.position_variance,
        velocity_variance=args.velocity_variance,
    )
    try:
        result = replay_bag(args.bag, config)
        trajectory_path, metrics_path = write_replay_outputs(result, args.output_dir)
    except (BagReplayError, ValueError) as exc:
        parser.exit(2, f"error: {exc}\n")
    print(json.dumps(result.metrics, indent=2, sort_keys=True))
    print(f"trajectory: {trajectory_path}")
    print(f"metrics: {metrics_path}")


if __name__ == "__main__":
    main()
