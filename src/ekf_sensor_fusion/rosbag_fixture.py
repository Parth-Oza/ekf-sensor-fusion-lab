"""Generate a small, deterministic rosbag2 fixture for the replay example."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore


def write_synthetic_rosbag(path: str | Path, *, include_odometry: bool = True) -> Path:
    """Write two IMU and two odometry samples to a rosbag2 MCAP directory."""

    destination = Path(path)
    typestore = get_typestore(Stores.ROS2_JAZZY)
    types = typestore.types

    def header(timestamp_ns: int):
        time = types["builtin_interfaces/msg/Time"](
            sec=timestamp_ns // 1_000_000_000,
            nanosec=timestamp_ns % 1_000_000_000,
        )
        return types["std_msgs/msg/Header"](stamp=time, frame_id="map")

    def imu(timestamp_ns: int, ax: float, ay: float):
        vector = types["geometry_msgs/msg/Vector3"]
        return types["sensor_msgs/msg/Imu"](
            header=header(timestamp_ns),
            orientation=types["geometry_msgs/msg/Quaternion"](
                x=0.0,
                y=0.0,
                z=0.0,
                w=1.0,
            ),
            orientation_covariance=np.zeros(9),
            angular_velocity=vector(x=0.0, y=0.0, z=0.0),
            angular_velocity_covariance=np.zeros(9),
            linear_acceleration=vector(x=ax, y=ay, z=0.0),
            linear_acceleration_covariance=np.zeros(9),
        )

    def odometry(timestamp_ns: int, x: float, y: float, vx: float, vy: float):
        vector = types["geometry_msgs/msg/Vector3"]
        pose = types["geometry_msgs/msg/Pose"](
            position=types["geometry_msgs/msg/Point"](x=x, y=y, z=0.0),
            orientation=types["geometry_msgs/msg/Quaternion"](
                x=0.0,
                y=0.0,
                z=0.0,
                w=1.0,
            ),
        )
        twist = types["geometry_msgs/msg/Twist"](
            linear=vector(x=vx, y=vy, z=0.0),
            angular=vector(x=0.0, y=0.0, z=0.0),
        )
        return types["nav_msgs/msg/Odometry"](
            header=header(timestamp_ns),
            child_frame_id="base_link",
            pose=types["geometry_msgs/msg/PoseWithCovariance"](
                pose=pose,
                covariance=np.zeros(36),
            ),
            twist=types["geometry_msgs/msg/TwistWithCovariance"](
                twist=twist,
                covariance=np.zeros(36),
            ),
        )

    imu_type = "sensor_msgs/msg/Imu"
    odometry_type = "nav_msgs/msg/Odometry"
    with Writer(
        destination,
        version=Writer.VERSION_LATEST,
        storage_plugin=StoragePlugin.MCAP,
    ) as writer:
        imu_connection = writer.add_connection(
            "/imu/data",
            imu_type,
            typestore=typestore,
        )
        odometry_connection = (
            writer.add_connection("/odom", odometry_type, typestore=typestore)
            if include_odometry
            else None
        )
        samples = (
            (1_000_000_000, "imu", imu(1_000_000_000, 0.2, 0.0)),
            (1_100_000_000, "odom", odometry(1_100_000_000, 0.11, 0.0, 1.0, 0.0)),
            (1_200_000_000, "imu", imu(1_200_000_000, 0.0, 0.0)),
            (1_300_000_000, "odom", odometry(1_300_000_000, 0.31, 0.0, 1.0, 0.0)),
        )
        for timestamp, kind, message in samples:
            if kind == "imu":
                writer.write(
                    imu_connection,
                    timestamp,
                    typestore.serialize_cdr(message, imu_type),
                )
            elif odometry_connection is not None:
                writer.write(
                    odometry_connection,
                    timestamp,
                    typestore.serialize_cdr(message, odometry_type),
                )
    return destination


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate the synthetic rosbag2 MCAP fixture"
    )
    parser.add_argument("path", nargs="?", default="examples/ros2_bag_fixture")
    args = parser.parse_args()
    destination = write_synthetic_rosbag(args.path)
    print(destination)


if __name__ == "__main__":
    main()
