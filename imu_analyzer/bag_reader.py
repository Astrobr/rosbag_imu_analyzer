from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from .data_model import ImuData


# Supported typestore names and their rosbags Stores enum values
_TYPESTORE_MAP = {
    "ROS1_NOETIC":  "ROS1_NOETIC",
    "ROS2_FOXY":    "ROS2_FOXY",
    "ROS2_GALACTIC":"ROS2_GALACTIC",
    "ROS2_HUMBLE":  "ROS2_HUMBLE",
    "ROS2_IRON":    "ROS2_IRON",
    "ROS2_JAZZY":   "ROS2_JAZZY",
    "LATEST":       "LATEST",
}


def _get_typestore(typestore_name: str):
    """Create a rosbags typestore for message deserialization."""
    from rosbags.typesys import Stores, get_typestore

    store_attr = _TYPESTORE_MAP.get(typestore_name)
    if store_attr is None:
        raise ValueError(
            f"Unknown typestore: '{typestore_name}'. "
            f"Valid options: {sorted(_TYPESTORE_MAP.keys())}"
        )
    store_enum = getattr(Stores, store_attr)
    return get_typestore(store_enum)


def detect_ros_version(bag_path: Path) -> str:
    """
    Detect bag format from path heuristics:
      - .bag extension -> ros1
      - directory containing metadata.yaml -> ros2
      - .db3 / .mcap extension -> ros2
    """
    bag_path = Path(bag_path)
    if bag_path.suffix == ".bag":
        return "ros1"
    if bag_path.is_dir() and (bag_path / "metadata.yaml").exists():
        return "ros2"
    if bag_path.suffix in (".db3", ".mcap"):
        return "ros2"
    raise ValueError(
        f"Cannot auto-detect ROS version for: {bag_path}\n"
        "Set ros_version: ros1 or ros2 explicitly in config.yaml."
    )


def get_bag_info(
    bag_path: Path,
    ros_version: str,
    typestore_name: str,
    imu_topic: str,
) -> Dict:
    """
    Read bag metadata without iterating all messages.

    Returns:
        dict with keys: start_time_ns, end_time_ns, duration_s,
                        topics, imu_topic_found, estimated_sample_count
    """
    bag_path = Path(bag_path)

    if ros_version == "ros1":
        from rosbags.rosbag1 import Reader
    else:
        from rosbags.rosbag2 import Reader

    with Reader(bag_path) as reader:
        start_ns = reader.start_time
        end_ns = reader.end_time
        all_topics = sorted({c.topic for c in reader.connections})
        imu_connections = [c for c in reader.connections if c.topic == imu_topic]
        estimated_count = sum(c.msgcount for c in imu_connections) if imu_connections else 0

    return {
        "start_time_ns": start_ns,
        "end_time_ns": end_ns,
        "duration_s": (end_ns - start_ns) / 1e9,
        "topics": all_topics,
        "imu_topic_found": bool(imu_connections),
        "estimated_sample_count": estimated_count,
    }


def read_imu_data(
    bag_path: Path,
    ros_version: str,
    typestore_name: str,
    imu_topic: str,
    start_ns: Optional[int] = None,
    stop_ns: Optional[int] = None,
) -> ImuData:
    """
    Read all IMU messages from the bag into numpy arrays.

    start_ns / stop_ns are absolute nanosecond timestamps for range filtering.
    """
    bag_path = Path(bag_path)
    store = _get_typestore(typestore_name)

    if ros_version == "ros1":
        from rosbags.rosbag1 import Reader
        deserialize = store.deserialize_ros1
    else:
        from rosbags.rosbag2 import Reader
        deserialize = store.deserialize_cdr

    timestamps: List[int] = []
    ax_list: List[float] = []
    ay_list: List[float] = []
    az_list: List[float] = []
    gx_list: List[float] = []
    gy_list: List[float] = []
    gz_list: List[float] = []

    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic == imu_topic]
        if not connections:
            available = sorted({c.topic for c in reader.connections})
            raise ValueError(
                f"Topic '{imu_topic}' not found in bag.\n"
                f"Available topics: {available}"
            )
        msgtype = connections[0].msgtype

        kwargs = {"connections": connections}
        if start_ns is not None:
            kwargs["start"] = start_ns
        if stop_ns is not None:
            kwargs["stop"] = stop_ns

        for _conn, ts, rawdata in reader.messages(**kwargs):
            msg = deserialize(rawdata, msgtype)
            timestamps.append(ts)
            ax_list.append(float(msg.linear_acceleration.x))
            ay_list.append(float(msg.linear_acceleration.y))
            az_list.append(float(msg.linear_acceleration.z))
            gx_list.append(float(msg.angular_velocity.x))
            gy_list.append(float(msg.angular_velocity.y))
            gz_list.append(float(msg.angular_velocity.z))

    if not timestamps:
        raise ValueError(
            f"No IMU messages found in topic '{imu_topic}' "
            f"within the specified time range."
        )

    ts_ns = np.array(timestamps, dtype=np.int64)
    ts_s = (ts_ns - ts_ns[0]).astype(np.float64) / 1e9

    dts = np.diff(ts_s)
    sample_rate = 1.0 / float(np.median(dts)) if len(dts) > 0 else 0.0

    return ImuData(
        timestamps_ns=ts_ns,
        timestamps_s=ts_s,
        accel_x=np.array(ax_list, dtype=np.float64),
        accel_y=np.array(ay_list, dtype=np.float64),
        accel_z=np.array(az_list, dtype=np.float64),
        gyro_x=np.array(gx_list, dtype=np.float64),
        gyro_y=np.array(gy_list, dtype=np.float64),
        gyro_z=np.array(gz_list, dtype=np.float64),
        sample_rate_hz=sample_rate,
        topic=imu_topic,
        bag_path=str(bag_path),
    )
