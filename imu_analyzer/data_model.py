from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class ImuData:
    """Container for IMU time-series data extracted from a bag file."""

    timestamps_ns: np.ndarray   # absolute nanosecond timestamps
    timestamps_s: np.ndarray    # relative seconds from first sample

    accel_x: np.ndarray
    accel_y: np.ndarray
    accel_z: np.ndarray

    gyro_x: np.ndarray
    gyro_y: np.ndarray
    gyro_z: np.ndarray

    sample_rate_hz: float       # computed from median dt
    topic: str
    bag_path: str

    filtered: Optional["ImuData"] = field(default=None, repr=False)

    @property
    def duration_s(self) -> float:
        if len(self.timestamps_s) < 2:
            return 0.0
        return float(self.timestamps_s[-1] - self.timestamps_s[0])

    @property
    def n_samples(self) -> int:
        return len(self.timestamps_s)

    def time_slice(self, start_s: Optional[float], end_s: Optional[float]) -> "ImuData":
        """Return a new ImuData sliced to [start_s, end_s] relative time."""
        t = self.timestamps_s
        mask = np.ones(len(t), dtype=bool)
        if start_s is not None:
            mask &= t >= start_s
        if end_s is not None:
            mask &= t <= end_s
        return ImuData(
            timestamps_ns=self.timestamps_ns[mask],
            timestamps_s=self.timestamps_s[mask],
            accel_x=self.accel_x[mask],
            accel_y=self.accel_y[mask],
            accel_z=self.accel_z[mask],
            gyro_x=self.gyro_x[mask],
            gyro_y=self.gyro_y[mask],
            gyro_z=self.gyro_z[mask],
            sample_rate_hz=self.sample_rate_hz,
            topic=self.topic,
            bag_path=self.bag_path,
        )
