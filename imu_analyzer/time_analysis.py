from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .data_model import ImuData


@dataclass
class AxisMetrics:
    label: str
    mean: float
    std: float
    rms: float
    min: float
    max: float
    peak_to_peak: float


@dataclass
class TimeDomainMetrics:
    accel_x: AxisMetrics
    accel_y: AxisMetrics
    accel_z: AxisMetrics
    gyro_x: AxisMetrics
    gyro_y: AxisMetrics
    gyro_z: AxisMetrics
    sample_rate_hz: float
    duration_s: float
    n_samples: int


def compute_axis_metrics(label: str, data: np.ndarray) -> AxisMetrics:
    return AxisMetrics(
        label=label,
        mean=float(np.mean(data)),
        std=float(np.std(data)),
        rms=float(np.sqrt(np.mean(data ** 2))),
        min=float(np.min(data)),
        max=float(np.max(data)),
        peak_to_peak=float(np.max(data) - np.min(data)),
    )


def compute_time_domain_metrics(imu_data: ImuData) -> TimeDomainMetrics:
    return TimeDomainMetrics(
        accel_x=compute_axis_metrics("accel_x", imu_data.accel_x),
        accel_y=compute_axis_metrics("accel_y", imu_data.accel_y),
        accel_z=compute_axis_metrics("accel_z", imu_data.accel_z),
        gyro_x=compute_axis_metrics("gyro_x",  imu_data.gyro_x),
        gyro_y=compute_axis_metrics("gyro_y",  imu_data.gyro_y),
        gyro_z=compute_axis_metrics("gyro_z",  imu_data.gyro_z),
        sample_rate_hz=imu_data.sample_rate_hz,
        duration_s=imu_data.duration_s,
        n_samples=imu_data.n_samples,
    )


def format_metrics_table(metrics: TimeDomainMetrics, title: str = "") -> str:
    """Return a plain-text formatted table for terminal output."""
    lines = []
    if title:
        lines.append(title)

    header = (
        f"{'Channel':<12} {'Mean':>12} {'Std':>12} {'RMS':>12} "
        f"{'Min':>12} {'Max':>12} {'P2P':>12}"
    )
    sep = "-" * len(header)
    lines += [header, sep]

    axes = [
        metrics.accel_x, metrics.accel_y, metrics.accel_z,
        metrics.gyro_x,  metrics.gyro_y,  metrics.gyro_z,
    ]
    for ax in axes:
        lines.append(
            f"{ax.label:<12} {ax.mean:>12.6f} {ax.std:>12.6f} {ax.rms:>12.6f} "
            f"{ax.min:>12.6f} {ax.max:>12.6f} {ax.peak_to_peak:>12.6f}"
        )

    lines += [sep, f"  Sample rate: {metrics.sample_rate_hz:.2f} Hz  |  "
                   f"Duration: {metrics.duration_s:.3f} s  |  "
                   f"Samples: {metrics.n_samples}"]
    return "\n".join(lines)
