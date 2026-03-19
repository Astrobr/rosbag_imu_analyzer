from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

import numpy as np
from scipy.signal import find_peaks as _sp_find_peaks

from .data_model import ImuData


@dataclass
class FreqPeak:
    frequency_hz: float
    magnitude: float
    relative_magnitude: float  # fraction of global spectrum maximum


@dataclass
class AxisFreqMetrics:
    label: str
    frequencies: np.ndarray   # Hz bins (real-valued, one-sided)
    magnitudes: np.ndarray    # normalized single-sided amplitude spectrum
    dominant_freq_hz: float   # frequency of the highest non-DC bin
    peaks: List[FreqPeak]
    total_power: float


@dataclass
class FreqDomainMetrics:
    accel_x: AxisFreqMetrics
    accel_y: AxisFreqMetrics
    accel_z: AxisFreqMetrics
    gyro_x: AxisFreqMetrics
    gyro_y: AxisFreqMetrics
    gyro_z: AxisFreqMetrics


def compute_axis_freq_metrics(
    label: str,
    data: np.ndarray,
    sample_rate_hz: float,
    window: str = "hann",
    peak_min_height: float = 0.1,
    peak_min_distance: int = 5,
) -> AxisFreqMetrics:
    n = len(data)

    # Build window
    win_map = {
        "hann":    np.hanning(n),
        "hamming": np.hamming(n),
        "blackman":np.blackman(n),
        "none":    np.ones(n),
    }
    win = win_map.get(window, np.hanning(n))

    windowed = data * win
    fft_vals = np.fft.rfft(windowed)
    freqs = np.fft.rfftfreq(n, d=1.0 / sample_rate_hz)

    # Normalized single-sided amplitude spectrum
    magnitudes = np.abs(fft_vals) * 2.0 / n
    magnitudes[0] /= 2.0  # DC bin is not doubled

    # Dominant frequency: highest non-DC bin
    non_dc_mag = magnitudes[1:].copy()
    dominant_idx = int(np.argmax(non_dc_mag)) + 1
    dominant_freq = float(freqs[dominant_idx])

    total_power = float(np.sum(magnitudes ** 2))

    peaks = _find_peaks(freqs, magnitudes, peak_min_height, peak_min_distance)

    return AxisFreqMetrics(
        label=label,
        frequencies=freqs,
        magnitudes=magnitudes,
        dominant_freq_hz=dominant_freq,
        peaks=peaks,
        total_power=total_power,
    )


def _find_peaks(
    freqs: np.ndarray,
    magnitudes: np.ndarray,
    min_height: float,
    min_distance: int,
) -> List[FreqPeak]:
    """Find local maxima above a threshold fraction of the global maximum."""
    global_max = float(np.max(magnitudes))
    if global_max == 0:
        return []
    threshold = min_height * global_max

    idxs, _ = _sp_find_peaks(magnitudes, height=threshold, distance=min_distance)

    return [
        FreqPeak(
            frequency_hz=float(freqs[i]),
            magnitude=float(magnitudes[i]),
            relative_magnitude=float(magnitudes[i]) / global_max,
        )
        for i in idxs
    ]


def compute_freq_domain_metrics(
    imu_data: ImuData,
    window: str = "hann",
    peak_min_height: float = 0.1,
    peak_min_distance: int = 5,
) -> FreqDomainMetrics:
    fs = imu_data.sample_rate_hz
    kwargs = dict(
        sample_rate_hz=fs,
        window=window,
        peak_min_height=peak_min_height,
        peak_min_distance=peak_min_distance,
    )
    return FreqDomainMetrics(
        accel_x=compute_axis_freq_metrics("accel_x", imu_data.accel_x, **kwargs),
        accel_y=compute_axis_freq_metrics("accel_y", imu_data.accel_y, **kwargs),
        accel_z=compute_axis_freq_metrics("accel_z", imu_data.accel_z, **kwargs),
        gyro_x=compute_axis_freq_metrics("gyro_x",  imu_data.gyro_x,  **kwargs),
        gyro_y=compute_axis_freq_metrics("gyro_y",  imu_data.gyro_y,  **kwargs),
        gyro_z=compute_axis_freq_metrics("gyro_z",  imu_data.gyro_z,  **kwargs),
    )


def format_freq_summary(metrics: FreqDomainMetrics, title: str = "") -> str:
    """Return a terminal-friendly summary of frequency domain peaks."""
    lines = []
    if title:
        lines.append(title)

    header = f"{'Channel':<12} {'Dominant (Hz)':>14} {'Top Peaks (Hz @ magnitude)':}"
    sep = "-" * 70
    lines += [header, sep]

    all_axes = [
        metrics.accel_x, metrics.accel_y, metrics.accel_z,
        metrics.gyro_x,  metrics.gyro_y,  metrics.gyro_z,
    ]
    for fm in all_axes:
        top_peaks = sorted(fm.peaks, key=lambda p: p.magnitude, reverse=True)[:5]
        peak_str = "  ".join(
            f"{p.frequency_hz:.2f}@{p.magnitude:.4f}" for p in top_peaks
        ) or "(none)"
        lines.append(f"{fm.label:<12} {fm.dominant_freq_hz:>14.2f}     {peak_str}")
    lines.append(sep)
    return "\n".join(lines)
