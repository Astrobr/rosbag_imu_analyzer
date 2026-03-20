from __future__ import annotations

from pathlib import Path
from typing import List, Optional

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from .data_model import ImuData
from .time_analysis import TimeDomainMetrics
from .freq_analysis import FreqDomainMetrics, FreqPeak

# Use non-interactive backend when not showing plots interactively
matplotlib.rcParams["figure.dpi"] = 100


def _deduplicate_peaks(peaks: List[FreqPeak], min_gap_hz: float) -> List[FreqPeak]:
    """
    Within any frequency window of `min_gap_hz`, keep only the peak with the
    highest magnitude. Returns a sorted (by frequency) list of survivors.

    Algorithm: greedy scan over magnitude-sorted peaks; accept a candidate only
    if it is more than `min_gap_hz` away from every already-accepted peak.
    """
    # Sort descending by magnitude so the strongest peak in each window wins
    by_magnitude = sorted(peaks, key=lambda p: p.magnitude, reverse=True)
    accepted: List[FreqPeak] = []
    for candidate in by_magnitude:
        too_close = any(
            abs(candidate.frequency_hz - kept.frequency_hz) < min_gap_hz
            for kept in accepted
        )
        if not too_close:
            accepted.append(candidate)
    # Return sorted by frequency for consistent left-to-right rendering
    return sorted(accepted, key=lambda p: p.frequency_hz)


def _save_or_show(
    fig: plt.Figure,
    save_path: Optional[Path],
    show: bool,
) -> None:
    if save_path:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"  Saved: {save_path}")
    if show:
        plt.show()
    plt.close(fig)


def plot_time_domain(
    imu_data: ImuData,
    metrics: TimeDomainMetrics,
    title_prefix: str = "",
    save_path: Optional[Path] = None,
    show: bool = True,
) -> None:
    """
    Plot time-domain waveforms in a 2x3 grid:
      Row 0: Accel X/Y/Z  |  Row 1: Gyro X/Y/Z
    Each subplot shows the signal with a dashed mean line and a stats box.
    """
    fig, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
    prefix = f"{title_prefix} " if title_prefix else ""
    fig.suptitle(f"{prefix}IMU Time Domain", fontsize=14, fontweight="bold")

    pairs = [
        (axes[0, 0], imu_data.accel_x, metrics.accel_x, "Accel X (m/s²)"),
        (axes[0, 1], imu_data.accel_y, metrics.accel_y, "Accel Y (m/s²)"),
        (axes[0, 2], imu_data.accel_z, metrics.accel_z, "Accel Z (m/s²)"),
        (axes[1, 0], imu_data.gyro_x,  metrics.gyro_x,  "Gyro X (rad/s)"),
        (axes[1, 1], imu_data.gyro_y,  metrics.gyro_y,  "Gyro Y (rad/s)"),
        (axes[1, 2], imu_data.gyro_z,  metrics.gyro_z,  "Gyro Z (rad/s)"),
    ]

    t = imu_data.timestamps_s
    for ax, data, m, ylabel in pairs:
        ax.plot(t, data, linewidth=0.6, color="steelblue", label=ylabel)
        ax.axhline(m.mean, color="darkorange", linestyle="--", linewidth=1.0,
                   label=f"mean={m.mean:.4f}")
        ax.set_ylabel(ylabel, fontsize=9)
        ax.set_xlabel("Time (s)", fontsize=9)
        ax.legend(fontsize=8, loc="upper right")
        ax.grid(True, alpha=0.3)

        info_text = (
            f"std  = {m.std:.6f}\n"
            f"rms  = {m.rms:.6f}\n"
            f"p2p  = {m.peak_to_peak:.6f}\n"
            f"min  = {m.min:.6f}\n"
            f"max  = {m.max:.6f}"
        )
        ax.text(
            0.02, 0.97, info_text,
            transform=ax.transAxes,
            ha="left", va="top", fontsize=7, family="monospace",
            bbox=dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.8),
        )

    fig.text(
        0.5, 0.01,
        f"Sample rate: {metrics.sample_rate_hz:.2f} Hz  |  "
        f"Duration: {metrics.duration_s:.3f} s  |  "
        f"Samples: {metrics.n_samples}",
        ha="center", fontsize=9, color="gray",
    )
    plt.tight_layout(rect=[0, 0.03, 1, 0.97])
    _save_or_show(fig, save_path, show)


def plot_freq_domain(
    freq_metrics: FreqDomainMetrics,
    title_prefix: str = "",
    save_path: Optional[Path] = None,
    show: bool = True,
) -> None:
    """
    Plot frequency spectra in a 2x3 grid.
    Detected peaks are marked with red dashed vertical lines and annotations.
    """
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    prefix = f"{title_prefix} " if title_prefix else ""
    fig.suptitle(f"{prefix}IMU Frequency Domain", fontsize=14, fontweight="bold")

    channel_metrics = [
        (axes[0, 0], freq_metrics.accel_x, "Accel X"),
        (axes[0, 1], freq_metrics.accel_y, "Accel Y"),
        (axes[0, 2], freq_metrics.accel_z, "Accel Z"),
        (axes[1, 0], freq_metrics.gyro_x,  "Gyro X"),
        (axes[1, 1], freq_metrics.gyro_y,  "Gyro Y"),
        (axes[1, 2], freq_metrics.gyro_z,  "Gyro Z"),
    ]

    for ax, fm, label in channel_metrics:
        ax.plot(fm.frequencies, fm.magnitudes, linewidth=0.8, color="steelblue")
        ax.axvline(
            fm.dominant_freq_hz, color="green", linestyle=":", linewidth=1.0,
            label=f"dominant={fm.dominant_freq_hz:.2f}Hz",
        )

        # Mark detected peaks — deduplicate within 20 Hz windows first
        top_peaks = sorted(fm.peaks, key=lambda p: p.magnitude, reverse=True)[:8]
        display_peaks = _deduplicate_peaks(top_peaks, min_gap_hz=20.0)
        for peak in display_peaks:
            ax.axvline(peak.frequency_hz, color="red", alpha=0.5,
                       linestyle="--", linewidth=0.8)
            ax.annotate(
                f"{peak.frequency_hz:.1f}",
                xy=(peak.frequency_hz, peak.magnitude),
                xytext=(4, 4), textcoords="offset points",
                fontsize=7, color="red",
                arrowprops=dict(arrowstyle="-", color="red", alpha=0.5),
            )

        ax.set_title(
            f"{label}  [dom={fm.dominant_freq_hz:.2f}Hz, "
            f"power={fm.total_power:.4f}]",
            fontsize=9,
        )
        ax.set_xlabel("Frequency (Hz)", fontsize=9)
        ax.set_ylabel("Amplitude", fontsize=9)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        if len(fm.frequencies):
            ax.set_xlim(0, fm.frequencies[-1])

    plt.tight_layout()
    _save_or_show(fig, save_path, show)


def plot_comparison(
    raw: ImuData,
    filtered: ImuData,
    save_path: Optional[Path] = None,
    show: bool = True,
) -> None:
    """
    Overlay plot: raw (blue, semi-transparent) vs filtered (orange) for all 6 channels.
    """
    fig, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
    fig.suptitle("Raw vs Filtered IMU Data", fontsize=14, fontweight="bold")

    channels = [
        (axes[0, 0], raw.accel_x, filtered.accel_x, "Accel X (m/s²)"),
        (axes[0, 1], raw.accel_y, filtered.accel_y, "Accel Y (m/s²)"),
        (axes[0, 2], raw.accel_z, filtered.accel_z, "Accel Z (m/s²)"),
        (axes[1, 0], raw.gyro_x,  filtered.gyro_x,  "Gyro X (rad/s)"),
        (axes[1, 1], raw.gyro_y,  filtered.gyro_y,  "Gyro Y (rad/s)"),
        (axes[1, 2], raw.gyro_z,  filtered.gyro_z,  "Gyro Z (rad/s)"),
    ]

    t = raw.timestamps_s
    for ax, raw_data, filt_data, ylabel in channels:
        ax.plot(t, raw_data, linewidth=0.5, color="steelblue", alpha=0.5, label="Raw")
        ax.plot(t, filt_data, linewidth=0.9, color="darkorange", label="Filtered")
        ax.set_ylabel(ylabel, fontsize=9)
        ax.set_xlabel("Time (s)", fontsize=9)
        ax.legend(fontsize=8, loc="upper right")
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    _save_or_show(fig, save_path, show)


def plot_freq_comparison(
    raw_freq: FreqDomainMetrics,
    filtered_freq: FreqDomainMetrics,
    save_path: Optional[Path] = None,
    show: bool = True,
) -> None:
    """
    Overlay frequency spectra: raw vs filtered for all 6 channels.
    """
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle("Raw vs Filtered Frequency Spectra", fontsize=14, fontweight="bold")

    channels = [
        (axes[0, 0], raw_freq.accel_x, filtered_freq.accel_x, "Accel X"),
        (axes[0, 1], raw_freq.accel_y, filtered_freq.accel_y, "Accel Y"),
        (axes[0, 2], raw_freq.accel_z, filtered_freq.accel_z, "Accel Z"),
        (axes[1, 0], raw_freq.gyro_x,  filtered_freq.gyro_x,  "Gyro X"),
        (axes[1, 1], raw_freq.gyro_y,  filtered_freq.gyro_y,  "Gyro Y"),
        (axes[1, 2], raw_freq.gyro_z,  filtered_freq.gyro_z,  "Gyro Z"),
    ]

    for ax, raw_fm, filt_fm, label in channels:
        ax.plot(raw_fm.frequencies, raw_fm.magnitudes,
                linewidth=0.6, color="steelblue", alpha=0.6, label="Raw")
        ax.plot(filt_fm.frequencies, filt_fm.magnitudes,
                linewidth=0.9, color="darkorange", label="Filtered")
        ax.set_title(label, fontsize=9)
        ax.set_xlabel("Frequency (Hz)", fontsize=9)
        ax.set_ylabel("Amplitude", fontsize=9)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        if len(raw_fm.frequencies):
            ax.set_xlim(0, raw_fm.frequencies[-1])

    plt.tight_layout()
    _save_or_show(fig, save_path, show)


def plot_filter_attenuation(
    fc,
    save_path: Optional[Path] = None,
    show: bool = True,
) -> None:
    """
    2x3 grid of dB attenuation curves (one per channel).
    Shows how much the filter suppresses each frequency.
    Reference lines at -3 dB and -6 dB are drawn for orientation.
    NaN bins (below noise floor) are skipped by matplotlib automatically.
    """
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle("Filter Frequency Attenuation", fontsize=14, fontweight="bold")

    channels = [
        (axes[0, 0], fc.accel_x_atten, "Accel X"),
        (axes[0, 1], fc.accel_y_atten, "Accel Y"),
        (axes[0, 2], fc.accel_z_atten, "Accel Z"),
        (axes[1, 0], fc.gyro_x_atten,  "Gyro X"),
        (axes[1, 1], fc.gyro_y_atten,  "Gyro Y"),
        (axes[1, 2], fc.gyro_z_atten,  "Gyro Z"),
    ]

    for ax, am, label in channels:
        ax.plot(am.frequencies, am.attenuation_db,
                linewidth=0.9, color="steelblue", label="Attenuation")
        ax.axhline(-3.0, color="darkorange", linestyle="--",
                   linewidth=1.0, alpha=0.8, label="-3 dB")
        ax.axhline(-6.0, color="red", linestyle=":",
                   linewidth=1.0, alpha=0.8, label="-6 dB")
        ax.axhline(0.0, color="gray", linestyle="-",
                   linewidth=0.5, alpha=0.5)

        ax.set_title(label, fontsize=9)
        ax.set_xlabel("Frequency (Hz)", fontsize=9)
        ax.set_ylabel("Attenuation (dB)", fontsize=9)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # Clamp y-axis to [-60, +6] dB for readability
        valid = am.attenuation_db[np.isfinite(am.attenuation_db)]
        if len(valid):
            y_lo = max(float(np.min(valid)) - 2.0, -60.0)
            y_hi = min(float(np.max(valid)) + 2.0,   6.0)
            ax.set_ylim(y_lo, y_hi)

        # Annotate probe frequencies
        for f_probe, db_val in sorted(am.db_at_key_freqs.items()):
            if np.isfinite(db_val):
                ax.annotate(
                    f"{f_probe:.0f}Hz\n{db_val:.1f}dB",
                    xy=(f_probe, db_val),
                    xytext=(6, 0), textcoords="offset points",
                    fontsize=6, color="steelblue",
                    bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.7),
                )

    plt.tight_layout()
    _save_or_show(fig, save_path, show)


def plot_filter_delay(
    raw: ImuData,
    filtered: ImuData,
    fc,
    window_s: float = 0.5,
    save_path: Optional[Path] = None,
    show: bool = True,
) -> None:
    """
    2x3 grid showing a zoomed time window of raw vs filtered with the
    measured cross-correlation delay annotated per channel.

    Args:
        window_s: Duration (seconds) of the time window to display.
    """
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle(
        "Filter Time Delay (Cross-Correlation Analysis)",
        fontsize=14, fontweight="bold",
    )

    channels = [
        (axes[0, 0], raw.accel_x, filtered.accel_x, fc.accel_x_delay, "Accel X (m/s²)"),
        (axes[0, 1], raw.accel_y, filtered.accel_y, fc.accel_y_delay, "Accel Y (m/s²)"),
        (axes[0, 2], raw.accel_z, filtered.accel_z, fc.accel_z_delay, "Accel Z (m/s²)"),
        (axes[1, 0], raw.gyro_x,  filtered.gyro_x,  fc.gyro_x_delay,  "Gyro X (rad/s)"),
        (axes[1, 1], raw.gyro_y,  filtered.gyro_y,  fc.gyro_y_delay,  "Gyro Y (rad/s)"),
        (axes[1, 2], raw.gyro_z,  filtered.gyro_z,  fc.gyro_z_delay,  "Gyro Z (rad/s)"),
    ]

    t = raw.timestamps_s
    t_start = float(t[0])
    t_end = min(t_start + window_s, float(t[-1]))
    mask = (t >= t_start) & (t <= t_end)
    t_zoom = t[mask]

    for ax, raw_ch, filt_ch, dm, ylabel in channels:
        raw_zoom  = raw_ch[mask]
        filt_zoom = filt_ch[mask]

        ax.plot(t_zoom, raw_zoom,  linewidth=0.8, color="steelblue",
                alpha=0.7, label="Raw")
        ax.plot(t_zoom, filt_zoom, linewidth=0.9, color="darkorange",
                label="Filtered")

        # Annotate delay with a horizontal arrow at the signal peak
        if len(raw_zoom) > 0:
            peak_local = int(np.argmax(np.abs(raw_zoom)))
            t_peak = float(t_zoom[peak_local])
            y_peak = float(raw_zoom[peak_local])
            lag_s  = dm.lag_ms / 1000.0

            if abs(lag_s) > 0 and (t_peak + lag_s) <= t_end:
                ax.annotate(
                    "",
                    xy=(t_peak + lag_s, y_peak * 0.8),
                    xytext=(t_peak,     y_peak * 0.8),
                    arrowprops=dict(
                        arrowstyle="->", color="black", lw=1.2,
                        connectionstyle="arc3,rad=0",
                    ),
                )

        # Stats box
        delay_str = (
            f"delay = {dm.lag_ms:+.3f} ms\n"
            f"({dm.lag_samples:+d} samples)\n"
            f"xcorr = {dm.correlation_peak:.4f}"
        )
        ax.text(
            0.98, 0.97, delay_str,
            transform=ax.transAxes,
            ha="right", va="top", fontsize=7, family="monospace",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow", alpha=0.85),
        )

        ax.set_ylabel(ylabel, fontsize=9)
        ax.set_xlabel("Time (s)", fontsize=9)
        ax.legend(fontsize=8, loc="upper left")
        ax.grid(True, alpha=0.3)

    fig.text(
        0.5, 0.01,
        "Positive delay = filtered signal lags behind raw  |  "
        f"Showing first {window_s:.2f}s of signal",
        ha="center", fontsize=8, color="gray",
    )
    plt.tight_layout(rect=[0, 0.03, 1, 1.0])
    _save_or_show(fig, save_path, show)
