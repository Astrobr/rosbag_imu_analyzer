"""
Filter characterization analysis: frequency attenuation and time-domain delay.

Both computations use only numpy (no scipy required):
  - Attenuation: FFT-based |H(f)| = filtered_mag / raw_mag -> 20*log10(|H(f)|) in dB
  - Delay:       Normalized cross-correlation to find lag between raw and filtered
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

from .data_model import ImuData


@dataclass
class AxisAttenuationMetrics:
    label: str
    frequencies: np.ndarray    # Hz bins, one-sided (same grid as FFT analysis)
    attenuation_db: np.ndarray # 20*log10(|H(f)|); NaN where raw is below noise floor
    db_at_key_freqs: Dict[float, float]  # {freq_hz: db_value} at probe points


@dataclass
class AxisDelayMetrics:
    label: str
    lag_samples: int           # positive = filtered lags behind raw
    lag_ms: float              # lag in milliseconds
    correlation_peak: float    # normalized xcorr peak value (0..1 for similar signals)


@dataclass
class FilterCharacterization:
    accel_x_atten: AxisAttenuationMetrics
    accel_y_atten: AxisAttenuationMetrics
    accel_z_atten: AxisAttenuationMetrics
    gyro_x_atten:  AxisAttenuationMetrics
    gyro_y_atten:  AxisAttenuationMetrics
    gyro_z_atten:  AxisAttenuationMetrics

    accel_x_delay: AxisDelayMetrics
    accel_y_delay: AxisDelayMetrics
    accel_z_delay: AxisDelayMetrics
    gyro_x_delay:  AxisDelayMetrics
    gyro_y_delay:  AxisDelayMetrics
    gyro_z_delay:  AxisDelayMetrics

    sample_rate_hz: float


# ---------------------------------------------------------------------------
# Attenuation
# ---------------------------------------------------------------------------

def _compute_axis_attenuation(
    label: str,
    raw: np.ndarray,
    filtered: np.ndarray,
    sample_rate_hz: float,
    window: str = "none",
    key_freqs_hz: Optional[List[float]] = None,
    freq_start_hz: float = 0.1,
) -> AxisAttenuationMetrics:
    n = len(raw)

    win_map = {
        "hann":     np.hanning(n),
        "hamming":  np.hamming(n),
        "blackman": np.blackman(n),
        "none":     np.ones(n, dtype=np.float64),
    }
    win = win_map.get(window, np.ones(n, dtype=np.float64))

    raw_mag      = np.abs(np.fft.rfft(raw.astype(np.float64)      * win))
    filtered_mag = np.abs(np.fft.rfft(filtered.astype(np.float64) * win))
    freqs        = np.fft.rfftfreq(n, d=1.0 / sample_rate_hz)

    # Mask bins where raw is effectively zero (noise floor guard)
    raw_peak  = np.max(raw_mag)
    raw_floor = raw_peak * 1e-9 if raw_peak > 0 else 1e-12

    valid_mask = raw_mag > raw_floor

    # |H(f)| = filtered / raw, guarded
    ratio = np.where(valid_mask, filtered_mag / np.where(valid_mask, raw_mag, 1.0), np.nan)

    # -120 dB floor to prevent log10(0) -> -inf
    RATIO_FLOOR = 1e-6
    ratio_clamped = np.where(
        np.isfinite(ratio) & (ratio > 0),
        np.maximum(ratio, RATIO_FLOOR),
        RATIO_FLOOR,
    )
    attenuation_db = 20.0 * np.log10(ratio_clamped)
    # Restore NaN for noise-floor bins
    attenuation_db = np.where(valid_mask, attenuation_db, np.nan)

    # Probe at key frequencies (nearest bin, before trimming)
    db_at_key: Dict[float, float] = {}
    if key_freqs_hz:
        for f in key_freqs_hz:
            if f <= 0 or f >= sample_rate_hz / 2.0:
                continue
            idx = int(np.argmin(np.abs(freqs - f)))
            db_at_key[float(freqs[idx])] = float(attenuation_db[idx])

    # Trim to [freq_start_hz, Nyquist] for display (matching MATLAB approach)
    if freq_start_hz > 0.0:
        start_idx = int(np.searchsorted(freqs, freq_start_hz))
        freqs = freqs[start_idx:]
        attenuation_db = attenuation_db[start_idx:]

    return AxisAttenuationMetrics(
        label=label,
        frequencies=freqs,
        attenuation_db=attenuation_db,
        db_at_key_freqs=db_at_key,
    )


# ---------------------------------------------------------------------------
# Delay (cross-correlation)
# ---------------------------------------------------------------------------

def _compute_axis_delay(
    label: str,
    raw: np.ndarray,
    filtered: np.ndarray,
    sample_rate_hz: float,
) -> AxisDelayMetrics:
    # DC removal before correlation to eliminate offset dominance
    r = raw.astype(np.float64)      - np.mean(raw)
    f = filtered.astype(np.float64) - np.mean(filtered)

    n = len(r)

    # FFT-based cross-correlation: O(n log n), pure numpy
    nfft = 1 << int(np.ceil(np.log2(2 * n - 1)))   # next power of 2
    R = np.fft.rfft(r, n=nfft)
    F = np.fft.rfft(f, n=nfft)
    xcorr_full = np.fft.irfft(R * np.conj(F), n=nfft)

    # Reorder from [0..n-1, -(n-1)..-1] to [-(n-1)..0..n-1]
    xcorr = np.concatenate([xcorr_full[nfft - (n - 1):], xcorr_full[:n]])

    # Normalize to [-1, 1]
    norm = np.sqrt(np.sum(r ** 2) * np.sum(f ** 2))
    if norm > 0:
        xcorr = xcorr / norm

    # Center index corresponds to zero lag
    center = n - 1
    peak_idx = int(np.argmax(xcorr))
    lag_samples = peak_idx - center   # positive = filtered lags behind raw
    lag_ms = (lag_samples / sample_rate_hz) * 1000.0
    correlation_peak = float(xcorr[peak_idx])

    return AxisDelayMetrics(
        label=label,
        lag_samples=lag_samples,
        lag_ms=lag_ms,
        correlation_peak=correlation_peak,
    )


# ---------------------------------------------------------------------------
# Top-level compute
# ---------------------------------------------------------------------------

def compute_filter_characterization(
    raw: ImuData,
    filtered: ImuData,
    window: str = "none",
    key_freqs_hz: Optional[List[float]] = None,
    freq_start_hz: float = 0.1,
) -> FilterCharacterization:
    """
    Compute frequency attenuation and time delay for all 6 IMU channels.

    Args:
        raw:          Original unfiltered ImuData.
        filtered:     Filtered ImuData (same timestamps as raw).
        window:       FFT window type (hann/hamming/blackman/none).
        key_freqs_hz: Frequencies (Hz) at which to report attenuation values.
                      Defaults to [1, 5, 10, 20, 50, 100] clipped to Nyquist.
        freq_start_hz: Low-frequency cutoff for display (default 0.1 Hz, matching MATLAB).
    """
    fs = raw.sample_rate_hz
    nyq = fs / 2.0

    if key_freqs_hz is None:
        candidates = [1.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0]
        key_freqs_hz = [f for f in candidates if f < nyq]

    def _at(label, r, f):
        return _compute_axis_attenuation(label, r, f, fs, window, key_freqs_hz, freq_start_hz)

    def _dl(label, r, f):
        return _compute_axis_delay(label, r, f, fs)

    return FilterCharacterization(
        accel_x_atten=_at("accel_x", raw.accel_x, filtered.accel_x),
        accel_y_atten=_at("accel_y", raw.accel_y, filtered.accel_y),
        accel_z_atten=_at("accel_z", raw.accel_z, filtered.accel_z),
        gyro_x_atten= _at("gyro_x",  raw.gyro_x,  filtered.gyro_x),
        gyro_y_atten= _at("gyro_y",  raw.gyro_y,  filtered.gyro_y),
        gyro_z_atten= _at("gyro_z",  raw.gyro_z,  filtered.gyro_z),
        accel_x_delay=_dl("accel_x", raw.accel_x, filtered.accel_x),
        accel_y_delay=_dl("accel_y", raw.accel_y, filtered.accel_y),
        accel_z_delay=_dl("accel_z", raw.accel_z, filtered.accel_z),
        gyro_x_delay= _dl("gyro_x",  raw.gyro_x,  filtered.gyro_x),
        gyro_y_delay= _dl("gyro_y",  raw.gyro_y,  filtered.gyro_y),
        gyro_z_delay= _dl("gyro_z",  raw.gyro_z,  filtered.gyro_z),
        sample_rate_hz=fs,
    )


# ---------------------------------------------------------------------------
# Terminal formatter
# ---------------------------------------------------------------------------

def format_filter_characterization(fc: FilterCharacterization) -> str:
    """Return a terminal-friendly summary of filter attenuation and delay."""
    lines: List[str] = ["=== Filter Characterization ==="]

    # ---- Attenuation table ----
    all_atten = [
        fc.accel_x_atten, fc.accel_y_atten, fc.accel_z_atten,
        fc.gyro_x_atten,  fc.gyro_y_atten,  fc.gyro_z_atten,
    ]

    probe_freqs = sorted(all_atten[0].db_at_key_freqs.keys())

    if probe_freqs:
        lines.append("")
        lines.append("Frequency Attenuation:")
        freq_cols = "".join(f"  {f:>8.1f}Hz" for f in probe_freqs)
        header = f"  {'Channel':<12}{freq_cols}"
        sep = "-" * len(header)
        lines += [header, sep]

        for am in all_atten:
            row = f"  {am.label:<12}"
            for f in probe_freqs:
                db = am.db_at_key_freqs.get(f, float("nan"))
                row += f"  {db:>8.2f}dB"
            lines.append(row)

    # ---- Delay table ----
    all_delay = [
        fc.accel_x_delay, fc.accel_y_delay, fc.accel_z_delay,
        fc.gyro_x_delay,  fc.gyro_y_delay,  fc.gyro_z_delay,
    ]

    lines.append("")
    lines.append("Time Delay (cross-correlation):")
    header2 = f"  {'Channel':<12} {'Lag (samples)':>14} {'Lag (ms)':>10} {'xcorr peak':>12}"
    sep2 = "-" * len(header2)
    lines += [header2, sep2]

    for dm in all_delay:
        sign = "+" if dm.lag_samples > 0 else ""
        lines.append(
            f"  {dm.label:<12} {sign}{dm.lag_samples:>13d} "
            f"{dm.lag_ms:>10.3f} {dm.correlation_peak:>12.4f}"
        )

    lines.append("")
    lines.append(
        "  Note: positive lag means filtered signal is delayed relative to raw."
    )
    return "\n".join(lines)
