"""
Filter implementations for IMU signal processing.

Supported filter types:
  - mean:        Moving average (FIR, numpy only)
  - pt1:         First-order PT1 low-pass IIR (numpy only)
  - biquad:      Direct Form II biquad (user-supplied coefficients, numpy only)
  - butterworth: Butterworth IIR via scipy
  - notch:       IIR notch filter via scipy

Up to two stages can be chained via apply_filter_pipeline().
"""
from __future__ import annotations

from typing import Any, Dict, List

import numpy as np
from scipy.signal import butter, sosfiltfilt, iirnotch, lfilter


def apply_filter_pipeline(
    data: np.ndarray,
    sample_rate_hz: float,
    stages: List[Any],  # List[FilterStageConfig]
) -> np.ndarray:
    """Apply up to two filter stages sequentially."""
    result = data.copy()
    for stage in stages[:2]:
        result = apply_filter_stage(result, sample_rate_hz, stage.type, stage.params)
    return result


def apply_filter_stage(
    data: np.ndarray,
    sample_rate_hz: float,
    stage_type: str,
    params: Dict[str, Any],
) -> np.ndarray:
    """Dispatch to the correct filter implementation."""
    dispatch = {
        "mean":        _mean_filter,
        "pt1":         _pt1_filter,
        "biquad":      _biquad_filter,
        "butterworth": _butterworth_filter,
        "notch":       _notch_filter,
    }
    if stage_type not in dispatch:
        raise ValueError(
            f"Unknown filter type: '{stage_type}'. "
            f"Valid: {sorted(dispatch.keys())}"
        )
    return dispatch[stage_type](data, sample_rate_hz, params)


# ---------------------------------------------------------------------------
# Individual filter implementations
# ---------------------------------------------------------------------------

def _mean_filter(
    data: np.ndarray, fs: float, params: Dict[str, Any]
) -> np.ndarray:
    """
    Moving average (uniform FIR).
    params:
      window_size (int): number of samples in the averaging window (default 5)
    """
    window = int(params.get("window_size", 5))
    if window < 1:
        raise ValueError("mean filter: window_size must be >= 1")
    kernel = np.ones(window, dtype=np.float64) / window
    return np.convolve(data, kernel, mode="same")


def _pt1_filter(
    data: np.ndarray, fs: float, params: Dict[str, Any]
) -> np.ndarray:
    """
    First-order PT1 low-pass IIR filter.
    Discrete form: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    where alpha = dt / (tau + dt), tau = time_constant_s, dt = 1/fs

    params:
      time_constant_s (float): filter time constant in seconds (default 0.01)
    """
    tau = float(params.get("time_constant_s", 0.01))
    if tau <= 0:
        raise ValueError("pt1 filter: time_constant_s must be > 0")
    dt = 1.0 / fs
    alpha = dt / (tau + dt)
    out = np.empty_like(data, dtype=np.float64)
    out[0] = data[0]
    for i in range(1, len(data)):
        out[i] = alpha * data[i] + (1.0 - alpha) * out[i - 1]
    return out


def _biquad_filter(
    data: np.ndarray, fs: float, params: Dict[str, Any]
) -> np.ndarray:
    """
    Direct Form II biquad section with user-supplied coefficients.
    Difference equation:
      y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]

    params (all required):
      b0, b1, b2 (float): numerator coefficients
      a1, a2 (float): denominator coefficients (a0 is assumed to be 1)
    """
    try:
        b0 = float(params["b0"])
        b1 = float(params["b1"])
        b2 = float(params["b2"])
        a1 = float(params["a1"])
        a2 = float(params["a2"])
    except KeyError as e:
        raise ValueError(f"biquad filter: missing required parameter {e}") from e

    n = len(data)
    out = np.zeros(n, dtype=np.float64)
    x = data.astype(np.float64)

    for i in range(n):
        x_n  = x[i]
        x_n1 = x[i - 1] if i >= 1 else 0.0
        x_n2 = x[i - 2] if i >= 2 else 0.0
        y_n1 = out[i - 1] if i >= 1 else 0.0
        y_n2 = out[i - 2] if i >= 2 else 0.0
        out[i] = b0 * x_n + b1 * x_n1 + b2 * x_n2 - a1 * y_n1 - a2 * y_n2

    return out


def _butterworth_filter(
    data: np.ndarray, fs: float, params: Dict[str, Any]
) -> np.ndarray:
    """
    Zero-phase Butterworth IIR filter (forward + backward pass).

    params:
      order (int):        filter order (default 4)
      cutoff_hz (float):  cutoff frequency in Hz (required for low/high)
      filter_kind (str):  'low', 'high', or 'band' (default 'low')
      low_cutoff_hz (float):  lower cutoff for band filter
      high_cutoff_hz (float): upper cutoff for band filter
    """
    order = int(params.get("order", 4))
    kind = str(params.get("filter_kind", "low")).lower()
    nyq = fs / 2.0

    if kind == "band":
        low = float(params.get("low_cutoff_hz", params.get("cutoff_hz", nyq * 0.1))) / nyq
        high = float(params.get("high_cutoff_hz", params.get("cutoff_hz", nyq * 0.9))) / nyq
        if not (0 < low < high < 1.0):
            raise ValueError(
                f"butterworth band filter: invalid normalized cutoffs "
                f"low={low:.4f}, high={high:.4f} (must be 0 < low < high < 1)"
            )
        sos = butter(order, [low, high], btype="band", output="sos")
    else:
        if "cutoff_hz" not in params:
            raise ValueError("butterworth filter: 'cutoff_hz' is required")
        wn = float(params["cutoff_hz"]) / nyq
        if not (0 < wn < 1.0):
            raise ValueError(
                f"butterworth filter: cutoff_hz={params['cutoff_hz']} is out of range "
                f"(must be between 0 and Nyquist={nyq:.1f} Hz)"
            )
        sos = butter(order, wn, btype=kind, output="sos")

    return sosfiltfilt(sos, data)


def _notch_filter(
    data: np.ndarray, fs: float, params: Dict[str, Any]
) -> np.ndarray:
    """
    IIR notch filter to attenuate a specific frequency.

    params:
      notch_freq_hz (float):  frequency to notch out in Hz (required)
      quality_factor (float): Q factor controlling notch bandwidth (default 30.0)
                              Higher Q = narrower notch
    """
    if "notch_freq_hz" not in params:
        raise ValueError("notch filter: 'notch_freq_hz' is required")
    freq = float(params["notch_freq_hz"])
    Q = float(params.get("quality_factor", 30.0))
    nyq = fs / 2.0
    w0 = freq / nyq
    if not (0 < w0 < 1.0):
        raise ValueError(
            f"notch filter: notch_freq_hz={freq} is out of range "
            f"(must be between 0 and Nyquist={nyq:.1f} Hz)"
        )
    b, a = iirnotch(w0, Q)
    return lfilter(b, a, data)
