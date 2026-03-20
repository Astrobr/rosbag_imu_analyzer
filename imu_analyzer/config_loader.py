from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml


VALID_FILTER_TYPES = {"mean", "pt1", "biquad", "butterworth", "notch"}
VALID_WINDOWS = {"hann", "hamming", "blackman", "none"}
VALID_ROS_VERSIONS = {"auto", "ros1", "ros2"}
VALID_TYPESTORES = {
    "ROS1_NOETIC", "ROS2_FOXY", "ROS2_GALACTIC", "ROS2_HUMBLE",
    "ROS2_IRON", "ROS2_JAZZY", "LATEST",
}


@dataclass
class FilterStageConfig:
    type: str
    params: Dict[str, Any]


@dataclass
class FilterConfig:
    enabled: bool
    stages: List[FilterStageConfig]


@dataclass
class AnalyzerConfig:
    imu_topic: str
    ros_version: str
    ros_typestore: str
    output_dir: Path
    save_plots: bool
    show_plots: bool
    fft_window: str
    fft_freq_start_hz: float
    fft_peak_min_height: float
    fft_peak_min_distance: int
    filters: FilterConfig


def _get(d: dict, key: str, default: Any = None) -> Any:
    return d.get(key, default)


def load_config(config_path: str | Path) -> AnalyzerConfig:
    """Load and validate YAML config, applying defaults for missing keys."""
    config_path = Path(config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    with open(config_path, encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}

    imu_topic = _get(raw, "imu_topic", "/imu/data")

    ros_version = str(_get(raw, "ros_version", "auto")).lower()
    if ros_version not in VALID_ROS_VERSIONS:
        raise ValueError(
            f"Invalid ros_version '{ros_version}'. Valid: {sorted(VALID_ROS_VERSIONS)}"
        )

    ros_typestore = str(_get(raw, "ros_typestore", "ROS2_HUMBLE"))
    if ros_typestore not in VALID_TYPESTORES:
        raise ValueError(
            f"Invalid ros_typestore '{ros_typestore}'. Valid: {sorted(VALID_TYPESTORES)}"
        )

    output_dir = Path(_get(raw, "output_dir", "./output"))
    save_plots = bool(_get(raw, "save_plots", True))
    show_plots = bool(_get(raw, "show_plots", False))

    fft_window = str(_get(raw, "fft_window", "none")).lower()
    if fft_window not in VALID_WINDOWS:
        raise ValueError(
            f"Invalid fft_window '{fft_window}'. Valid: {sorted(VALID_WINDOWS)}"
        )

    fft_freq_start_hz = float(_get(raw, "fft_freq_start_hz", 0.1))
    if fft_freq_start_hz < 0.0:
        raise ValueError("fft_freq_start_hz must be >= 0")

    fft_peak_min_height = float(_get(raw, "fft_peak_min_height", 0.1))
    fft_peak_min_distance = int(_get(raw, "fft_peak_min_distance", 5))

    # Parse filter config
    raw_filters = _get(raw, "filters", {}) or {}
    filters_enabled = bool(_get(raw_filters, "enabled", False))
    raw_stages = _get(raw_filters, "stages", []) or []

    if len(raw_stages) > 2:
        print(
            f"Warning: {len(raw_stages)} filter stages specified; only the first 2 will be used."
        )

    stages: List[FilterStageConfig] = []
    for i, stage_raw in enumerate(raw_stages[:2]):
        stype = str(stage_raw.get("type", "")).lower()
        if stype not in VALID_FILTER_TYPES:
            raise ValueError(
                f"Filter stage {i+1}: invalid type '{stype}'. "
                f"Valid: {sorted(VALID_FILTER_TYPES)}"
            )
        sparams = dict(stage_raw.get("params", {}) or {})
        stages.append(FilterStageConfig(type=stype, params=sparams))

    filters = FilterConfig(enabled=filters_enabled, stages=stages)

    return AnalyzerConfig(
        imu_topic=imu_topic,
        ros_version=ros_version,
        ros_typestore=ros_typestore,
        output_dir=output_dir,
        save_plots=save_plots,
        show_plots=show_plots,
        fft_window=fft_window,
        fft_freq_start_hz=fft_freq_start_hz,
        fft_peak_min_height=fft_peak_min_height,
        fft_peak_min_distance=fft_peak_min_distance,
        filters=filters,
    )
