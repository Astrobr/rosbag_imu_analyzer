#!/usr/bin/env python3
"""
IMU ROS Bag Analyzer
====================
Analyze IMU data (acceleration & angular velocity) from ROS1/ROS2 bag files.

Usage examples:
  # Inspect a bag file (print time range and topics, no analysis)
  python imu_analyzer.py --bag /path/to/recording.bag --info

  # Analyze the full bag with default config
  python imu_analyzer.py --bag /path/to/recording.bag

  # Analyze a specific time window (seconds relative to bag start)
  python imu_analyzer.py --bag /path/to/recording.bag --start 10.0 --end 30.0

  # Use a custom config file
  python imu_analyzer.py --bag /path/to/recording.bag --config my_config.yaml

  # ROS2 bag directory
  python imu_analyzer.py --bag /path/to/ros2_bag_dir/ --config config.yaml
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # non-interactive by default; overridden if show_plots=true


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="IMU ROS bag analyzer: time/frequency domain analysis with filtering",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument(
        "--bag", required=True,
        help="Path to a ROS1 .bag file, ROS2 .db3/.mcap file, or ROS2 bag directory",
    )
    p.add_argument(
        "--config", default="config.yaml",
        help="Path to YAML configuration file (default: config.yaml)",
    )
    p.add_argument(
        "--info", action="store_true",
        help="Print bag metadata (time range, topics) and exit without analysis",
    )
    p.add_argument(
        "--start", type=float, default=None, metavar="SEC",
        help="Start time in seconds relative to bag start (default: beginning of bag)",
    )
    p.add_argument(
        "--end", type=float, default=None, metavar="SEC",
        help="End time in seconds relative to bag start (default: end of bag)",
    )
    return p


def _print_bag_info(info: dict, bag_path: Path, imu_topic: str) -> None:
    start_s = info["start_time_ns"] / 1e9
    end_s = info["end_time_ns"] / 1e9
    print()
    print(f"Bag file : {bag_path}")
    print(f"Duration : {info['duration_s']:.3f} s  "
          f"({start_s:.3f} s  ->  {end_s:.3f} s  wall clock)")
    print(f"IMU topic: '{imu_topic}'  ->  "
          f"{'FOUND' if info['imu_topic_found'] else 'NOT FOUND'}  "
          f"(~{info['estimated_sample_count']} messages)")
    print(f"All topics ({len(info['topics'])}):")
    for t in info["topics"]:
        print(f"  {t}")
    print()
    print("Tip: use --start / --end (seconds, relative to bag start) "
          "to restrict the analysis window.")
    print()


def _apply_filters_to_imu(imu_data, stages):
    """Apply the filter pipeline to all 6 IMU channels and return a new ImuData."""
    import numpy as np
    from imu_analyzer.data_model import ImuData
    from imu_analyzer.filters import apply_filter_pipeline

    fs = imu_data.sample_rate_hz
    return ImuData(
        timestamps_ns=imu_data.timestamps_ns.copy(),
        timestamps_s=imu_data.timestamps_s.copy(),
        accel_x=apply_filter_pipeline(imu_data.accel_x, fs, stages),
        accel_y=apply_filter_pipeline(imu_data.accel_y, fs, stages),
        accel_z=apply_filter_pipeline(imu_data.accel_z, fs, stages),
        gyro_x=apply_filter_pipeline(imu_data.gyro_x,  fs, stages),
        gyro_y=apply_filter_pipeline(imu_data.gyro_y,  fs, stages),
        gyro_z=apply_filter_pipeline(imu_data.gyro_z,  fs, stages),
        sample_rate_hz=fs,
        topic=imu_data.topic,
        bag_path=imu_data.bag_path,
    )


def main() -> None:
    args = build_parser().parse_args()

    # -- Load config ----------------------------------------------------------
    from imu_analyzer.config_loader import load_config
    try:
        cfg = load_config(args.config)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        print("Create a config.yaml file or specify one with --config.")
        sys.exit(1)
    except (ValueError, Exception) as e:
        print(f"Config error: {e}")
        sys.exit(1)

    bag_path = Path(args.bag).resolve()
    if not bag_path.exists():
        print(f"Error: bag path not found: {bag_path}")
        sys.exit(1)

    # -- Detect ROS version ---------------------------------------------------
    from imu_analyzer.bag_reader import detect_ros_version, get_bag_info, read_imu_data

    ros_version = cfg.ros_version
    if ros_version == "auto":
        try:
            ros_version = detect_ros_version(bag_path)
        except ValueError as e:
            print(f"Error: {e}")
            sys.exit(1)

    print(f"Format   : {ros_version.upper()}  |  typestore: {cfg.ros_typestore}")

    # -- Bag info -------------------------------------------------------------
    try:
        info = get_bag_info(bag_path, ros_version, cfg.ros_typestore, cfg.imu_topic)
    except Exception as e:
        print(f"Error reading bag metadata: {e}")
        sys.exit(1)

    _print_bag_info(info, bag_path, cfg.imu_topic)

    if args.info:
        sys.exit(0)

    if not info["imu_topic_found"]:
        print(
            f"Error: IMU topic '{cfg.imu_topic}' not found in bag.\n"
            f"Available topics: {info['topics']}\n"
            f"Update 'imu_topic' in {args.config} to match."
        )
        sys.exit(1)

    # -- Convert relative time range to absolute nanoseconds ------------------
    bag_start_ns = info["start_time_ns"]
    start_ns = int(bag_start_ns + args.start * 1e9) if args.start is not None else None
    stop_ns  = int(bag_start_ns + args.end   * 1e9) if args.end   is not None else None

    if args.start is not None or args.end is not None:
        s_str = f"{args.start:.3f}s" if args.start is not None else "start"
        e_str = f"{args.end:.3f}s"   if args.end   is not None else "end"
        print(f"Time window: [{s_str}, {e_str}] (relative to bag start)")

    # -- Read IMU data --------------------------------------------------------
    print(f"Reading IMU data from '{cfg.imu_topic}' ...")
    try:
        imu_data = read_imu_data(
            bag_path, ros_version, cfg.ros_typestore, cfg.imu_topic,
            start_ns, stop_ns,
        )
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)

    print(
        f"  Loaded {imu_data.n_samples} samples  |  "
        f"duration={imu_data.duration_s:.3f}s  |  "
        f"rate={imu_data.sample_rate_hz:.2f}Hz"
    )

    # -- matplotlib backend ---------------------------------------------------
    if cfg.show_plots:
        matplotlib.use("TkAgg")  # switch to interactive if show=true

    from imu_analyzer.time_analysis import compute_time_domain_metrics, format_metrics_table
    from imu_analyzer.freq_analysis import compute_freq_domain_metrics, format_freq_summary
    from imu_analyzer.filter_analysis import (
        compute_filter_characterization,
        format_filter_characterization,
    )
    from imu_analyzer.plotter import (
        plot_time_domain, plot_freq_domain,
        plot_comparison, plot_freq_comparison,
        plot_filter_attenuation, plot_filter_delay,
    )

    out = cfg.output_dir

    # =========================================================================
    # RAW DATA ANALYSIS
    # =========================================================================
    print("\n" + "=" * 70)
    print("RAW DATA ANALYSIS")
    print("=" * 70)

    raw_metrics = compute_time_domain_metrics(imu_data)
    print("\n" + format_metrics_table(raw_metrics, title="--- Time Domain Metrics ---"))

    raw_freq = compute_freq_domain_metrics(
        imu_data, cfg.fft_window, cfg.fft_peak_min_height, cfg.fft_peak_min_distance,
    )
    print("\n" + format_freq_summary(raw_freq, title="--- Frequency Domain Peaks ---"))

    # Plots
    print("\nGenerating raw plots ...")
    plot_time_domain(
        imu_data, raw_metrics, title_prefix="Raw",
        save_path=out / "raw_time_domain.png" if cfg.save_plots else None,
        show=cfg.show_plots,
    )
    plot_freq_domain(
        raw_freq, title_prefix="Raw",
        save_path=out / "raw_freq_domain.png" if cfg.save_plots else None,
        show=cfg.show_plots,
    )

    # =========================================================================
    # FILTERED DATA ANALYSIS (if enabled)
    # =========================================================================
    if cfg.filters.enabled and cfg.filters.stages:
        print("\n" + "=" * 70)
        print("FILTERED DATA ANALYSIS")
        print("=" * 70)

        for i, stage in enumerate(cfg.filters.stages[:2]):
            print(f"  Stage {i + 1}: {stage.type}  params={stage.params}")

        try:
            filtered_data = _apply_filters_to_imu(imu_data, cfg.filters.stages)
        except RuntimeError as e:
            # scipy not available
            print(f"\nFilter error: {e}")
            sys.exit(1)
        except Exception as e:
            print(f"\nFilter error: {e}")
            sys.exit(1)

        imu_data.filtered = filtered_data

        filt_metrics = compute_time_domain_metrics(filtered_data)
        print("\n" + format_metrics_table(filt_metrics, title="--- Filtered Time Domain Metrics ---"))

        filt_freq = compute_freq_domain_metrics(
            filtered_data, cfg.fft_window,
            cfg.fft_peak_min_height, cfg.fft_peak_min_distance,
        )
        print("\n" + format_freq_summary(filt_freq, title="--- Filtered Frequency Domain Peaks ---"))

        print("\nGenerating filtered plots ...")
        plot_time_domain(
            filtered_data, filt_metrics, title_prefix="Filtered",
            save_path=out / "filtered_time_domain.png" if cfg.save_plots else None,
            show=cfg.show_plots,
        )
        plot_freq_domain(
            filt_freq, title_prefix="Filtered",
            save_path=out / "filtered_freq_domain.png" if cfg.save_plots else None,
            show=cfg.show_plots,
        )
        plot_comparison(
            imu_data, filtered_data,
            save_path=out / "comparison_time.png" if cfg.save_plots else None,
            show=cfg.show_plots,
        )
        plot_freq_comparison(
            raw_freq, filt_freq,
            save_path=out / "comparison_freq.png" if cfg.save_plots else None,
            show=cfg.show_plots,
        )

        # -- Filter Characterization: attenuation + delay --------------------
        print("\n" + "=" * 70)
        print("FILTER CHARACTERIZATION")
        print("=" * 70)
        print("Computing frequency attenuation and time delay ...")

        filter_char = compute_filter_characterization(
            imu_data, filtered_data, window=cfg.fft_window,
        )
        print("\n" + format_filter_characterization(filter_char))

        print("\nGenerating filter characterization plots ...")
        plot_filter_attenuation(
            filter_char,
            save_path=out / "filter_attenuation.png" if cfg.save_plots else None,
            show=cfg.show_plots,
        )
        plot_filter_delay(
            imu_data, filtered_data, filter_char,
            save_path=out / "filter_delay.png" if cfg.save_plots else None,
            show=cfg.show_plots,
        )
    else:
        print("\nFilters: disabled (or no stages configured).")

    print("\nDone.")
    if cfg.save_plots:
        print(f"Output plots saved to: {out.resolve()}")


if __name__ == "__main__":
    main()
