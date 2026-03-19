# rosbag_imu_analyzer

A Python-based IMU data analysis tool supporting ROS1 and ROS2 bag files. Performs time-domain analysis, frequency-domain analysis, signal filtering, and filter characterization on accelerometer and gyroscope data.

## Features

- **Time-domain analysis**: Plot acceleration and angular velocity waveforms; compute mean, standard deviation, RMS, max, min, and peak-to-peak values
- **Frequency-domain analysis**: FFT spectra with Hann/Hamming/Blackman windowing and automatic peak detection
- **Multi-stage filtering**: Up to two cascaded filter stages — mean, PT1, Biquad, Butterworth, and notch filters
- **Filter characterization**:
  - Frequency attenuation curve (dB) with -3 dB and -6 dB reference lines and per-frequency annotations
  - Time-domain delay analysis via cross-correlation, reporting lag in samples and milliseconds
- **Flexible time window**: Restrict analysis to any time range via command-line arguments
- **ROS1 / ROS2 support**: Auto-detects `.bag` (ROS1), `.db3` / `.mcap` / directory (ROS2) formats

## Project Structure

```
rosbag_imu_analyzer/
├── imu_analyzer.py              # Main entry point
├── config.yaml                  # Configuration file (with detailed comments)
├── requirements.txt             # Python dependencies
└── imu_analyzer/
    ├── data_model.py            # ImuData data structure
    ├── config_loader.py         # YAML config loading and validation
    ├── bag_reader.py            # ROS1/ROS2 bag reading
    ├── filters.py               # Filter implementations
    ├── time_analysis.py         # Time-domain metrics
    ├── freq_analysis.py         # Frequency-domain metrics
    ├── filter_analysis.py       # Filter characterization (attenuation + delay)
    └── plotter.py               # All plotting functions
```

---

## Requirements

| Item | Requirement |
|------|-------------|
| Python | >= 3.9 |
| OS | Linux / macOS / Windows (WSL) |
| ROS | Not required — bag files are read by a pure-Python library |

### Python Dependencies

| Library | Version | Description |
|---------|---------|-------------|
| `numpy` | >= 1.24 | Numerical computation |
| `matplotlib` | >= 3.7 | Plotting |
| `PyYAML` | >= 5.4 | Configuration file parsing |
| `rosbags` | >= 0.9.0 | Pure-Python ROS1/ROS2 bag reading |
| `scipy` | >= 1.11 | Signal processing (filters and peak detection) |

---

## Installation

### Option 1: Virtual environment (recommended)

```bash
# Clone the repository
git clone <repo_url> rosbag_imu_analyzer
cd rosbag_imu_analyzer

# Create and activate a virtual environment
python3 -m venv venv
source venv/bin/activate        # Linux / macOS
# venv\Scripts\activate         # Windows

# Install dependencies
pip install -r requirements.txt
```

### Option 2: Install directly into user environment

```bash
pip install --user numpy matplotlib PyYAML "rosbags>=0.9.0" "scipy>=1.11"
```

### Verify installation

```bash
python3 imu_analyzer.py --help
```

Expected output:

```
usage: imu_analyzer.py [-h] --bag BAG [--config CONFIG] [--info]
                        [--start SEC] [--end SEC]

IMU ROS bag analyzer: time/frequency domain analysis with filtering
```

---

## Quick Start

### 1. Inspect a bag file

Before running analysis, check the bag's time range and available topics:

```bash
python3 imu_analyzer.py --bag /path/to/recording.bag --info
```

Example output:

```
Format   : ROS1  |  typestore: ROS1_NOETIC

Bag file : /path/to/recording.bag
Duration : 120.500 s  (1700000000.000 s  ->  1700000120.500 s  wall clock)
IMU topic: '/imu/data'  ->  FOUND  (~12050 messages)
All topics (3):
  /imu/data
  /tf
  /camera/image_raw

Tip: use --start / --end (seconds, relative to bag start) to restrict the analysis window.
```

### 2. Analyze the full bag

```bash
python3 imu_analyzer.py --bag /path/to/recording.bag
```

### 3. Analyze a specific time window

`--start` and `--end` are in **seconds**, relative to the bag start time:

```bash
# Analyze from 10 s to 30 s
python3 imu_analyzer.py --bag /path/to/recording.bag --start 10.0 --end 30.0

# From a start offset to the end
python3 imu_analyzer.py --bag /path/to/recording.bag --start 5.0

# From the beginning up to an end offset
python3 imu_analyzer.py --bag /path/to/recording.bag --end 60.0
```

### 4. Use a custom configuration file

```bash
python3 imu_analyzer.py --bag /path/to/recording.bag --config my_config.yaml
```

### 5. ROS2 bag files

```bash
# .db3 file
python3 imu_analyzer.py --bag /path/to/recording.db3

# .mcap file
python3 imu_analyzer.py --bag /path/to/recording.mcap

# ROS2 bag directory (contains metadata.yaml)
python3 imu_analyzer.py --bag /path/to/ros2_bag_dir/
```

---

## Configuration File

The configuration file is in YAML format. The default path is `config.yaml` in the current directory.

### Basic settings

```yaml
# IMU topic name in the bag file
imu_topic: /imu/data

# ROS version: auto (detect from file) | ros1 | ros2
ros_version: auto

# ROS message typestore
# ROS1: ROS1_NOETIC
# ROS2: ROS2_HUMBLE | ROS2_IRON | ROS2_JAZZY | ROS2_FOXY | LATEST
ros_typestore: ROS2_HUMBLE

# Directory for saved plot images
output_dir: ./output

# Save plots to disk as PNG files
save_plots: true

# Show interactive plot windows (requires a display environment)
show_plots: false
```

### Frequency-domain settings

```yaml
# FFT window function: hann | hamming | blackman | none
fft_window: hann

# Minimum peak height as a fraction of the spectrum maximum (0.0–1.0)
fft_peak_min_height: 0.05

# Minimum separation between detected peaks (in frequency bins)
fft_peak_min_distance: 5
```

### Filter configuration

```yaml
filters:
  enabled: true   # Set to true to enable filtering

  stages:         # Up to 2 stages applied sequentially
    - type: butterworth
      params:
        order: 4
        cutoff_hz: 80.0
        filter_kind: low   # low | high | band
    - type: notch
      params:
        notch_freq_hz: 50.0
        quality_factor: 30.0
```

### Filter parameter reference

| Filter | Parameters | Description |
|--------|------------|-------------|
| `mean` | `window_size` (int) | Moving average window length in samples |
| `pt1` | `time_constant_s` (float) | Time constant in seconds; approx. `1/(2π·fc)` |
| `biquad` | `b0, b1, b2, a1, a2` (float) | Direct Form II coefficients; a0 is assumed to be 1 |
| `butterworth` | `order`, `cutoff_hz`, `filter_kind` | Filter order, cutoff frequency, type (low/high/band) |
| `notch` | `notch_freq_hz`, `quality_factor` | Notch frequency in Hz; higher Q = narrower notch |

**PT1 cutoff frequency:**

```
fc ≈ 1 / (2π · time_constant_s)
```

For example, `time_constant_s: 0.01` gives a cutoff of approximately 15.9 Hz.

**Biquad coefficients** can be computed with external tools (e.g., MATLAB's Filter Designer or `scipy.signal.butter`) and entered directly in the config file.

---

## Output

### Terminal output

The tool prints the following sections in order:

**Raw data analysis:**
```
======================================================================
RAW DATA ANALYSIS
======================================================================

--- Time Domain Metrics ---
Channel              Mean          Std          RMS          Min          Max          P2P
------------------------------------------------------------------------------------------
accel_x          0.012345     0.987654     0.987731    -3.141593     3.141593     6.283185
...
  Sample rate: 200.00 Hz  |  Duration: 30.000 s  |  Samples: 6000

--- Frequency Domain Peaks ---
Channel       Dominant (Hz) Top Peaks (Hz @ magnitude)
----------------------------------------------------------------------
accel_x               12.34     12.34@0.4521  50.00@0.1203
...
```

**Filtered analysis (when `filters.enabled: true`):**
```
======================================================================
FILTERED DATA ANALYSIS
======================================================================
  Stage 1: butterworth  params={'order': 4, 'cutoff_hz': 80.0, 'filter_kind': 'low'}

--- Filtered Time Domain Metrics ---
...

======================================================================
FILTER CHARACTERIZATION
======================================================================

=== Filter Characterization ===

Frequency Attenuation:
  Channel          1.0Hz       5.0Hz      10.0Hz      20.0Hz      50.0Hz     100.0Hz
  -----------------------------------------------------------------------------------
  accel_x        -0.00dB     -0.02dB     -0.08dB     -0.34dB     -3.01dB    -23.47dB
  ...

Time Delay (cross-correlation):
  Channel    Lag (samples)   Lag (ms)   xcorr peak
  --------------------------------------------------
  accel_x    +           0      0.000       1.0000
  ...
  Note: positive lag means filtered signal is delayed relative to raw.
```

### Output plot files

All plots are saved to the directory specified by `output_dir` (default: `./output/`):

| File | Content | Condition |
|------|---------|-----------|
| `raw_time_domain.png` | Raw time-domain waveforms (2×3 grid, 6 channels) | Always |
| `raw_freq_domain.png` | Raw frequency spectra with peak markers | Always |
| `filtered_time_domain.png` | Filtered time-domain waveforms | Filtering enabled |
| `filtered_freq_domain.png` | Filtered frequency spectra | Filtering enabled |
| `comparison_time.png` | Raw vs filtered time-domain overlay | Filtering enabled |
| `comparison_freq.png` | Raw vs filtered frequency spectra overlay | Filtering enabled |
| `filter_attenuation.png` | Filter frequency attenuation curve (dB) | Filtering enabled |
| `filter_delay.png` | Time-domain delay visualization (cross-correlation) | Filtering enabled |

---

## Command-Line Arguments

| Argument | Required | Default | Description |
|----------|----------|---------|-------------|
| `--bag PATH` | Yes | — | Path to bag file (`.bag` / `.db3` / `.mcap` / ROS2 directory) |
| `--config PATH` | No | `config.yaml` | Path to YAML configuration file |
| `--info` | No | — | Print bag metadata (time range, topics) and exit without analysis |
| `--start SEC` | No | bag start | Analysis start time in seconds, relative to bag start |
| `--end SEC` | No | bag end | Analysis end time in seconds, relative to bag start |

---

## FAQ

### Q: `Topic '/imu/data' not found`

Use `--info` to list the topics actually present in the bag:

```bash
python3 imu_analyzer.py --bag recording.bag --info
```

Then update `imu_topic` in `config.yaml` to match, for example:

```yaml
imu_topic: /sensor/imu_raw
```

### Q: Which `ros_typestore` should I use for a ROS2 bag?

Choose the value matching the ROS2 version used to record the bag:

| ROS2 version | Config value |
|--------------|-------------|
| Foxy | `ROS2_FOXY` |
| Galactic | `ROS2_GALACTIC` |
| Humble | `ROS2_HUMBLE` |
| Iron | `ROS2_IRON` |
| Jazzy | `ROS2_JAZZY` |
| Unknown / latest | `LATEST` |

### Q: Running on a headless server

Ensure `config.yaml` has:

```yaml
show_plots: false   # do not open windows
save_plots: true    # save plots to disk
```

The tool uses the non-interactive `Agg` matplotlib backend by default and requires no display environment.

### Q: The attenuation curve shows large positive values at low frequencies

This is expected. When the raw signal has little or no energy at a given frequency bin, the attenuation ratio is dominated by noise and can take arbitrary values. The tool automatically marks bins where the raw energy is below $10^{-9}$ of the spectrum peak as NaN, which matplotlib renders as gaps in the curve. This does not affect results at frequencies where the signal has meaningful content.

---

## Analysis Methods

### Time-domain metrics

| Metric | Formula |
|--------|---------|
| Mean | $\bar{x} = \frac{1}{N}\sum x_i$ |
| Std dev | $\sigma = \sqrt{\frac{1}{N}\sum(x_i - \bar{x})^2}$ |
| RMS | $x_{\text{rms}} = \sqrt{\frac{1}{N}\sum x_i^2}$ |
| Peak-to-peak | $x_{\max} - x_{\min}$ |

### Frequency-domain analysis

Signals are windowed before FFT. The normalized single-sided amplitude spectrum is:

$$X[k] = \frac{2}{N} \left| \text{FFT}(x \cdot w)[k] \right|$$

### Filter attenuation

$$|H(f)| = \frac{|X_{\text{filtered}}(f)|}{|X_{\text{raw}}(f)|}$$

$$\text{Attenuation}(f) = 20 \log_{10}|H(f)| \quad \text{[dB]}$$

### Time-domain delay

Delay is estimated via normalized cross-correlation:

$$\text{lag} = \arg\max_\tau \frac{\sum r[n] \cdot f[n - \tau]}{\sqrt{\sum r^2[n] \cdot \sum f^2[n]}}$$

A positive lag means the filtered signal is delayed relative to the raw signal. This is the expected behavior for causal IIR filters such as PT1. Butterworth filtering uses a zero-phase forward-backward pass (`sosfiltfilt`), so its theoretical delay is zero.

---

## License

See the [LICENSE](LICENSE) file.
