# rosbag_imu_analyzer

基于 Python 的 IMU 数据分析工具，支持 ROS1 和 ROS2 格式的 bag 文件。可对加速度计和陀螺仪数据进行时域分析、频域分析，以及滤波处理与滤波器特性评估。

## 功能特性

- **时域分析**：绘制加速度与角速度时域波形，计算均值、标准差、RMS、最大值、最小值、峰峰值
- **频域分析**：FFT 频谱图，支持 Hann/Hamming/Blackman 窗，自动检测频率峰值
- **多级滤波**：最多两级串联滤波，支持均值滤波、PT1、Biquad、Butterworth、陷波滤波
- **滤波器特性分析**：
  - 频率衰减曲线（dB），标注各频率点衰减量、-3 dB 和 -6 dB 参考线
  - 时域延迟分析（互相关法），输出延迟样本数与毫秒数
- **灵活的时间窗口**：运行时通过命令行参数指定分析的起止时间段
- **支持 ROS1 / ROS2**：自动识别 `.bag`（ROS1）、`.db3` / `.mcap` / 目录（ROS2）格式

## 目录结构

```
rosbag_imu_analyzer/
├── imu_analyzer.py              # 主入口脚本
├── config.yaml                  # 配置文件（含详细注释）
├── requirements.txt             # Python 依赖
└── imu_analyzer/
    ├── data_model.py            # ImuData 数据结构
    ├── config_loader.py         # YAML 配置加载与校验
    ├── bag_reader.py            # ROS1/ROS2 bag 读取
    ├── filters.py               # 滤波器实现
    ├── time_analysis.py         # 时域指标计算
    ├── freq_analysis.py         # 频域指标计算
    ├── filter_analysis.py       # 滤波器特性分析（衰减 + 延迟）
    └── plotter.py               # 绘图
```

---

## 环境要求

| 项目 | 要求 |
|------|------|
| Python | >= 3.9 |
| 操作系统 | Linux / macOS / Windows（WSL） |
| ROS（可选） | 不需要安装 ROS 环境，bag 文件由纯 Python 库读取 |

### Python 依赖

| 库 | 版本 | 说明 |
|----|------|------|
| `numpy` | >= 1.24 | 核心数值计算 |
| `matplotlib` | >= 3.7 | 绘图 |
| `PyYAML` | >= 5.4 | 配置文件解析 |
| `rosbags` | >= 0.9.0 | 纯 Python ROS1/ROS2 bag 读取 |
| `scipy` | >= 1.11 | 信号处理（滤波器与峰值检测） |

---

## 安装

### 方式一：虚拟环境安装（推荐）

```bash
# 克隆仓库
git clone <repo_url> rosbag_imu_analyzer
cd rosbag_imu_analyzer

# 创建并激活虚拟环境
python3 -m venv venv
source venv/bin/activate        # Linux / macOS
# venv\Scripts\activate         # Windows

# 安装依赖
pip install -r requirements.txt
```

### 方式二：直接安装到用户环境

```bash
pip install --user numpy matplotlib PyYAML "rosbags>=0.9.0" "scipy>=1.11"
```

### 验证安装

```bash
python3 imu_analyzer.py --help
```

输出示例：

```
usage: imu_analyzer.py [-h] --bag BAG [--config CONFIG] [--info]
                        [--start SEC] [--end SEC]

IMU ROS bag analyzer: time/frequency domain analysis with filtering
```

---

## 快速开始

### 1. 查看 bag 文件信息

在进行分析前，先查看 bag 文件的时间范围与话题列表：

```bash
python3 imu_analyzer.py --bag /path/to/recording.bag --info
```

输出示例：

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

### 2. 分析完整数据

```bash
python3 imu_analyzer.py --bag /path/to/recording.bag
```

### 3. 分析指定时间段

`--start` 和 `--end` 的单位为**秒**，相对于 bag 文件的起始时刻：

```bash
# 分析第 10 秒到第 30 秒的数据
python3 imu_analyzer.py --bag /path/to/recording.bag --start 10.0 --end 30.0

# 只指定起始时间（到结束）
python3 imu_analyzer.py --bag /path/to/recording.bag --start 5.0

# 只指定结束时间（从开头）
python3 imu_analyzer.py --bag /path/to/recording.bag --end 60.0
```

### 4. 使用自定义配置文件

```bash
python3 imu_analyzer.py --bag /path/to/recording.bag --config my_config.yaml
```

### 5. ROS2 bag 文件

```bash
# .db3 文件
python3 imu_analyzer.py --bag /path/to/recording.db3

# .mcap 文件
python3 imu_analyzer.py --bag /path/to/recording.mcap

# ROS2 bag 目录（包含 metadata.yaml）
python3 imu_analyzer.py --bag /path/to/ros2_bag_dir/
```

---

## 配置文件

配置文件为 YAML 格式，默认路径为当前目录下的 `config.yaml`。

### 基本配置

```yaml
# IMU 话题名称
imu_topic: /imu/data

# ROS 版本：auto（自动检测）| ros1 | ros2
ros_version: auto

# ROS 消息类型库
# ROS1: ROS1_NOETIC
# ROS2: ROS2_HUMBLE | ROS2_IRON | ROS2_JAZZY | ROS2_FOXY | LATEST
ros_typestore: ROS2_HUMBLE

# 输出目录（保存图片）
output_dir: ./output

# 是否保存图片到磁盘
save_plots: true

# 是否弹出交互式图形窗口（需要显示环境）
show_plots: false
```

### 频域设置

```yaml
# FFT 窗函数：hann | hamming | blackman | none
fft_window: hann

# 峰值检测最小高度（相对于频谱最大值的比例，0.0~1.0）
fft_peak_min_height: 0.05

# 峰值之间的最小间距（频率 bin 数）
fft_peak_min_distance: 5
```

### 滤波器配置

```yaml
filters:
  enabled: true   # 设为 true 以启用滤波

  stages:         # 最多配置两级，依次串联执行
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

### 各滤波器参数说明

| 滤波器 | 参数 | 说明 |
|--------|------|------|
| `mean` | `window_size` (int) | 移动平均窗口长度（样本数） |
| `pt1` | `time_constant_s` (float) | 时间常数（秒），约等于 `1/(2π·fc)` |
| `biquad` | `b0, b1, b2, a1, a2` (float) | Direct Form II 系数，a0 默认为 1 |
| `butterworth` | `order`, `cutoff_hz`, `filter_kind` | 阶数、截止频率、类型（low/high/band） |
| `notch` | `notch_freq_hz`, `quality_factor` | 陷波频率（Hz）、Q 值（越大陷波越窄） |

**PT1 滤波器截止频率计算：**

```
fc ≈ 1 / (2π · time_constant_s)
```

例如 `time_constant_s: 0.01` 对应截止频率约 15.9 Hz。

**Biquad 系数可通过外部工具预先计算**（如 MATLAB 的 `filterDesigner`、Python 的 `scipy.signal.butter` 等），再填入配置文件。

---

## 输出说明

### 终端输出

运行后，终端依次输出以下内容：

**原始数据分析：**
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

**滤波后分析（filters.enabled: true 时）：**
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

### 输出图片文件

所有图片保存在 `output_dir` 指定的目录（默认 `./output/`）：

| 文件名 | 内容 | 条件 |
|--------|------|------|
| `raw_time_domain.png` | 原始数据时域波形（2×3，6通道） | 始终生成 |
| `raw_freq_domain.png` | 原始数据频谱图，标注峰值 | 始终生成 |
| `filtered_time_domain.png` | 滤波后时域波形 | 滤波启用时 |
| `filtered_freq_domain.png` | 滤波后频谱图 | 滤波启用时 |
| `comparison_time.png` | 原始与滤波时域对比（叠加） | 滤波启用时 |
| `comparison_freq.png` | 原始与滤波频谱对比（叠加） | 滤波启用时 |
| `filter_attenuation.png` | 滤波器频率衰减曲线（dB） | 滤波启用时 |
| `filter_delay.png` | 时域延迟可视化（互相关） | 滤波启用时 |

---

## 命令行参数

| 参数 | 必填 | 默认值 | 说明 |
|------|------|--------|------|
| `--bag PATH` | 是 | — | bag 文件路径（`.bag` / `.db3` / `.mcap` / ROS2 目录） |
| `--config PATH` | 否 | `config.yaml` | YAML 配置文件路径 |
| `--info` | 否 | — | 仅输出 bag 元信息（时间范围、话题列表），不执行分析 |
| `--start SEC` | 否 | bag 起始 | 分析起始时间（秒，相对于 bag 起始） |
| `--end SEC` | 否 | bag 结束 | 分析结束时间（秒，相对于 bag 起始） |

---

## 常见问题

### Q: 提示 `Topic '/imu/data' not found`

使用 `--info` 查看 bag 中实际存在的话题：

```bash
python3 imu_analyzer.py --bag recording.bag --info
```

然后在 `config.yaml` 中修改 `imu_topic` 为实际话题名称，例如：

```yaml
imu_topic: /sensor/imu_raw
```

### Q: ROS2 bag 应选择哪个 `ros_typestore`

根据录制 bag 时使用的 ROS2 版本选择：

| ROS2 版本 | 配置值 |
|-----------|--------|
| Foxy | `ROS2_FOXY` |
| Galactic | `ROS2_GALACTIC` |
| Humble | `ROS2_HUMBLE` |
| Iron | `ROS2_IRON` |
| Jazzy | `ROS2_JAZZY` |
| 未知/最新 | `LATEST` |

### Q: 在无显示器的服务器上运行

确保 `config.yaml` 中设置：

```yaml
show_plots: false   # 不弹窗
save_plots: true    # 保存图片到磁盘
```

工具默认使用非交互式 matplotlib 后端（`Agg`），无需 GUI 环境。

### Q: 滤波器衰减曲线在低频段出现异常大的正值

这是正常现象：当原始信号在某些频率 bin 上几乎没有能量时（信号无该频率分量），该 bin 的衰减比值在噪声中随机波动。工具会自动将原始能量低于峰值 $10^{-9}$ 的 bin 标记为 NaN（图中显示为断线）。这不影响有意义频率段的衰减结果。

---

## 分析方法说明

### 时域指标

| 指标 | 公式 |
|------|------|
| 均值 | $\bar{x} = \frac{1}{N}\sum x_i$ |
| 标准差 | $\sigma = \sqrt{\frac{1}{N}\sum(x_i - \bar{x})^2}$ |
| RMS | $x_{\text{rms}} = \sqrt{\frac{1}{N}\sum x_i^2}$ |
| 峰峰值 | $x_{\max} - x_{\min}$ |

### 频域分析

对信号加窗后进行 FFT，输出归一化单边幅度谱：

$$X[k] = \frac{2}{N} \left| \text{FFT}(x \cdot w)[k] \right|$$

### 滤波器衰减

$$|H(f)| = \frac{|X_{\text{filtered}}(f)|}{|X_{\text{raw}}(f)|}$$

$$\text{Attenuation}(f) = 20 \log_{10}|H(f)| \quad \text{[dB]}$$

### 时域延迟

使用归一化互相关估计延迟：

$$\text{lag} = \arg\max_\tau \frac{\sum r[n] \cdot f[n - \tau]}{\sqrt{\sum r^2[n] \cdot \sum f^2[n]}}$$

正延迟表示滤波后信号相对于原始信号滞后，这是 IIR 滤波器（PT1、Butterworth 单向滤波）的正常特性。Butterworth 使用零相位双向滤波（`sosfiltfilt`），理论延迟为 0。

---

## 许可证

见 [LICENSE](LICENSE) 文件。
