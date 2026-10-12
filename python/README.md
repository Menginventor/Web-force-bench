# Web Force-Bench Python Tools

This directory contains standalone Python utilities for serial hardware communication, real-time plotting, CSV data logging, FOPDT system identification, and interactive zero-overshoot PI pole placement simulation.

---

## Prerequisites

Install Python dependencies:
```bash
pip install pyserial matplotlib numpy scipy
```

---

## Script 1: Real-Time Serial Plotter & CSV Logger (`realtime_logger.py`)

Connects to the Arduino Nano over serial, parses `$DAT:` telemetry packets, displays a 60fps-style real-time plot, and writes data to CSV in real-time.

### Usage
```bash
# Auto-detects port, sends TUNE 80 120 0 127 5, and plots 5 relay cycles in real-time
python realtime_logger.py

# Custom autotune thresholds or cycles:
python realtime_logger.py --tmin 80 --tmax 120 --pmin 0 --pmax 127 --cycles 5

# Only monitor & log without triggering autotune:
python realtime_logger.py --no-tune
```

### Features:
- **Automatic Port Detection**: Detects USB hardware serial chips (CH340, CP210x, FTDI, Arduino Nano).
- **$DAT: Filter**: Extracts `Temp`, `PWM`, `Voltage`, `RawADC` into real-time plots and CSV records. Non-`$DAT:` firmware messages (`[CONFIG]`, `[TUNE]`, `[STATUS]`) are printed cleanly to the console.
- **CSV Output**: Saved directly to `data/force_bench_YYYYMMDD_HHMMSS.csv` with ISO timestamps, elapsed seconds, temperature, and PWM.

---

## Script 2: FOPDT Model Fitting & Interactive PI Pole Placement (`fopdt_tuning_sim.py`)

Identifies a First-Order Plus Dead-Time (FOPDT) transfer function model from recorded step test data and designs a zero-overshoot PI controller with an interactive settling time slider.

### Model Transfer Function
$$G(s) = \frac{K}{\tau s + 1} e^{-\theta s}$$
- **$K$**: Process Gain ($^\circ\text{C} / \text{PWM}$)
- **$\tau$**: Time constant (seconds)
- **$\theta$**: Dead time / Transport delay (seconds)

### Zero-Overshoot Pole Placement Tuning
Cancels the plant pole by choosing $T_i = \tau$ and sets the closed-loop pole $\lambda$ to guarantee strictly monotonic step response:
$$K_p = \frac{\tau}{K (\lambda + \theta)}$$
$$K_i = \frac{1}{K (\lambda + \theta)}$$
$$\lambda = \frac{T_s - \theta}{4}$$

### Usage
```bash
# Uses the most recent CSV file recorded by Script 1
python fopdt_tuning_sim.py

# Specify a particular recorded CSV file
python fopdt_tuning_sim.py --csv data/force_bench_20261012.csv

# Run synthetic benchmark demonstration (no CSV needed)
python fopdt_tuning_sim.py --demo

# Target different setpoint (e.g. 80 °C or 100 °C)
python fopdt_tuning_sim.py --target 100.0
```

### Features:
- **Interactive Slider**: Drag the "Desired Settling Time $T_s$" slider to observe:
  - Real-time step response to 100 °C without overshoot.
  - Required PWM control effort ($0-255$) with saturation and anti-windup.
  - Live calculated $K_p$, $K_i$, and $T_i$ values ready to copy into your Arduino firmware.
