#!/usr/bin/env python3
"""
Web Force-Bench - Real-time Serial Plotter & CSV Logger (Script 1)
------------------------------------------------------------------
Connects to serial device (Arduino Nano / Heated Force-Bench),
automatically executes the autotune protocol:
    TUNE <t_min> <t_max> <pwm_min> <pwm_max> <cycles>
(Default: 80 °C to 120 °C with 0 to 127 PWM for 5 cycles),
parses structured $DAT: telemetry, displays live plots with cycle thresholds,
and logs all incoming data to CSV.

Usage:
    python realtime_logger.py
    python realtime_logger.py --tmin 80 --tmax 120 --pmin 0 --pmax 127 --cycles 5
    python realtime_logger.py --port COM7 --baud 115200 --no-tune
"""

import argparse
import csv
import datetime
import os
import re
import sys
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import serial.tools.list_ports


def find_serial_port(preferred_vid="1a86"):
    """
    Auto-detect suitable USB serial port.
    Prefers CH340 (VID 1a86), FTDI (0403), CP210x (10c4), Arduino (2341).
    """
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None

    # 1. Look for known USB serial hardware
    for p in ports:
        desc = (p.description or "").lower()
        vid = f"{p.vid:04x}".lower() if p.vid else ""
        if preferred_vid and preferred_vid.lower() in vid:
            return p.device
        if any(keyword in desc for keyword in ["ch340", "cp210", "ftdi", "arduino", "usb-serial", "nano"]):
            return p.device

    # 2. Return first non-bluetooth port if available
    for p in ports:
        desc = (p.description or "").lower()
        if "bluetooth" not in desc:
            return p.device

    # 3. Fallback to first available port
    return ports[0].device


class SerialDataLogger:
    def __init__(self, port, baud=115200, output_csv=None, max_window_sec=120, tune_config=None):
        self.port_name = port
        self.baud = baud
        self.max_window_sec = max_window_sec
        self.tune_config = tune_config or {"enabled": True, "tmin": 80.0, "tmax": 120.0, "pmin": 0, "pmax": 127, "cycles": 5}
        self.running = False
        self.serial_conn = None

        # Prepare output CSV path
        if output_csv:
            self.csv_path = output_csv
        else:
            data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
            os.makedirs(data_dir, exist_ok=True)
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.csv_path = os.path.join(data_dir, f"force_bench_tune_{ts}.csv")

        # Data buffers (time-aligned)
        self.lock = threading.Lock()
        self.start_time = None
        self.total_points = 0

        # Sliding window deques for plotting
        self.time_data = deque()
        self.temp_data = deque()
        self.pwm_data = deque()
        self.volt_data = deque()
        self.adc_data = deque()

        # CSV file handle
        self.csv_file = None
        self.csv_writer = None

    def start_csv(self):
        self.csv_file = open(self.csv_path, mode="w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "Timestamp_ISO",
            "Elapsed_s",
            "Temperature_C",
            "PWM",
            "Voltage_V",
            "Raw_ADC"
        ])
        self.csv_file.flush()
        print(f"[CSV] Logging to: {self.csv_path}")

    def log_row(self, iso_ts, elapsed, temp, pwm, volt, adc):
        if self.csv_writer and self.csv_file:
            self.csv_writer.writerow([
                iso_ts,
                f"{elapsed:.3f}",
                f"{temp:.2f}" if temp is not None else "",
                pwm if pwm is not None else "",
                f"{volt:.4f}" if volt is not None else "",
                adc if adc is not None else ""
            ])
            # Periodic flush
            if self.total_points % 10 == 0:
                self.csv_file.flush()

    def parse_dat_line(self, raw_line):
        """
        Parses $DAT: formatted line into dictionary of values:
        e.g. '$DAT:Temp=25.40,PWM=64,Voltage=0.7540,RawADC=234'
        """
        clean = raw_line.strip()
        if not clean.upper().startswith("$DAT:"):
            return None

        payload = clean[5:].strip()
        # Regex to match key=value or key:value
        pattern = re.compile(r"([a-zA-Z0-9_\-\.]+)\s*[:=]\s*([-+]?[0-9]*\.?[0-9]+)")
        matches = pattern.findall(payload)
        if not matches:
            return None

        parsed = {}
        for k, v in matches:
            key_lower = k.lower()
            try:
                val = float(v)
                if key_lower in ["temp", "temperature"]:
                    parsed["temp"] = val
                elif key_lower == "pwm":
                    parsed["pwm"] = int(val)
                elif key_lower in ["volt", "voltage", "vout"]:
                    parsed["volt"] = val
                elif key_lower in ["raw", "adc", "rawadc"]:
                    parsed["adc"] = int(val)
                else:
                    parsed[k] = val
            except ValueError:
                pass

        return parsed if "temp" in parsed or "pwm" in parsed else None

    def serial_worker(self):
        """Background thread reading from serial port non-blockingly."""
        try:
            self.serial_conn = serial.Serial(self.port_name, self.baud, timeout=0.1)
            time.sleep(1.8)  # Allow Arduino bootloader reset to settle
            print(f"[SERIAL] Connected to {self.port_name} at {self.baud} baud.")

            # 1. Enable telemetry stream
            self.serial_conn.write(b"REPORT ON\r\n")
            self.serial_conn.flush()
            time.sleep(0.2)

            # 2. Issue TUNE command if enabled
            if self.tune_config.get("enabled", True):
                tmin = self.tune_config.get("tmin", 80.0)
                tmax = self.tune_config.get("tmax", 120.0)
                pmin = self.tune_config.get("pmin", 0)
                pmax = self.tune_config.get("pmax", 127)
                cycles = self.tune_config.get("cycles", 5)

                cmd_str = f"TUNE {tmin} {tmax} {pmin} {pmax} {cycles}\r\n"
                print(f"[TUNE] Starting autotune protocol on device: {cmd_str.strip()}")
                self.serial_conn.write(cmd_str.encode("utf-8"))
                self.serial_conn.flush()

        except Exception as e:
            print(f"[ERROR] Failed to open {self.port_name}: {e}")
            self.running = False
            return

        self.start_time = time.time()

        while self.running and self.serial_conn and self.serial_conn.is_open:
            try:
                line_bytes = self.serial_conn.readline()
                if not line_bytes:
                    continue

                line_str = line_bytes.decode("utf-8", errors="replace").strip()
                if not line_str:
                    continue

                # Check if telemetry
                parsed = self.parse_dat_line(line_str)
                if parsed:
                    now = time.time()
                    elapsed = now - self.start_time
                    iso_ts = datetime.datetime.now().isoformat()

                    temp = parsed.get("temp", None)
                    pwm = parsed.get("pwm", None)
                    volt = parsed.get("volt", None)
                    adc = parsed.get("adc", None)

                    with self.lock:
                        self.total_points += 1
                        self.time_data.append(elapsed)
                        self.temp_data.append(temp)
                        self.pwm_data.append(pwm)
                        self.volt_data.append(volt)
                        self.adc_data.append(adc)

                        # Evict old points from plotting window if window length set
                        if self.max_window_sec > 0:
                            cutoff = elapsed - self.max_window_sec
                            while self.time_data and self.time_data[0] < cutoff:
                                self.time_data.popleft()
                                self.temp_data.popleft()
                                self.pwm_data.popleft()
                                self.volt_data.popleft()
                                self.adc_data.popleft()

                    self.log_row(iso_ts, elapsed, temp, pwm, volt, adc)
                else:
                    # Non-telemetry line (Status, Diagnostic, Config, Autotune event)
                    print(f"[DEVICE LOG] {line_str}")

            except Exception as ex:
                if self.running:
                    print(f"[SERIAL READ ERROR] {ex}")
                break

    def stop(self):
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            try:
                # Safely shut off heater on exit
                self.serial_conn.write(b"STOP\r\n")
                self.serial_conn.flush()
                print("[SAFE] Sent STOP command to turn off heater.")
                self.serial_conn.close()
            except Exception:
                pass
        if self.csv_file:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:
                pass
        print(f"\n[DONE] Recorded {self.total_points} telemetry points to {self.csv_path}")


def main():
    parser = argparse.ArgumentParser(description="Web Force-Bench Real-time Serial Logger & Autotune Plotter")
    parser.add_argument("--port", type=str, default=None, help="Serial port (e.g. COM7). Auto-detects if omitted.")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--window", type=float, default=120.0, help="Sliding plot window in seconds (0 for full history, default: 120s)")
    parser.add_argument("--output", type=str, default=None, help="Path to output CSV file")

    # Autotune parameters (Default: 80°C to 120°C with 0 to 127 PWM for 5 cycles)
    parser.add_argument("--tmin", type=float, default=80.0, help="Autotune minimum temperature in °C (default: 80.0)")
    parser.add_argument("--tmax", type=float, default=120.0, help="Autotune maximum temperature in °C (default: 120.0)")
    parser.add_argument("--pmin", type=int, default=0, help="Autotune minimum PWM (default: 0)")
    parser.add_argument("--pmax", type=int, default=127, help="Autotune maximum PWM (default: 127)")
    parser.add_argument("--cycles", type=int, default=5, help="Autotune cycle count (default: 5)")
    parser.add_argument("--no-tune", action="store_true", help="Do not trigger autotune; only monitor and log")

    args = parser.parse_args()

    port = args.port or find_serial_port()
    if not port:
        print("[ERROR] No serial ports found. Please connect your Arduino device.")
        sys.exit(1)

    tune_config = {
        "enabled": not args.no_tune,
        "tmin": args.tmin,
        "tmax": args.tmax,
        "pmin": args.pmin,
        "pmax": args.pmax,
        "cycles": args.cycles
    }

    print("=" * 65)
    print("      Web Force-Bench: Real-time Plotter & Autotune Logger")
    print("=" * 65)
    print(f"Target Port    : {port}")
    print(f"Baud Rate      : {args.baud}")
    print(f"Plot Window    : {args.window}s ({'Sliding' if args.window > 0 else 'Full history'})")
    if tune_config["enabled"]:
        print(f"Autotune Protocol: TUNE {args.tmin}°C to {args.tmax}°C | PWM: {args.pmin}..{args.pmax} | Cycles: {args.cycles}")
    else:
        print("Autotune Protocol: Disabled (--no-tune)")
    print("=" * 65)

    logger = SerialDataLogger(port, baud=args.baud, output_csv=args.output, max_window_sec=args.window, tune_config=tune_config)
    logger.start_csv()
    logger.running = True

    # Start serial background thread
    t = threading.Thread(target=logger.serial_worker, daemon=True)
    t.start()

    # Setup Matplotlib Real-time figure
    plt.style.use("seaborn-v0_8-darkgrid" if "seaborn-v0_8-darkgrid" in plt.style.available else "default")
    fig, (ax_temp, ax_pwm) = plt.subplots(2, 1, figsize=(11, 7.5), sharex=True)
    fig.canvas.manager.set_window_title("Web Force-Bench - Real-time Serial Plotter & Autotune")
    fig.subplots_adjust(hspace=0.25, top=0.91, bottom=0.1)

    # Styling colors
    color_temp = "#0284c7"  # Sky blue
    color_pwm = "#7c3aed"   # Purple

    line_temp, = ax_temp.plot([], [], color=color_temp, lw=2.2, label="Temperature (°C)")
    line_pwm, = ax_pwm.plot([], [], color=color_pwm, lw=1.8, label="Heater PWM (0-255)")

    # Guide lines for Autotune thresholds
    if tune_config["enabled"]:
        ax_temp.axhline(args.tmin, color="#06b6d4", linestyle="--", lw=1.5, alpha=0.85, label=f"T_min ({args.tmin:.1f} °C)")
        ax_temp.axhline(args.tmax, color="#f43f5e", linestyle="--", lw=1.5, alpha=0.85, label=f"T_max ({args.tmax:.1f} °C)")
        ax_pwm.axhline(args.pmax, color="#e11d48", linestyle=":", lw=1.5, alpha=0.85, label=f"PWM_max ({args.pmax})")

    ax_temp.set_ylabel("Temperature (°C)", fontsize=11, fontweight="bold", color="#0f172a")
    ax_temp.grid(True, linestyle="--", alpha=0.6)
    ax_temp.legend(loc="upper left", frameon=True, fontsize=9)

    ax_pwm.set_ylabel("PWM Output", fontsize=11, fontweight="bold", color="#0f172a")
    ax_pwm.set_xlabel("Elapsed Time (seconds)", fontsize=11, fontweight="bold", color="#0f172a")
    ax_pwm.set_ylim(-5, max(140, args.pmax + 20))
    ax_pwm.grid(True, linestyle="--", alpha=0.6)
    ax_pwm.legend(loc="upper left", frameon=True, fontsize=9)

    title_text = fig.suptitle(f"Real-time Telemetry: {port} | Initializing Autotune...", fontsize=12, fontweight="bold")

    def update_plot(frame):
        with logger.lock:
            times = list(logger.time_data)
            temps = list(logger.temp_data)
            pwms = list(logger.pwm_data)
            pts = logger.total_points

        if not times or len(times) != len(temps):
            return line_temp, line_pwm

        # Update line data
        line_temp.set_data(times, temps)
        line_pwm.set_data(times, pwms)

        # Dynamic X Limits
        x_min, x_max = times[0], times[-1]
        if x_max - x_min < 10:
            x_max = x_min + 10
        ax_pwm.set_xlim(x_min, x_max)

        # Dynamic Temp Y Limits
        valid_temps = [t for t in temps if t is not None]
        if valid_temps:
            y_min = min(valid_temps + ([args.tmin] if tune_config["enabled"] else [])) - 4.0
            y_max = max(valid_temps + ([args.tmax] if tune_config["enabled"] else [])) + 4.0
            if y_max - y_min < 10.0:
                y_max = y_min + 10.0
            ax_temp.set_ylim(y_min, y_max)

            latest_t = valid_temps[-1]
            latest_pwm = pwms[-1] if pwms else 0
            title_text.set_text(
                f"Web Force-Bench [{port}] | Temp: {latest_t:.2f} °C | PWM: {latest_pwm} | Autotune ({args.tmin}..{args.tmax} °C) | Samples: {pts}"
            )

        return line_temp, line_pwm

    # 20 Hz visual refresh (50ms interval)
    ani = animation.FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        logger.stop()


if __name__ == "__main__":
    main()
