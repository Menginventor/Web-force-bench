#!/usr/bin/env python3
"""
Web Force-Bench - Real-time PI Step Response Tuner & Plotter
------------------------------------------------------------
Allows interactive tuning of Kp, Ki, and Target Setpoint, executes
real-time single-step closed-loop tests directly on the hardware,
renders live temperature and PWM curves, and computes step metrics.

Usage:
    python pi_step_tuner.py
    python pi_step_tuner.py --port COM7 --baud 115200
    python pi_step_tuner.py --sp 100 --kp 6.47 --ki 0.180
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
from matplotlib.widgets import Slider, Button
import numpy as np
import serial
import serial.tools.list_ports


def find_serial_port(preferred_vid="1a86"):
    """Auto-detect suitable USB serial port (prefers CH340 / Arduino)."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    for p in ports:
        desc = (p.description or "").lower()
        vid = f"{p.vid:04x}".lower() if p.vid else ""
        if preferred_vid and preferred_vid.lower() in vid:
            return p.device
        if any(k in desc for k in ["ch340", "cp210", "ftdi", "arduino", "nano"]):
            return p.device
    for p in ports:
        if "bluetooth" not in (p.description or "").lower():
            return p.device
    return ports[0].device


class PiStepControllerApp:
    def __init__(self, port, baud=115200, init_kp=6.47, init_ki=0.180, init_sp=100.0):
        self.port_name = port
        self.baud = baud
        self.kp = init_kp
        self.ki = init_ki
        self.setpoint = init_sp

        self.running = True
        self.is_controlling = False
        self.serial_conn = None
        self.lock = threading.Lock()

        # Step response metrics & markers
        self.step_start_time = None
        self.step_start_temp = None
        self.peak_temp = -999.0

        # Ring buffers for real-time visualization (up to 3000 points @ 10 Hz = 300s)
        self.max_pts = 3000
        self.time_data = deque(maxlen=self.max_pts)
        self.temp_data = deque(maxlen=self.max_pts)
        self.pwm_data = deque(maxlen=self.max_pts)
        self.sp_data = deque(maxlen=self.max_pts)

        # CSV Logging
        data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
        os.makedirs(data_dir, exist_ok=True)
        ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(data_dir, f"pi_step_test_{ts_str}.csv")
        self.csv_file = open(self.csv_path, mode="w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["Timestamp_ISO", "Elapsed_s", "Temperature_C", "PWM", "Setpoint_C"])
        self.csv_file.flush()

        self.start_wall_time = None

    def send_cmd(self, cmd_str):
        """Sends a serial command string to the device."""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                line = (cmd_str.strip() + "\r\n").encode("utf-8")
                self.serial_conn.write(line)
                self.serial_conn.flush()
                print(f"[TX -> DEVICE] {cmd_str.strip()}")
            except Exception as e:
                print(f"[SERIAL TX ERROR] {e}")

    def start_step_test(self, kp, ki, sp):
        """Applies tuning parameters and triggers a single step test."""
        self.kp = kp
        self.ki = ki
        self.setpoint = sp

        with self.lock:
            # Mark step start
            latest_t = self.time_data[-1] if self.time_data else 0.0
            latest_temp = self.temp_data[-1] if self.temp_data else 25.0
            self.step_start_time = latest_t
            self.step_start_temp = latest_temp
            self.peak_temp = latest_temp
            self.is_controlling = True

        print("\n" + "=" * 55)
        print(f">>> STARTING STEP TEST: Target = {sp:.1f} °C | Kp = {kp:.3f} | Ki = {ki:.4f}")
        print("=" * 55)

        self.send_cmd(f"KP {kp:.4f}")
        time.sleep(0.05)
        self.send_cmd(f"KI {ki:.5f}")
        time.sleep(0.05)
        self.send_cmd(f"SP {sp:.2f}")
        time.sleep(0.05)
        self.send_cmd("START CONTROL")

    def stop_heater(self):
        """Safely stops controller and turns off heater."""
        with self.lock:
            self.is_controlling = False
        print("[CONTROL] Stopping controller & setting PWM to 0...")
        self.send_cmd("STOP CONTROL")

    def parse_telemetry(self, raw_line):
        """Extracts Temp, PWM, SP from $DAT: packet."""
        clean = raw_line.strip()
        if not clean.upper().startswith("$DAT:"):
            return None

        payload = clean[5:].strip()
        matches = re.findall(r"([a-zA-Z0-9_\-\.]+)\s*[:=]\s*([-+]?[0-9]*\.?[0-9]+)", payload)
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
                elif key_lower in ["sp", "setpoint"]:
                    parsed["sp"] = val
            except ValueError:
                pass
        return parsed

    def serial_worker(self):
        """Reads incoming telemetry non-blockingly."""
        try:
            self.serial_conn = serial.Serial(self.port_name, self.baud, timeout=0.1)
            time.sleep(1.8)
            print(f"[SERIAL] Connected to {self.port_name} at {self.baud} baud.")

            # Enable 10 Hz telemetry stream
            self.send_cmd("REPORT ON")
            time.sleep(0.1)
            self.send_cmd("STATUS")
        except Exception as e:
            print(f"[ERROR] Failed to open serial port {self.port_name}: {e}")
            self.running = False
            return

        self.start_wall_time = time.time()

        while self.running and self.serial_conn and self.serial_conn.is_open:
            try:
                raw_bytes = self.serial_conn.readline()
                if not raw_bytes:
                    continue
                raw_str = raw_bytes.decode("utf-8", errors="replace").strip()
                if not raw_str:
                    continue

                parsed = self.parse_telemetry(raw_str)
                if parsed:
                    now = time.time()
                    elapsed = now - self.start_wall_time
                    iso_ts = datetime.datetime.now().isoformat()

                    t_val = parsed.get("temp", None)
                    p_val = parsed.get("pwm", None)
                    sp_val = parsed.get("sp", self.setpoint)

                    if t_val is not None:
                        with self.lock:
                            self.time_data.append(elapsed)
                            self.temp_data.append(t_val)
                            self.pwm_data.append(p_val if p_val is not None else 0)
                            self.sp_data.append(sp_val)
                            if self.is_controlling and t_val > self.peak_temp:
                                self.peak_temp = t_val

                        # Write to CSV
                        if self.csv_writer and self.csv_file:
                            self.csv_writer.writerow([
                                iso_ts,
                                f"{elapsed:.3f}",
                                f"{t_val:.2f}",
                                p_val if p_val is not None else 0,
                                f"{sp_val:.2f}"
                            ])
                else:
                    print(f"[DEVICE LOG] {raw_str}")
            except Exception as ex:
                if self.running:
                    print(f"[READ ERROR] {ex}")
                break

    def close(self):
        """Cleans up connection and flushes files."""
        self.running = False
        self.stop_heater()
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        if self.csv_file:
            try:
                self.csv_file.flush()
                self.csv_file.close()
                print(f"[CSV] Session logged to {self.csv_path}")
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(description="Web Force-Bench Real-time PI Step Response Tuner")
    parser.add_argument("--port", type=str, default=None, help="Serial port (auto-detects if omitted)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--sp", type=float, default=120.0, help="Initial target setpoint in °C (default: 120.0)")
    parser.add_argument("--kp", type=float, default=6.47, help="Initial proportional gain Kp (default: 6.47)")
    parser.add_argument("--ki", type=float, default=0.180, help="Initial integral gain Ki (default: 0.180)")
    args = parser.parse_args()

    port = args.port or find_serial_port()
    if not port:
        print("[ERROR] No serial ports found. Please connect your device.")
        sys.exit(1)

    app = PiStepControllerApp(port, baud=args.baud, init_kp=args.kp, init_ki=args.ki, init_sp=args.sp)
    th = threading.Thread(target=app.serial_worker, daemon=True)
    th.start()

    # Setup Matplotlib GUI Figure
    plt.style.use("seaborn-v0_8-darkgrid" if "seaborn-v0_8-darkgrid" in plt.style.available else "default")
    fig = plt.figure(figsize=(12, 8.5))
    fig.canvas.manager.set_window_title("Web Force-Bench - Real-time PI Step Tuner")

    # Grid layout: 2 Plot rows + Bottom Controls
    gs = fig.add_gridspec(3, 1, height_ratios=[1.2, 0.7, 0.65], hspace=0.32, top=0.93, bottom=0.06, left=0.09, right=0.95)

    ax_temp = fig.add_subplot(gs[0])
    ax_pwm = fig.add_subplot(gs[1], sharex=ax_temp)

    # Plot styling
    color_temp = "#0284c7"   # Cyan / Sky blue
    color_sp = "#dc2626"     # Bright red dashed
    color_pwm = "#7c3aed"    # Purple

    line_temp, = ax_temp.plot([], [], color=color_temp, lw=2.4, label="Temperature (°C)")
    line_sp = ax_temp.axhline(args.sp, color=color_sp, linestyle="--", lw=1.8, label=f"Setpoint ({args.sp:.1f} °C)")
    band_span = ax_temp.axhspan(args.sp - 2.0, args.sp + 2.0, color="#10b981", alpha=0.15, label="±2% Tolerance Band")

    ax_temp.set_ylabel("Temperature (°C)", fontsize=11, fontweight="bold")
    ax_temp.grid(True, linestyle="--", alpha=0.5)
    ax_temp.legend(loc="upper left", frameon=True, fontsize=9)

    line_pwm, = ax_pwm.plot([], [], color=color_pwm, lw=1.8, label="Control Effort (PWM)")
    ax_pwm.axhline(255, color="#ef4444", linestyle=":", lw=1.2, label="Saturation (255)")
    ax_pwm.set_ylabel("PWM Output", fontsize=11, fontweight="bold")
    ax_pwm.set_xlabel("Elapsed Time (seconds)", fontsize=11, fontweight="bold")
    ax_pwm.set_ylim(-5, 275)
    ax_pwm.grid(True, linestyle="--", alpha=0.5)
    ax_pwm.legend(loc="upper left", frameon=True, fontsize=9)

    title_text = fig.suptitle(f"Real-time PI Step Tuner: {port} | Ready", fontsize=12, fontweight="bold")

    # Live Metrics HUD Annotation
    hud_text = ax_temp.text(
        0.02, 0.45, "Status: Ready", transform=ax_temp.transAxes,
        fontsize=9, verticalalignment="top",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="#f8fafc", edgecolor="#cbd5e1", alpha=0.92)
    )

    # Bottom UI Controls Subplot Area
    sub_gs = gs[2].subgridspec(4, 3, height_ratios=[1.0, 1.0, 1.0, 1.2], hspace=0.45, wspace=0.25)

    ax_slider_sp = fig.add_subplot(sub_gs[0, :2])
    ax_slider_kp = fig.add_subplot(sub_gs[1, :2])
    ax_slider_ki = fig.add_subplot(sub_gs[2, :2])

    ax_btn_start = fig.add_subplot(sub_gs[3, 0])
    ax_btn_stop = fig.add_subplot(sub_gs[3, 1])
    ax_btn_save = fig.add_subplot(sub_gs[3, 2])

    # Sliders
    slider_sp = Slider(ax_slider_sp, "Target Setpoint (°C): ", 30.0, 160.0, valinit=args.sp, valstep=1.0, color="#dc2626")
    slider_kp = Slider(ax_slider_kp, "Proportional Gain Kp: ", 0.1, 30.0, valinit=args.kp, valstep=0.01, color="#0284c7")
    slider_ki = Slider(ax_slider_ki, "Integral Gain Ki: ", 0.005, 1.00, valinit=args.ki, valstep=0.001, color="#7c3aed")

    for s in [slider_sp, slider_kp, slider_ki]:
        s.label.set_fontsize(9.5)
        s.label.set_fontweight("bold")

    # Buttons
    btn_start = Button(ax_btn_start, "START STEP TEST", color="#10b981", hovercolor="#059669")
    btn_start.label.set_fontsize(10)
    btn_start.label.set_fontweight("bold")
    btn_start.label.set_color("white")

    btn_stop = Button(ax_btn_stop, "STOP HEATER", color="#ef4444", hovercolor="#dc2626")
    btn_stop.label.set_fontsize(10)
    btn_stop.label.set_fontweight("bold")
    btn_stop.label.set_color("white")

    btn_save = Button(ax_btn_save, "SAVE TO EEPROM", color="#0284c7", hovercolor="#0369a1")
    btn_save.label.set_fontsize(9.5)
    btn_save.label.set_fontweight("bold")
    btn_save.label.set_color("white")

    def on_sp_change(val):
        target = slider_sp.val
        line_sp.set_ydata([target, target])
        line_sp.set_label(f"Setpoint ({target:.1f} °C)")
        band_span.set_xy([
            [0, target - 2.0], [1, target - 2.0],
            [1, target + 2.0], [0, target + 2.0]
        ])

    slider_sp.on_changed(on_sp_change)

    def on_click_start(event):
        sp = slider_sp.val
        kp = slider_kp.val
        ki = slider_ki.val
        app.start_step_test(kp, ki, sp)

    def on_click_stop(event):
        app.stop_heater()

    def on_click_save(event):
        kp = slider_kp.val
        ki = slider_ki.val
        app.send_cmd(f"KP {kp:.4f}")
        time.sleep(0.05)
        app.send_cmd(f"KI {ki:.5f}")
        time.sleep(0.05)
        app.send_cmd("SAVE")

    btn_start.on_clicked(on_click_start)
    btn_stop.on_clicked(on_click_stop)
    btn_save.on_clicked(on_click_save)

    def update_plot(frame):
        with app.lock:
            times = list(app.time_data)
            temps = list(app.temp_data)
            pwms = list(app.pwm_data)
            is_ctrl = app.is_controlling
            t_start = app.step_start_time
            t_init = app.step_start_temp
            peak = app.peak_temp

        if not times or len(times) != len(temps):
            return line_temp, line_pwm

        line_temp.set_data(times, temps)
        line_pwm.set_data(times, pwms)

        # Dynamic X range
        x_min = max(0, times[-1] - 120.0)
        x_max = max(times[-1] + 5.0, times[0] + 30.0)
        ax_pwm.set_xlim(x_min, x_max)

        # Dynamic Y range for temp
        current_sp = slider_sp.val
        all_y = [t for t in temps if t is not None] + [current_sp]
        y_min = min(all_y) - 5.0
        y_max = max(all_y) + 5.0
        ax_temp.set_ylim(y_min, y_max)

        # Real-time Metrics HUD
        latest_temp = temps[-1]
        latest_pwm = pwms[-1] if pwms else 0
        error = current_sp - latest_temp

        metrics_lines = [
            f"State: {'CONTROL ACTIVE (10 Hz)' if is_ctrl else 'IDLE / OFF'}",
            f"Current Temp: {latest_temp:.2f} °C",
            f"Setpoint SP : {current_sp:.1f} °C (Err: {error:+.2f} °C)",
            f"PWM Output  : {latest_pwm} / 255"
        ]

        if is_ctrl and t_start is not None and t_init is not None:
            step_elapsed = times[-1] - t_start
            delta_target = current_sp - t_init

            # Overshoot
            if delta_target > 0:
                overshoot_pct = max(0.0, ((peak - current_sp) / delta_target) * 100.0)
            else:
                overshoot_pct = 0.0

            # In ±2% tolerance band check
            in_band = abs(error) <= (0.02 * abs(delta_target) if delta_target != 0 else 2.0)

            metrics_lines.append(f"Step Elapsed: {step_elapsed:.1f} s")
            metrics_lines.append(f"Peak Temp   : {peak:.2f} °C")
            metrics_lines.append(f"Overshoot   : {overshoot_pct:.2f} %")
            metrics_lines.append(f"Settled ±2% : {'YES' if in_band else 'No (Rising)'}")

        hud_text.set_text("\n".join(metrics_lines))
        title_text.set_text(
            f"PI Step Tuner [{port}] | SP: {current_sp:.1f} °C | Temp: {latest_temp:.2f} °C | Kp: {slider_kp.val:.2f} | Ki: {slider_ki.val:.3f}"
        )

        return line_temp, line_pwm

    ani = animation.FuncAnimation(fig, update_plot, interval=60, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        app.close()


if __name__ == "__main__":
    main()
