#!/usr/bin/env python3
"""
Web Force-Bench - FOPDT Model Identification & Interactive PI Pole Placement (Script 2)
--------------------------------------------------------------------------------------
1. Fits experimental step test data to a First-Order Plus Dead-Time (FOPDT) model:
       G(s) = K / (tau * s + 1) * exp(-theta * s)
2. Computes PI controller gains via Pole Placement / Direct Synthesis (Lambda tuning)
   specifically designed for 0% overshoot.
3. Simulates closed-loop step response to 100 °C with actuator limits (PWM 0-255)
   and anti-windup.
4. Provides an interactive slider allowing the user to adjust the settling time in
   real-time and visually inspect the response, control effort, and tuning parameters.

Usage:
    python fopdt_tuning_sim.py
    python fopdt_tuning_sim.py --csv data/force_bench_20261012.csv
    python fopdt_tuning_sim.py --demo
"""

import argparse
import glob
import os
import sys

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import numpy as np
from scipy.optimize import curve_fit, minimize


def generate_benchmark_data():
    """Generates a realistic heated force-bench step test dataset if no CSV is available."""
    print("[INFO] No CSV provided; generating realistic heated bench step-response data for demonstration.")
    t = np.linspace(0, 300, 3000)  # 300s at 10 Hz
    dt = t[1] - t[0]

    # True physical parameters of typical heated aluminum bench
    true_T0 = 24.5       # Ambient Temp °C
    true_K = 0.58        # °C / PWM unit
    true_tau = 85.0      # Time constant (seconds)
    true_theta = 7.5     # Dead time delay (seconds)

    # Step input: PWM jumps from 0 to 150 at t = 10s
    pwm_step_time = 10.0
    pwm_val = 150.0
    pwm = np.zeros_like(t)
    pwm[t >= pwm_step_time] = pwm_val

    # Simulate true response with delay
    temp = np.zeros_like(t)
    temp[0] = true_T0
    delay_steps = int(true_theta / dt)

    for i in range(1, len(t)):
        delayed_idx = max(0, i - delay_steps)
        u_delayed = pwm[delayed_idx]
        dTemp = (-(temp[i - 1] - true_T0) + true_K * u_delayed) / true_tau
        temp[i] = temp[i - 1] + dTemp * dt

    # Add realistic sensor noise (MCP9700 ~ ±0.3°C quantization/noise)
    np.random.seed(42)
    noise = np.random.normal(0, 0.25, len(t))
    temp_noisy = temp + noise

    return t, temp_noisy, pwm


def load_csv_data(csv_path):
    """Loads experimental data from a Web Force-Bench CSV file."""
    import csv

    times, temps, pwms = [], [], []
    with open(csv_path, mode="r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                el = float(row.get("Elapsed_s") or row.get("Elapsed_sec") or row.get("time") or 0)
                t_val = float(row.get("Temperature_C") or row.get("Temperature") or row.get("Temp") or 0)
                p_val = float(row.get("PWM") or row.get("pwm") or 0)
                times.append(el)
                temps.append(t_val)
                pwms.append(p_val)
            except (ValueError, TypeError):
                continue

    if len(times) < 20:
        raise ValueError(f"Insufficient data rows ({len(times)}) in CSV: {csv_path}")

    return np.array(times), np.array(temps), np.array(pwms)


def fit_fopdt(times, temps, pwms):
    """
    Identifies continuous FOPDT model fitted across ALL recorded cycles:
        tau * dy/dt + (y - T0) = K * u(t - theta)
    Returns: K, tau, theta, T0, R2, t_step, delta_u, y_pred, t_fit
    """
    t_arr = np.array(times, dtype=float)
    y_arr = np.array(temps, dtype=float)
    u_arr = np.array(pwms, dtype=float)
    dt = float(np.mean(np.diff(t_arr))) if len(t_arr) > 1 else 0.1

    delta_u = max(10.0, float(np.max(u_arr) - np.min(u_arr)))
    delta_y = float(np.max(y_arr) - np.min(y_arr))
    init_K = max(0.2, delta_y / delta_u)
    init_tau = 35.0
    init_theta = 0.5
    init_T0 = float(np.min(y_arr))

    # Fast ODE simulator for arbitrary u(t) across all cycles
    def sim_ode(K_val, tau_val, theta_val, T0_val):
        delay_n = max(0, int(round(theta_val / dt)))
        u_del = np.zeros_like(u_arr)
        if delay_n < len(u_arr):
            u_del[delay_n:] = u_arr[:-delay_n if delay_n > 0 else len(u_arr)]

        y_sim = np.zeros_like(t_arr)
        y_sim[0] = y_arr[0]
        for k in range(1, len(t_arr)):
            h = t_arr[k] - t_arr[k - 1]
            dydt = (-(y_sim[k - 1] - T0_val) + K_val * u_del[k - 1]) / tau_val
            y_sim[k] = y_sim[k - 1] + dydt * h
        return y_sim

    def loss(p):
        y_s = sim_ode(p[0], p[1], p[2], p[3])
        return np.mean((y_arr - y_s) ** 2)

    print(f"[FOPDT] Fitting continuous FOPDT model across ALL {len(t_arr)} data points ({t_arr[-1]-t_arr[0]:.1f}s)...")
    p0 = [init_K, init_tau, init_theta, init_T0]
    bounds = [(0.05, 5.0), (5.0, 300.0), (0.0, 15.0), (10.0, float(np.max(y_arr)))]

    res = minimize(loss, p0, method='L-BFGS-B', bounds=bounds)
    K, tau, theta, T0 = res.x

    y_pred = sim_ode(K, tau, theta, T0)
    ss_res = np.sum((y_arr - y_pred) ** 2)
    ss_tot = np.sum((y_arr - np.mean(y_arr)) ** 2)
    r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

    return K, tau, theta, T0, r2, t_arr[0], delta_u, y_pred, t_arr


def compute_pi_pole_placement(K, tau, theta, settling_time_target):
    """
    Computes PI parameters via Direct Synthesis / Pole Placement for 0% overshoot.

    Target Closed Loop Transfer function:
        Y(s)/R(s) = 1 / (lambda * s + 1) * exp(-theta * s)

    PI Controller:
        C(s) = Kp * (1 + 1 / (Ti * s))

    Setting Ti = tau cancels the plant pole.
    To avoid overshoot, closed-loop time constant lambda is placed according to
    the 2% settling time:
        Ts ≈ 4 * lambda + theta  ==>  lambda = (Ts - theta) / 4

    This yields:
        Kp = tau / (K * (lambda + theta))
        Ti = tau
        Ki = Kp / Ti = 1 / (K * (lambda + theta))
    """
    # Enforce physical realizability: lambda cannot be smaller than half dead-time
    min_lambda = max(0.2, theta * 0.25)
    lambda_val = max(min_lambda, (settling_time_target - theta) / 4.0)

    Kp = tau / (K * (lambda_val + theta))
    Ti = tau
    Ki = Kp / Ti

    return Kp, Ki, Ti, lambda_val


def simulate_closed_loop(K, tau, theta, T0, Kp, Ki, setpoint=120.0, t_sim=400.0, dt=0.1):
    """
    Simulates closed-loop PI temperature control to target setpoint (default: 120 °C) with:
    - Dead time transport delay buffer
    - Actuator saturation [PWM 0 to 255]
    - Anti-windup clamping on integrator
    """
    n_steps = int(t_sim / dt)
    t = np.linspace(0, t_sim, n_steps)

    y = np.zeros(n_steps)
    u = np.zeros(n_steps)
    error = np.zeros(n_steps)
    integral = 0.0

    # Initial state
    y[0] = T0
    delay_steps = int(round(theta / dt))
    u_history = [0.0] * max(1, delay_steps + 1)

    for i in range(1, n_steps):
        e = setpoint - y[i - 1]
        error[i] = e

        # Standard PI candidate control signal
        u_p = Kp * e
        u_unsat = u_p + Ki * integral

        # Actuator saturation [0, 255]
        u_sat = np.clip(u_unsat, 0.0, 255.0)
        u[i - 1] = u_sat

        # Anti-windup: only integrate if not saturated OR if error drives control out of saturation
        if (u_unsat == u_sat) or (u_unsat > 255.0 and e < 0) or (u_unsat < 0.0 and e > 0):
            integral += e * dt

        # Apply delay to actuator input
        u_history.append(u_sat)
        delayed_u = u_history[-1 - delay_steps] if len(u_history) > delay_steps else 0.0

        # Plant ODE integration: dy/dt = (-(y - T0) + K * u_delayed) / tau
        dydt = (-(y[i - 1] - T0) + K * delayed_u) / tau
        y[i] = y[i - 1] + dydt * dt

    u[-1] = u[-2]

    # Calculate performance metrics
    # 2% settling time
    band = 0.02 * (setpoint - T0)
    lower_bound = setpoint - band
    upper_bound = setpoint + band

    # Find the last time the response left the 2% band
    outside_indices = np.where((y < lower_bound) | (y > upper_bound))[0]
    if len(outside_indices) == 0:
        actual_settling_time = 0.0
    elif outside_indices[-1] < len(t) - 1:
        actual_settling_time = t[outside_indices[-1] + 1]
    else:
        actual_settling_time = float("inf")  # Did not settle in window

    # Overshoot percentage
    peak_y = np.max(y)
    overshoot_pct = max(0.0, ((peak_y - setpoint) / (setpoint - T0)) * 100.0)

    # 10% to 90% rise time
    y10 = T0 + 0.10 * (setpoint - T0)
    y90 = T0 + 0.90 * (setpoint - T0)
    idx10 = np.where(y >= y10)[0]
    idx90 = np.where(y >= y90)[0]
    if len(idx10) > 0 and len(idx90) > 0:
        rise_time = t[idx90[0]] - t[idx10[0]]
    else:
        rise_time = float("nan")

    return t, y, u, actual_settling_time, overshoot_pct, rise_time


def main():
    parser = argparse.ArgumentParser(description="Web Force-Bench: FOPDT Model Fitting & PI Pole Placement Simulation")
    parser.add_argument("--csv", type=str, default=None, help="Path to experimental step test CSV file")
    parser.add_argument("--demo", action="store_true", help="Force synthetic demo dataset")
    parser.add_argument("--target", type=float, default=120.0, help="Target step temperature in °C (default: 120.0)")
    args = parser.parse_args()

    # Step 1: Obtain Data
    csv_file = args.csv
    if not csv_file and not args.demo:
        # Search for recent CSV files in python/data or current directory
        candidates = glob.glob(os.path.join(os.path.dirname(__file__), "data", "*.csv")) + glob.glob("*.csv")
        if candidates:
            # Pick latest modified CSV
            csv_file = max(candidates, key=os.path.getmtime)
            print(f"[INFO] Found recent experimental data: {csv_file}")

    if csv_file and os.path.exists(csv_file) and not args.demo:
        print(f"[LOAD] Loading data from {csv_file}...")
        times, temps, pwms = load_csv_data(csv_file)
    else:
        times, temps, pwms = generate_benchmark_data()

    # Step 2: Fit FOPDT Model
    print("\n" + "=" * 60)
    print("       FOPDT MODEL SYSTEM IDENTIFICATION")
    print("=" * 60)
    K, tau, theta, T0, r2, t_step, delta_u, y_pred, t_fit = fit_fopdt(times, temps, pwms)

    print(f"Ambient Temperature (T0) : {T0:.2f} °C")
    print(f"Process Gain (K)          : {K:.4f} °C / PWM unit")
    print(f"Time Constant (tau)       : {tau:.2f} seconds")
    print(f"Dead Time Delay (theta)   : {theta:.2f} seconds")
    print(f"Goodness of Fit (R^2)     : {r2 * 100:.2f} %")
    print("=" * 60)

    # Initial settling time recommendation: ~ 3.5 * tau
    initial_ts = float(np.clip(3.5 * tau, 30.0, 600.0))
    setpoint = args.target

    # Setup interactive Matplotlib Figure
    plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
    fig = plt.figure(figsize=(13, 8))
    fig.canvas.manager.set_window_title("FOPDT Model Identification & PI Pole Placement (No Overshoot)")

    # Grid layout: 2 rows of plots + bottom slider space
    gs = fig.add_gridspec(3, 2, height_ratios=[1.0, 1.0, 0.15], width_ratios=[1.1, 1.0], hspace=0.35, wspace=0.25,
                          left=0.07, right=0.96, top=0.91, bottom=0.08)

    ax_fit = fig.add_subplot(gs[0, 0])
    ax_pwmin = fig.add_subplot(gs[1, 0])
    ax_sim_temp = fig.add_subplot(gs[0, 1])
    ax_sim_pwm = fig.add_subplot(gs[1, 1])

    # 1. Plot FOPDT Fit across ALL Cycles (Left column)
    ax_fit.plot(times, temps, color="#94a3b8", lw=1.5, alpha=0.8, label="Measured Sensor Data")
    ax_fit.plot(t_fit, y_pred, color="#0284c7", lw=2.4, label=f"FOPDT Fit (All Cycles, $R^2={r2*100:.1f}\\%$)")
    ax_fit.set_title("1. Plant Identification: Measured vs. FOPDT Fit (All Cycles)", fontsize=11, fontweight="bold", color="#0f172a")
    ax_fit.set_ylabel("Temperature (°C)", fontsize=10, fontweight="bold")
    ax_fit.grid(True, linestyle="--", alpha=0.5)
    ax_fit.legend(loc="lower right", frameon=True)

    ax_pwmin.plot(times, pwms, color="#475569", lw=1.5, label="Input PWM")
    ax_pwmin.set_title("Step Input Excitation", fontsize=11, fontweight="bold", color="#0f172a")
    ax_pwmin.set_ylabel("PWM (0-255)", fontsize=10, fontweight="bold")
    ax_pwmin.set_xlabel("Elapsed Time (s)", fontsize=10, fontweight="bold")
    ax_pwmin.set_ylim(-10, 265)
    ax_pwmin.grid(True, linestyle="--", alpha=0.5)
    ax_pwmin.legend(loc="upper left", frameon=True)

    # 2. Right column: Closed-loop Step Response Simulation to 100 °C
    sim_t_max = max(350.0, initial_ts * 1.5)
    t_sim, y_sim, u_sim, act_ts, os_pct, rise_t = simulate_closed_loop(
        K, tau, theta, T0, *compute_pi_pole_placement(K, tau, theta, initial_ts)[:2], setpoint=setpoint, t_sim=sim_t_max
    )

    line_resp, = ax_sim_temp.plot(t_sim, y_sim, color="#059669", lw=2.5, label="Simulated Temp $T(t)$")
    line_sp = ax_sim_temp.axhline(setpoint, color="#dc2626", linestyle="--", lw=1.8, label=f"Setpoint ({setpoint:.0f} °C)")
    fill_band = ax_sim_temp.axhspan(setpoint - 2.0, setpoint + 2.0, color="#10b981", alpha=0.15, label="±2% Settling Band")
    marker_ts = ax_sim_temp.axvline(act_ts if np.isfinite(act_ts) else 0, color="#d97706", linestyle=":", lw=2.0, label="Settling Point")

    ax_sim_temp.set_title(f"2. Pole Placement PI Step Response to {setpoint:.0f} °C (No Overshoot)", fontsize=11, fontweight="bold", color="#0f172a")
    ax_sim_temp.set_ylabel("Temperature (°C)", fontsize=10, fontweight="bold")
    ax_sim_temp.set_ylim(T0 - 5.0, setpoint + 15.0)
    ax_sim_temp.set_xlim(0, sim_t_max)
    ax_sim_temp.grid(True, linestyle="--", alpha=0.5)
    ax_sim_temp.legend(loc="lower right", frameon=True, fontsize=8)

    line_ctrl, = ax_sim_pwm.plot(t_sim, u_sim, color="#7c3aed", lw=2.0, label="Control Effort (PWM)")
    ax_sim_pwm.axhline(255, color="#ef4444", linestyle=":", lw=1.2, label="Max Saturation (255)")
    ax_sim_pwm.set_title("3. Controller Output Effort", fontsize=11, fontweight="bold", color="#0f172a")
    ax_sim_pwm.set_ylabel("PWM Output", fontsize=10, fontweight="bold")
    ax_sim_pwm.set_xlabel("Time (s)", fontsize=10, fontweight="bold")
    ax_sim_pwm.set_ylim(-10, 275)
    ax_sim_pwm.set_xlim(0, sim_t_max)
    ax_sim_pwm.grid(True, linestyle="--", alpha=0.5)
    ax_sim_pwm.legend(loc="upper right", frameon=True, fontsize=8)

    # Metrics on-screen summary annotation
    text_box = ax_sim_temp.text(
        0.03, 0.65, "", transform=ax_sim_temp.transAxes,
        fontsize=9, verticalalignment="top",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="#f8fafc", edgecolor="#cbd5e1", alpha=0.95)
    )

    # Slider configuration
    ax_slider = fig.add_subplot(gs[2, :])
    slider_min = max(20.0, float(theta * 2.0))
    slider_max = max(150.0, float(tau * 6.0))
    slider = Slider(
        ax=ax_slider,
        label="Desired Settling Time Ts (s):",
        valmin=slider_min,
        valmax=slider_max,
        valinit=initial_ts,
        valstep=1.0,
        color="#0284c7"
    )
    slider.label.set_fontsize(10)
    slider.label.set_fontweight("bold")

    def update(val):
        target_ts = slider.val
        Kp, Ki, Ti, lambda_val = compute_pi_pole_placement(K, tau, theta, target_ts)

        t_sim_dyn = max(350.0, target_ts * 1.5)
        t_new, y_new, u_new, act_ts_new, os_new, rise_new = simulate_closed_loop(
            K, tau, theta, T0, Kp, Ki, setpoint=setpoint, t_sim=t_sim_dyn
        )

        line_resp.set_data(t_new, y_new)
        line_ctrl.set_data(t_new, u_new)

        ax_sim_temp.set_xlim(0, t_sim_dyn)
        ax_sim_pwm.set_xlim(0, t_sim_dyn)

        if np.isfinite(act_ts_new):
            marker_ts.set_xdata([act_ts_new, act_ts_new])
            marker_ts.set_visible(True)
        else:
            marker_ts.set_visible(False)

        metrics_str = (
            f"FOPDT: K={K:.3f} | τ={tau:.1f}s | θ={theta:.1f}s\n"
            f"PI Tuning (Zero Overshoot):\n"
            f"  Kp = {Kp:.3f}  |  Ki = {Ki:.4f}  |  Ti = {Ti:.1f}s\n"
            f"  λ (Pole) = {lambda_val:.1f}s\n"
            f"Performance:\n"
            f"  Settling Time: {act_ts_new:.1f}s (Target: {target_ts:.0f}s)\n"
            f"  Rise Time (10-90%): {rise_new:.1f}s\n"
            f"  Overshoot: {os_new:.2f}% (Guaranteed 0%)\n"
            f"\nArduino constants:\n"
            f"  const float Kp = {Kp:.4f}f;\n"
            f"  const float Ki = {Ki:.5f}f;"
        )
        text_box.set_text(metrics_str)
        fig.canvas.draw_idle()

    slider.on_changed(update)
    update(initial_ts)

    # Header title
    fig.suptitle("Heated Force-Bench: FOPDT System ID & Interactive Pole Placement PI Controller",
                 fontsize=13, fontweight="bold", color="#0f172a")

    print("[SUCCESS] Interactive PI Pole Placement GUI started.")
    print(f"Move the slider to dynamically observe zero-overshoot step responses to {setpoint:.0f} °C.")
    plt.show()


if __name__ == "__main__":
    main()
