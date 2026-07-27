#!/usr/bin/env python3
"""
allan_analyze.py - overlapping Allan deviation for the ISM330DHCX capture.

Reads the CSV produced by allan_logger.py, recovers the true sample rate from
the host timestamps, computes overlapping ADEV per axis, and extracts:

  gyro : ARW (angle random walk), bias instability, RRW (rate random walk)
  accel: VRW (velocity random walk), bias instability

Usage:
    python allan_analyze.py allan_run.csv
    python allan_analyze.py allan_run.csv --plot allan.png
"""

import argparse
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

# ISM330DHCX scale factors -- VERIFY against the datasheet for your CTRL1_XL /
# CTRL2_G full-scale settings before trusting the absolute numbers.
GYRO_DPS_PER_LSB = 0.01750    # +/-500 dps
ACC_G_PER_LSB = 0.000122      # +/-4 g
G0 = 9.80665                  # m/s^2
TEMP_LSB_PER_C = 256.0
TEMP_OFFSET_C = 25.0


def overlapping_adev(y, dt, taus):
    """Overlapping Allan deviation of a rate series y sampled at 1/dt."""
    theta = np.concatenate(([0.0], np.cumsum(y) * dt))  # integrated signal
    out_tau, out_sigma = [], []
    seen_m = set()
    for tau in taus:
        m = int(round(tau / dt))
        # log-spaced taus collapse onto the same m at the short end; duplicates
        # produce zero-width intervals and break the later slope estimate
        if m < 1 or 2 * m >= len(theta) - 1 or m in seen_m:
            continue
        seen_m.add(m)
        d = theta[2 * m:] - 2.0 * theta[m:-m] + theta[:-2 * m]
        tau_m = m * dt
        var = np.sum(d * d) / (2.0 * tau_m * tau_m * len(d))
        out_tau.append(tau_m)
        out_sigma.append(np.sqrt(var))
    return np.array(out_tau), np.array(out_sigma)


def fit_slope_region(tau, sigma, target_slope, tau_lo, tau_hi, tol=0.2):
    """Fit sigma = C * tau**target_slope over points whose local log-log slope
    is close to target_slope. Returns C, or None if no usable region."""
    lt, ls = np.log10(tau), np.log10(sigma)
    local = np.gradient(ls, lt)
    sel = (tau >= tau_lo) & (tau <= tau_hi) & (np.abs(local - target_slope) < tol)
    if sel.sum() < 3:
        return None
    # least squares on the intercept with slope forced
    c = np.mean(ls[sel] - target_slope * lt[sel])
    return 10.0 ** c


def characterize(tau, sigma, kind):
    """Return a dict of noise parameters. sigma in deg/s (gyro) or m/s^2 (accel)."""
    res = {}
    tmax = tau[-1]

    # white noise: slope -1/2, read the coefficient at tau = 1 s
    c = fit_slope_region(tau, sigma, -0.5, tau[0], min(10.0, tmax / 10))
    if c is None:  # fall back to the value nearest tau = 1 s
        c = float(np.interp(1.0, tau, sigma))
    res["white_at_1s"] = c

    # bias instability: curve minimum, sigma_min = 0.664 * B
    i = int(np.argmin(sigma))
    res["bias_instability"] = sigma[i] / 0.664
    res["bi_tau"] = tau[i]

    # random walk of the bias: slope +1/2, K such that sigma = K*sqrt(tau/3)
    k = fit_slope_region(tau, sigma, 0.5, tau[i], tmax)
    res["rrw"] = (k * np.sqrt(3.0)) if k is not None else None
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--plot", default="allan.png")
    ap.add_argument("--rate", type=float, default=None,
                    help="override sample rate (Hz); default derives it from host timestamps")
    args = ap.parse_args()

    data = np.genfromtxt(args.csv, delimiter=",", names=True)
    n = data["n"]
    host_t = data["host_t"]
    N = len(n)
    if N < 1000:
        sys.exit(f"only {N} samples - not enough for Allan analysis")

    if args.rate:
        fs = args.rate
    else:
        fs = (n[-1] - n[0]) / (host_t[-1] - host_t[0])
    dt = 1.0 / fs

    temp = data["temp"] / TEMP_LSB_PER_C + TEMP_OFFSET_C
    print(f"samples      : {N}")
    print(f"duration     : {N*dt/3600:.3f} h")
    print(f"sample rate  : {fs:.4f} Hz  (dt = {dt*1000:.4f} ms)")
    print(f"temperature  : {temp.min():.2f} -> {temp.max():.2f} C "
          f"(drift {temp.max()-temp.min():.2f} C)")
    gaps = np.diff(n)
    if np.any(gaps != 1):
        print(f"WARNING: {int(np.sum(gaps[gaps != 1] - 1))} missing samples; "
              f"series treated as uniform")
    print()

    taus = np.logspace(np.log10(dt), np.log10(N * dt / 9.0), 120)

    channels = [
        ("gx", GYRO_DPS_PER_LSB, "gyro"), ("gy", GYRO_DPS_PER_LSB, "gyro"),
        ("gz", GYRO_DPS_PER_LSB, "gyro"),
        ("ax", ACC_G_PER_LSB * G0, "accel"), ("ay", ACC_G_PER_LSB * G0, "accel"),
        ("az", ACC_G_PER_LSB * G0, "accel"),
    ]

    curves, results = {}, {}
    for name, scale, kind in channels:
        y = data[name] * scale
        t, s = overlapping_adev(y - 0.0, dt, taus)
        curves[name] = (t, s, kind)
        results[name] = characterize(t, s, kind)
        results[name]["mean"] = float(np.mean(y))
        results[name]["std"] = float(np.std(y))

    print("=== GYRO ===")
    print(f"{'axis':5} {'mean dps':>10} {'ARW':>12} {'ARW':>12} "
          f"{'bias inst':>12} {'@tau':>8} {'RRW':>12}")
    print(f"{'':5} {'':>10} {'deg/sqrt(hr)':>12} {'dps/sqrt(Hz)':>12} "
          f"{'deg/hr':>12} {'s':>8} {'deg/hr/sqrt(hr)':>12}")
    for name in ("gx", "gy", "gz"):
        r = results[name]
        arw_dps_rthz = r["white_at_1s"]              # dps/sqrt(Hz) == deg/sqrt(s)
        arw_deg_rthr = arw_dps_rthz * 60.0
        bi_deg_hr = r["bias_instability"] * 3600.0
        rrw = r["rrw"]
        rrw_s = f"{rrw*3600.0*60.0:12.4g}" if rrw is not None else f"{'n/a':>12}"
        print(f"{name:5} {r['mean']:10.4f} {arw_deg_rthr:12.4g} {arw_dps_rthz:12.4g} "
              f"{bi_deg_hr:12.4g} {r['bi_tau']:8.1f} {rrw_s}")

    print("\n=== ACCEL ===")
    print(f"{'axis':5} {'mean m/s2':>10} {'VRW':>12} {'noise dens':>14} "
          f"{'bias inst':>12} {'@tau':>8}")
    print(f"{'':5} {'':>10} {'m/s/sqrt(hr)':>12} {'(m/s2)/sqrt(Hz)':>14} "
          f"{'ug':>12} {'s':>8}")
    for name in ("ax", "ay", "az"):
        r = results[name]
        dens = r["white_at_1s"]
        vrw = dens * 60.0
        bi_ug = r["bias_instability"] / G0 * 1e6
        print(f"{name:5} {r['mean']:10.4f} {vrw:12.4g} {dens:14.4g} "
              f"{bi_ug:12.4g} {r['bi_tau']:8.1f}")

    print("\n--- per-sample white noise at this rate (for the filter) ---")
    for name in ("gx", "gy", "gz"):
        s = results[name]["white_at_1s"] / np.sqrt(dt)
        print(f"  {name}: sigma = {s:.5f} dps  ({np.deg2rad(s):.3e} rad/s)")
    for name in ("ax", "ay", "az"):
        s = results[name]["white_at_1s"] / np.sqrt(dt)
        print(f"  {name}: sigma = {s:.5f} m/s^2  "
              f"(~{np.rad2deg(s/G0):.4f} deg of tilt error)")

    if plt is None:
        print("\nmatplotlib not installed - skipping plot")
        return

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
    for ax, kind, title, unit in (
        (axes[0], "gyro", "Gyroscope", "deg/s"),
        (axes[1], "accel", "Accelerometer", "m/s$^2$"),
    ):
        for name, (t, s, k) in curves.items():
            if k != kind:
                continue
            ax.loglog(t, s, label=name)
            r = results[name]
            ax.loglog(t, r["white_at_1s"] / np.sqrt(t), "--", lw=0.7, alpha=0.4,
                      color="grey")
        ax.set_xlabel(r"$\tau$ (s)")
        ax.set_ylabel(rf"$\sigma(\tau)$ ({unit})")
        ax.set_title(f"{title} - overlapping Allan deviation")
        ax.grid(True, which="both", alpha=0.3)
        ax.legend()
    fig.suptitle(f"ISM330DHCX, {N*dt/3600:.2f} h @ {fs:.2f} Hz, "
                 f"{temp.min():.1f}-{temp.max():.1f} C")
    fig.tight_layout()
    fig.savefig(args.plot, dpi=140)
    print(f"\nplot written to {args.plot}")


if __name__ == "__main__":
    main()
