#!/usr/bin/env python3
"""
compare_filters.py - the Phase 4 deliverable.

Scores complementary and Kalman variants against the same synthetic records
and emits a markdown table per scenario.

Rows are filter variants, columns are metrics, one table per scenario. The
two rows that carry the argument are the bias column (the complementary
filter has no entry -- not worse, absent) and the tuning column (the
complementary filter needs one number found by sweeping against ground truth
that does not exist on real hardware; the Kalman filter needs three numbers
measured in Phase 2).

Usage:
    python3 compare_filters.py
    python3 compare_filters.py --md phase4.md --seeds 1 2 3 4 5
"""

import argparse

import numpy as np

import filter_testbench as fb
import kalman as km
from accel_disturbance import lever_arm

D = np.degrees

# Scenario -> (profile, simulate_imu kwargs)
SCENARIOS = [
    ("clean, gentle",        "gentle", {}),
    ("clean, brisk",         "brisk",  {}),
    ("lever 30 mm, brisk",   "brisk",  dict(lever_arm_r=(0, 0, 0.03))),
    ("lever+vib+taps, brisk", "brisk", dict(lever_arm_r=(0, 0, 0.03),
                                            vib_rms=0.05, tap_rate=0.3)),
]

# label, config, tuning-inputs description
VARIANTS = [
    ("CF, tuned per scenario", dict(kind="cf", tau=None), "1 swept"),
    ("CF, tuned on clean only", dict(kind="cf", tau=0.0221), "1 swept"),
    ("KF, bare", dict(kind="kf"), "3 measured"),
    ("KF + gate", dict(kind="kf", gate=0.01, dwell=0.15),
     "3 measured + 2 tuned"),
    ("KF + lever comp", dict(kind="kf", comp_rz=0.03),
     "3 measured + r"),
    ("KF + gate + comp", dict(kind="kf", gate=0.01, dwell=0.15,
                              comp_rz=0.03), "3 measured + 2 + r"),
]

TAUS = np.logspace(np.log10(0.005), np.log10(100.0), 41)

# Steady state is a property of Q and R, not of the record. Compute once.
_SS = km.steady_state_iterate(km.QR_PITCH)
KF_TAU_EQ = km.equivalent_tau(_SS["K0"])


def lowpass(x, fc, dt=fb.DT):
    """First-order causal low-pass. Used on the rate before differencing."""
    if fc is None or fc <= 0:
        return x.copy()
    a = np.exp(-2.0 * np.pi * fc * dt)
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = a * y[i - 1] + (1.0 - a) * x[i]
    return y


def compensate_lever_arm(ax, az, gyro_rad, r_z, fc=8.0, dt=fb.DT):
    """Subtract the predicted lever-arm acceleration from the specific force.

    Angular acceleration comes from differencing the measured rate, which
    amplifies gyro noise by 1/dt. At dt = 9.88 ms the gyro's 8.1e-4 rad/s
    becomes 0.082 rad/s^2, and at r_z = 30 mm that is 2.5e-3 m/s^2 -- a third
    of the accelerometer's own noise, injected by the compensation itself.
    Hence the low-pass on the rate before differencing.

    Rate bias is NOT removed here; it contributes only through the
    centripetal term and is negligible at these rates.
    """
    n = len(gyro_rad)
    w = lowpass(gyro_rad, fc, dt)
    omega = np.zeros((n, 3))
    omega[:, 1] = w
    alpha = np.zeros((n, 3))
    alpha[:, 1] = np.gradient(w, dt)
    pred = lever_arm(omega, alpha, (0.0, 0.0, r_z))
    return ax - pred[:, 0], az - pred[:, 2]


def score(err_deg, static):
    return dict(
        rms_all=fb.rms(err_deg),
        rms_static=fb.rms(err_deg[static]),
        rms_dyn=fb.rms(err_deg[~static]),
        peak=float(np.abs(err_deg).max()),
    )


def bias_convergence(bias_est_dps, bias_true_dps, dt=fb.DT, tol=0.01):
    """Time until the bias error stays inside tol, from a cold start."""
    e = np.abs(bias_est_dps - bias_true_dps)
    bad = np.flatnonzero(e > tol)
    if len(bad) == 0:
        return 0.0
    if bad[-1] == len(e) - 1:
        return float("nan")          # never settled
    return (bad[-1] + 1) * dt


def run_variant(cfg, gyro, ax, az, truth, static, bias_true):
    """One filter variant on one record. Returns a metrics dict."""
    if cfg["kind"] == "cf":
        aa = fb.accel_angle(ax, az)
        tau = cfg["tau"]
        if tau is None:
            errs = [fb.rms(fb.complementary(gyro, aa, x) - truth)
                    for x in TAUS]
            tau = TAUS[int(np.argmin(errs))]
        err = fb.complementary(gyro, aa, tau) - truth
        m = score(err, static)
        m.update(tau=tau, bias_rms=np.nan, bias_t=np.nan, accepted=100.0)
        return m

    ax_u, az_u = ax, az
    if cfg.get("comp_rz"):
        ax_u, az_u = compensate_lever_arm(ax, az, np.deg2rad(gyro),
                                          cfg["comp_rz"])
    aa = fb.accel_angle(ax_u, az_u)
    norm = np.hypot(ax_u, az_u)

    out = km.run(np.deg2rad(gyro), np.deg2rad(aa), km.QR_PITCH,
                 theta0=np.deg2rad(aa[0]), b0=0.0,
                 accel_norm=norm if cfg.get("gate") else None,
                 g=fb.G,
                 gate_thresh=cfg.get("gate"),
                 dwell_s=cfg.get("dwell", 0.0))

    err = D(out["theta"]) - truth
    m = score(err, static)
    b_est = D(out["bias"])
    m.update(
        tau=KF_TAU_EQ,
        bias_rms=fb.rms(b_est[static] - bias_true[static]),
        bias_t=bias_convergence(b_est, bias_true),
        accepted=100.0 * out["accepted"].mean(),
    )
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3, 4, 5])
    ap.add_argument("--md", default="phase4_comparison.md")
    args = ap.parse_args()

    lines = ["# Phase 4 - filter comparison\n",
             f"Synthetic records, {args.duration:.0f} s at {fb.FS:.4f} Hz "
             f"(dt = {fb.DT:.7f} s), seeds {args.seeds}, values are means "
             f"over seeds. All errors in degrees.\n",
             "Kalman Q and R derived from the Phase 2 Allan deviation "
             "results; no tuning against these records.\n"]

    for sc_label, profile, kw in SCENARIOS:
        print("=" * 100)
        print(sc_label)
        print("=" * 100)
        print(f"{'variant':<26} {'tau':>7} {'static':>8} {'dynamic':>8} "
              f"{'peak':>8} {'biasRMS':>8} {'bias t':>8} {'upd%':>6}  tuning")
        lines.append(f"\n## {sc_label}\n")
        lines.append("| variant | tau (s) | RMS static | RMS dynamic | peak "
                     "| bias RMS (deg/s) | bias settle (s) | updates used "
                     "| tuning inputs |")
        lines.append("|---|---|---|---|---|---|---|---|---|")

        for v_label, cfg, tuning in VARIANTS:
            acc = []
            for seed in args.seeds:
                rng = np.random.default_rng(seed)
                t, truth, rate, static = fb.PROFILES[profile](args.duration)
                gyro, ax, az, bias_true, dist = fb.simulate_imu(
                    truth, rate, rng, **kw)
                acc.append(run_variant(cfg, gyro, ax, az, truth, static,
                                       bias_true))
            def _mean(key):
                vals = np.array([a[key] for a in acc], dtype=float)
                return float("nan") if np.isnan(vals).all() else float(
                    np.nanmean(vals))

            agg = {k: _mean(k) for k in acc[0]}

            def f(x, n=4):
                return "n/a" if np.isnan(x) else f"{x:.{n}f}"

            print(f"{v_label:<26} {agg['tau']:>7.4f} {agg['rms_static']:>8.4f} "
                  f"{agg['rms_dyn']:>8.4f} {agg['peak']:>8.4f} "
                  f"{f(agg['bias_rms']):>8} {f(agg['bias_t'],1):>8} "
                  f"{agg['accepted']:>5.1f}%  {tuning}")
            lines.append(
                f"| {v_label} | {agg['tau']:.4f} | {agg['rms_static']:.4f} "
                f"| {agg['rms_dyn']:.4f} | {agg['peak']:.4f} "
                f"| {f(agg['bias_rms'])} | {f(agg['bias_t'],1)} "
                f"| {agg['accepted']:.1f}% | {tuning} |")
        print()

    lines.append(
        "\n## Caveats\n\n"
        "* Synthetic data throughout. The filters are validated against a "
        "simulator built from the Phase 2 measurements, not against "
        "independent truth.\n"
        "* The 1/f flicker component responsible for the bias-instability "
        "minimum is not synthesised. Its omission flatters any filter "
        "carrying an explicit bias state, so it flatters the Kalman filter "
        "more than the complementary filter.\n"
        "* The gate uses the in-plane specific-force magnitude only, since "
        "this is a single-axis testbench. A three-axis implementation would "
        "use the full norm.\n"
        "* Lever-arm compensation assumes r is known. The sensitivity of the "
        "result to an incorrect r is not swept here.\n")

    with open(args.md, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"markdown written to {args.md}")


if __name__ == "__main__":
    main()
