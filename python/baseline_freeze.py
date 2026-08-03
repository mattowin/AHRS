#!/usr/bin/env python3
"""
baseline_freeze.py - generate the frozen complementary-filter baseline that
the Phase 4 Kalman comparison is scored against.

Every number the Kalman filter is compared to must come from here, produced
by one script with fixed seeds, so the comparison table cannot drift as the
testbench evolves. Emits a markdown table for the design document.

Usage:
    python3 baseline_freeze.py
    python3 baseline_freeze.py --md baseline.md --seeds 1 2 3 4 5
"""

import argparse

import numpy as np

import filter_testbench as fb

# Scenarios: (label, profile, simulate_imu kwargs)
SCENARIOS = [
    ("clean",                 "gentle", {}),
    ("clean",                 "brisk",  {}),
    ("lever 30 mm",           "brisk",  dict(lever_arm_r=(0, 0, 0.03))),
    ("lever 30 mm + vib",     "brisk",  dict(lever_arm_r=(0, 0, 0.03),
                                             vib_rms=0.05)),
    ("lever + vib + taps",    "brisk",  dict(lever_arm_r=(0, 0, 0.03),
                                             vib_rms=0.05, tap_rate=0.3)),
    ("legacy dist 0.5",       "gentle", dict(legacy_disturbance=0.5)),
]


def sweep(gyro, angle_acc, truth, static, taus):
    A, S, D = [], [], []
    for tau in taus:
        err = fb.complementary(gyro, angle_acc, tau) - truth
        A.append(fb.rms(err))
        S.append(fb.rms(err[static]))
        D.append(fb.rms(err[~static]))
    return np.array(A), np.array(S), np.array(D)


def run_one(profile, kwargs, seed, taus, duration):
    rng = np.random.default_rng(seed)
    t, truth, rate, static = fb.PROFILES[profile](duration)
    gyro, ax, az, bias, dist = fb.simulate_imu(truth, rate, rng, **kwargs)
    angle_acc = fb.accel_angle(ax, az)
    A, S, D = sweep(gyro, angle_acc, truth, static, taus)

    i = int(np.argmin(A))
    err_best = fb.complementary(gyro, angle_acc, taus[i]) - truth
    dmag = np.linalg.norm(dist, axis=1)
    norm_dev = np.abs(np.hypot(ax, az) - fb.G) / fb.G
    tilt_dist = np.degrees(np.arctan2(dist[:, 0], fb.G))

    return dict(
        tau_opt=taus[i], railed=(i in (0, len(taus) - 1)),
        rms_all=A[i], rms_static=S[i], rms_dyn=D[i],
        peak=np.abs(err_best).max(),
        tau_static=taus[int(np.argmin(S))],
        tau_dyn=taus[int(np.argmin(D))],
        acc_only=fb.rms(angle_acc - truth),
        dist_rms=fb.rms(dmag),
        tilt_rms=fb.rms(tilt_dist),
        norm_rms=fb.rms(norm_dev) * 100.0,
        gate_ratio=(fb.rms(tilt_dist) / (fb.rms(norm_dev) * 100.0)
                    if dmag.max() > 0 else float("nan")),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3, 4, 5])
    ap.add_argument("--tau-min", type=float, default=0.005)
    ap.add_argument("--tau-max", type=float, default=100.0)
    ap.add_argument("--md", default="baseline.md")
    args = ap.parse_args()

    taus = np.logspace(np.log10(args.tau_min), np.log10(args.tau_max), 41)

    rows = []
    for label, profile, kw in SCENARIOS:
        runs = [run_one(profile, kw, s, taus, args.duration)
                for s in args.seeds]
        agg = {k: np.array([r[k] for r in runs], dtype=float)
               for k in runs[0] if k != "railed"}
        rows.append((label, profile, agg,
                     any(r["railed"] for r in runs)))

    hdr = (f"{'scenario':<20} {'profile':<7} {'tau_opt':>9} {'RMS all':>9} "
           f"{'static':>8} {'dynamic':>9} {'peak':>8} {'accel-only':>11}")
    print(hdr)
    print("-" * len(hdr))
    for label, profile, a, railed in rows:
        flag = " RAILED" if railed else ""
        print(f"{label:<20} {profile:<7} {a['tau_opt'].mean():9.4f} "
              f"{a['rms_all'].mean():9.4f} {a['rms_static'].mean():8.4f} "
              f"{a['rms_dyn'].mean():9.4f} {a['peak'].mean():8.4f} "
              f"{a['acc_only'].mean():11.4f}{flag}")

    print(f"\nMean over seeds {args.seeds}. Spread (max-min) on RMS all:")
    for label, profile, a, _ in rows:
        r = a['rms_all']
        print(f"  {label:<20} {profile:<7} {r.max()-r.min():.4f} deg "
              f"({(r.max()-r.min())/r.mean()*100:.1f}% of mean)")

    print("\nDisturbance geometry (0.573 deg/% = isotropic; higher = gate-blind):")
    for label, profile, a, _ in rows:
        if np.isnan(a['gate_ratio']).all():
            continue
        print(f"  {label:<20} {profile:<7} tilt {a['tilt_rms'].mean():7.4f} deg  "
              f"norm {a['norm_rms'].mean():7.4f}%  "
              f"ratio {a['gate_ratio'].mean():6.2f}")

    with open(args.md, "w") as f:
        f.write("# Phase 3 baseline - complementary filter\n\n")
        f.write(f"Seeds {args.seeds}, {args.duration:.0f} s, "
                f"{fb.FS:.4f} Hz, dt = {fb.DT:.7f} s. "
                f"Tau swept over [{args.tau_min}, {args.tau_max}] s, "
                f"41 points. Values are means over seeds.\n\n")
        f.write("| scenario | profile | tau_opt (s) | RMS all | RMS static | "
                "RMS dynamic | peak | accel-only |\n")
        f.write("|---|---|---|---|---|---|---|---|\n")
        for label, profile, a, railed in rows:
            f.write(f"| {label} | {profile} | {a['tau_opt'].mean():.4f}"
                    f"{' **RAILED**' if railed else ''} "
                    f"| {a['rms_all'].mean():.4f} | {a['rms_static'].mean():.4f} "
                    f"| {a['rms_dyn'].mean():.4f} | {a['peak'].mean():.4f} "
                    f"| {a['acc_only'].mean():.4f} |\n")
        f.write("\nAll errors in degrees RMS unless stated.\n\n")
        f.write("Caveats: synthetic data only, validated against a simulator "
                "built from the Phase 2 measurements rather than against "
                "independent truth. The 1/f flicker component is not "
                "synthesised, which flatters any filter carrying an explicit "
                "bias state more than it flatters the complementary filter.\n")
    print(f"\nmarkdown written to {args.md}")


if __name__ == "__main__":
    main()
