#!/usr/bin/env python3
"""
filter_testbench.py - develop and tune attitude filters offline, against
synthetic data with known ground truth.

The sensor model uses the noise parameters measured from the 8 h Allan
deviation capture, so the synthetic gyro and accelerometer behave like the
real ISM330DHCX rather than like idealised sensors.

Pitch axis only (gyro Y, accel X/Z). Roll is structurally identical with
gyro X and accel Y/Z.

v2 changes
----------
* simulate_imu() now returns the specific-force components (ax, az) rather
  than only the derived angle, so disturbances can be injected in the
  measurement domain where they physically act.
* Physically-modelled disturbances from accel_disturbance.py: lever-arm
  coupling (motion-correlated), band-limited vibration, impulsive taps.
* The v1 disturbance model is retained as --legacy-disturbance so earlier
  results remain reproducible.
* Motion profile registry: --profile gentle|brisk. 'gentle' is v1's
  make_truth unchanged and regenerates earlier baselines bit-identically.
* Tau sweep floor lowered 0.05 -> 0.005 s. The clean-case optimum is
  0.026 s, so every earlier "best tau = 0.050" was the sweep reporting its
  own lower bound. Boundary minima are now flagged explicitly.

Usage:
    python3 filter_testbench.py
    python3 filter_testbench.py --profile brisk --lever-arm 0.03
    python3 filter_testbench.py --profile brisk --lever-arm 0.03 --vib 0.05 --taps 0.3
    python3 filter_testbench.py --legacy-disturbance 0.5
"""

import argparse

import numpy as np

from accel_disturbance import lever_arm, band_limited_vibration, transient_taps

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

# ---------------------------------------------------------------------------
# Measured in Phase 2 (8 h stationary capture, 101.2229 Hz, 16.4-18.6 C)
# ---------------------------------------------------------------------------
FS = 101.2229
DT = 1.0 / FS

GYRO_WN_DPS       = 0.04651    # gy per-sample white noise sigma, deg/s
GYRO_RRW_DPH_RTHR = 20.06      # gy rate random walk, deg/hr/sqrt(hr)
GYRO_BIAS0_DPS    = -0.4295    # gy measured static bias, deg/s

ACC_WN_X_MS2      = 0.00669    # ax per-sample sigma, m/s^2
ACC_WN_Z_MS2      = 0.00752    # az per-sample sigma, m/s^2

G = 9.80665

# NOTE: this model includes white noise and rate random walk, but not the
# 1/f (flicker) component responsible for the bias-instability minimum on the
# Allan curve. Flicker is awkward to synthesise and its omission makes the
# simulation mildly optimistic at intermediate timescales. Worth stating in
# the write-up rather than glossing over.
#
# v2 addendum: this omission is NOT symmetric between the two filters. The
# Kalman bias state exists precisely to track slow bias wander, so leaving
# out the component that dominates at tau = 100-250 s flatters the Kalman
# filter more than the complementary filter. Do not quote the Phase 4
# improvement factor without this caveat attached.
#
# NOTE ON G: G cancels identically inside atan2(-ax, az), so this testbench
# cannot exercise accelerometer scale-factor error. The measured gravity
# magnitude is 9.8871 m/s^2 against a local g of ~9.8030, a 0.86% scale
# error that is real but structurally invisible here. It belongs in the
# error budget, not in this simulation.


def make_truth(duration_s):
    """Rate profile defined analytically, angle obtained by integrating it.

    Defining rate first and integrating (rather than differentiating an angle)
    keeps truth exactly consistent with how the filter propagates state.

    Layout: static -> manoeuvring -> static, so steady-state and dynamic
    behaviour can be scored separately.
    """
    t = np.arange(0.0, duration_s, DT)

    # smooth on/off envelope for the manoeuvring segment
    env = (0.5 * (1 + np.tanh((t - 60.0) / 4.0))
           * 0.5 * (1 + np.tanh((240.0 - t) / 4.0)))

    rate = env * (10.0 * np.sin(2 * np.pi * 0.05 * t)
                  + 4.0 * np.sin(2 * np.pi * 0.17 * t + 1.1))   # deg/s
    angle = np.cumsum(rate) * DT                                # deg

    static = (t < 55.0) | (t > 245.0)
    return t, angle, rate, static


def make_truth_brisk(duration_s):
    """As make_truth, plus a 25 deg/s 0.8 Hz term in the manoeuvring segment.

    Rationale: the 'gentle' profile peaks at 14 deg/s and 0.129 rad/s^2, about
    27x gentler in angular acceleration than a brisk hand rotation. Lever-arm
    coupling scales as alpha*r_z, so at that acceleration a 30 mm offset
    produces 0.009 deg RMS tilt error -- an order of magnitude BELOW the
    accelerometer noise floor. The gentle profile therefore cannot exercise
    the dominant dynamic error mechanism, and a filter comparison run on it
    is a comparison under conditions neither filter finds difficult.

    This profile reaches 39 deg/s and 2.32 rad/s^2, which puts lever-arm
    tilt error at 0.208 deg RMS / 0.407 deg peak for r_z = 30 mm.

    make_truth is left untouched so existing baselines regenerate exactly.
    """
    t = np.arange(0.0, duration_s, DT)

    env = (0.5 * (1 + np.tanh((t - 60.0) / 4.0))
           * 0.5 * (1 + np.tanh((240.0 - t) / 4.0)))

    rate = env * (10.0 * np.sin(2 * np.pi * 0.05 * t)
                  + 4.0 * np.sin(2 * np.pi * 0.17 * t + 1.1)
                  + 25.0 * np.sin(2 * np.pi * 0.80 * t + 0.4))   # deg/s
    angle = np.cumsum(rate) * DT                                 # deg

    static = (t < 55.0) | (t > 245.0)
    return t, angle, rate, static


# Score Phase 4 on both and report both columns: "here is the filter under
# gentle motion, here it is under motion that excites the disturbance
# mechanisms" is a stronger result than either alone.
PROFILES = {
    "gentle": make_truth,
    "brisk": make_truth_brisk,
}


def simulate_imu(angle_deg, rate_dps, rng, legacy_disturbance=0.0,
                 lever_arm_r=(0.0, 0.0, 0.0), vib_rms=0.0, tap_rate=0.0,
                 tap_peak=3.0):
    """Synthesise gyro and 3-axis specific force with realistic error sources.

    Returns
    -------
    gyro      : (n,) deg/s
    ax, az    : (n,) m/s^2, specific force including all disturbances
    bias      : (n,) deg/s, true gyro bias (for scoring the Kalman bias state)
    dist      : (n,3) m/s^2, the injected disturbance alone, for diagnostics
    """
    n = len(angle_deg)

    # gyro: constant offset + random-walk bias + white noise
    k = GYRO_RRW_DPH_RTHR / 3600.0 / 60.0          # deg/s per sqrt(s)
    bias = GYRO_BIAS0_DPS + np.cumsum(rng.normal(0.0, k * np.sqrt(DT), n))
    gyro = rate_dps + bias + rng.normal(0.0, GYRO_WN_DPS, n)

    # accel: gravity projected onto body axes (clean)
    th = np.deg2rad(angle_deg)
    ax = -G * np.sin(th)
    az = G * np.cos(th)

    # ---- physically-modelled disturbances -------------------------------
    # Pitch axis: rotation about body y, so omega = [0, w, 0].
    # Lever arm r = (r_x, 0, r_z) gives, on the pitch-sensing x axis,
    #     alpha*r_z - w^2*r_x
    # The tangential term is perpendicular to gravity: it tips the specific
    # force vector to first order while changing its magnitude only to
    # second order. That is the term a norm-based gate cannot see.
    dist = np.zeros((n, 3))

    omega = np.zeros((n, 3))
    omega[:, 1] = np.deg2rad(rate_dps)
    alpha = np.zeros((n, 3))
    alpha[:, 1] = np.gradient(omega[:, 1], DT)
    if np.any(np.asarray(lever_arm_r) != 0.0):
        dist += lever_arm(omega, alpha, lever_arm_r)

    if vib_rms > 0.0:
        dist += band_limited_vibration(n, DT, vib_rms, 5.0, 45.0, rng=rng)

    if tap_rate > 0.0:
        dist += transient_taps(n, DT, tap_rate, tap_peak, rng=rng)

    # ---- v1 disturbance model, retained for reproducibility -------------
    # 2 Hz lowpassed noise applied independently per axis. Not motion-
    # correlated, so it does not reproduce the lever-arm mechanism; kept
    # only so v1 numbers can be regenerated.
    if legacy_disturbance > 0.0:
        a = np.exp(-2 * np.pi * 2.0 * DT)
        for axis in (0, 2):
            w = rng.normal(0.0, 1.0, n)
            f = np.empty(n)
            f[0] = w[0]
            for i in range(1, n):
                f[i] = a * f[i - 1] + (1 - a) * w[i]
            f *= legacy_disturbance / (f.std() + 1e-12)
            dist[:, axis] += f

    ax = ax + dist[:, 0] + rng.normal(0.0, ACC_WN_X_MS2, n)
    az = az + dist[:, 2] + rng.normal(0.0, ACC_WN_Z_MS2, n)

    return gyro, ax, az, bias, dist


def accel_angle(ax, az):
    """Pitch from specific force, as the filter's measurement update does."""
    return np.rad2deg(np.arctan2(-ax, az))


def complementary(gyro_dps, angle_acc_deg, tau_s):
    """theta = a*(theta + w*dt) + (1-a)*theta_acc, with a set by time constant."""
    a = tau_s / (tau_s + DT)
    th = np.empty_like(gyro_dps)
    th[0] = angle_acc_deg[0]
    for i in range(1, len(gyro_dps)):
        th[i] = a * (th[i - 1] + gyro_dps[i] * DT) + (1 - a) * angle_acc_deg[i]
    return th


def rms(x):
    return float(np.sqrt(np.mean(x * x)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--profile", choices=sorted(PROFILES), default="gentle",
                    help="motion profile; 'gentle' reproduces earlier "
                         "baselines exactly, 'brisk' exercises lever-arm "
                         "coupling (default: gentle)")
    ap.add_argument("--lever-arm", type=float, default=0.0,
                    help="sensor offset r_z from rotation axis, metres "
                         "(try 0.03)")
    ap.add_argument("--lever-arm-x", type=float, default=0.0,
                    help="sensor offset r_x, metres (centripetal term)")
    ap.add_argument("--vib", type=float, default=0.0,
                    help="band-limited vibration RMS per axis, m/s^2")
    ap.add_argument("--taps", type=float, default=0.0,
                    help="impulsive tap rate, events/s (try 0.3)")
    ap.add_argument("--legacy-disturbance", type=float, default=0.0,
                    help="v1 disturbance model RMS in m/s^2 (try 0.5)")
    ap.add_argument("--tau-min", type=float, default=0.005,
                    help="lower bound of the tau sweep, s (default 0.005)")
    ap.add_argument("--tau-max", type=float, default=100.0,
                    help="upper bound of the tau sweep, s (default 100)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--plot", default="filter_tuning.png")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    t, truth, rate, static = PROFILES[args.profile](args.duration)
    gyro, ax, az, bias, dist = simulate_imu(
        truth, rate, rng,
        legacy_disturbance=args.legacy_disturbance,
        lever_arm_r=(args.lever_arm_x, 0.0, args.lever_arm),
        vib_rms=args.vib, tap_rate=args.taps)
    angle_acc = accel_angle(ax, az)

    # reference points: what each sensor gives you alone
    gyro_only = np.cumsum(gyro) * DT
    alpha_pk = np.abs(np.gradient(np.deg2rad(rate), DT)).max()
    print(f"samples          : {len(t)}  ({args.duration:.0f} s @ {FS:.2f} Hz)")
    print(f"profile          : {args.profile}  "
          f"(peak {np.abs(rate).max():.1f} deg/s, {alpha_pk:.3f} rad/s^2)")
    print(f"lever arm        : r_z={args.lever_arm:.3f} m  r_x={args.lever_arm_x:.3f} m")
    print(f"vibration        : {args.vib:.3f} m/s^2 RMS/axis")
    print(f"taps             : {args.taps:.2f} /s")
    print(f"legacy dist.     : {args.legacy_disturbance:.2f} m/s^2 RMS")
    print(f"gyro-only drift  : {gyro_only[-1] - truth[-1]:+.2f} deg after "
          f"{args.duration:.0f} s")
    print(f"accel-only RMS   : {rms(angle_acc - truth):.4f} deg  "
          f"(static {rms((angle_acc - truth)[static]):.4f}, "
          f"dynamic {rms((angle_acc - truth)[~static]):.4f})")

    dmag = np.linalg.norm(dist, axis=1)
    if dmag.max() > 0:
        print(f"disturbance      : RMS {rms(dmag):.4f}, peak {dmag.max():.4f} m/s^2")
        acc_norm = np.hypot(ax, az)
        nd = np.abs(acc_norm - G) / G
        print(f"norm deviation   : RMS {rms(nd)*100:.3f}%, peak {nd.max()*100:.3f}% "
              f"-- what a norm gate actually sees")
    print()

    # Floor is 0.005 s, not 0.05 s. The clean-case optimum is 0.026 s, so the
    # original range railed against its own lower bound and reported that
    # bound as "best tau". Rail detection below guards against a recurrence.
    taus = np.logspace(np.log10(args.tau_min), np.log10(args.tau_max), 41)
    all_rms, stat_rms, dyn_rms = [], [], []
    for tau in taus:
        est = complementary(gyro, angle_acc, tau)
        err = est - truth
        all_rms.append(rms(err))
        stat_rms.append(rms(err[static]))
        dyn_rms.append(rms(err[~static]))
    all_rms = np.array(all_rms)
    stat_rms = np.array(stat_rms)
    dyn_rms = np.array(dyn_rms)

    print(f"{'tau (s)':>9} {'alpha':>9} {'RMS all':>10} {'RMS static':>11} "
          f"{'RMS dynamic':>12}")
    for i, tau in enumerate(taus):
        mark = "  <-- best" if i == int(np.argmin(all_rms)) else ""
        print(f"{tau:9.4f} {tau/(tau+DT):9.6f} {all_rms[i]:10.4f} "
              f"{stat_rms[i]:11.4f} {dyn_rms[i]:12.4f}{mark}")

    print()
    railed = False
    for label, arr in (("all", all_rms), ("static", stat_rms),
                       ("dynamic", dyn_rms)):
        i = int(np.argmin(arr))
        edge = ""
        if i == 0:
            edge = "  [RAILED at tau_min - true optimum is lower]"
            railed = True
        elif i == len(taus) - 1:
            edge = "  [RAILED at tau_max - true optimum is higher]"
            railed = True
        print(f"best tau ({label:<7}) = {taus[i]:8.4f} s  "
              f"(alpha = {taus[i]/(taus[i]+DT):.6f}, "
              f"RMS = {arr[i]:.4f} deg){edge}")
    if railed:
        print("\n*** A minimum at a sweep boundary is not an optimum. Widen the "
              "range with --tau-min / --tau-max before quoting these numbers.")
    best = taus[int(np.argmin(all_rms))]

    if plt is None:
        print("matplotlib not installed - skipping plot")
        return

    est = complementary(gyro, angle_acc, best)
    fig, axes = plt.subplots(3, 1, figsize=(11, 11))

    axes[0].semilogx(taus, all_rms, "o-", label="all")
    axes[0].semilogx(taus, stat_rms, "s--", lw=0.8, alpha=0.7, label="static")
    axes[0].semilogx(taus, dyn_rms, "^--", lw=0.8, alpha=0.7, label="dynamic")
    axes[0].axvline(best, color="k", ls=":", lw=0.8)
    axes[0].set_xlabel(r"filter time constant $\tau$ (s)")
    axes[0].set_ylabel("RMS pitch error (deg)")
    axes[0].set_title(f"Tuning sweep - {args.profile} profile")
    axes[0].grid(True, which="both", alpha=0.3)
    axes[0].legend()

    axes[1].plot(t, angle_acc, lw=0.4, alpha=0.35, label="accel only")
    axes[1].plot(t, truth, "k", lw=1.4, label="truth")
    axes[1].plot(t, est, lw=1.0, label=rf"complementary, $\tau$={best:.2f} s")
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("pitch (deg)")
    axes[1].set_title("Best-tuned filter against ground truth")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    axes[2].plot(t, np.rad2deg(np.arctan2(dist[:, 0], G)), lw=0.5,
                 label="tilt error from disturbance")
    axes[2].plot(t, (np.hypot(ax, az) - G) / G * 100.0, lw=0.5,
                 label="norm deviation (%)")
    axes[2].set_xlabel("time (s)")
    axes[2].set_title("Disturbance: what it does vs what a norm gate sees")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()

    fig.tight_layout()
    fig.savefig(args.plot, dpi=140)
    print(f"plot written to {args.plot}")


if __name__ == "__main__":
    main()
