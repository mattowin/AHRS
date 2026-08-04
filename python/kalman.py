#!/usr/bin/env python3
"""
kalman.py - 2-state Kalman filter for the AHRS, state [angle, gyro_bias].

Everything internal is SI radians. Degrees appear only at the scoring and
plotting boundary. The firmware works in radians, so keeping the Python in
radians means the port is a transcription rather than a re-derivation, and a
scale error cannot hide in the unit conversion.

The propagation is written as scalar operations rather than 2x2 matrix
algebra. A general matrix implementation of a filter this structured wastes
most of its cycles on multiplying by known zeros and ones; the expanded form
is ~15 scalar operations and ports directly to the Cortex-M4.

Model
-----
    w_meas = w_true + b + n_g          n_g PSD N^2   (ARW)
    theta_dot = w_meas - b - n_g
    b_dot     = n_b                    n_b PSD K^2   (RRW)

    F = [[1, -dt], [0, 1]]   B = [dt, 0]^T   H = [1, 0]

A is nilpotent (A^2 = 0), so F = I + A*dt is exact, not a truncation.

Discrete process noise, exact by Van Loan:

    Q11 = N^2*dt + K^2*dt^3/3
    Q12 = Q21   = -K^2*dt^2/2
    Q22 = K^2*dt

The off-diagonal and the K^2*dt^3/3 correction are 8 and 4 orders of
magnitude below the leading terms respectively for this sensor, so the
diagonal form is used. USE_EXACT_Q below switches to the full form; it is
kept so the approximation can be shown to be harmless rather than asserted.

Run this file directly to execute the validation suite.
"""

import numpy as np

# ---------------------------------------------------------------------------
# Phase 2 measured parameters
# ---------------------------------------------------------------------------
DT = 0.0098792              # s, measured sample interval (101.2229 Hz)

# Gyro angle random walk N [rad/sqrt(s)] and rate random walk K [rad/s/sqrt(s)]
N_GX, K_GX = 9.2648e-5, 2.101e-6      # roll  channel
N_GY, K_GY = 8.0700e-5, 1.621e-6      # pitch channel

# Accelerometer per-sample sigma [m/s^2]
SIG_AX, SIG_AY = 0.00669, 0.00600

# Gravity. G_MEAS is the magnitude actually measured in Phase 2; using it for
# R makes the 0.86% accelerometer scale-factor error cancel, since the tilt
# estimate normalises by the same quantity. G_STD is what filter_testbench.py
# synthesises with -- R differs by 1.7% between them, which is immaterial.
G_MEAS = 9.8871
G_STD = 9.80665

USE_EXACT_Q = False


def make_QR(N, K, sigma_acc, dt=DT, g=G_MEAS, exact=USE_EXACT_Q):
    """Discrete process and measurement noise from measured noise densities.

    Returns (Q11, Q12, Q22, R). Units: Q11 rad^2, Q12 rad^2/s, Q22 rad^2/s^2,
    R rad^2.
    """
    if exact:
        Q11 = N * N * dt + K * K * dt ** 3 / 3.0
        Q12 = -K * K * dt * dt / 2.0
    else:
        Q11 = N * N * dt
        Q12 = 0.0
    Q22 = K * K * dt
    R = (sigma_acc / g) ** 2
    return Q11, Q12, Q22, R


QR_ROLL = make_QR(N_GX, K_GX, SIG_AY)
QR_PITCH = make_QR(N_GY, K_GY, SIG_AX)


def wrap(a):
    """Wrap an angle to [-pi, pi). Free, and the roll channel needs it."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


# ---------------------------------------------------------------------------
# The filter
# ---------------------------------------------------------------------------
class KF2:
    """2-state Kalman filter, scalar form.

    Parameters
    ----------
    Q11, Q12, Q22, R : discrete noise terms from make_QR
    theta0           : initial angle [rad], normally the first accel reading
    b0               : initial bias [rad/s]
    P00_0            : initial angle variance; R if theta0 came from the accel
    P11_0            : initial bias variance. With an M-sample stationary boot
                       average, sigma_b0 = N/sqrt(M*dt): 2 s gives 4.3e-9.
                       Without any boot calibration use the observed bias
                       magnitude squared, ~7.6e-5.
    joseph           : use the Joseph form for the covariance update. Slower
                       but preserves symmetry and positive-definiteness under
                       float32, which matters on the M4 and not here.
    """

    def __init__(self, Q11, Q12, Q22, R, theta0=0.0, b0=0.0,
                 P00_0=None, P11_0=7.6e-5, joseph=False, dt=DT):
        self.Q11, self.Q12, self.Q22, self.R = Q11, Q12, Q22, R
        self.dt = dt
        self.theta = float(theta0)
        self.b = float(b0)
        self.P00 = float(R if P00_0 is None else P00_0)
        self.P01 = 0.0
        self.P10 = 0.0
        self.P11 = float(P11_0)
        self.joseph = joseph
        self.K0 = self.K1 = 0.0

    # -- prediction ---------------------------------------------------------
    def predict(self, omega):
        """Propagate one step with gyro measurement omega [rad/s].

        P00 needs the old P01, P10 and P11; P01 and P10 need the old P11; P11
        is updated last. Written this way it is correct in place.

        Caveat, measured rather than assumed: at THIS sensor's Q22 the
        mis-ordering is numerically invisible (see TEST 2 -- the error it
        introduces is dt*Q22 against Q11, a ratio of 4e-6). Keep the order
        correct on principle, but do not go hunting here first if the filter
        misbehaves. The update step's prior-P00 caching is the trap that
        actually bites.
        """
        dt = self.dt
        self.theta += (omega - self.b) * dt

        P00, P01, P10, P11 = self.P00, self.P01, self.P10, self.P11
        self.P00 = P00 - dt * (P01 + P10) + dt * dt * P11 + self.Q11
        self.P01 = P01 - dt * P11 + self.Q12
        self.P10 = P10 - dt * P11 + self.Q12
        self.P11 = P11 + self.Q22

    # -- measurement update -------------------------------------------------
    def update(self, z, R=None):
        """Correct with accelerometer-derived angle z [rad]."""
        R = self.R if R is None else R
        y = wrap(z - self.theta)
        S = self.P00 + R
        K0 = self.P00 / S
        K1 = self.P10 / S
        self.K0, self.K1 = K0, K1

        self.theta = wrap(self.theta + K0 * y)
        self.b += K1 * y

        P00, P01, P10, P11 = self.P00, self.P01, self.P10, self.P11
        if self.joseph:
            a = (1.0 - K0) * P00
            bb = (1.0 - K0) * P01
            c = P10 - K1 * P00
            d = P11 - K1 * P01
            self.P00 = a * (1.0 - K0) + R * K0 * K0
            self.P01 = -a * K1 + bb + R * K0 * K1
            self.P10 = c * (1.0 - K0) + R * K0 * K1
            self.P11 = -c * K1 + d + R * K1 * K1
        else:
            # P10 and P11 need the PRIOR P00 and P01, so cache before writing.
            self.P00 = (1.0 - K0) * P00
            self.P01 = (1.0 - K0) * P01
            self.P10 = P10 - K1 * P00
            self.P11 = P11 - K1 * P01

    def symmetrize(self):
        """P01 and P10 are analytically equal but diverge in float32."""
        m = 0.5 * (self.P01 + self.P10)
        self.P01 = self.P10 = m

    @property
    def asymmetry(self):
        return abs(self.P01 - self.P10)


# ---------------------------------------------------------------------------
# Batch driver, with optional accelerometer gating
# ---------------------------------------------------------------------------
def run(omega, z, qr, theta0=None, b0=0.0, P11_0=7.6e-5, joseph=False,
        dt=DT, accel_norm=None, g=G_STD, gate_thresh=None, dwell_s=0.0,
        adaptive_k=None, symmetrize_every=0):
    """Run the filter over a record.

    omega       : (n,) gyro [rad/s]
    z           : (n,) accel-derived angle [rad]
    qr          : (Q11, Q12, Q22, R)
    accel_norm  : (n,) |a| [m/s^2], required for gating
    gate_thresh : fractional norm deviation above which the update is skipped
    dwell_s     : hold-off after a gate violation, seconds
    adaptive_k  : if set, inflate R by (k*resid/g)^2 instead of hard gating

    Returns dict with theta, bias, P00, P11, K0, K1, accepted.
    """
    n = len(omega)
    Q11, Q12, Q22, R = qr
    kf = KF2(Q11, Q12, Q22, R,
             theta0=z[0] if theta0 is None else theta0,
             b0=b0, P11_0=P11_0, joseph=joseph, dt=dt)

    theta = np.empty(n)
    bias = np.empty(n)
    P00 = np.empty(n)
    P11 = np.empty(n)
    K0 = np.empty(n)
    K1 = np.empty(n)
    accepted = np.zeros(n, dtype=bool)
    asym = 0.0

    dwell_n = int(round(dwell_s / dt))
    blocked_until = -1

    for i in range(n):
        kf.predict(omega[i])

        do_update = True
        Ri = None
        if gate_thresh is not None and accel_norm is not None:
            resid = abs(accel_norm[i] - g) / g
            if resid > gate_thresh:
                blocked_until = i + dwell_n
            if i <= blocked_until:
                do_update = False
        elif adaptive_k is not None and accel_norm is not None:
            resid = abs(accel_norm[i] - g) / g
            Ri = R + (adaptive_k * resid) ** 2

        if do_update:
            kf.update(z[i], Ri)
            accepted[i] = True

        if symmetrize_every and (i % symmetrize_every == 0):
            kf.symmetrize()

        asym = max(asym, kf.asymmetry)
        theta[i] = kf.theta
        bias[i] = kf.b
        P00[i] = kf.P00
        P11[i] = kf.P11
        K0[i] = kf.K0
        K1[i] = kf.K1

    return dict(theta=theta, bias=bias, P00=P00, P11=P11, K0=K0, K1=K1,
                accepted=accepted, max_asymmetry=asym)


# ---------------------------------------------------------------------------
# Steady state
# ---------------------------------------------------------------------------
def steady_state_iterate(qr, dt=DT, iters=2_000_000, tol=1e-24):
    """Converge the covariance by running predict/update with no data.

    This exercises the ACTUAL implementation, so it catches ordering and
    caching bugs that a separate closed-form solution would not.
    """
    Q11, Q12, Q22, R = qr
    kf = KF2(Q11, Q12, Q22, R, P00_0=R, P11_0=7.6e-5, dt=dt)
    prev = None
    for i in range(iters):
        kf.predict(0.0)
        kf.update(0.0)
        cur = (kf.P00, kf.P01, kf.P10, kf.P11)
        if prev is not None:
            if max(abs(a - b) for a, b in zip(cur, prev)) < tol * max(
                    1e-30, abs(cur[0])):
                break
        prev = cur
    return dict(P00=kf.P00, P01=kf.P01, P10=kf.P10, P11=kf.P11,
                K0=kf.K0, K1=kf.K1, iters=i + 1)


def steady_state_dare(qr, dt=DT):
    """Closed-form steady state via the filter algebraic Riccati equation.

    scipy's solve_discrete_are solves the CONTROL Riccati equation, so the
    filter form is obtained by transposing: solve_discrete_are(F.T, H.T,Q,R)
    returns the a priori (predicted) covariance.
    """
    try:
        from scipy.linalg import solve_discrete_are
    except ImportError:
        return None
    Q11, Q12, Q22, R = qr
    F = np.array([[1.0, -dt], [0.0, 1.0]])
    H = np.array([[1.0, 0.0]])
    Q = np.array([[Q11, Q12], [Q12, Q22]])
    P_pri = solve_discrete_are(F.T, H.T, Q, np.array([[R]]))
    S = P_pri[0, 0] + R
    K = P_pri @ H.T / S
    KH = K @ H
    P_post = (np.eye(2) - KH) @ P_pri
    return dict(P_pri=P_pri, P_post=P_post, K0=K[0, 0], K1=K[1, 0])


def equivalent_tau(K0, dt=DT):
    """The steady-state KF is a complementary filter. Report its tau.

    With constant gains the angle recursion is
        theta_k = (1-K0)*(theta_{k-1} + w*dt) + K0*z_k
    which is exactly the complementary filter with alpha = 1-K0, hence
        tau = alpha*dt/(1-alpha) = (1-K0)*dt/K0
    The bias state adds the integral term the complementary filter lacks.
    """
    return (1.0 - K0) * dt / K0


# ---------------------------------------------------------------------------
# Validation suite
# ---------------------------------------------------------------------------
def _validate():
    D = np.degrees
    ok = True

    print("=" * 74)
    print("DERIVED Q AND R")
    print("=" * 74)
    for name, (N, K, sig) in (("roll  (gx, ay)", (N_GX, K_GX, SIG_AY)),
                              ("pitch (gy, ax)", (N_GY, K_GY, SIG_AX))):
        Q11, Q12, Q22, R = make_QR(N, K, sig)
        e11, e12, e22, _ = make_QR(N, K, sig, exact=True)
        print(f"  {name}")
        print(f"    N = {N:.4e} rad/sqrt(s)   K = {K:.4e} rad/s/sqrt(s)")
        print(f"    Q11 = {Q11:.4e} rad^2      (exact {e11:.4e}, "
              f"delta {abs(e11-Q11)/Q11:.2e})")
        print(f"    Q22 = {Q22:.4e} rad^2/s^2")
        print(f"    Q12 = {e12:.4e}  -> correlation "
              f"{abs(e12)/np.sqrt(e11*e22):.2e}, dropped")
        print(f"    R   = {R:.4e} rad^2       (sigma = {D(np.sqrt(R)):.4f} deg)")

    print()
    print("=" * 74)
    print("TEST 1  steady state: implementation vs closed-form DARE")
    print("=" * 74)
    for name, qr in (("roll", QR_ROLL), ("pitch", QR_PITCH)):
        it = steady_state_iterate(qr)
        da = steady_state_dare(qr)
        print(f"  {name}: converged in {it['iters']} steps")
        print(f"    iterated  K0 = {it['K0']:.6e}   K1 = {it['K1']:.6e}")
        if da is None:
            print("    scipy unavailable - closed-form check skipped")
        else:
            d0 = abs(it['K0'] - da['K0']) / da['K0']
            d1 = abs(it['K1'] - da['K1']) / abs(da['K1'])
            print(f"    DARE      K0 = {da['K0']:.6e}   K1 = {da['K1']:.6e}")
            print(f"    relative difference: K0 {d0:.2e}, K1 {d1:.2e}")
            if max(d0, d1) > 1e-6:
                print("    *** MISMATCH")
                ok = False
        tau = equivalent_tau(it['K0'])
        print(f"    equivalent complementary tau = {tau:.4f} s "
              f"(alpha = {1-it['K0']:.6f})")

    print()
    print("=" * 74)
    print("TEST 2  ordering sensitivity of the predict step")
    print("=" * 74)
    Q11, Q12, Q22, R = QR_PITCH

    def wrong_class(q22):
        class Wrong(KF2):
            def predict(self, omega):
                dt = self.dt
                self.theta += (omega - self.b) * dt
                self.P11 = self.P11 + self.Q22      # WRONG: moved to top
                P00, P01, P10, P11 = self.P00, self.P01, self.P10, self.P11
                self.P00 = P00 - dt * (P01 + P10) + dt * dt * P11 + self.Q11
                self.P01 = P01 - dt * P11 + self.Q12
                self.P10 = P10 - dt * P11 + self.Q12
        return Wrong

    good = steady_state_iterate(QR_PITCH)
    print(f"  {'Q22 scale':>10} {'K0 rel diff':>14} {'K1 rel diff':>14}")
    for scale in (1.0, 1e3, 1e6, 1e9):
        q22 = Q22 * scale
        kfw = wrong_class(q22)(Q11, Q12, q22, R, P00_0=R, P11_0=7.6e-5)
        for _ in range(300000):
            kfw.predict(0.0)
            kfw.update(0.0)
        ref = steady_state_iterate((Q11, Q12, q22, R))
        d0 = abs(kfw.K0 - ref["K0"]) / ref["K0"]
        d1 = abs(kfw.K1 - ref["K1"]) / abs(ref["K1"])
        print(f"  {scale:>10.0e} {d0:>14.2e} {d1:>14.2e}")
    print("  At the measured Q22 the mis-ordering is invisible: the error it")
    print("  introduces is dt*Q22 against Q11, a ratio of "
          f"{DT*Q22/Q11:.1e}. Order the lines")
    print("  correctly on principle, but this is NOT the bug to hunt for if")
    print("  the filter misbehaves. The DARE check is the real safety net.")

    print()
    print("=" * 74)
    print("TEST 3  bias convergence from a cold start")
    print("=" * 74)
    rng = np.random.default_rng(0)
    n = int(300.0 / DT)
    true_b = np.deg2rad(-0.4295)
    omega = true_b + rng.normal(0.0, 8.118e-4, n)
    z = rng.normal(0.0, np.sqrt(QR_PITCH[3]), n)
    out = run(omega, z, QR_PITCH, theta0=0.0, b0=0.0)
    berr = D(out["bias"] - true_b)
    ss = steady_state_iterate(QR_PITCH)
    tau_b = DT / ss["K1"] * ss["K0"] if ss["K1"] else float("nan")
    print(f"    true bias        {D(true_b):+.4f} deg/s")
    print(f"    final estimate   {D(out['bias'][-1]):+.4f} deg/s  "
          f"(error {berr[-1]:+.5f})")
    within = np.abs(berr) < 0.01
    t_conv = np.argmax(within) * DT if within.any() else float("nan")
    print(f"    time to |err| < 0.01 deg/s: {t_conv:.1f} s")
    print(f"    steady-state sqrt(P11) = {D(np.sqrt(out['P11'][-1])):.5f} deg/s, "
          f"actual |error| = {abs(berr[-1]):.5f} deg/s")
    print(f"    final angle error {D(out['theta'][-1]):+.5f} deg")
    if abs(berr[-1]) > 0.05:
        print("    *** bias did not converge")
        ok = False

    print()
    print("=" * 74)
    print("TEST 4  covariance symmetry and consistency")
    print("=" * 74)
    print(f"    max |P01 - P10| over run (float64): {out['max_asymmetry']:.3e}")
    outj = run(omega, z, QR_PITCH, theta0=0.0, b0=0.0, joseph=True)
    print(f"    Joseph form, same:                  "
          f"{outj['max_asymmetry']:.3e}")
    print(f"    angle difference simple vs Joseph:  "
          f"{np.abs(out['theta']-outj['theta']).max():.3e} rad")
    print("    -> float64 keeps them identical; the Joseph form is insurance")
    print("       for float32 on the M4, not a correctness fix here.")

    # NEES-style consistency: is the reported P honest?
    nsteady = out["theta"][n // 2:]
    print(f"    steady-state sqrt(P00) = {D(np.sqrt(out['P00'][-1])):.5f} deg")
    print(f"    actual angle error RMS = {D(np.sqrt(np.mean(nsteady**2))):.5f} deg")
    print("    -> these should be the same order; a large gap means Q or R is"
          " misstated.")

    print()
    print("=" * 74)
    print("RESULT:", "ALL CHECKS PASSED" if ok else "FAILURES PRESENT")
    print("=" * 74)
    return ok


if __name__ == "__main__":
    raise SystemExit(0 if _validate() else 1)
