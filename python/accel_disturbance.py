"""
accel_disturbance.py — linear-acceleration disturbance injection for the
2-axis AHRS filter testbench.

Purpose
-------
The accelerometer measures specific force and cannot distinguish gravity from
linear acceleration.  A testbench whose only accelerometer error is white noise
(sigma ~ 0.0067 m/s^2, i.e. 0.039 deg of tilt) will make any filter look good,
because it never exercises the failure mode that actually matters.

This module produces the *additional* body-frame specific force caused by
sensor motion, to be summed with the clean gravity projection before sensor
noise is added:

    f_body = gravity_body(roll, pitch) + disturbance + white_noise

Frame convention
----------------
Body frame, level board reads f = [0, 0, +g].  This matches the measured
Phase 2 means (ax=-0.3958, ay=-0.3473, az=+9.8730).

    f_x = -g sin(pitch)
    f_y = +g cos(pitch) sin(roll)
    f_z = +g cos(pitch) cos(roll)

so, small angle:   roll ~ f_y/g,   pitch ~ -f_x/g

VERIFY the sign of each axis against the ISM330DHCX silkscreen before
trusting the pitch sign; the magnitudes are convention-independent but the
signs are not.

Units: SI throughout.  angles rad, rates rad/s, accel m/s^2.
"""

from dataclasses import dataclass, field
import numpy as np

# Measured Phase 2 constants -------------------------------------------------
DT = 0.0098792              # s, measured sample interval (101.2229 Hz)
FS = 1.0 / DT               # Hz
G_MEAS = 9.8871             # m/s^2, measured gravity magnitude (see note)
G_LOCAL = 9.8030            # m/s^2, computed local g, Ann Arbor 42.28N 256m

# Per-sample accelerometer sigmas, Phase 2
SIGMA_AX = 0.00669          # m/s^2
SIGMA_AY = 0.00600
SIGMA_AZ = 0.00752


# --------------------------------------------------------------------------
# Clean gravity projection
# --------------------------------------------------------------------------
def gravity_body(roll, pitch, g=G_MEAS):
    """Body-frame specific force from gravity alone. Returns (N,3)."""
    roll = np.atleast_1d(roll)
    pitch = np.atleast_1d(pitch)
    return np.column_stack([
        -g * np.sin(pitch),
        g * np.cos(pitch) * np.sin(roll),
        g * np.cos(pitch) * np.cos(roll),
    ])


def tilt_from_accel(f, g=None):
    """Recover (roll, pitch) from specific force, as the filter's update does."""
    f = np.atleast_2d(f)
    roll = np.arctan2(f[:, 1], f[:, 2])
    pitch = np.arctan2(-f[:, 0], np.hypot(f[:, 1], f[:, 2]))
    return roll, pitch


# --------------------------------------------------------------------------
# 1. Lever-arm coupling  — a = alpha x r + omega x (omega x r)
# --------------------------------------------------------------------------
def lever_arm(omega, alpha, r):
    """
    Body-frame acceleration of a sensor displaced by r from the rotation axis.

    omega, alpha : (N,3) angular rate [rad/s] and angular acceleration [rad/s^2]
    r            : (3,)  sensor position in body frame [m]

    This is the disturbance that matters most, because it is *correlated with
    the motion itself*: it is largest exactly when the board is being rotated,
    which is exactly when the accelerometer update is most needed.
    """
    omega = np.atleast_2d(omega)
    alpha = np.atleast_2d(alpha)
    r = np.asarray(r, dtype=float)
    tangential = np.cross(alpha, r)                      # alpha x r
    centripetal = np.cross(omega, np.cross(omega, r))    # omega x (omega x r)
    return tangential + centripetal


def omega_alpha_from_angles(roll, pitch, dt=DT):
    """Numerical rate and angular acceleration from an angle trajectory."""
    n = len(roll)
    omega = np.zeros((n, 3))
    omega[:, 0] = np.gradient(roll, dt)
    omega[:, 1] = np.gradient(pitch, dt)
    alpha = np.zeros((n, 3))
    alpha[:, 0] = np.gradient(omega[:, 0], dt)
    alpha[:, 1] = np.gradient(omega[:, 1], dt)
    return omega, alpha


# --------------------------------------------------------------------------
# 2. Band-limited vibration
# --------------------------------------------------------------------------
def band_limited_vibration(n, dt=DT, rms=0.0, f_lo=5.0, f_hi=45.0,
                           axes=(1, 1, 1), rng=None):
    """
    Band-limited random vibration, per axis, scaled to `rms` m/s^2 each.

    Relevant to the reaction-wheel follow-on project (wheel imbalance) and to
    any airframe.  f_hi is capped below Nyquist (~50.6 Hz here); real vibration
    above Nyquist aliases down and this model does NOT capture that.
    """
    rng = np.random.default_rng() if rng is None else rng
    if rms <= 0:
        return np.zeros((n, 3))
    freqs = np.fft.rfftfreq(n, dt)
    band = (freqs >= f_lo) & (freqs <= min(f_hi, 0.98 * 0.5 / dt))
    out = np.zeros((n, 3))
    for k, on in enumerate(axes):
        if not on:
            continue
        spec = np.fft.rfft(rng.standard_normal(n))
        spec[~band] = 0.0
        sig = np.fft.irfft(spec, n=n)
        s = np.std(sig)
        out[:, k] = sig * (rms / s) if s > 0 else 0.0
    return out


# --------------------------------------------------------------------------
# 3. Impulsive transients (bench knocks, footsteps, cable tugs)
# --------------------------------------------------------------------------
def transient_taps(n, dt=DT, rate_hz=0.2, peak=2.0, decay_tau=0.04,
                   ring_hz=25.0, rng=None):
    """
    Poisson-distributed decaying-sinusoid impulses on random axes.
    Tests gate *recovery* rather than steady-state rejection.
    """
    rng = np.random.default_rng() if rng is None else rng
    out = np.zeros((n, 3))
    if peak <= 0 or rate_hz <= 0:
        return out
    n_events = rng.poisson(rate_hz * n * dt)
    length = int(6 * decay_tau / dt)
    t = np.arange(length) * dt
    shape = np.exp(-t / decay_tau) * np.sin(2 * np.pi * ring_hz * t)
    for _ in range(n_events):
        i0 = rng.integers(0, n)
        amp = peak * rng.uniform(0.4, 1.0)
        direction = rng.standard_normal(3)
        direction /= np.linalg.norm(direction)
        seg = min(length, n - i0)
        out[i0:i0 + seg, :] += amp * shape[:seg, None] * direction[None, :]
    return out


# --------------------------------------------------------------------------
# 4. Coordinated turn — the case norm-gating cannot see
# --------------------------------------------------------------------------
def coordinated_turn(bank, g=G_MEAS):
    """
    Specific force during a coordinated turn at the given bank angle.

    The resultant of gravity and centripetal acceleration lies along the body
    z-axis, so the accelerometer reports ZERO tilt while the true tilt is
    `bank`.  Magnitude is g/cos(bank).

    Key property:  tilt error is O(bank), magnitude deviation is O(bank^2/2).
    A norm-based gate therefore under-detects this class of disturbance to
    first order.  This is not a tuning problem, it is structural.
    """
    bank = np.atleast_1d(bank)
    f = np.zeros((len(bank), 3))
    f[:, 2] = g / np.cos(bank)
    return f


# --------------------------------------------------------------------------
# 5. Gating utility
# --------------------------------------------------------------------------
def norm_gate(f, g=G_MEAS, threshold=0.05):
    """
    True where the accelerometer update should be ACCEPTED.
    `threshold` is fractional: 0.05 = accept when ||f|| within 5% of g.
    """
    f = np.atleast_2d(f)
    return np.abs(np.linalg.norm(f, axis=1) - g) / g <= threshold


def adaptive_R(f, R_static, g=G_MEAS, k=1.0):
    """
    Continuous alternative to hard gating: inflate R by the observed
    specific-force residual instead of switching the update off.
    """
    f = np.atleast_2d(f)
    resid = np.abs(np.linalg.norm(f, axis=1) - g)
    return R_static + (k * resid / g) ** 2


# --------------------------------------------------------------------------
# Composite scenario builder
# --------------------------------------------------------------------------
@dataclass
class DisturbanceSpec:
    """All disturbance sources off by default — opt in explicitly."""
    lever_arm_r: tuple = (0.0, 0.0, 0.0)   # m, sensor offset from rotation axis
    vib_rms: float = 0.0                   # m/s^2 per axis
    vib_band: tuple = (5.0, 45.0)          # Hz
    tap_rate_hz: float = 0.0               # events/s
    tap_peak: float = 2.0                  # m/s^2
    seed: int = 0


def inject(roll, pitch, spec, dt=DT, g=G_MEAS):
    """
    Build the full body-frame specific force for a trajectory.

    Returns (f_clean, f_disturbed, components) so the testbench can report
    each contribution separately in the Phase 4 comparison table.
    """
    n = len(roll)
    rng = np.random.default_rng(spec.seed)
    f_clean = gravity_body(roll, pitch, g)

    omega, alpha = omega_alpha_from_angles(roll, pitch, dt)
    comps = {
        "lever_arm": lever_arm(omega, alpha, spec.lever_arm_r),
        "vibration": band_limited_vibration(n, dt, spec.vib_rms,
                                            *spec.vib_band, rng=rng),
        "taps": transient_taps(n, dt, spec.tap_rate_hz, spec.tap_peak,
                               rng=rng),
    }
    total = sum(comps.values())
    return f_clean, f_clean + total, comps
