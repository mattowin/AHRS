import numpy as np, filter_testbench as fb

def run(label, **kw):
    rng = np.random.default_rng(1)
    t, truth, rate, static = fb.make_truth(300.0)
    gyro, ax, az, bias, dist = fb.simulate_imu(truth, rate, rng, **kw)
    aa = fb.accel_angle(ax, az)
    taus = np.logspace(np.log10(0.005), np.log10(100.0), 41)
    A=[];S=[];D=[]
    for tau in taus:
        e = fb.complementary(gyro, aa, tau) - truth
        A.append(fb.rms(e)); S.append(fb.rms(e[static])); D.append(fb.rms(e[~static]))
    A,S,D = map(np.array,(A,S,D))
    dmag = np.linalg.norm(dist,axis=1)
    nd = np.abs(np.hypot(ax,az)-fb.G)/fb.G
    tilt_dist = np.rad2deg(np.arctan2(dist[:,0], fb.G))
    print(f"\n{label}")
    print(f"  accel-only RMS   : {fb.rms(aa-truth):.4f} deg "
          f"(static {fb.rms((aa-truth)[static]):.4f}, dyn {fb.rms((aa-truth)[~static]):.4f})")
    if dmag.max()>0:
        print(f"  disturbance RMS  : {fb.rms(dmag):.4f} m/s2, peak {dmag.max():.4f}")
        print(f"  tilt err from it : RMS {fb.rms(tilt_dist):.4f} deg, peak {np.abs(tilt_dist).max():.4f} deg")
        print(f"  norm dev (gate)  : RMS {fb.rms(nd)*100:.4f}%, peak {nd.max()*100:.3f}%")
        ratio = fb.rms(np.abs(tilt_dist))/ (fb.rms(nd)*100 + 1e-12)
        print(f"  deg of tilt error per 1% of norm deviation: {ratio:.2f}")
    for nm, arr in (("all",A),("static",S),("dynamic",D)):
        i = int(np.argmin(arr))
        rail = "  [RAILED at sweep edge]" if i in (0,len(taus)-1) else ""
        print(f"  tau_opt {nm:<8}: {taus[i]:8.4f} s   RMS {arr[i]:.4f} deg{rail}")

run("A. clean")
run("B. lever arm r_z=30mm", lever_arm_r=(0,0,0.03))
run("C. lever arm r_z=30mm + vib 0.05 + taps 0.3", lever_arm_r=(0,0,0.03), vib_rms=0.05, tap_rate=0.3)
run("D. legacy v1 disturbance 0.5 (motion-UNcorrelated)", legacy_disturbance=0.5)
run("E. vibration only 0.10 (motion-uncorrelated)", vib_rms=0.10)
