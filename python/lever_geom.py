import numpy as np, filter_testbench as fb
from accel_disturbance import lever_arm
DT=fb.DT
print("Lever arm r=(0,0,r_z), rotation about y, sinusoid theta_amp * sin(2 pi f t)")
print("  tangential  = alpha*r_z  on x  (PERPENDICULAR to g -> tilt error)")
print("  centripetal = -w^2*r_z   on z  (PARALLEL to g      -> norm deviation)")
print("  ratio alpha/w^2 = (2 pi f)^2 A / ((2 pi f) A)^2 = 1/A   [A in rad]\n")
print(f"  {'amp(deg)':>9} {'f(Hz)':>7} {'peak w':>9} {'peak a':>9} {'RMS x':>9} {'RMS z':>9} {'x/z':>7} {'1/A':>7}")
for A_deg, f in [(31.8,0.05),(20,0.5),(10,1.0),(5,2.0),(2,5.0),(45,0.2)]:
    n=30000; t=np.arange(n)*DT
    A=np.radians(A_deg)
    th = A*np.sin(2*np.pi*f*t)
    om=np.zeros((n,3)); om[:,1]=A*2*np.pi*f*np.cos(2*np.pi*f*t)
    al=np.zeros((n,3)); al[:,1]=-A*(2*np.pi*f)**2*np.sin(2*np.pi*f*t)
    d=lever_arm(om,al,(0,0,0.03))
    rx,rz=np.sqrt(np.mean(d[:,0]**2)),np.sqrt(np.mean(d[:,2]**2))
    print(f"  {A_deg:>9.1f} {f:>7.2f} {np.abs(om[:,1]).max():>9.3f} "
          f"{np.abs(al[:,1]).max():>9.3f} {rx:>9.5f} {rz:>9.5f} {rx/rz:>7.2f} {1/A:>7.2f}")
print("\n  -> x/z tracks 1/A exactly. Small-amplitude fast motion is tangential-dominated")
print("     (norm-gate blind); large-amplitude slow motion is centripetal-visible.")

print("\nHow brisk is the current truth profile?")
t,truth,rate,static = fb.make_truth(300.0)
om=np.radians(rate); al=np.gradient(om,DT)
print(f"  peak rate  = {np.abs(rate).max():.1f} deg/s")
print(f"  peak alpha = {np.abs(al).max():.4f} rad/s^2")
print(f"  angle amplitude ~ {np.abs(truth).max():.1f} deg")
print("  For comparison, a brisk hand rotation is ~60 deg/s and ~3 rad/s^2:")
print(f"  this profile is {3.44/np.abs(al).max():.0f}x gentler in angular acceleration.")
