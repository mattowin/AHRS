#!/usr/bin/env python3
"""
allan_logger.py - long-duration IMU capture for Allan deviation analysis.

Expects one ASCII CSV line per sample from the STM32:
    n,gx,gy,gz,ax,ay,az,temp\n
with all values as raw signed integers (no scaling on the MCU).

Writes a CSV with an added host monotonic timestamp column, which is used
later to recover the TRUE sample rate (the F446 is running off HSI, so the
nominal 100 Hz is only accurate to ~1%).

Usage:
    python allan_logger.py --port /dev/ttyACM0 --out warmup.csv --minutes 1
    python allan_logger.py --port /dev/ttyACM0 --out allan_run.csv --minutes 180
"""

import argparse
import os
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed:  pip install pyserial")

NFIELDS = 8  # n, gx, gy, gz, ax, ay, az, temp
HEADER = "host_t,n,gx,gy,gz,ax,ay,az,temp\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="e.g. /dev/ttyACM0 or COM4")
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--out", required=True, help="output CSV path")
    ap.add_argument("--minutes", type=float, default=180.0)
    ap.add_argument("--status-sec", type=float, default=60.0)
    args = ap.parse_args()

    if os.path.exists(args.out):
        sys.exit(f"refusing to overwrite existing file: {args.out}")

    ser = serial.Serial(args.port, args.baud, timeout=2.0)
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.readline()  # discard first (likely partial) line

    duration = args.minutes * 60.0
    n_ok = n_bad = n_missing = n_resets = 0
    prev_n = None
    first_n = first_t = None
    last_n = last_t = None
    last_status = time.monotonic()
    last_temp_raw = 0

    print(f"logging to {args.out}  ({args.minutes:.1f} min @ {args.baud} baud)")
    print("Ctrl-C stops early and closes the file cleanly.\n")

    f = open(args.out, "w", buffering=1 << 20)
    f.write(HEADER)
    t_start = time.monotonic()

    try:
        while True:
            now = time.monotonic()
            if now - t_start >= duration:
                break

            raw = ser.readline()
            if not raw:
                print("  ! serial timeout - no data for 2 s")
                continue

            t_rx = time.monotonic()
            try:
                parts = raw.decode("ascii", "strict").strip().split(",")
                if len(parts) != NFIELDS:
                    raise ValueError
                vals = [int(p) for p in parts]
            except (UnicodeDecodeError, ValueError):
                n_bad += 1
                continue

            n = vals[0]
            if prev_n is not None and n != prev_n + 1:
                gap = n - prev_n - 1
                if gap > 0:
                    n_missing += gap
                    print(f"  ! dropped {gap} sample(s) at n={n}")
                else:
                    # counter went backwards -> the board rebooted mid-run
                    n_resets += 1
                    print(f"  !! BOARD RESET at n={n} (was {prev_n}). "
                          f"The time base is broken from here on - "
                          f"this capture is not usable.")
            prev_n = n
            last_temp_raw = vals[7]

            if first_n is None:
                first_n, first_t = n, t_rx
            last_n, last_t = n, t_rx

            f.write(f"{t_rx:.6f},{raw.decode('ascii').strip()}\n")
            n_ok += 1

            if t_rx - last_status >= args.status_sec:
                last_status = t_rx
                elapsed = t_rx - t_start
                rate = (last_n - first_n) / (last_t - first_t) if last_t > first_t else 0
                temp_c = last_temp_raw / 256.0 + 25.0
                os.fsync(f.fileno())
                print(
                    f"  {elapsed/60:6.1f} min | {n_ok:9d} samples | "
                    f"{rate:7.3f} Hz | {temp_c:5.2f} C | "
                    f"drops {n_missing} | bad lines {n_bad}"
                )

    except KeyboardInterrupt:
        print("\ninterrupted by user")
    finally:
        f.flush()
        os.fsync(f.fileno())
        f.close()
        ser.close()

    print("\n--- capture summary ---")
    print(f"file             : {args.out}")
    print(f"samples written  : {n_ok}")
    print(f"malformed lines  : {n_bad}")

    if n_ok == 0:
        print("\nNo parseable samples received.")
        if n_bad:
            print(f"{n_bad} line(s) arrived but did not match the expected "
                  f"{NFIELDS}-field CSV format -> likely a baud mismatch "
                  f"(garbage bytes) or firmware still emitting the old "
                  f"human-readable format.")
        else:
            print("Nothing arrived at all -> board not transmitting, wrong "
                  "port, or firmware not running.")
        return

    span = last_t - first_t
    true_fs = (last_n - first_n) / span if span > 0 else float("nan")
    print(f"duration         : {span/60:.2f} min")
    print(f"measured rate    : {true_fs:.4f} Hz")
    print(f"dropped samples  : {n_missing}")
    print(f"board resets     : {n_resets}")
    print(f"final temp       : {last_temp_raw/256.0 + 25.0:.2f} C")
    if n_resets:
        print("\n*** The board reset during this capture. The sample counter "
              "restarted, so the derived sample rate and every tau on the "
              "Allan curve are wrong. Re-run. ***")
    if n_missing or n_bad:
        print("\nWARNING: gaps present. A handful over 3 h is tolerable "
              "(the analysis treats the series as uniform), but hundreds "
              "means a bandwidth/blocking problem - fix and re-run.")


if __name__ == "__main__":
    main()
