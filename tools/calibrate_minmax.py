#!/usr/bin/env python3
import json, time
import numpy as np
import bluerobotics_navigator as navigator

def main(seconds=60.0, rate_hz=100.0, out_path="mag_cal.json"):
    navigator.init()
    n = int(seconds * rate_hz)
    period = 1.0 / rate_hz

    mins = np.array([+1e9, +1e9, +1e9], dtype=float)
    maxs = np.array([-1e9, -1e9, -1e9], dtype=float)

    print(f"Collecting {seconds:.0f}s of mag data @ {rate_hz:.0f}Hz.")
    print("Rotate the ROV slowly through as many orientations as possible (figure-8 style).")
    t_next = time.perf_counter()

    for _ in range(n):
        m = navigator.read_mag()
        v = np.array([m.x, m.y, m.z], dtype=float)
        mins = np.minimum(mins, v)
        maxs = np.maximum(maxs, v)
        t_next += period
        dt = t_next - time.perf_counter()
        if dt > 0:
            time.sleep(dt)

    bias = (maxs + mins) / 2.0
    half_range = (maxs - mins) / 2.0
    avg = float(np.mean(half_range))

    # diagonal soft-iron scale (simple)
    scale = avg / np.maximum(half_range, 1e-9)
    softiron = np.diag(scale)

    result = {
        "mag_bias_uT": bias.tolist(),
        "mag_softiron_3x3": softiron.tolist(),
        "mins_uT": mins.tolist(),
        "maxs_uT": maxs.tolist(),
    }

    with open(out_path, "w") as f:
        json.dump(result, f, indent=2)

    print("Saved:", out_path)
    print("mag_bias_uT:", bias)
    print("scale_diag :", scale)

if __name__ == "__main__":
    main()
