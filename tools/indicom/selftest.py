"""Headless check of the IndiCom acquisition + analysis against a running engine-sim.

Usage: python selftest.py <seconds> [pegging]
Prints per-cylinder averages; compare with engine-sim's ENGINE_SIM_INDI_LOG.
"""

import math
import sys
import time

import numpy as np

from acquisition import Acquisition
from analysis import Geometry, Settings


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    pegging = sys.argv[2] if len(sys.argv) > 2 else "thermodynamic"

    acquisition = Acquisition(5005)
    acquisition.start()

    # Wait for the engine info, then configure from it (like "Carregar do simulador")
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline and (acquisition.info.cylinders == 0 or min(acquisition.info.tdc_deg) < 0):
        time.sleep(0.2)
    info = acquisition.info
    print(f"engine: {info.name}, {info.cylinders} cyl, bore {info.bore_mm:.2f} stroke {info.stroke_mm:.2f} "
          f"rod {info.rod_mm:.2f} CR {info.compression_ratio:.2f}, {info.frequency_hz:.0f} Hz")
    print("TDC:", ", ".join(f"{t:.1f}" for t in info.tdc_deg))

    acquisition.configure(
        Geometry(info.bore_mm, info.stroke_mm, info.rod_mm, info.compression_ratio),
        Settings(pegging=pegging), info.tdc_deg, [True] * info.cylinders)

    results = {}
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            result, _ = acquisition.results.get(timeout=0.5)
        except Exception:
            continue
        results.setdefault(result.cylinder, []).append(result)

    print(f"rpm {acquisition.status.rpm:.0f}, lost packets {acquisition.status.lost_packets}")
    print("cyl  cycles  Pmax  aPmax  IMEPn  IMEPg   CA10   CA50   CA90  peg_offset")
    for cylinder in sorted(results):
        fired = [r for r in results[cylinder] if not r.misfire]
        avg = lambda name: np.nanmean([getattr(r, name) for r in fired]) if fired else math.nan
        print(f"{cylinder:3d} {len(results[cylinder]):6d} {avg('pmax_bar'):5.1f} {avg('pmax_angle'):6.1f} "
              f"{avg('imep_net_bar'):6.2f} {avg('imep_gross_bar'):6.2f} {avg('ca10'):6.1f} {avg('ca50'):6.1f} "
              f"{avg('ca90'):6.1f} {avg('peg_offset_bar'):8.2f}")

    acquisition.stop()


if __name__ == "__main__":
    main()
