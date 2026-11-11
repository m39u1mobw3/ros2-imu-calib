"""Summarise an imu_calib YAML and flag values that need a re-run.

    python3 tools/report.py imu_calibration.yaml

Exit status is 1 when any check fails, so it can gate a CI job or a shell
script wrapping the calibration run.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

# Thresholds come from docs/calibration-procedure.md.
LIMITS = {
    "gyro_bias_rad_s": 0.02,
    "accel_bias_m_s2": 0.2,
    "residual_mps2": 0.2,
}


def parse(path: Path) -> tuple[dict[str, list[float]], float]:
    """Read flat vector fields and the residual from the calibration file."""
    vectors: dict[str, list[float]] = {}
    residual = math.inf
    for line in path.read_text(encoding="utf-8").splitlines():
        key, _, raw = line.partition(":")
        key = key.strip()
        raw = raw.strip()
        if key == "residual_mps2":
            residual = float(raw)
        elif raw.startswith("[") and raw.endswith("]"):
            vectors[key] = [float(v) for v in raw[1:-1].split(",") if v.strip()]
    return vectors, residual


def check(name: str, values: list[float], limit: float) -> bool:
    return all(abs(v) <= limit for v in values)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("calibration", type=Path)
    args = parser.parse_args(argv)

    vectors, residual = parse(args.calibration)
    failures = 0
    for name, limit in LIMITS.items():
        values = [residual] if name == "residual_mps2" else vectors.get(name)
        if not values:
            print(f"missing  {name}")
            failures += 1
            continue
        peak = max(abs(v) for v in values)
        status = "ok" if check(name, values, limit) else "FAIL"
        failures += status == "FAIL"
        print(f"{status:6}  {name:24} peak {peak:.6f}  limit {limit}")

    print(f"{failures} check(s) failed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
