"""Static characterisation of an IMU: bias, noise density and axis misalignment.

The estimator is deliberately free of ROS imports so it can be unit tested and
reused offline on recorded bags.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

GRAVITY = 9.80665
MIN_SAMPLES = 200


@dataclass
class SampleSet:
    """Accelerometer and gyroscope readings taken while the sensor is still."""

    accel: list[np.ndarray] = field(default_factory=list)
    gyro: list[np.ndarray] = field(default_factory=list)

    def add(self, accel: np.ndarray, gyro: np.ndarray) -> None:
        self.accel.append(np.asarray(accel, dtype=float))
        self.gyro.append(np.asarray(gyro, dtype=float))

    def __len__(self) -> int:
        return len(self.accel)

    def arrays(self) -> tuple[np.ndarray, np.ndarray]:
        return np.vstack(self.accel), np.vstack(self.gyro)


@dataclass
class Calibration:
    """Result of one static capture."""

    frame_id: str
    sample_rate_hz: float
    gyro_bias: np.ndarray
    accel_bias: np.ndarray
    gyro_noise_density: np.ndarray
    accel_noise_density: np.ndarray
    accel_misalignment: np.ndarray
    residual_mps2: float

    def to_dict(self) -> dict:
        return {
            "frame_id": self.frame_id,
            "sample_rate_hz": round(self.sample_rate_hz, 3),
            "gyro_bias_rad_s": [round(float(v), 6) for v in self.gyro_bias],
            "accel_bias_m_s2": [round(float(v), 6) for v in self.accel_bias],
            "gyro_noise_density": [round(float(v), 6) for v in self.gyro_noise_density],
            "accel_noise_density": [round(float(v), 6) for v in self.accel_noise_density],
            "accel_misalignment": [[round(float(v), 6) for v in row] for row in self.accel_misalignment],
            "residual_mps2": round(float(self.residual_mps2), 6),
        }

    def to_yaml(self) -> str:
        lines = [f"frame_id: {self.frame_id}", f"sample_rate_hz: {self.sample_rate_hz:.3f}"]
        for name, values in (
            ("gyro_bias_rad_s", self.gyro_bias),
            ("accel_bias_m_s2", self.accel_bias),
            ("gyro_noise_density", self.gyro_noise_density),
            ("accel_noise_density", self.accel_noise_density),
        ):
            lines.append(f"{name}: [{', '.join(f'{v:.6f}' for v in values)}]")
        lines.append("accel_misalignment:")
        lines.extend(f"  - [{', '.join(f'{v:.6f}' for v in row)}]" for row in self.accel_misalignment)
        return "\n".join(lines) + "\n"


def misalignment_from(mean_accel: np.ndarray) -> np.ndarray:
    """Rotation (body -> world) that puts measured gravity on +Z."""
    norm = np.linalg.norm(mean_accel)
    if norm < 1e-6:
        raise ValueError("mean acceleration is zero, check the sensor feed")
    axis = np.cross(mean_accel / norm, np.array([0.0, 0.0, 1.0]))
    sin_theta = np.linalg.norm(axis)
    cos_theta = float(np.dot(mean_accel / norm, np.array([0.0, 0.0, 1.0])))
    if sin_theta < 1e-9:
        return np.eye(3)
    k = axis / sin_theta
    skew = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + sin_theta * skew + (1.0 - cos_theta) * (skew @ skew)


def estimate(samples: SampleSet, frame_id: str, sample_rate_hz: float) -> Calibration:
    """Estimate bias, noise density and misalignment from a still capture."""
    if len(samples) < MIN_SAMPLES:
        raise ValueError(f"need at least {MIN_SAMPLES} samples, got {len(samples)}")
    if sample_rate_hz <= 0.0:
        raise ValueError("sample rate must be positive")

    accel, gyro = samples.arrays()
    mean_accel = accel.mean(axis=0)
    rotation = misalignment_from(mean_accel)

    dt = 1.0 / sample_rate_hz
    accel_world = accel @ rotation.T
    bias_world = (accel_world - np.array([0.0, 0.0, GRAVITY])).mean(axis=0)
    residual = accel_world - bias_world - np.array([0.0, 0.0, GRAVITY])

    return Calibration(
        frame_id=frame_id,
        sample_rate_hz=sample_rate_hz,
        gyro_bias=gyro.mean(axis=0),
        accel_bias=rotation.T @ bias_world,
        gyro_noise_density=gyro.std(axis=0) * np.sqrt(dt),
        accel_noise_density=accel_world.std(axis=0) * np.sqrt(dt),
        accel_misalignment=rotation,
        residual_mps2=float(np.linalg.norm(residual, axis=1).mean()),
    )
