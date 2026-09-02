"""Deterministic synthetic data for exercising the filter."""

from __future__ import annotations

import json

import numpy as np

from .ekf import ConstantVelocityEKF


def run_simulation(steps: int = 120, dt: float = 0.1, seed: int = 7) -> dict[str, float | int]:
    if steps < 2:
        raise ValueError("steps must be at least 2")
    rng = np.random.default_rng(seed)
    truth = np.array([0.0, 0.0, 1.2, -0.35])
    filter_ = ConstantVelocityEKF(acceleration_variance=0.2)
    raw_errors: list[float] = []
    filtered_errors: list[float] = []

    for step in range(steps):
        truth[:2] += truth[2:] * dt
        measured_position = truth[:2] + rng.normal(0.0, 0.8, size=2)
        measured_velocity = truth[2:] + rng.normal(0.0, 0.18, size=2)
        filter_.predict(dt)
        filter_.update_position(measured_position, variance=0.8**2)
        if step % 2 == 0:
            filter_.update_velocity(measured_velocity, variance=0.18**2)
        raw_errors.append(float(np.linalg.norm(measured_position - truth[:2])))
        filtered_errors.append(float(np.linalg.norm(filter_.state[:2] - truth[:2])))

    raw_rmse = float(np.sqrt(np.mean(np.square(raw_errors))))
    filtered_rmse = float(np.sqrt(np.mean(np.square(filtered_errors))))
    return {
        "steps": steps,
        "raw_position_rmse": round(raw_rmse, 4),
        "filtered_position_rmse": round(filtered_rmse, 4),
        "improvement_percent": round((1.0 - filtered_rmse / raw_rmse) * 100.0, 2),
    }


def main() -> None:
    print(json.dumps(run_simulation(), indent=2))


if __name__ == "__main__":
    main()
