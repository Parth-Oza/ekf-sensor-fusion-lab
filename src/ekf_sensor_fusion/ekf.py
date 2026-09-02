"""A transparent 2D constant-velocity extended Kalman filter."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray


class ConstantVelocityEKF:
    """Track ``[x, y, vx, vy]`` from asynchronous position and velocity data."""

    def __init__(
        self,
        initial_state: ArrayLike | None = None,
        initial_covariance: ArrayLike | None = None,
        acceleration_variance: float = 0.35,
    ) -> None:
        self.state: NDArray[np.float64] = np.asarray(
            initial_state if initial_state is not None else np.zeros(4), dtype=float
        ).reshape(4)
        self.covariance: NDArray[np.float64] = np.asarray(
            initial_covariance if initial_covariance is not None else np.eye(4), dtype=float
        ).reshape(4, 4)
        if acceleration_variance <= 0:
            raise ValueError("acceleration_variance must be positive")
        self.acceleration_variance = float(acceleration_variance)

    def predict(self, dt: float) -> NDArray[np.float64]:
        if dt <= 0:
            raise ValueError("dt must be positive")
        transition = np.array(
            [[1.0, 0.0, dt, 0.0], [0.0, 1.0, 0.0, dt], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
        )
        noise_map = np.array([[0.5 * dt**2, 0.0], [0.0, 0.5 * dt**2], [dt, 0.0], [0.0, dt]])
        process_noise = noise_map @ (np.eye(2) * self.acceleration_variance) @ noise_map.T
        self.state = transition @ self.state
        self.covariance = transition @ self.covariance @ transition.T + process_noise
        return self.state.copy()

    def update_position(self, measurement: ArrayLike, variance: float) -> NDArray[np.float64]:
        observation = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        return self._update(measurement, observation, variance)

    def update_velocity(self, measurement: ArrayLike, variance: float) -> NDArray[np.float64]:
        observation = np.array([[0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        return self._update(measurement, observation, variance)

    def _update(self, measurement: ArrayLike, observation: NDArray[np.float64], variance: float) -> NDArray[np.float64]:
        if variance <= 0:
            raise ValueError("variance must be positive")
        value = np.asarray(measurement, dtype=float).reshape(2)
        measurement_noise = np.eye(2) * variance
        innovation = value - observation @ self.state
        innovation_covariance = observation @ self.covariance @ observation.T + measurement_noise
        gain = np.linalg.solve(innovation_covariance.T, (self.covariance @ observation.T).T).T
        self.state = self.state + gain @ innovation

        # Joseph form preserves symmetry and positive semi-definiteness better.
        identity = np.eye(4)
        residual = identity - gain @ observation
        self.covariance = residual @ self.covariance @ residual.T + gain @ measurement_noise @ gain.T
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        return self.state.copy()
