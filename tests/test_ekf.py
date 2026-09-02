import unittest

import numpy as np

from ekf_sensor_fusion import ConstantVelocityEKF
from ekf_sensor_fusion.simulation import run_simulation


class EKFTests(unittest.TestCase):
    def test_position_update_moves_estimate_toward_measurement(self) -> None:
        filter_ = ConstantVelocityEKF()
        before = np.linalg.norm(filter_.state[:2] - np.array([2.0, -1.0]))
        filter_.update_position([2.0, -1.0], variance=0.1)
        after = np.linalg.norm(filter_.state[:2] - np.array([2.0, -1.0]))
        self.assertLess(after, before)

    def test_covariance_remains_symmetric(self) -> None:
        filter_ = ConstantVelocityEKF()
        filter_.predict(0.1)
        filter_.update_velocity([1.0, 0.5], variance=0.2)
        np.testing.assert_allclose(filter_.covariance, filter_.covariance.T, atol=1e-12)

    def test_synthetic_fusion_beats_raw_position_measurement(self) -> None:
        result = run_simulation()
        self.assertLess(result["filtered_position_rmse"], result["raw_position_rmse"])


if __name__ == "__main__":
    unittest.main()
