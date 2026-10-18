# Evidence record

## Verified synthetic run

Command:

```bash
ekf-demo
```

Expected seeded result:

```json
{
  "steps": 120,
  "raw_position_rmse": 1.0101,
  "filtered_position_rmse": 0.2829,
  "improvement_percent": 71.99
}
```

The experiment uses seed `7`, a 0.1-second time step, 0.8 position-noise standard deviation, 0.18 velocity-noise standard deviation, and a velocity update every second step.

## Test record

The test suite checks that:

- a position update moves the estimate toward the measurement;
- covariance remains symmetric; and
- the seeded fused estimate outperforms the raw position measurement;
- an acceleration control input advances position and velocity correctly;
- a real rosbag2 MCAP fixture containing standard IMU and odometry messages replays successfully;
- trajectory CSV and metrics JSON exports have the expected records; and
- a missing required topic is rejected with a clear diagnostic.

Run:

```bash
python -m unittest discover -s tests -v
```

## Interpretation boundary

This result validates controlled synthetic scenarios and the ROS 2 storage/serialization integration. It is not evidence of field performance, clock synchronization, frame correctness, IMU gravity compensation, robustness to outliers, delayed measurements, calibration errors, or production deployment. The replay residual uses odometry as its reference and must not be presented as independent ground-truth accuracy. Those questions belong in a hardware-backed localization reliability benchmark.
