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
- the seeded fused estimate outperforms the raw position measurement.

Run:

```bash
python -m unittest discover -s tests -v
```

## Interpretation boundary

This result validates one controlled synthetic scenario. It is not evidence of field performance, ROS 2 timing behavior, robustness to outliers, delayed measurements, calibration errors, or production deployment. Those questions belong in the planned ROS 2 localization reliability benchmark.
