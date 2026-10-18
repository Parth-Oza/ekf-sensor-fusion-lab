import csv
import json
import tempfile
import unittest
from pathlib import Path

from ekf_sensor_fusion.bag_replay import (
    BagReplayError,
    replay_bag,
    write_replay_outputs,
)
from ekf_sensor_fusion.rosbag_fixture import write_synthetic_rosbag


class BagReplayTests(unittest.TestCase):
    def test_checked_in_fixture_matches_reference_metrics(self) -> None:
        fixture = Path(__file__).parents[1] / "examples" / "ros2_bag_fixture"

        result = replay_bag(fixture)

        self.assertEqual(
            result.metrics,
            {
                "reference": "odometry_measurement",
                "imu_messages": 2,
                "odometry_messages": 2,
                "duration_seconds": 0.3,
                "position_residual_rmse_m": 0.003047,
                "position_residual_max_m": 0.00311,
            },
        )

    def test_replays_standard_messages_and_exports_outputs(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            bag_path = Path(directory) / "fixture"
            output_path = Path(directory) / "output"
            write_synthetic_rosbag(bag_path)

            result = replay_bag(bag_path)
            trajectory_path, metrics_path = write_replay_outputs(result, output_path)

            self.assertEqual(result.metrics["imu_messages"], 2)
            self.assertEqual(result.metrics["odometry_messages"], 2)
            self.assertEqual(result.metrics["reference"], "odometry_measurement")
            self.assertTrue(trajectory_path.is_file())
            with trajectory_path.open(encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 2)
            self.assertEqual(int(rows[-1]["timestamp_ns"]), 1_300_000_000)
            metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
            self.assertIn("position_residual_rmse_m", metrics)

    def test_missing_required_topic_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            bag_path = Path(directory) / "fixture"
            write_synthetic_rosbag(bag_path, include_odometry=False)

            with self.assertRaisesRegex(BagReplayError, "missing required topic.*odom"):
                replay_bag(bag_path)


if __name__ == "__main__":
    unittest.main()
