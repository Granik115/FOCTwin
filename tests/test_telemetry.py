import csv
import tempfile
import unittest
from pathlib import Path

from foctwin.domain import TelemetrySample
from foctwin.telemetry import TelemetryRecorder, TelemetryStatistics, monitor_stale_timeout


class TelemetryTests(unittest.TestCase):
    def test_statistics_report_rate_and_jitter(self):
        statistics = TelemetryStatistics()
        for timestamp in (0.0, 0.1, 0.2, 0.3):
            statistics.add(timestamp)

        self.assertAlmostEqual(statistics.frequency_hz, 10.0)
        self.assertAlmostEqual(statistics.jitter_s, 0.0)
        self.assertEqual(statistics.sample_count, 4)

    def test_recorder_flushes_normalized_samples_to_csv(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "telemetry.csv"
            recorder = TelemetryRecorder()
            recorder.start(path)
            recorder.append(
                TelemetrySample(
                    timestamp_s=0.25,
                    sequence=1,
                    received_at_utc="2026-07-20T00:00:00+00:00",
                    current_q_a=0.4,
                    raw="raw row",
                )
            )
            recorder.stop()

            with path.open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))

            self.assertEqual(rows[0]["current_q_a"], "0.4")
            self.assertEqual(rows[0]["raw"], "raw row")

    def test_monitor_timeout_scales_with_downsample(self):
        self.assertEqual(monitor_stale_timeout(20), 2.0)
        self.assertGreater(monitor_stale_timeout(100000), 300.0)
        with self.assertRaises(ValueError):
            monitor_stale_timeout(0)
