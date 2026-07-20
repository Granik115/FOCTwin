import csv
import tempfile
import unittest
from pathlib import Path

from foctwin.domain import TelemetrySample
from foctwin.telemetry import TelemetryRecorder, TelemetryStatistics


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

            with path.open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            recorder.stop()

            self.assertEqual(rows[0]["current_q_a"], "0.4")
            self.assertEqual(rows[0]["raw"], "raw row")
