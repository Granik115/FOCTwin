import json
import sqlite3
import tempfile
import unittest
from contextlib import closing
from pathlib import Path

from foctwin.domain import MotorProfile
from foctwin.project_store import ProjectStore


class ProjectStoreTests(unittest.TestCase):
    def test_project_is_durable_and_checkpoint_is_valid_json(self):
        with tempfile.TemporaryDirectory() as temporary:
            store = ProjectStore(Path(temporary) / "test.foctwin")
            store.initialize(MotorProfile())
            experiment_id = store.create_experiment("identification", {"signal": "step"})
            store.update_experiment(experiment_id, "running", started_at="now")
            checkpoint = store.save_checkpoint("identification", {"next_experiment": experiment_id})
            telemetry_path = store.new_telemetry_path("manual test")

            payload = json.loads(checkpoint.read_text(encoding="utf-8"))
            self.assertEqual(payload["payload"]["next_experiment"], experiment_id)
            self.assertTrue(store.db_path.exists())
            self.assertEqual(telemetry_path.parent, store.telemetry_dir)
            self.assertTrue(telemetry_path.name.endswith("_manual_test.csv"))

            with closing(sqlite3.connect(store.db_path)) as connection:
                result = connection.execute(
                    "SELECT status FROM experiments WHERE id = ?", (experiment_id,)
                ).fetchone()
            self.assertEqual(result, ("running",))
