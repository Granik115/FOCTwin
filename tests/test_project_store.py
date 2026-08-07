import json
import sqlite3
import tempfile
import unittest
import zipfile
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
            store.update_experiment(
                experiment_id,
                "completed",
                result_json=json.dumps({"position_map": {"observations": [{"position": 1.0}]}}),
            )
            checkpoint = store.save_checkpoint("identification", {"next_experiment": experiment_id})
            loaded_checkpoint = store.load_checkpoint("identification")
            telemetry_path = store.new_telemetry_path("manual test")
            export_path = store.save_export("friction", {"coulomb": 0.2})
            accepted_id = store.accept_parameters(
                "test profile",
                "friction_velocity",
                {"coulomb_friction_nm": 0.2},
                score=0.9,
            )

            payload = json.loads(checkpoint.read_text(encoding="utf-8"))
            self.assertEqual(payload["payload"]["next_experiment"], experiment_id)
            self.assertEqual(loaded_checkpoint["next_experiment"], experiment_id)
            self.assertTrue(store.db_path.exists())
            self.assertEqual(telemetry_path.parent, store.telemetry_dir)
            self.assertTrue(telemetry_path.name.endswith("_manual_test.csv"))
            self.assertEqual(json.loads(export_path.read_text(encoding="utf-8"))["coulomb"], 0.2)
            self.assertGreater(accepted_id, 0)
            results = store.experiment_results("identification")
            self.assertEqual(results[0][0], experiment_id)
            self.assertEqual(
                results[0][1]["position_map"]["observations"][0]["position"],
                1.0,
            )

            with closing(sqlite3.connect(store.db_path)) as connection:
                result = connection.execute(
                    "SELECT status FROM experiments WHERE id = ?", (experiment_id,)
                ).fetchone()
            self.assertEqual(result, ("completed",))

    def test_save_bundle_collects_existing_evidence_files_once(self):
        with tempfile.TemporaryDirectory() as temporary:
            store = ProjectStore(Path(temporary) / "project.foctwin")
            store.initialize()
            report = store.save_export("current_trial_1", {"status": "completed"})
            telemetry = store.new_telemetry_path("current_trial_1")
            telemetry.write_text("timestamp_s,current_q_a\n0.0,0.0\n", encoding="utf-8")

            bundle = store.save_bundle(
                "current_trial_1_send_me",
                [report, telemetry, telemetry],
            )

            self.assertTrue(bundle.is_file())
            with zipfile.ZipFile(bundle) as archive:
                self.assertEqual(
                    sorted(archive.namelist()),
                    sorted([report.name, telemetry.name]),
                )
