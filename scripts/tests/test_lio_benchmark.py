from contextlib import nullcontext
import fcntl
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import lio_benchmark as bench


class PipelineTest(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.repo = self.root / "repo"
        self.repo.mkdir()
        self.package = self.repo / bench.PACKAGE
        (self.package / "src").mkdir(parents=True)
        self.source = self.package / "src/test.cpp"
        self.source.write_text("int original;\n")
        self.git("init", "-q")
        self.git("add", ".")
        self.git("-c", "user.email=test@example.invalid", "-c", "user.name=Test",
                 "commit", "-qm", "fixture")

    def git(self, *args):
        return subprocess.check_output(["git", "-C", str(self.repo), *args])

    def test_outdoor_only_recipe_schedules_only_outdoor_and_freezes_scope(self):
        cfg = bench.read_json(bench.ROOT / "experiments/recipes/lio_default.json")
        cfg.update(reference="outdoor-only test", hypothesis="test configured datasets",
                   patch="candidate.patch", allowed_files=[str(bench.PACKAGE / "src/test.cpp")],
                   overlay_setup=str(self.repo / "ws_livox/install/setup.bash"),
                   bags={"outdoor": {"path": "/outdoor"}}, repetitions=2)
        recipe = self.root / "recipe.json"
        bench.write_json(recipe, cfg)
        cfg = bench.read_config(recipe, self.repo)
        runs = list(bench.replay_order(cfg))
        self.assertEqual([(v, d, r) for _, v, d, r in runs], [
            ("baseline", "outdoor", 1), ("candidate", "outdoor", 1),
            ("candidate", "outdoor", 2), ("baseline", "outdoor", 2)])
        contract = bench.contract_for(cfg, {"commit": "source"}, cfg["bags"])
        self.assertEqual(contract["required_datasets"], ["outdoor"])

    def test_snapshot_keeps_staged_and_unstaged_changes_and_untracked_source(self):
        self.source.write_text("int staged;\n")
        self.git("add", ".")
        self.source.write_text("int current;\n")
        (self.package / "src/local.cpp").write_text("int local;\n")
        (self.package / "src/raw.db3").write_bytes(b"large recording stand-in")
        snap = bench.source_snapshot(self.repo, "HEAD")
        destination = self.root / "worktree"
        self.git("worktree", "add", "--detach", str(destination), snap["commit"])
        bench.restore_source(self.repo, destination, snap)
        self.assertEqual((destination / bench.PACKAGE / "src/test.cpp").read_text(), "int current;\n")
        self.assertEqual((destination / bench.PACKAGE / "src/local.cpp").read_text(), "int local;\n")
        self.assertFalse((destination / bench.PACKAGE / "src/raw.db3").exists())
        self.assertEqual(self.source.read_text(), "int current;\n")

    def test_patch_scope_rejects_undeclared_and_special_paths(self):
        approved = str(bench.PACKAGE / "src/test.cpp")
        for name in ("outside.cpp", "/tmp/absolute.cpp", "../escape.cpp", "dir with space.cpp"):
            patch = f"diff --git a/{name} b/{name}\n--- a/{name}\n+++ b/{name}\n@@ -1 +1 @@\n-a\n+b\n"
            with self.subTest(name=name), self.assertRaises(bench.BenchmarkError):
                bench.patch_paths(patch.encode(), {approved, name})
        patch = f"diff --git a/{approved} b/{approved}\n--- a/{approved}\n+++ b/{approved}\n@@ -1 +1 @@\n-a\n+b\n"
        self.assertEqual(bench.patch_paths(patch.encode(), {approved}), {approved})
        with self.assertRaises(bench.BenchmarkError):
            bench.patch_paths(patch.replace(f"+++ b/{approved}", "+++ b/escape.cpp").encode(),
                              {approved})

    def test_resume_rejects_changed_identity(self):
        frozen = {"files": {str(self.source): bench.digest(self.source)}}
        bench.verify_files(frozen["files"])
        self.source.write_text("int changed;\n")
        with self.assertRaisesRegex(bench.BenchmarkError, "changed"):
            bench.verify_files(frozen["files"])

    def test_non_head_ref_is_rejected_before_worktree_creation(self):
        previous = self.git("rev-parse", "HEAD").decode().strip()
        self.source.write_text("int next_version;\n")
        self.git("add", ".")
        self.git("-c", "user.email=test@example.invalid", "-c", "user.name=Test",
                 "commit", "-qm", "next version")
        with self.assertRaisesRegex(bench.BenchmarkError, "current source HEAD"):
            bench.source_snapshot(self.repo, previous)

    def test_resume_checks_frozen_files_before_dispatching_steps(self):
        root = self.repo / "experiments/example/unit"
        root.mkdir(parents=True)
        cfg = {"scheme_id": "unit", "output_root": str(root.parent), "record_ledger": False}
        bench.write_json(root / "frozen.json", {"config": cfg, "files": {str(self.source): "stale"}, "bags": {}})
        with patch.object(bench, "run_lock", return_value=nullcontext()), patch.object(bench, "guard_mapping"), \
                patch.object(bench, "run_pipeline") as execute:
            with self.assertRaisesRegex(bench.BenchmarkError, "Frozen file changed"):
                bench.resume(root, self.repo)
            execute.assert_not_called()
        self.assertEqual(bench.read_json(root / "failure/verdict.json")["verdict"], "RETEST")

    def test_resume_rejects_changed_cpu_affinity_or_openmp_environment(self):
        root = self.repo / "experiments/example/unit"
        root.mkdir(parents=True)
        cfg = {"scheme_id": "unit", "output_root": str(root.parent), "record_ledger": False}
        recipe = root / "recipe.json"
        recipe.write_text("{}")
        frozen = {"config": cfg, "config_path": str(recipe), "files": {}, "bags": {},
                  "machine_identity": {"cpu_affinity": [0, 1], "inherited_thread_env": {"OMP_NUM_THREADS": "2"}}}
        bench.write_json(root / "frozen.json", frozen)
        variations = [{"cpu_affinity": [0], "inherited_thread_env": {"OMP_NUM_THREADS": "2"}},
                      {"cpu_affinity": [0, 1], "inherited_thread_env": {"OMP_NUM_THREADS": "3"}}]
        for identity in variations:
            with self.subTest(identity=identity), patch.object(bench, "run_lock", return_value=nullcontext()), \
                    patch.object(bench, "guard_mapping"), patch.object(bench, "read_config", return_value=cfg), \
                    patch.object(bench, "machine_identity", return_value=identity), \
                    patch.object(bench, "source_snapshot") as snapshot:
                with self.assertRaisesRegex(bench.BenchmarkError, "Frozen machine"):
                    bench.resume(root, self.repo)
                snapshot.assert_not_called()

    def test_broken_replay_evidence_keeps_stage_retryable(self):
        root = self.root / "run"
        root.mkdir()
        state = bench.State(root)
        cfg = {"ros_setup": "ros", "overlay_setup": "overlay", "surf_filter": 0.15, "post_play_seconds": 30}
        tools = {"python": sys.executable, "bash": "bash", "tegrastats": "tegrastats"}
        binary = {"path": "/binary", "sha256": "binary"}
        bag = {"path": "/bag", "duration_s": 26}

        def fake_observer(attempt, label, argv, cwd):
            output = attempt / "replay"
            events = output / "timing/run_test/events.csv"
            events.parent.mkdir(parents=True)
            for file in ("commands.json", "effective_surf_filter.txt", "observer_ready",
                         "observer.log", "player.log", "node.log"):
                (output / file).touch()
            code = 4 if attempt.name == "001" else 0
            bench.write_json(output / "returncodes.json", {"observer": 0, "node": 0, "player": code})
            bench.write_json(output / "observer_counts.json", {"cloud_messages": 1, "tf_endpoint_gids": ["1ab"]})
            bench.write_json(output / "run_summary.json", {"full_cycles": 1, "odom_events": 1, "tf_creations": 1,
                                                          "csv": str(events)})
            (output / "outputs.jsonl").write_text('{"event":"odom"}\n{"event":"tf"}\n')
            events.write_text("event,status,count\nfull_cycle,1,1\nodom_publish,1,1\ntf_broadcaster_create,0,1\n")
            if attempt.name == "002":
                with events.open("a") as stream:
                    stream.write("full_cycle,1")

        with patch.object(bench, "guard_mapping"), patch.object(bench, "verify_binary"), \
                patch.object(bench, "verify_bag"), patch.object(bench, "telemetry", return_value=nullcontext()), \
                patch.object(bench, "command", side_effect=fake_observer):
            run = lambda attempt: bench.replay(attempt, cfg, tools, self.repo, binary, bag, bench.machine_identity())
            with self.assertRaisesRegex(bench.BenchmarkError, "returncodes"):
                state.step("replay", run)
            with self.assertRaisesRegex(bench.BenchmarkError, "diagnostic CSV"):
                state.step("replay", run)
            state.step("replay", run)
        self.assertEqual([entry["status"] for entry in state.data["steps"]["replay"]["attempts"]],
                         ["failed", "failed", "done"])
        self.assertTrue((root / "attempts/replay/002/replay/timing/run_test/events.csv").exists())

    def test_pipeline_rejects_changed_manifest_gates_before_dispatch(self):
        root = self.repo / "experiments/example/unit"
        root.mkdir(parents=True)
        cfg = {"warmup_seconds": 5, "high_load_start_s": 40, "high_load_end_s": 100, "record_ledger": False,
               "regression_gates": {"tf_odom_position_max_m": 1e-12}}
        snapshot = {"commit": "frozen"}
        files = {str(self.repo / path): "frozen" for path in
                 (Path("scripts/lio_benchmark_report.py"), Path("scripts/summarize_pr465_ab.py"), bench.LEGACY_PARSER)}
        frozen = {"config": cfg, "tools": {}, "bags": {}, "files": files, "machine_identity": {}}
        contract = bench.frozen_manifest(frozen, snapshot, self.repo)["contract"]
        contract["regression_gates"] = {"tf_odom_position_max_m": 1}
        bench.write_json(root / "manifest.json", {"contract": contract, "runs": {}})
        with patch.object(bench, "prepare") as prepare, \
                patch.object(bench, "machine_identity", return_value={}):
            with self.assertRaisesRegex(bench.BenchmarkError, "manifest contract"):
                bench.run_pipeline(bench.State(root), frozen, snapshot, self.repo)
            prepare.assert_not_called()
        self.assertEqual(bench.read_json(root / "manifest.json")["contract"], contract)

    def test_manifest_runs_are_rebuilt_from_done_state_with_verified_hashes(self):
        root = self.root / "run"
        root.mkdir()
        cfg = {"warmup_seconds": 5, "high_load_start_s": 40, "high_load_end_s": 100, "repetitions": 1}
        paths = (Path("scripts/lio_benchmark_report.py"), Path("scripts/summarize_pr465_ab.py"), bench.LEGACY_PARSER)
        frozen = {"config": cfg, "bags": {}, "files": {str(self.repo / path): "frozen" for path in paths},
                  "machine_identity": {}}
        snapshot = {"commit": "frozen"}
        manifest = {**bench.frozen_manifest(frozen, snapshot, self.repo), "runs": {}}
        state = bench.State(root)

        def complete(attempt):
            output = attempt / "replay"
            output.mkdir()
            data = output / "evidence"
            data.write_text("actual")
            return {"path": str(output), "exit_code": 0, "binary_sha256": "binary",
                    "evidence_sha256": {str(data): bench.digest(data)}}

        state.step("replay-baseline_indoor_r1", complete)
        runs = bench.verify_manifest(state, manifest, frozen, snapshot, self.repo)
        self.assertEqual(list(runs), ["baseline_indoor_r1"])
        record = runs["baseline_indoor_r1"]
        self.assertFalse(Path(record["path"]).is_absolute())
        manifest["runs"] = {"candidate_indoor_r1": {**record, "variant": "candidate"}}
        with self.assertRaisesRegex(bench.BenchmarkError, "differs from verified state"):
            bench.verify_manifest(state, manifest, frozen, snapshot, self.repo)
        (root / record["path"] / "evidence").write_text("changed")
        with self.assertRaisesRegex(bench.BenchmarkError, "Frozen file changed"):
            bench.verify_manifest(state, manifest, frozen, snapshot, self.repo)

    def test_report_failure_keeps_attempt_and_success_can_publish(self):
        root = self.root / "report-run"
        root.mkdir()
        bench.write_json(root / "manifest.json", {"runs": {}})
        state = bench.State(root)
        with patch.object(bench, "command", side_effect=bench.BenchmarkError("report error")):
            with self.assertRaisesRegex(bench.BenchmarkError, "report error"):
                state.step("report", lambda attempt: bench.generate_report(attempt, root, {"python": sys.executable}))
        self.assertFalse((root / "REPORT.md").exists())

        def success(attempt, label, argv, cwd):
            for name in ("comparison.json", "REPORT.md", "metrics.csv"):
                (attempt / name).write_text("evidence")
            bench.write_json(attempt / "verdict.json", {"verdict": "RETEST"})

        with patch.object(bench, "command", side_effect=success):
            state = bench.State(root)
            result = state.step("report", lambda attempt: bench.generate_report(attempt, root, {"python": sys.executable}))
        self.assertEqual(result["verdict"], "RETEST")
        bench.publish_report(root, result)
        bench.publish_report(root, result)
        self.assertEqual((root / "REPORT.md").read_text(), "evidence")
        self.assertEqual(len(state.data["steps"]["report"]["attempts"]), 2)

    def test_failed_stage_retries_in_new_directory_and_done_stage_skips(self):
        directory = self.root / "run"
        directory.mkdir()
        state = bench.State(directory)
        calls = []

        def failure(attempt):
            calls.append(attempt)
            (attempt / "output.log").write_text("failure")
            raise bench.BenchmarkError("failed")

        with self.assertRaisesRegex(bench.BenchmarkError, "failed"):
            state.step("build", failure)
        state = bench.State(directory)
        state.step("build", calls.append)
        state.step("build", calls.append)
        self.assertEqual(len(calls), 2)
        self.assertNotEqual(calls[0], calls[1])
        self.assertEqual((calls[0] / "output.log").read_text(), "failure")

    def test_existing_output_and_live_lock(self):
        directory = self.root / "run"
        bench.create_run(directory)
        with self.assertRaises(FileExistsError):
            bench.create_run(directory)
        lock = self.root / "main.lock"
        lock.write_text("999999\n")
        with bench.run_lock(lock), lock.open("r+") as competing:
            with self.assertRaises(BlockingIOError):
                fcntl.flock(competing, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with bench.run_lock(lock):
            pass

    def test_command_exit_and_argv_are_saved(self):
        attempt = self.root / "attempt"
        attempt.mkdir()
        argv = [sys.executable, "-c", "print('one log line');raise SystemExit(7)"]
        with self.assertRaises(bench.BenchmarkError) as error:
            bench.command(attempt, "stub", argv, cwd=self.repo)
        self.assertEqual(error.exception.exit_code, 7)
        self.assertIn("one log line", (attempt / "stub.log").read_text())
        self.assertEqual(bench.read_json(attempt / "stub.json")["argv"], argv)


if __name__ == "__main__":
    unittest.main()
