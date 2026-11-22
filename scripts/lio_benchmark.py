#!/usr/bin/env python3
"""Prepare and run an isolated FAST-LIVO2 offline latency comparison."""

import argparse
from contextlib import closing, contextmanager
import csv
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import sqlite3
import subprocess
import sys
import tempfile
import time


REPORT_FILES = ("comparison.json", "REPORT.md", "verdict.json", "metrics.csv")


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = Path("ws_livox/src/FAST-LIVO2-master")
OBSERVER = Path("scripts/benchmark_tf_lifetime.py")
LEGACY_PARSER = PACKAGE / "scripts/summarize_surf_filter_ab.py"
COMMON_PATCH_SHA = "6e60a9446d12841af5038a4568cd80b26a387144638aa8b4289315097aa3051c"
COMMON_FILE = str(PACKAGE / "src/main.cpp")
VARIANTS = ("baseline", "candidate")
TOPICS = {"/livox/imu": "sensor_msgs/msg/Imu", "/livox/lidar": "livox_ros_driver2/msg/CustomMsg"}
GENERATED_DIRS = {"Log", "build", "install", "log", "__pycache__", "CMakeFiles"}
SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx", ".ipp",
                   ".inl", ".tpp", ".cmake", ".py", ".xml", ".yaml", ".yml", ".in"}
SOURCE_ENV = 'source "$1" && source "$2" && shift 2 && exec "$@"'


class BenchmarkError(RuntimeError):
    def __init__(self, message, exit_code=1):
        super().__init__(message)
        self.exit_code = exit_code if 0 < exit_code < 126 else 1


def read_json(path):
    return json.loads(path.read_text())


def write_json(path, value):
    temp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temp.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temp.replace(path)


def digest(path):
    with path.open("rb") as stream:
        checksum = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(block)
    return checksum.hexdigest()


def verify_files(files):
    for name, expected in files.items():
        path = Path(name)
        if not path.is_file() or digest(path) != expected:
            raise BenchmarkError(f"Frozen file changed or missing: {path}")


def git(repo, *args):
    result = subprocess.run(["git", "-C", str(repo), *map(str, args)],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if result.returncode:
        raise BenchmarkError(f"Git {args[0]} failed: {os.fsdecode(result.stderr).strip()}")
    return result.stdout


def source_files(repo):
    package = repo / PACKAGE
    if not package.is_dir() or package.resolve() != package:
        raise BenchmarkError(f"Source package missing or reached through a symlink: {package}")
    tracked = {Path(os.fsdecode(name)) for name in
               git(repo, "ls-files", "-z", "--", PACKAGE).split(b"\0") if name}
    paths = tracked.copy()
    for directory, subdirs, files in os.walk(package):
        subdirs[:] = [name for name in subdirs if name not in GENERATED_DIRS]
        for name in subdirs:
            if (Path(directory) / name).is_symlink():
                raise BenchmarkError(f"Unexpected source directory symlink: {Path(directory) / name}")
        for name in files:
            file = Path(directory) / name
            path = file.relative_to(repo)
            if name == "CMakeLists.txt" or file.suffix in SOURCE_SUFFIXES:
                paths.add(path)
    identities = {}
    for path in sorted(paths):
        file = repo / path
        if file.is_symlink() or (file.exists() and not file.is_file()):
            raise BenchmarkError(f"Unsupported source path: {file}")
        if file.is_file():
            identities[str(path)] = {"sha256": digest(file),
                                     "mode": file.stat().st_mode & 0o777}
    return identities, [str(path) for path in sorted(paths - tracked)]


def source_snapshot(repo, reference):
    commit = git(repo, "rev-parse", "--verify", "--end-of-options", reference + "^{commit}").decode().strip()
    head = git(repo, "rev-parse", "HEAD").decode().strip()
    if commit != head:
        raise BenchmarkError("baseline_ref must resolve to the current source HEAD")
    patch = git(repo, "diff", "--binary", "--no-ext-diff", "--no-textconv",
                "--no-renames", "--no-color", "HEAD", "--", PACKAGE)
    files, untracked = source_files(repo)
    return {"commit": commit, "source_head": head, "dirty": patch,
            "files": files, "untracked": untracked}


def patch_paths(patch, allowed):
    try:
        lines = patch.decode("utf-8").split("\n")
    except UnicodeDecodeError as error:
        raise BenchmarkError("Patch must be UTF-8 text") from error
    paths = set()
    current = None
    headers = set()
    for line in lines:
        if line.startswith("diff --git "):
            if current and headers != {"old", "new", "hunk"}:
                raise BenchmarkError(f"Incomplete textual patch for {current}")
            match = re.fullmatch(r"diff --git a/([\w./-]+) b/([\w./-]+)", line, re.ASCII)
            if not match or match[1] != match[2]:
                raise BenchmarkError(f"Unsupported patch paths: {line}")
            current = match[1]
            path = Path(current)
            if (current not in allowed or path.is_absolute() or path.as_posix() != current
                    or ".." in path.parts or path.parent == PACKAGE or PACKAGE not in path.parents):
                raise BenchmarkError(f"Patch path outside approved package scope: {current}")
            if current in paths:
                raise BenchmarkError(f"Duplicate patch section: {current}")
            paths.add(current)
            headers = set()
        elif line.startswith("@@ "):
            if not current or not {"old", "new"} <= headers:
                raise BenchmarkError("Patch hunk has no verified file headers")
            headers.add("hunk")
        elif line.startswith(("--- ", "+++ ")) and "hunk" not in headers:
            key, prefix = ("old", "--- a/") if line.startswith("--- ") else ("new", "+++ b/")
            if current is None or key in headers or line != prefix + current:
                raise BenchmarkError(f"Unexpected patch file header: {line}")
            headers.add(key)
        elif ("hunk" in headers and line.startswith((" ", "+", "-", "\\ "))) or not line:
            continue
        elif current and re.fullmatch(r"index [0-9a-f]+\.\.[0-9a-f]+ 100(?:644|755)", line):
            continue
        else:
            raise BenchmarkError(f"Unsupported patch line: {line}")
    if not paths or headers != {"old", "new", "hunk"}:
        raise BenchmarkError("Expected a complete textual patch")
    return paths


def apply_patch(worktree, patch, *, common=False, paths=None):
    if not patch:
        return
    if paths is not None:
        parsed = subprocess.run(["git", "-C", str(worktree), "apply", "--numstat", "-z", "-"],
                                input=patch, capture_output=True, check=False)
        actual = {os.fsdecode(entry.split(b"\t", 2)[-1]) for entry in parsed.stdout.split(b"\0") if entry}
        if parsed.returncode or actual != set(paths):
            raise BenchmarkError(f"Git interpreted unexpected patch paths: {sorted(actual)}")
    if common:
        check = subprocess.run(["git", "-C", str(worktree), "apply", "--reverse", "--check", "-"],
                               input=patch, capture_output=True, check=False)
        if check.returncode == 0:
            return
    for flags in (["--check"], []):
        result = subprocess.run(["git", "-C", str(worktree), "apply", *flags, "-"],
                                input=patch, capture_output=True, check=False)
        if result.returncode:
            raise BenchmarkError(f"Patch rejected in {worktree}: {os.fsdecode(result.stderr).strip()}")


def restore_source(repo, destination, snapshot):
    apply_patch(destination, snapshot["dirty"])
    for name in snapshot["untracked"]:
        source, target = repo / name, destination / name
        if target.exists() or target.is_symlink():
            raise BenchmarkError(f"Untracked source conflicts with baseline ref: {name}")
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    for name, entry in snapshot["files"].items():
        if destination.joinpath(name).is_file() and digest(destination / name) == entry["sha256"]:
            (destination / name).chmod(entry["mode"])
    if source_files(destination)[0] != snapshot["files"]:
        raise BenchmarkError("Isolated source differs from the frozen live source")


def bag_identity(path):
    metadata = path / "metadata.yaml"
    contents = metadata.read_text()
    if not re.search(r"^  storage_identifier: sqlite3$", contents, re.M):
        raise BenchmarkError(f"Unsupported rosbag storage: {path}")
    files_section = re.search(r"^  relative_file_paths:\n((?:    - [^\n]+\n)+)", contents, re.M)
    duration = re.search(r"^  duration:\n    nanoseconds: ([0-9]+)$", contents, re.M)
    start = re.search(r"^  starting_time:\n    nanoseconds_since_epoch: ([0-9]+)$", contents, re.M)
    if not (files_section and duration and start):
        raise BenchmarkError(f"Incomplete rosbag metadata: {path}")
    filenames = [line.removeprefix("    - ") for line in files_section[1].splitlines()]
    if (len(set(filenames)) != len(filenames) or
            any(not re.fullmatch(r"[a-zA-Z0-9_.-]+\.db3", name) for name in filenames)):
        raise BenchmarkError(f"Unsupported rosbag filenames: {path}")
    actual = {item.name for item in path.iterdir()}
    if actual != {"metadata.yaml", *filenames} or any(item.is_symlink() for item in path.iterdir()):
        raise BenchmarkError(f"Unexpected rosbag files or symlinks: {path}")
    counts = dict.fromkeys(TOPICS, 0)
    starts, ends = [], []
    for name in filenames:
        file = path / name
        if not file.is_file():
            raise BenchmarkError(f"Missing rosbag data file: {file}")
        with closing(sqlite3.connect(file.as_uri() + "?mode=ro&immutable=1", uri=True)) as db:
            first, last = db.execute("SELECT min(timestamp), max(timestamp) FROM messages").fetchone()
            if first is None or last is None:
                raise BenchmarkError(f"Empty rosbag data file: {file}")
            starts.append(first)
            ends.append(last)
            for topic, type_name, count in db.execute("""
                SELECT topics.name, topics.type, count(messages.id)
                FROM topics LEFT JOIN messages ON messages.topic_id = topics.id
                WHERE topics.name IN (?, ?) GROUP BY topics.id
            """, tuple(TOPICS)):
                if type_name != TOPICS[topic]:
                    raise BenchmarkError(f"Unexpected ROS message type for {topic}: {type_name}")
                counts[topic] += count
    if int(start[1]) != min(starts) or int(duration[1]) != max(ends) - min(starts):
        raise BenchmarkError(f"Rosbag metadata time bounds differ from the SQLite storage: {path}")
    if not all(counts.values()):
        raise BenchmarkError(f"Missing recorded LiDAR or IMU messages: {path}")
    return {"path": str(path), "start_s": int(start[1]) / 1e9, "duration_s": int(duration[1]) / 1e9,
            "input_counts": counts, "input_types": TOPICS,
            "sha256": {name: digest(path / name) for name in ["metadata.yaml", *filenames]}}


def validate_gates(gates):
    distances = ("tf_odom_position_max_m", "tf_odom_quaternion_component_max",
                 "paired_pose_max_m", "paired_quaternion_component_max")
    margins = {"outdoor_high_load_wall_mapped_age_p95_candidate_max": ("p95", " ms"),
               "outdoor_high_load_lidar_q_max_candidate_max": ("max", ""),
               "wall_output_interval_p95_candidate_max": ("p95", " ms")}
    descriptions = ("successful_source_scan_sets", "inputs", "state_age_validity",
                    "absolute_accuracy", "performance_claim")
    if not isinstance(gates, dict):
        raise BenchmarkError("Missing regression_gates")
    for key in distances:
        value = gates.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise BenchmarkError(f"Invalid regression gate: {key}")
    for key, (stat, units) in margins.items():
        value = gates.get(key)
        match = (re.fullmatch(rf"baseline {stat} \+ ([0-9]+(?:\.[0-9]+)?){units}", value)
                 if isinstance(value, str) else None)
        if not match or not math.isfinite(float(match[1])) or (not units and not float(match[1]).is_integer()):
            raise BenchmarkError(f"Invalid baseline-plus-margin regression gate: {key}")
    for key in descriptions:
        if not isinstance(gates.get(key), str) or not gates[key].strip():
            raise BenchmarkError(f"Missing regression rule: {key}")


def read_config(config_path, repo):
    cfg = read_json(config_path)
    if not isinstance(cfg, dict):
        raise BenchmarkError("Recipe must contain a JSON object")
    for key in ("scheme_id", "reference", "hypothesis", "patch", "common_patch", "output_root",
                "ros_setup", "overlay_setup"):
        if not isinstance(cfg.get(key), str) or not cfg[key].strip():
            raise BenchmarkError(f"Required recipe field: {key}")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", cfg["scheme_id"]) or cfg["scheme_id"] == "example":
        raise BenchmarkError("Expected a concrete scheme_id")
    if cfg.get("baseline_ref", "HEAD") in ("", None):
        raise BenchmarkError("Expected a baseline Git reference")
    if not isinstance(cfg.get("baseline_ref", "HEAD"), str):
        raise BenchmarkError("baseline_ref must be a string")
    if type(cfg.get("build_jobs")) is not int or cfg["build_jobs"] != 2:
        raise BenchmarkError("build_jobs must be 2")
    thread_counts = cfg.get("openmp_threads", {})
    if thread_counts and (not isinstance(thread_counts, dict)
                          or set(thread_counts) != set(VARIANTS)
                          or any(type(value) is not int or value < 1 for value in thread_counts.values())):
        raise BenchmarkError("openmp_threads must specify positive baseline and candidate counts")
    if type(cfg.get("repetitions")) is not int or cfg["repetitions"] < 1:
        raise BenchmarkError("repetitions must be a positive integer")
    if type(cfg.get("minimum_successful_pairs", 2)) is not int or cfg.get("minimum_successful_pairs", 2) < 2:
        raise BenchmarkError("minimum_successful_pairs must be an integer of at least 2")
    for key in ("target_ms", "warmup_seconds", "high_load_start_s", "high_load_end_s",
                "post_play_seconds", "surf_filter"):
        value = cfg.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise BenchmarkError(f"{key} must be finite and positive")
    if tuple(cfg[key] for key in ("warmup_seconds", "high_load_start_s", "high_load_end_s", "target_ms")) != (5, 40, 100, 80):
        raise BenchmarkError("The existing evaluator requires 5 s warmup, 40–100 s high-load window and 80 ms target")
    if type(cfg.get("require_p95_target", False)) is not bool:
        raise BenchmarkError("require_p95_target must be a boolean")
    if cfg["post_play_seconds"] != 30 or cfg["surf_filter"] not in (0.1, 0.15):
        raise BenchmarkError("Unsupported drain duration or observer surface filter")
    validate_gates(cfg.get("regression_gates"))
    if type(cfg.get("record_ledger", True)) is not bool:
        raise BenchmarkError("record_ledger must be a boolean")
    approved = cfg.get("allowed_files")
    if (not isinstance(approved, list) or not all(isinstance(x, str) for x in approved)
            or len(set(approved)) != len(approved)):
        raise BenchmarkError("allowed_files must be a nonempty list of distinct paths")
    for name in approved:
        path = Path(name)
        if not (len(path.parts) > len(PACKAGE.parts) + 1
                and path.parts[:len(PACKAGE.parts)] == PACKAGE.parts
                and path.parts[len(PACKAGE.parts)] in {"src", "include"}
                and path.suffix in SOURCE_SUFFIXES - {".cmake", ".py", ".xml", ".yaml", ".yml", ".in"}):
            raise BenchmarkError(f"Candidate may only patch FAST-LIVO2 C/C++ sources: {name}")
    if (not isinstance(cfg.get("bags"), dict)
            or set(cfg["bags"]) not in ({"outdoor"}, {"indoor", "outdoor"})):
        raise BenchmarkError("Recipe must specify an outdoor input bag; indoor is optional")
    for key, item in cfg["bags"].items():
        if not isinstance(item, dict) or not isinstance(item.get("path"), str) or not item["path"]:
            raise BenchmarkError(f"Missing path for {key} bag")
    for key in ("patch", "common_patch", "output_root", "ros_setup", "overlay_setup"):
        cfg[key] = str((repo / cfg[key]).resolve())
    if cfg["ros_setup"] != "/opt/ros/humble/setup.bash" or cfg["overlay_setup"] != str(repo / "ws_livox/install/setup.bash"):
        raise BenchmarkError("Expected the fixed ROS Humble and workspace setup paths")
    if (repo / "experiments").resolve() not in Path(cfg["output_root"]).parents:
        raise BenchmarkError("output_root must be below the workspace experiments directory")
    protected = {"pr465", "runs", "recipes"}
    if Path(cfg["output_root"]).relative_to(repo / "experiments").parts[0] in protected:
        raise BenchmarkError("output_root overlaps protected experiment data")
    return cfg


def preflight(config_path, repo=ROOT):
    cfg = read_config(config_path, repo)
    files = {str(path): digest(path) for path in [config_path, Path(cfg["patch"]), Path(cfg["common_patch"]),
             Path(cfg["ros_setup"]), Path(cfg["overlay_setup"]), Path(__file__).resolve(),
             repo / "scripts/lio_benchmark_report.py", repo / "scripts/summarize_pr465_ab.py", repo / LEGACY_PARSER]}
    patch = Path(cfg["patch"]).read_bytes()
    if (patch_paths(patch, set(cfg["allowed_files"])) if patch.strip() else set()) != set(cfg["allowed_files"]):
        raise BenchmarkError("Candidate patch paths differ from allowed_files")
    common = Path(cfg["common_patch"]).read_bytes()
    if hashlib.sha256(common).hexdigest() != COMMON_PATCH_SHA:
        raise BenchmarkError("Common patch differs from the frozen shutdown patch")
    patch_paths(common, {COMMON_FILE})
    if COMMON_FILE in cfg["allowed_files"]:
        raise BenchmarkError("Candidate cannot change the common shutdown patch file")
    tools = {}
    for name in ("git", "cmake", "bash", "ldd", "c++", "tegrastats"):
        executable = shutil.which(name)
        if executable is None:
            raise BenchmarkError(f"Required executable is missing: {name}")
        path = Path(executable).resolve()
        files[str(path)] = digest(path)
        tools[name] = str(path)
    python = Path(sys.executable).resolve()
    files[str(python)] = digest(python)
    tools["python"] = str(python)
    for file in (Path("/opt/ros/humble/bin/ros2"), Path("/opt/ros/humble/lib/librmw_cyclonedds_cpp.so")):
        files[str(file)] = digest(file)
    bags = {name: bag_identity((repo / item["path"]).resolve()) for name, item in cfg["bags"].items()}
    if (bags["outdoor"]["duration_s"] < cfg["high_load_end_s"]
            or ("indoor" in bags and (
                bags["indoor"]["duration_s"] <= cfg["warmup_seconds"]
                or bags["indoor"]["path"] == bags["outdoor"]["path"]))):
        raise BenchmarkError("Input bag durations or pairing do not match the frozen windows")
    source = source_snapshot(repo, cfg.get("baseline_ref", "HEAD"))
    verify_files(files)
    return cfg, source, files, tools, bags


def create_run(path):
    path.mkdir(parents=True, exist_ok=False)


def guard_mapping():
    for entry in Path("/proc").iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            argv0 = (entry / "cmdline").read_bytes().split(b"\0", 1)[0]
            comm = (entry / "comm").read_text().strip()
        except OSError:
            continue
        if Path(os.fsdecode(argv0)).name == "fastlivo_mapping" or comm == "fastlivo_mappin":
            raise BenchmarkError(f"Existing fastlivo_mapping PID {entry.name}; offline run blocked")


@contextmanager
def run_lock(path):
    flags = os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW
    with os.fdopen(os.open(path, flags, 0o600), "r+") as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise BenchmarkError(f"Another offline LIO command holds {path}") from error
        stream.seek(0)
        stream.truncate()
        stream.write(f"{os.getpid()}\n")
        stream.flush()
        yield


def stop_command(process):
    # Let the existing observer handle SIGINT and flush its own child logs first.
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=45)
        except subprocess.TimeoutExpired:
            pass
    # A failed observer can exit while leaving descendants in its own session.
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait()


def command(attempt, label, argv, *, cwd, env=None):
    argv = [str(arg) for arg in argv]
    started = time.time()
    metadata = attempt / f"{label}.json"
    record = {"argv": argv, "cwd": str(cwd), "start_unix_s": started, "exit_code": None}
    write_json(metadata, record)
    try:
        with (attempt / f"{label}.log").open("xb") as log:
            process = subprocess.Popen(argv, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            try:
                record["exit_code"] = process.wait()
            finally:
                stop_command(process)
    finally:
        if "process" in locals():
            record["exit_code"] = process.returncode
        record["end_unix_s"] = time.time()
        write_json(metadata, record)
    if record["exit_code"] != 0:
        raise BenchmarkError(f"Command {label} exited {record['exit_code']}. See {attempt / (label + '.log')}",
                             record["exit_code"])


class State:
    def __init__(self, root):
        self.root = root
        self.path = root / "state.json"
        self.data = read_json(self.path) if self.path.exists() else {"steps": {}}

    def step(self, name, run):
        steps = self.data["steps"]
        prior = steps.get(name, {"attempts": []})
        if prior.get("status") == "done":
            return prior["result"]
        attempt = self.root / "attempts" / name / f"{len(prior['attempts']) + 1:03d}"
        attempt.mkdir(parents=True, exist_ok=False)
        entry = {"status": "running", "path": str(attempt)}
        prior["attempts"].append(entry)
        prior["status"] = "running"
        steps[name] = prior
        write_json(self.path, self.data)
        try:
            result = run(attempt)
        except (Exception, KeyboardInterrupt, SystemExit) as error:
            entry.update(status="failed", error=str(error) or type(error).__name__)
            prior["status"] = "failed"
            write_json(self.path, self.data)
            raise
        entry["status"] = "done"
        prior.update(status="done", result=result)
        write_json(self.path, self.data)
        return result


def environment(cfg, tools, *argv):
    return [tools["bash"], "--noprofile", "--norc", "-c", SOURCE_ENV, "lio-benchmark",
            cfg["ros_setup"], cfg["overlay_setup"], *argv]


def contract_for(cfg, source, bags):
    contract = {**cfg, "bags": bags, "required_datasets": list(bags),
                "source_sha": source["commit"], "diagnostics": True,
                "ros_domain_id": 176, "localhost_only": True, "replay_rate": 1, "clock_hz": 200,
                "ground_truth": "missing", "rmw": "rmw_cyclonedds_cpp",
                "windows": {"warmup_seconds": cfg["warmup_seconds"],
                            "outdoor_high_load_start_s": cfg["high_load_start_s"],
                            "outdoor_high_load_end_s": cfg["high_load_end_s"]}}
    return contract


def frozen_manifest(frozen, snapshot, repo):
    files = frozen["files"]
    contract = contract_for(frozen["config"], snapshot, frozen["bags"])
    evaluators = (Path("scripts/lio_benchmark_report.py"), Path("scripts/summarize_pr465_ab.py"), LEGACY_PARSER)
    contract["frozen_files_sha256"] = {str(path): files[str(repo / path)] for path in evaluators}
    return {"contract": contract, "source_snapshot": "source/initial_source.json",
            "frozen_files_sha256": files, "machine_identity": frozen["machine_identity"]}


def verify_bag(bag):
    path = Path(bag["path"])
    actual = {item.name for item in path.iterdir() if item.is_file() and not item.is_symlink()}
    if actual != set(bag["sha256"]) or len(list(path.iterdir())) != len(actual):
        raise BenchmarkError(f"Frozen bag file list changed: {path}")
    verify_files({str(path / name): sha for name, sha in bag["sha256"].items()})


def freeze_source(state, cfg, snapshot, repo):
    evidence = state.root / "source"
    evidence.mkdir()
    (evidence / "baseline_dirty.patch").write_bytes(snapshot["dirty"])
    (evidence / "git_status.txt").write_bytes(git(repo, "status", "--porcelain=v1", "--untracked-files=all",
                                                   "--", PACKAGE))
    for key in ("patch", "common_patch"):
        shutil.copyfile(cfg[key], evidence / f"{key}.patch")
    for name in snapshot["untracked"]:
        target = evidence / "untracked" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(repo / name, target)
    write_json(evidence / "initial_source.json", {k: v for k, v in snapshot.items() if k != "dirty"})


def prepare(attempt, cfg, snapshot, repo, evidence):
    isolation = Path(tempfile.mkdtemp(prefix="lio-benchmark-", dir="/tmp"))
    (attempt / "isolation.txt").write_text(str(isolation) + "\n")
    results = {}
    for variant in VARIANTS:
        checkout = isolation / variant
        command(attempt, variant, ["git", "-C", repo, "worktree", "add", "--detach", checkout,
                                   snapshot["commit"]], cwd=repo)
        restore_source(repo, checkout, snapshot)
        apply_patch(checkout, (evidence / "common_patch.patch").read_bytes(), common=True, paths={COMMON_FILE})
        if variant == "candidate":
            apply_patch(checkout, (evidence / "patch.patch").read_bytes(), paths=cfg["allowed_files"])
        tree = checkout / PACKAGE
        if (tree / "Log").exists() or (tree / "Log").is_symlink():
            raise BenchmarkError(f"Expected a clean isolated Log path: {tree}")
        for path in (OBSERVER, Path("config/nav_test.yaml"), Path("config/camera_pinhole.yaml")):
            if not (tree / path).is_file():
                raise BenchmarkError(f"Missing replay helper or config: {tree / path}")
        results[variant] = {"checkout": str(checkout), "files": source_files(checkout)[0]}
        diff = git(checkout, "diff", "--binary", "--no-ext-diff", "--no-color", "HEAD", "--", PACKAGE)
        (attempt / f"{variant}.patch").write_bytes(diff)
    baseline, candidate = (results[name]["files"] for name in VARIANTS)
    delta = {name for name in baseline.keys() | candidate.keys() if baseline.get(name) != candidate.get(name)}
    if delta != set(cfg["allowed_files"]):
        raise BenchmarkError(f"Actual candidate scope differs from recipe: {sorted(delta)}")
    write_json(attempt / "prepared_source.json", results)
    return results


def verify_prepared(prepared):
    for variant, details in prepared.items():
        checkout = Path(details["checkout"])
        if not checkout.is_dir() or source_files(checkout)[0] != details["files"]:
            raise BenchmarkError(f"Frozen {variant} worktree source changed or is missing")


def build(attempt, cfg, tools, checkout):
    tree = checkout / PACKAGE
    path = checkout.parent / f"{checkout.name}-{attempt.name}-build"
    install = checkout.parent / f"{checkout.name}-{attempt.name}-install"
    path.mkdir()
    install.mkdir()
    (attempt / "directories.txt").write_text(f"{path}\n{install}\n")
    for label, args in (
        ("configure", ["-S", tree, "-B", path, "-DENABLE_LIO_DIAGNOSTICS=ON",
                       *([f"-DPROC_NUM={cfg['openmp_threads'][checkout.name]}"]
                         if cfg.get("openmp_threads") else []),
                       f"-DCMAKE_INSTALL_PREFIX={install}", f"-DCMAKE_CXX_COMPILER={tools['c++']}"]),
        ("build", ["--build", path, "--target", "fastlivo_mapping", "-j2"]),
    ):
        command(attempt, label, environment(cfg, tools, tools["cmake"], *args), cwd=path)
    binary = path / "fastlivo_mapping"
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise BenchmarkError(f"Expected compiled executable: {binary}")
    command(attempt, "ldd", environment(cfg, tools, tools["ldd"], binary), cwd=path)
    text = (attempt / "ldd.log").read_text()
    if "not found" in text:
        raise BenchmarkError(f"Unresolved binary dependencies. See {attempt / 'ldd.log'}")
    dependencies = {}
    for line in text.splitlines():
        found = re.search(r"(?:=>\s*|^\s*)(/[^ ]+)\s+\(", line)
        if found:
            dependency = Path(found[1])
            dependencies[str(dependency)] = digest(dependency)
    if not dependencies:
        raise BenchmarkError("Unable to identify dynamic binary dependencies")
    return {"path": str(binary), "sha256": digest(binary), "dependencies": dependencies,
            "cmake_cache_sha256": digest(path / "CMakeCache.txt")}


def verify_binary(binary):
    verify_files({binary["path"]: binary["sha256"], **binary["dependencies"],
                  str(Path(binary["path"]).parent / "CMakeCache.txt"): binary["cmake_cache_sha256"]})


def package_tests(attempt, tools, checkout):
    directory = checkout / PACKAGE / "tests"
    if not (directory / "test_timing_diagnostics.py").is_file():
        raise BenchmarkError(f"Existing diagnostic tests missing: {directory}")
    command(attempt, "tests", [tools["python"], "-B", "-m", "unittest", "discover", "-s", directory, "-v"],
            cwd=checkout)


def machine_identity():
    files = [Path("/etc/nvpmodel.conf"), Path("/var/lib/nvpmodel/status"),
             Path("/sys/devices/system/cpu/online")]
    for expression in ("cpufreq/policy*/scaling_governor", "cpufreq/policy*/scaling_max_freq"):
        files.extend(Path("/sys/devices/system/cpu").glob(expression))
    fields = {}
    for path in files:
        if path.is_file() and os.access(path, os.R_OK):
            fields[str(path)] = path.read_text()
    thread_keys = ("OMP_NUM_THREADS", "OMP_DYNAMIC", "OMP_THREAD_LIMIT", "OMP_PROC_BIND",
                   "OMP_PLACES", "GOMP_CPU_AFFINITY", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")
    return {"uname": list(os.uname()), "cpu_affinity": sorted(os.sched_getaffinity(0)),
            "settings": fields, "inherited_thread_env": {key: os.environ.get(key) for key in thread_keys}}


def verify_machine(expected):
    actual = machine_identity()
    if actual != expected:
        changed = sorted(key for key in actual.keys() | expected.keys() if actual.get(key) != expected.get(key))
        raise BenchmarkError(f"Frozen machine environment changed: {', '.join(changed)}")


def machine_snapshot(identity):
    return {**identity, "load_average": list(os.getloadavg())}


@contextmanager
def telemetry(attempt, output, executable):
    argv = [executable, "--interval", "1000"]
    with (output.parent / f"{output.name}_tegrastats.log").open("xb") as log:
        record = {"argv": argv, "cwd": str(attempt), "exit_code": None}
        try:
            process = subprocess.Popen(argv, cwd=attempt, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            try:
                yield
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                record["exit_code"] = process.returncode
        finally:
            write_json(attempt / "tegrastats.json", record)


def audit_replay(output):
    required = ("commands.json", "outputs.jsonl", "effective_surf_filter.txt", "returncodes.json",
                "observer_counts.json", "run_summary.json", "observer_ready",
                "observer.log", "node.log", "player.log")
    csvs = list(output.glob("timing/run_*/events.csv"))
    if not all((output / name).is_file() for name in required) or len(csvs) != 1:
        raise BenchmarkError(f"Missing observer replay evidence: {output}")
    try:
        codes = read_json(output / "returncodes.json")
        if (not isinstance(codes, dict) or set(codes) != {"observer", "node", "player"}
                or any(type(value) is not int or value != 0 for value in codes.values())):
            raise BenchmarkError(f"Invalid observer returncodes in {output}: {codes}")
        observer = read_json(output / "observer_counts.json")
        clouds, endpoints = observer["cloud_messages"], observer["tf_endpoint_gids"]
        if (type(clouds) is not int or clouds < 0 or not isinstance(endpoints, list)
                or not endpoints or any(not isinstance(gid, str) or not gid for gid in endpoints)):
            raise BenchmarkError(f"Invalid basic observer counts in {output}: {observer}")
        events = set()
        with (output / "outputs.jsonl").open() as stream:
            for line in stream:
                item = json.loads(line)
                if not line.endswith("\n") or not isinstance(item, dict) or item.get("event") not in ("odom", "tf", "path"):
                    raise BenchmarkError(f"Malformed observer output in {output}")
                events.add(item["event"])
        if not {"odom", "tf"} <= events:
            raise BenchmarkError(f"Observer output is missing odometry or TF in {output}")

        with csvs[0].open("rb") as stream:
            stream.seek(0, os.SEEK_END)
            if not stream.tell():
                raise BenchmarkError(f"Empty diagnostic CSV: {csvs[0]}")
            stream.seek(-1, os.SEEK_END)
            if stream.read() != b"\n":
                raise BenchmarkError(f"Unterminated diagnostic CSV row: {csvs[0]}")
        counts = {"full_cycle": 0, "odom_publish": 0, "tf_broadcaster_create": 0}
        with csvs[0].open(newline="") as stream:
            rows = csv.DictReader(stream, strict=True)
            if (not rows.fieldnames or len(set(rows.fieldnames)) != len(rows.fieldnames)
                    or not {"event", "status", "count"} <= set(rows.fieldnames)):
                raise BenchmarkError(f"Invalid diagnostic CSV header: {csvs[0]}")
            for row in rows:
                if None in row or any(value is None or value == "" for value in row.values()):
                    raise BenchmarkError(f"Incomplete diagnostic CSV row {rows.line_num}: {csvs[0]}")
                event = row["event"]
                if event in counts:
                    counts[event] += 1
        summary = read_json(output / "run_summary.json")
        labels = ("full_cycles", "odom_events", "tf_creations")
        if (not isinstance(summary, dict) or not all(counts.values())
                or any(type(summary.get(name)) is not int for name in labels)
                or [summary[name] for name in labels] != list(counts.values())
                or Path(summary["csv"]).resolve() != csvs[0].resolve()):
            raise BenchmarkError(f"Diagnostic CSV does not match observer summary: {csvs[0]}")
    except (OSError, ValueError, TypeError, KeyError, csv.Error) as error:
        raise BenchmarkError(f"Unreadable observer or diagnostic CSV evidence in {output}: {error}") from error


def replay(attempt, cfg, tools, checkout, binary, bag, expected_machine):
    tree = checkout / PACKAGE
    output = attempt / "replay"
    source_logs = tree / "Log"
    # Preserve source logs left by an interrupted earlier attempt before opening a new ROOT_DIR/Log.
    if source_logs.is_symlink() or (source_logs.exists() and not source_logs.is_dir()):
        raise BenchmarkError(f"Unsafe isolated source Log: {source_logs}")
    if source_logs.exists():
        shutil.move(source_logs, attempt / "prior_source_logs")
    source_logs.mkdir()
    guard_mapping()
    verify_binary(binary)
    verify_bag(bag)
    verify_machine(expected_machine)
    write_json(attempt / "environment.json", machine_snapshot(expected_machine))
    argv = [tools["python"], "-B", tree / OBSERVER, "run", "--binary", binary["path"],
            "--out", output, "--bag", bag["path"], "--domain", "176", "--surf-filter",
            cfg["surf_filter"], "--playback-timeout", bag["duration_s"] + 90,
            "--post-play-seconds", cfg["post_play_seconds"]]
    try:
        with telemetry(attempt, output, tools["tegrastats"]):
            command(attempt, "replay", environment(cfg, tools, *argv), cwd=attempt)
    finally:
        if source_logs.is_dir():
            destination = (output if output.is_dir() else attempt) / "source_logs"
            shutil.move(source_logs, destination)
    verify_binary(binary)
    verify_bag(bag)
    verify_machine(expected_machine)
    audit_replay(output)
    evidence = [*output.rglob("*"), output.parent / f"{output.name}_tegrastats.log"]
    return {"path": str(output), "exit_code": 0, "binary_sha256": binary["sha256"],
            "evidence_sha256": {str(file): digest(file) for file in evidence if file.is_file()}}


def generate_report(attempt, root, tools):
    # Evaluate in the attempt directory. Every previously published artifact is preserved on retry.
    manifest = read_json(root / "manifest.json")
    write_json(attempt / "manifest.json", manifest)
    invoke = ("from pathlib import Path; import sys; import lio_benchmark_report as report; "
              "report.write_report(Path(sys.argv[2]), report.analyze(Path(sys.argv[1]), write=False))")
    command(attempt, "report", [tools["python"], "-B", "-c", invoke, root, attempt], cwd=ROOT / "scripts")
    result = read_json(attempt / "verdict.json")
    if result.get("verdict") not in ("ACCEPT", "REJECT", "RETEST"):
        raise BenchmarkError("Report returned an invalid verdict")
    if not all((attempt / name).is_file() and (attempt / name).stat().st_size > 0 for name in REPORT_FILES):
        raise BenchmarkError("Report is missing required output")
    return {**result, "artifacts": {str(attempt / name): digest(attempt / name) for name in REPORT_FILES}}


def publish_report(root, result):
    verify_files(result["artifacts"])
    failure = root / "failure"
    for path, sha in result["artifacts"].items():
        source = Path(path)
        target = root / source.name
        if target.exists():
            placeholder = failure / f"initial-{source.name}"
            if digest(target) == sha:
                continue
            if placeholder.is_file() and digest(target) == digest(placeholder):
                target.unlink()
            else:
                raise BenchmarkError(f"Report output already exists with different contents: {target}")
        temporary = root / f".{source.name}.{os.getpid()}.publish"
        try:
            shutil.copyfile(source, temporary)
            os.link(temporary, target)
        finally:
            temporary.unlink(missing_ok=True)


def record_ledger(repo, cfg, root, verdict, stage, *, final):
    if not cfg.get("record_ledger", True):
        return
    ledger = repo / "plans/latency_schemes.md"
    marker = f"<!-- lio-benchmark:{root}:{'complete' if final else 'failed'} -->"
    with ledger.open("r+", encoding="utf-8") as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        if marker in stream.read():
            return
        details = (f"mean≤{cfg['target_ms']:g} ms={verdict.get('performance_mean_pass')}, "
                   f"p95≤{cfg['target_ms']:g} ms={verdict.get('performance_p95_pass')}" if final else "未执行完毕")
        link = "REPORT.md" if final else "failure/REPORT.md"
        path = root.relative_to(repo)
        stream.seek(0, os.SEEK_END)
        stream.write(f"\n\n{marker}\n### 方案 {cfg['scheme_id']} 自动化记录\n\n"
                     f"- 来源：{cfg['reference']}\n- 假设：{cfg['hypothesis']}\n"
                     f"- 运行目录：`{path}`\n- 阶段：{stage}\n"
                     f"- 性能：{details}\n- 判定：{verdict['verdict']}。"
                     f"[报告](../{path / link})\n")
        stream.flush()
        os.fsync(stream.fileno())


def failure_report(state, cfg, repo, error):
    failure = state.root / "failure"
    failure.mkdir(exist_ok=True)
    number = len(list(failure.glob("attempt_*.json"))) + 1
    reason = str(error) or type(error).__name__
    stage = state.data.get("active_stage", "initialization")
    result = {"verdict": "RETEST", "stage": stage, "reasons": [reason]}
    write_json(failure / f"attempt_{number:03d}.json", result)
    write_json(failure / "verdict.json", result)
    (failure / "REPORT.md").write_text(
        f"# FAST-LIVO2 离线运行失败\n\n判定 **RETEST**。\n\n阶段 `{stage}`。\n\n{reason}\n")
    if not (state.root / "REPORT.md").exists():
        for name in ("REPORT.md", "verdict.json"):
            target = state.root / name
            if not target.exists():
                shutil.copyfile(failure / name, failure / f"initial-{name}")
                shutil.copyfile(failure / name, target)
    try:
        record_ledger(repo, cfg, state.root, result, stage, final=False)
    except OSError as ledger_error:
        print(f"Ledger update failed: {ledger_error}", file=sys.stderr)


def replay_order(cfg):
    for repeat in range(1, cfg["repetitions"] + 1):
        order = VARIANTS if repeat % 2 else VARIANTS[::-1]
        for dataset in ("indoor", "outdoor"):
            if dataset not in cfg.get("bags", ("indoor", "outdoor")):
                continue
            for variant in order:
                yield f"{variant}_{dataset}_r{repeat}", variant, dataset, repeat


def verified_replay(state, name, variant, dataset, repeat):
    stage = state.data["steps"].get(f"replay-{name}", {})
    if stage.get("status") != "done":
        return None
    result = stage["result"]
    path = Path(result["path"])
    attempt = Path(stage["attempts"][-1]["path"])
    if (result.get("exit_code") != 0 or not result.get("evidence_sha256")
            or path.resolve() != attempt / "replay" or not path.is_relative_to(state.root)):
        raise BenchmarkError(f"Invalid completed replay in state: {name}")
    verify_files(result["evidence_sha256"])
    return {**result, "variant": variant, "dataset": dataset, "repeat": repeat,
            "path": str(path.relative_to(state.root))}


def verify_manifest(state, manifest, frozen, snapshot, repo):
    expected = frozen_manifest(frozen, snapshot, repo)
    for key, value in expected.items():
        if manifest.get(key) != value:
            raise BenchmarkError(f"Frozen manifest {key} changed")
    recorded = manifest.get("runs")
    if not isinstance(recorded, dict):
        raise BenchmarkError("Invalid manifest runs")
    runs = {}
    for args in replay_order(frozen["config"]):
        result = verified_replay(state, *args)
        if result is not None:
            runs[args[0]] = result
    for name, record in recorded.items():
        if name not in runs or runs[name] != record:
            raise BenchmarkError(f"Manifest replay record differs from verified state: {name}")
    return runs


def run_pipeline(state, frozen, snapshot, repo, *, prepare_only=False):
    cfg, tools, bags, expected_machine = (frozen[key] for key in ("config", "tools", "bags", "machine_identity"))
    manifest_path = state.root / "manifest.json"
    if state.data.get("complete"):
        raise BenchmarkError(f"Run already completed: {state.root}")
    try:
        manifest = read_json(manifest_path)
        verify_machine(expected_machine)
        manifest["runs"] = verify_manifest(state, manifest, frozen, snapshot, repo)

        def step(name, callback):
            state.data["active_stage"] = name
            write_json(state.path, state.data)
            return state.step(name, callback)

        evidence = state.root / "source"
        prepared = step("prepare", lambda attempt: prepare(attempt, cfg, snapshot, repo, evidence))
        verify_prepared(prepared)
        manifest["prepared_sources"] = prepared
        manifest["configs_sha256"] = {
            str(path): prepared["baseline"]["files"][str(PACKAGE / path)]["sha256"]
            for path in (OBSERVER, Path("config/nav_test.yaml"), Path("config/camera_pinhole.yaml"))}
        write_json(manifest_path, manifest)
        if prepare_only:
            print(f"Prepared {state.root}")
            return 0

        binaries = {}
        for variant in VARIANTS:
            checkout = Path(prepared[variant]["checkout"])
            binaries[variant] = step(f"build-{variant}", lambda attempt: build(attempt, cfg, tools, checkout))
            verify_binary(binaries[variant])
        if binaries["baseline"]["dependencies"] != binaries["candidate"]["dependencies"]:
            raise BenchmarkError("Baseline and candidate have different dynamic dependencies")
        manifest["binaries"] = binaries
        write_json(manifest_path, manifest)
        for variant in VARIANTS:
            checkout = Path(prepared[variant]["checkout"])
            step(f"tests-{variant}", lambda attempt: package_tests(attempt, tools, checkout))

        for bag in bags.values():
            verify_bag(bag)
        manifest["bag_hashes_verified"] = True
        write_json(manifest_path, manifest)
        for name, variant, dataset, repeat in replay_order(cfg):
            verify_machine(expected_machine)
            verify_prepared(prepared)
            checkout = Path(prepared[variant]["checkout"])
            step(f"replay-{name}", lambda attempt: replay(
                attempt, cfg, tools, checkout, binaries[variant], bags[dataset], expected_machine))
            manifest["runs"][name] = verified_replay(state, name, variant, dataset, repeat)
            write_json(manifest_path, manifest)
        result = step("report", lambda attempt: generate_report(attempt, state.root, tools))
        step("publish", lambda attempt: publish_report(state.root, result))
        step("ledger", lambda attempt: record_ledger(repo, cfg, state.root, result, "report", final=True))
        state.data.update(complete=True, verdict=result["verdict"], active_stage=None)
        write_json(state.path, state.data)
        print(f"{result['verdict']} {state.root}")
        return 1 if result["verdict"] == "RETEST" else 0
    except (Exception, KeyboardInterrupt, SystemExit) as error:
        failure_report(state, cfg, repo, error)
        raise


def start(config_path, *, prepare_only, preflight_only, repo=ROOT):
    if preflight_only:
        guard_mapping()
        cfg, snapshot, _, _, bags = preflight(config_path, repo)
        print(json.dumps({"status": "preflight_ok", "scheme_id": cfg["scheme_id"],
                          "source_sha": snapshot["commit"], "bags": bags}, indent=2))
        return 0
    lock = Path("/tmp") / f"fastlivo-benchmark-{os.getuid()}.lock"
    with run_lock(lock):
        guard_mapping()
        cfg, snapshot, files, tools, bags = preflight(config_path, repo)
        root = Path(cfg["output_root"]) / cfg["scheme_id"]
        create_run(root)
        state = State(root)
        try:
            freeze_source(state, cfg, snapshot, repo)
            for path in (root / "source").rglob("*"):
                if path.is_file():
                    files[str(path)] = digest(path)
            identity = machine_identity()
            frozen = {"config": cfg, "config_path": str(config_path), "files": files,
                      "tools": tools, "bags": bags, "machine_identity": identity}
            write_json(root / "frozen.json", frozen)
            write_json(root / "manifest.json", {**frozen_manifest(frozen, snapshot, repo),
                                                "environment": machine_snapshot(identity), "runs": {}})
        except (Exception, KeyboardInterrupt, SystemExit) as error:
            failure_report(state, cfg, repo, error)
            raise
        return run_pipeline(state, frozen, snapshot, repo, prepare_only=prepare_only)


def resume(root, repo=ROOT):
    lock = Path("/tmp") / f"fastlivo-benchmark-{os.getuid()}.lock"
    with run_lock(lock):
        guard_mapping()
        state = State(root)
        if state.data.get("complete"):
            raise BenchmarkError(f"Run already completed: {root}")
        frozen = read_json(root / "frozen.json")
        cfg = frozen["config"]
        try:
            if root != Path(cfg["output_root"]) / cfg["scheme_id"]:
                raise BenchmarkError("Run has moved since its paths were frozen")
            verify_files(frozen["files"])
            if cfg != read_config(Path(frozen["config_path"]), repo):
                raise BenchmarkError("Run recipe differs from the frozen configuration")
            verify_machine(frozen["machine_identity"])
            for bag in frozen["bags"].values():
                verify_bag(bag)
            current = source_snapshot(repo, cfg.get("baseline_ref", "HEAD"))
            snapshot = read_json(root / "source/initial_source.json")
            snapshot["dirty"] = (root / "source/baseline_dirty.patch").read_bytes()
            if snapshot != current:
                raise BenchmarkError("Live FAST-LIVO2 source changed since the run was frozen")
            for key, stage in state.data["steps"].items():
                if stage.get("status") == "done":
                    if key.startswith("build-"):
                        verify_binary(stage["result"])
        except (Exception, KeyboardInterrupt, SystemExit) as error:
            failure_report(state, cfg, repo, error)
            raise
        return run_pipeline(state, frozen, snapshot, repo)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    run = modes.add_parser("run")
    run.add_argument("--config", type=Path, required=True)
    options = run.add_mutually_exclusive_group()
    options.add_argument("--prepare-only", action="store_true")
    options.add_argument("--preflight-only", action="store_true")
    modes.add_parser("preflight").add_argument("--config", type=Path, required=True)
    modes.add_parser("resume").add_argument("run_dir", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.mode == "resume":
            return resume(args.run_dir.resolve())
        return start(args.config.resolve(), prepare_only=getattr(args, "prepare_only", False),
                     preflight_only=args.mode == "preflight" or args.preflight_only)
    except (BenchmarkError, OSError, ValueError, KeyError, sqlite3.Error) as error:
        print(f"RETEST: {error}", file=sys.stderr)
        return getattr(error, "exit_code", 1)
    except KeyboardInterrupt:
        print("RETEST: interrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":
    def on_terminate(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_terminate)
    sys.exit(main())
