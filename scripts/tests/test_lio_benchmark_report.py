"""Synthetic evidence exercises report logic without ROS or benchmark runs."""
import csv
from hashlib import sha256
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

try:
    from scripts.lio_benchmark_report import analyze
except ModuleNotFoundError as error:
    if error.name != 'scripts.lio_benchmark_report':
        raise
    analyze = None


START = 2_000_000_000
GATES = {
    'tf_odom_position_max_m': 1e-12,
    'tf_odom_quaternion_component_max': 1e-12,
    'paired_pose_max_m': 1e-9,
    'paired_quaternion_component_max': 1e-12,
    'outdoor_high_load_wall_mapped_age_p95_candidate_max': 'baseline p95 + 5 ms',
    'outdoor_high_load_lidar_q_max_candidate_max': 'baseline max + 1',
    'wall_output_interval_p95_candidate_max': 'baseline p95 + 5 ms',
}


def write_run(root, durations, *, scan_offset, reject=False, duplicate=False, missing=False, nan_header=False):
    rows, outputs = [], []
    defaults = dict(frame_id=0, status=1)
    for key in ('scan_end_s', 'target_end_s', 'raw_stamp_s', 'corrected_stamp_s', 'age_s',
                'header_age_s', 'corrected_header_age_s', 'ros_s', 'callback_gap_ms'):
        defaults[key] = 'nan'
    for key in ('imu_q', 'lidar_q', 'path_poses', 'down_points', 'input_points',
                'effective_points', 'iterations', 'map_voxels'):
        defaults[key] = 0
    defaults.update(count=1, duration_ms=0, start_steady_s=0)
    rows.append(dict(defaults, event='tf_broadcaster_create'))
    for i in range(20):
        rows.append(dict(defaults, event='imu_callback', status=-2 if reject and i == 0 else 1,
                         start_steady_s=3+i/10, ros_s=START+3+i/10, raw_stamp_s=START+i/10))
    for index, duration in enumerate(durations):
        frame = index + 1
        stamp = START + scan_offset + (0 if duplicate and index == 1 else index)
        steady = scan_offset + .25 + index
        rows.append(dict(defaults, event='lidar_callback', raw_stamp_s=START+scan_offset-.1+index,
                         corrected_stamp_s=START+scan_offset-.1+index))
        for stage in ('sync_before', 'sync_after'):
            rows.append(dict(defaults, event=stage, frame_id=frame, target_end_s=stamp))
        if missing and index == 1:
            continue
        for stage in ('full_cycle', 'process_imu', 'state_estimation', 'voxel_map_update', 'frame_period', 'odom_publish'):
            rows.append(dict(defaults, event=stage, frame_id=frame, scan_end_s=stamp, target_end_s=stamp,
                             start_steady_s=steady, duration_ms=duration, ros_s=stamp+.25, age_s=.25,
                             raw_stamp_s=stamp+.25))
        for event in ('odom', 'tf'):
            outputs.append(dict(event=event, steady_s=steady, parent='camera_init', child='aft_mapped',
                                stamp_s=float('nan') if nan_header and event == 'tf' else stamp+.25,
                                pos=[0, 0, 0], quat=[0, 0, 0, 1]))
    timing = root / 'timing' / 'run_fixture'
    timing.mkdir(parents=True)
    with (timing / 'events.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (root / 'outputs.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in outputs))
    (root / 'effective_surf_filter.txt').write_text('0.15')
    (root / 'returncodes.json').write_text(json.dumps(dict(observer=0, node=0, player=0)))
    (root / 'commands.json').write_text(json.dumps({'node': ['synthetic-node']}))
    (root / 'observer_counts.json').write_text(json.dumps({'odom': len(durations), 'tf': len(durations)}))
    (root / 'run_summary.json').write_text(json.dumps({'fixture': True}))
    telemetry = root.parent / (root.name + '_tegrastats.log')
    telemetry.write_text('cpu@40.0C 10%@1000')
    return {str(path): sha256(path.read_bytes()).hexdigest()
            for path in (*root.rglob('*'), telemetry) if path.is_file()}


class ReportTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(callable(analyze), 'report analyze has not been implemented')
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)

    def fixture(self, *, candidate=(50, 80, 100), repeat_count=1, both=None, candidate_options=None,
                gates=None, datasets=('indoor', 'outdoor')):
        runs = {}
        for repeat in range(1, repeat_count+1):
            for dataset in datasets:
                for variant, durations in [('baseline', (90, 105, 110)), ('candidate', candidate)]:
                    name = f'{variant}_{dataset}_{repeat}'
                    options = candidate_options if variant == 'candidate' and candidate_options else {}
                    hashes = write_run(self.root / name, durations,
                                       scan_offset=7 if dataset == 'indoor' else 41, **(both or options))
                    runs[name] = dict(variant=variant, dataset=dataset, repeat=repeat, path=name,
                                      exit_code=0, binary_sha256=('a' if variant == 'baseline' else 'b')*64,
                                      evidence_sha256=hashes)
        manifest = dict(bag_hashes_verified=True, runs=runs, contract=dict(
            bags={dataset: dict(path='unused synthetic', start_s=START,
                                duration_s=26 if dataset == 'indoor' else 120,
                                input_counts={'/livox/imu': 20, '/livox/lidar': 3},
                                sha256={'synthetic': 'a'*64}) for dataset in datasets},
            warmup_seconds=5, high_load_start_s=40, high_load_end_s=100, target_ms=80,
            replay_rate=1, clock_hz=200, regression_gates=GATES if gates is None else gates))
        (self.root / 'manifest.json').write_text(json.dumps(manifest))

    def test_strict_thresholds_and_independent_p95_result(self):
        self.fixture()
        result = analyze(self.root)
        self.assertTrue(result['performance_mean_pass'])
        self.assertFalse(result['performance_p95_pass'])
        window = result['datasets']['outdoor']['pairs']['1']['windows']['high_load_40_100s']
        self.assertEqual(window['candidate']['cycles_over_80ms'], 1)
        self.assertEqual(window['candidate']['cycles_over_100ms'], 0)
        self.assertEqual(window['candidate']['scans'], 3)
        self.assertIn('0/3', (self.root / 'REPORT.md').read_text())

    def test_nan_header_fails_closed(self):
        self.fixture(candidate_options={'nan_header': True})
        result = analyze(self.root, write=False)
        self.assertEqual(result['verdict'], 'RETEST')
        self.assertIsNone(result['performance_mean_pass'])
        self.assertIn('stamp_s', ' '.join(result['reasons']))

    def test_duplicate_scan_rejected(self):
        self.fixture(both={'duplicate': True})
        result = analyze(self.root, write=False)
        self.assertEqual(result['verdict'], 'RETEST')
        self.assertIn('duplicate', ' '.join(result['reasons']).lower())

    def test_common_missing_scan_rejected(self):
        self.fixture(both={'missing': True})
        result = analyze(self.root, write=False)
        self.assertEqual(result['verdict'], 'RETEST')
        self.assertIsNone(result['performance_mean_pass'])
        self.assertIn('scan', ' '.join(result['reasons']))

    def test_startup_difference_requires_retest_after_repeats(self):
        self.fixture(repeat_count=2, candidate=(50, 60, 70), candidate_options={'reject': True})
        result = analyze(self.root, write=False)
        self.assertEqual(result['verdict'], 'RETEST')
        self.assertTrue(result['performance_mean_pass'])
        self.assertIn('initialization', ' '.join(result['reasons']))

    def test_complete_comparable_repeats_can_pass(self):
        self.fixture(repeat_count=2, candidate=(50, 60, 70))
        result = analyze(self.root, write=False)
        self.assertEqual(result['verdict'], 'ACCEPT', result['reasons'])
        self.assertTrue(result['repeatability_verified'])
        self.assertEqual(result['datasets']['indoor']['successful_pairs'], 2)
        self.assertEqual(result['datasets']['outdoor']['successful_pairs'], 2)

    def test_outdoor_only_measurements_cannot_pass_adoption(self):
        self.fixture(repeat_count=2, candidate=(50, 60, 70), datasets=('outdoor',))
        result = analyze(self.root, write=False)
        self.assertEqual(result['verdict'], 'RETEST')
        self.assertTrue(result['performance_mean_pass'])
        self.assertTrue(result['performance_p95_pass'])
        self.assertIn('indoor', ' '.join(result['reasons']))

    def test_explicit_outdoor_only_contract_does_not_require_indoor(self):
        self.fixture(repeat_count=2, candidate=(50, 60, 70), datasets=('outdoor',))
        path = self.root / 'manifest.json'
        manifest = json.loads(path.read_text())
        manifest['contract']['required_datasets'] = ['outdoor']
        path.write_text(json.dumps(manifest))
        result = analyze(self.root, write=False)
        self.assertEqual(result['verdict'], 'ACCEPT', result['reasons'])
        self.assertEqual(set(result['datasets']), {'outdoor'})
        self.assertTrue(result['repeatability_verified'])

    def test_dataset_scope_cannot_hide_required_indoor_runs(self):
        self.fixture(repeat_count=2, candidate=(50, 60, 70))
        path = self.root / 'manifest.json'
        manifest = json.loads(path.read_text())
        manifest['contract']['required_datasets'] = ['outdoor']
        path.write_text(json.dumps(manifest))
        result = analyze(self.root, write=False)
        self.assertEqual(result['verdict'], 'RETEST')
        self.assertIn('dataset scope', ' '.join(result['reasons']))

    def test_valid_csv_duration_modified_after_freeze_is_rejected(self):
        self.fixture()
        path = self.root / 'candidate_outdoor_1' / 'timing' / 'run_fixture' / 'events.csv'
        with path.open(newline='') as stream:
            reader = csv.DictReader(stream)
            fields, rows = reader.fieldnames, list(reader)
        next(row for row in rows if row['event'] == 'full_cycle')['duration_ms'] = '40.5'
        with path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        result = analyze(self.root, write=False)
        self.assertEqual(result['verdict'], 'RETEST')
        self.assertIsNone(result['performance_mean_pass'])
        self.assertIn('frozen evidence SHA-256 mismatch', ' '.join(result['reasons']))
        self.assertNotIn('windows', result['datasets']['outdoor']['pairs']['1'])

    def test_manifest_must_freeze_all_required_evidence_files(self):
        self.fixture()
        manifest_path = self.root / 'manifest.json'
        manifest = json.loads(manifest_path.read_text())
        commands = self.root / 'candidate_outdoor_1' / 'commands.json'
        del manifest['runs']['candidate_outdoor_1']['evidence_sha256'][str(commands)]
        manifest_path.write_text(json.dumps(manifest))
        result = analyze(self.root, write=False)
        self.assertEqual(result['verdict'], 'RETEST')
        self.assertIsNone(result['performance_mean_pass'])
        self.assertIn('missing frozen evidence SHA-256', ' '.join(result['reasons']))

    def test_negative_or_nonfinite_processing_is_invalid(self):
        self.fixture(candidate=(float('nan'), -1, 65))
        result = analyze(self.root, write=False)
        self.assertEqual(result['verdict'], 'RETEST')
        self.assertIsNone(result['performance_mean_pass'])
        self.assertIn('duration_ms', ' '.join(result['reasons']))

    def test_unfrozen_regression_threshold_requires_retest(self):
        self.fixture(repeat_count=2, candidate=(50, 60, 70), gates={})
        result = analyze(self.root, write=False)
        self.assertEqual(result['verdict'], 'RETEST')
        self.assertTrue(result['performance_mean_pass'])
        self.assertIn('frozen regression threshold', ' '.join(result['reasons']))

    def test_failed_exit_is_not_treated_as_measurement(self):
        self.fixture()
        manifest_path = self.root / 'manifest.json'
        manifest = json.loads(manifest_path.read_text())
        manifest['runs']['candidate_outdoor_1']['exit_code'] = 1
        manifest_path.write_text(json.dumps(manifest))
        result = analyze(self.root, write=False)
        self.assertEqual(result['verdict'], 'RETEST')
        self.assertIsNone(result['performance_mean_pass'])
        self.assertIn('did not exit successfully', ' '.join(result['reasons']))

    def test_missing_planned_repeat_requires_retest(self):
        self.fixture(repeat_count=2, candidate=(50, 60, 70))
        manifest_path = self.root / 'manifest.json'
        manifest = json.loads(manifest_path.read_text())
        manifest['contract']['repetitions'] = 3
        manifest_path.write_text(json.dumps(manifest))
        result = analyze(self.root, write=False)
        self.assertEqual(result['verdict'], 'RETEST')
        self.assertIn('scheduled repeats', ' '.join(result['reasons']))

    def test_malformed_manifest_exports_retest(self):
        (self.root / 'manifest.json').write_text('{"contract": {"target_ms": NaN}}')
        result = analyze(self.root)
        self.assertEqual(result['verdict'], 'RETEST')
        self.assertIn('NaN', ' '.join(result['reasons']))
        self.assertTrue((self.root / 'verdict.json').is_file())

    def test_existing_historical_comparison_is_protected(self):
        comparison = self.root / 'comparison.json'
        comparison.write_text('historical evidence')
        with self.assertRaisesRegex(ValueError, 'already exists'):
            analyze(self.root)
        self.assertEqual(comparison.read_text(), 'historical evidence')

    def test_missing_retest_has_no_fake_accuracy(self):
        result = analyze(self.root)
        self.assertEqual(result['verdict'], 'RETEST')
        self.assertEqual(result['datasets'], {})
        self.assertEqual(result['absolute_accuracy']['availability'], 'missing')
        with (self.root / 'metrics.csv').open() as file:
            rows = list(csv.DictReader(file))
        self.assertFalse(any(row['availability'] == 'measured' for row in rows))
        accuracy = [row for row in rows if row['metric'] == 'absolute_accuracy']
        self.assertEqual(accuracy[0]['value'], '')


if __name__ == '__main__':
    unittest.main()
