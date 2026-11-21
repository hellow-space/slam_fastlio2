#!/usr/bin/env python3
"""PR465 entry point; reuse existing scan matching/statistics, expose the 80ms gate.

No change to existing parser or percentile definition. Inputs are actual replay
directories; this module does not supply example measurements or ground truth.
"""
import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PARSER = ROOT / 'ws_livox/src/FAST-LIVO2-master/scripts/summarize_surf_filter_ab.py'


def audit_recording(directory):
    odom = []
    successful_cycles = 0
    with next((directory / 'timing').glob('run_*/events.csv')).open() as f:
        for row in csv.DictReader(f):
            if row['event'] == 'odom_publish':
                odom.append({k: float(row[k]) for k in ['start_steady_s', 'raw_stamp_s', 'scan_end_s', 'age_s']})
            if row['event'] == 'full_cycle' and row['status'] == '1':
                successful_cycles += 1
    observed = [json.loads(line) for line in (directory / 'outputs.jsonl').read_text().splitlines()]
    observed = [row for row in observed if row['event'] == 'odom']
    ordered = sorted(odom, key=lambda row: row['start_steady_s'])
    if len(ordered) != len(observed) or successful_cycles != len(ordered):
        raise RuntimeError('Successful cycle / diagnostic / observer counts disagree')
    if any(not math.isclose(a['raw_stamp_s'], b['stamp_s'], rel_tol=0, abs_tol=2e-6) for a, b in zip(ordered, observed)):
        raise RuntimeError('Observer order does not match recorded odom headers')
    keys = [round(row['scan_end_s'] * 1e6) for row in ordered]
    if len(keys) != len(set(keys)):
        raise RuntimeError('Duplicate scan-end timestamp in full odometry output')
    return {'successful_cycles': successful_cycles, 'observed_odom': len(observed),
            'negative_raw_state_ages': sum(row['age_s'] < 0 for row in ordered),
            'nonfinite_raw_state_ages': sum(not math.isfinite(row['age_s']) for row in ordered),
            'diagnostic_header_observer_order_verified': True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('out', type=Path)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('existing_surf_summary', PARSER)
    existing = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(existing)
    manifest = json.loads((args.out / 'manifest.json').read_text())
    report = {'definition': manifest['contract'], 'datasets': {}, 'accuracy': 'No ground truth; differences are not accuracy.'}
    flat = []
    for dataset in ['indoor', 'outdoor']:
        bag = manifest['contract']['bags'][dataset]
        start = bag['start_s']
        runs = {v: existing.load(args.out / (v + '_' + dataset)) for v in ['baseline', 'candidate']}
        index, audits = {}, {}
        for variant, run in runs.items():
            audits[variant] = audit_recording(args.out / (variant + '_' + dataset))
            selected = [(fid, row) for fid, row in run['frames'].items() if row['scan_end_s'] >= start + 5]
            index[variant] = {round(row['scan_end_s'] * 1e6): fid for fid, row in selected}
            if len(index[variant]) != len(selected):
                raise RuntimeError('Duplicate rounded scan-end timestamp: ' + variant + '_' + dataset)
        common = set(index['baseline']) & set(index['candidate'])
        if not common:
            raise RuntimeError('No matched valid scans: ' + dataset)
        if set(index['baseline']) != set(index['candidate']):
            missing = {v: len(set(index[v]) - common) for v in index}
            raise RuntimeError('Processed source scans differ: ' + dataset + ' exclusive_counts=' + json.dumps(missing))
        group = {'common_scans': len(common),
                 'baseline_only_scans': len(set(index['baseline']) - common),
                 'candidate_only_scans': len(set(index['candidate']) - common), 'windows': {}, 'checks': {}}
        for variant, run in runs.items():
            assert run['returncodes'] == {'observer': 0, 'node': 0, 'player': 0}
            assert run['counts']['imu_callback'] == bag['input_counts']['/livox/imu']
            assert run['counts']['lidar_callback'] == bag['input_counts']['/livox/lidar']
            assert run['tf_creations'] == 1
            # A final scan can wait for an IMU sample beyond EOF. Keep this
            # explicit instead of hiding any common losses with scan matching.
            assert run['counts']['full_cycle'] >= bag['input_counts']['/livox/lidar'] - 1
            group['checks'][variant] = {k: run[k] for k in ['counts', 'imu_statuses', 'tf_creations',
                'tf_odom_position_difference_m', 'tf_odom_sign_equivalent_quat_difference',
                'cpu_temperature_c', 'cpu_frequency_mhz_samples', 'clock_mapping_offset_deviation_ms']}
            group['checks'][variant]['expected_raw_inputs'] = bag['input_counts']
            group['checks'][variant]['raw_recording_audit'] = audits[variant]
        windows = {'all_after_5s': common}
        if dataset == 'outdoor':
            windows['high_load_40_100s'] = {k for k in common if start + 40 <= k / 1e6 < start + 100}
        for window, keys in windows.items():
            if not keys:
                raise RuntimeError('Empty source-time window: ' + window)
            result = {}
            for variant, run in runs.items():
                selected = [run['frames'][index[variant][k]] for k in sorted(keys)]
                s = existing.summarize(run, keys)
                durations = [r['duration_ms'] for r in selected]
                s['cycles_over_80ms'] = sum(v > 80 for v in durations)
                s['fraction_over_80ms'] = s['cycles_over_80ms'] / len(durations)
                s['mean_le_80ms'] = s['stages_ms']['full_cycle']['mean'] <= 80
                s['p95_le_80ms'] = s['stages_ms']['full_cycle']['p95'] <= 80
                result[variant] = s
                for row in selected:
                    flat.append({'dataset': dataset, 'window': window, 'variant': variant,
                                 'scan_end_s': row['scan_end_s'], 'relative_source_s': row['scan_end_s'] - start,
                                 'full_cycle_ms': row['duration_ms'], 'state_age_ms': row['publish_age_ms'],
                                 'wall_mapped_state_age_ms': row['wall_mapped_publish_age_ms'], 'lidar_q': row['lidar_q']})
            position, orientation = [], []
            for key in sorted(keys):
                a, b = (runs[v]['poses'][key] for v in ['baseline', 'candidate'])
                position.append(math.sqrt(sum((x-y)**2 for x, y in zip(a['pos'], b['pos']))))
                direct = max(abs(x-y) for x, y in zip(a['quat'], b['quat']))
                opposite = max(abs(x+y) for x, y in zip(a['quat'], b['quat']))
                orientation.append(min(direct, opposite))
            result['paired_output_difference'] = {'position_m': existing.stats(position),
                                                   'quaternion_component_sign_equivalent': existing.stats(orientation)}
            result['mean_change_ms'] = result['candidate']['stages_ms']['full_cycle']['mean'] - result['baseline']['stages_ms']['full_cycle']['mean']
            group['windows'][window] = result
        report['datasets'][dataset] = group
    (args.out / 'comparison.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    with (args.out / 'matched_frames.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(flat[0]))
        writer.writeheader()
        writer.writerows(flat)
    compact = {d: {w: {v: {k: s[v]['stages_ms']['full_cycle'][k] for k in ['count', 'mean', 'p50', 'p95', 'max']}
                       for v in ['baseline', 'candidate']} for w, s in x['windows'].items()}
               for d, x in report['datasets'].items()}
    print(json.dumps(compact, indent=2), flush=True)


if __name__ == '__main__':
    main()
