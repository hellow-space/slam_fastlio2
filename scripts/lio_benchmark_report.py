#!/usr/bin/env python3
"""Audit and report frozen FAST-LIVO2 A/B replay evidence without launching ROS."""
import argparse
from collections import Counter, defaultdict
import csv
from hashlib import sha256
import importlib.util
import json
import math
from pathlib import Path
import re

if __package__:
    from . import summarize_pr465_ab as legacy_audit
else:
    import summarize_pr465_ab as legacy_audit


VARIANTS = ('baseline', 'candidate')
REQUIRED_DATASETS = ('indoor', 'outdoor')
REPORT_FILES = ('comparison.json', 'REPORT.md', 'verdict.json', 'metrics.csv')
METRIC_COLUMNS = ('dataset_id', 'pair_id', 'variant', 'event', 'event_status', 'metric',
                  'statistic', 'value', 'unit', 'sample_count', 'evidence_path',
                  'availability', 'missing_reason')
CORE_STAGES = ('process_imu', 'state_estimation', 'voxel_map_update', 'odom_publish')
FRAME = ('camera_init', 'aft_mapped')
HASH = re.compile(r'[0-9a-f]{64}')
ERRORS = (AssertionError, ArithmeticError, csv.Error, ImportError, KeyError,
          OSError, RuntimeError, StopIteration, TypeError, ValueError)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def finite(value, field, *, nonnegative=True):
    parsed = float(value)
    require(math.isfinite(parsed) and (not nonnegative or parsed >= 0),
            f'invalid {field}: {value}')
    return parsed


def valid_hash(value):
    return isinstance(value, str) and HASH.fullmatch(value) is not None


def scan_key(value):
    return round(finite(value, 'scan_end_s') * 1e6)


def digest(path):
    hasher = sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            hasher.update(block)
    return hasher.hexdigest()


def digest_values(values):
    return sha256('\n'.join(str(value) for value in values).encode()).hexdigest()


def read_json(path):
    def parse_number(text):
        return finite(text, f'{path.name} JSON number', nonnegative=False)

    def unique_fields(pairs):
        result = dict(pairs)
        require(len(result) == len(pairs), f'duplicate JSON field in {path.name}')
        return result

    value = json.loads(path.read_text(), parse_float=parse_number, parse_constant=parse_number,
                       object_pairs_hook=unique_fields)
    require(isinstance(value, dict), f'expected a JSON object in {path.name}')
    return value


def verify_recording_evidence(directory, diagnostics, frozen):
    required = (
        diagnostics,
        *(directory / filename for filename in ('outputs.jsonl', 'returncodes.json', 'commands.json',
                                                'effective_surf_filter.txt', 'observer_counts.json')),
        directory.with_name(directory.name + '_tegrastats.log'),
    )
    verified = {}
    for path in required:
        key = str(path)
        expected = frozen.get(key)
        require(valid_hash(expected), f'missing frozen evidence SHA-256 for {path}')
        require(path.is_file() and path.resolve() == path, f'missing or unsafe frozen evidence file {path}')
        actual = digest(path)
        require(actual == expected, f'frozen evidence SHA-256 mismatch for {path}')
        verified[key] = actual
    return verified


def read_recording(directory, bag, start, warmup, parser, *, frozen_evidence=None):
    paths = list((directory / 'timing').glob('run_*/events.csv'))
    require(len(paths) == 1, f'expected one events.csv, found {len(paths)}')
    diagnostics = paths[0]
    observed_path = directory / 'outputs.jsonl'
    verified_evidence = (verify_recording_evidence(directory, diagnostics, frozen_evidence)
                         if frozen_evidence is not None else None)
    returncodes = read_json(directory / 'returncodes.json')
    require(set(returncodes) == {'observer', 'node', 'player'} and
            all(type(code) is int and code == 0 for code in returncodes.values()),
            f'failed recording process: {returncodes}')

    frames = defaultdict(dict)
    statuses = defaultdict(Counter)
    callback_ages = defaultdict(list)
    clock_rows, imu_accepted, lidar_stamps = [], [], []
    sync_wait = []
    wait_reasons = Counter()
    fingerprint_count = 16
    with diagnostics.open(newline='') as stream:
        for row in csv.DictReader(stream):
            require(None not in row and all(value is not None and value != '' for value in row.values()),
                    'malformed diagnostic CSV row')
            event = row['event']
            fid = int(row['frame_id'])
            status = int(row['status'])
            count = int(row['count'])
            require(count > 0, f'invalid represented count for {event}')
            if event in ('imu_callback', 'lidar_callback'):
                require(count == 1, f'expected one callback per row for {event}')
                statuses[event][status] += 1
                raw = finite(row['raw_stamp_s'], f'{event}.raw_stamp_s')
                if event == 'imu_callback':
                    clock_rows.append((finite(row['start_steady_s'], 'imu_callback.start_steady_s'),
                                       finite(row['ros_s'], 'imu_callback.ros_s')))
                    if status == 1:
                        imu_accepted.append(raw)
                else:
                    require(status == 1, f'unaccepted LiDAR input status {status}')
                    lidar_stamps.append(scan_key(row['corrected_stamp_s']))
                if status == 1:
                    for field in ('header_age_s', 'corrected_header_age_s'):
                        value = float(row[field])
                        callback_ages[(event, field)].append(value * 1000)
            if event == 'wait_reason':
                wait_reasons[status] += count
            if event == 'sync_wait':
                sync_wait.append(finite(row['duration_ms'], 'sync_wait.duration_ms'))
            if fid > 0 and event in ('full_cycle', 'sync_after', 'sync_before', 'frame_period', *CORE_STAGES):
                require(event not in frames[fid], f'duplicate {event} frame {fid}')
                frames[fid][event] = row

    inputs = bag['input_counts']
    for event, topic in [('imu_callback', '/livox/imu'), ('lidar_callback', '/livox/lidar')]:
        expected = inputs[topic]
        require(type(expected) is int and expected > 0 and sum(statuses[event].values()) == expected,
                f'{event} count differs from bag input count ({expected})')
    require(all(a < b for a, b in zip(lidar_stamps, lidar_stamps[1:])),
            'duplicate or unordered LiDAR input timestamps')
    require(len(imu_accepted) >= fingerprint_count, 'insufficient accepted IMU for initialization fingerprint')

    full_ids = sorted(fid for fid, events in frames.items() if 'full_cycle' in events)
    sync_ids = sorted(fid for fid, events in frames.items() if 'sync_after' in events)
    require(full_ids and full_ids == sync_ids == list(range(1, len(full_ids) + 1)),
            'missing synchronized scan or full_cycle frame')
    pending = len(lidar_stamps) - len(sync_ids)
    require(pending in (0, 1), f'LiDAR input/scan count mismatch: {pending} pending')
    targets = [scan_key(frames[fid]['sync_after']['target_end_s']) for fid in sync_ids]
    require(all(a < b for a, b in zip(targets, targets[1:])), 'duplicate or unordered synchronized scan targets')
    if pending:
        # A single unexplained loss must not be excused just because both runs
        # lost it. Existing records must show the final LiDAR is still queued.
        with diagnostics.open(newline='') as stream:
            last_lidar = None
            for row in csv.DictReader(stream):
                if row['event'] == 'full_cycle':
                    last_lidar = row
        require(last_lidar is not None and int(last_lidar['lidar_q']) == 1
                and targets[-1] < lidar_stamps[-1],
                'one LiDAR scan is unaccounted for and EOF backlog is unverified')

    valid = {}
    previous_ros = previous_steady = None
    for fid in full_ids:
        events = frames[fid]
        full = events['full_cycle']
        status = int(full['status'])
        target = scan_key(events['sync_after']['target_end_s'])
        require(status in (0, 1), f'unexpected full_cycle.status {status}')
        require(status == 1 or target < round((start + warmup) * 1e6),
                f'unsuccessful post-warmup scan at frame {fid}')
        if status == 0:
            require('odom_publish' not in events, f'unsuccessful scan has odometry at frame {fid}')
            continue
        key = scan_key(full['scan_end_s'])
        require(key not in valid, f'duplicate successful scan key {key}')
        require(key == target, f'successful scan differs from synchronized target at frame {fid}')
        needed = set(CORE_STAGES) | {'sync_before'}
        if fid > 1:
            needed.add('frame_period')
        require(needed <= events.keys(), f'missing scan diagnostics at frame {fid}: {sorted(needed-events.keys())}')
        odom = events['odom_publish']
        require(odom['status'] == '1' and key == scan_key(odom['scan_end_s']),
                f'odometry scan link differs at frame {fid}')
        for stage in set(events) - {'sync_before', 'sync_after'}:
            finite(events[stage]['duration_ms'], f'{stage}.duration_ms')
        for event in ('full_cycle', 'sync_before'):
            for field in ('imu_q', 'lidar_q'):
                finite(events[event][field], f'{event}.{field}')
        ros = finite(odom['ros_s'], 'odom_publish.ros_s')
        header = finite(odom['raw_stamp_s'], 'odom_publish.raw_stamp_s')
        steady = finite(odom['start_steady_s'], 'odom_publish.start_steady_s')
        age = finite(odom['age_s'], 'odom_publish.age_s')
        computed_age = finite(ros - finite(odom['scan_end_s'], 'odom_publish.scan_end_s'), 'ROS state age')
        require(abs(age-computed_age) <= 2e-6, f'logged ROS state age mismatch at frame {fid}')
        # The diagnostic clock is sampled after TF send. A /clock tick can
        # advance between header capture and this age sample.
        require(header <= ros and (previous_ros is None or ros >= previous_ros),
                f'odom ROS clock/header semantics changed at frame {fid}')
        require(previous_steady is None or steady > previous_steady,
                f'odom diagnostic steady clock order invalid at frame {fid}')
        previous_ros, previous_steady = ros, steady
        valid[key] = fid
    require(valid, 'no successful source scans')

    outputs = [json.loads(line) for line in observed_path.read_text().splitlines()]
    clock_stamps = defaultdict(list)
    for row in outputs:
        event = row['event']
        require(event in ('odom', 'tf', 'path'), f'unexpected observer event {event}')
        stamp = finite(row['stamp_s'], f'{event}.stamp_s')
        finite(row['steady_s'], f'{event}.steady_s')
        if event in ('odom', 'tf'):
            require((row['parent'], row['child']) == FRAME, f'{event} frame semantics changed')
            for field, size in [('pos', 3), ('quat', 4)]:
                vector = row[field]
                require(isinstance(vector, list) and len(vector) == size, f'invalid {event}.{field}')
                for value in vector:
                    finite(value, f'{event}.{field}', nonnegative=False)
            require(math.isclose(math.hypot(*row['quat']), 1.0, abs_tol=1e-6),
                    f'invalid {event} quaternion norm')
            require(not clock_stamps[event] or stamp >= clock_stamps[event][-1],
                    f'{event} observer timestamps move backward')
            clock_stamps[event].append(stamp)
        else:
            finite(row['first_pose_s'], 'path.first_pose_s')
            finite(row['last_pose_s'], 'path.last_pose_s')
            require(type(row['size']) is int and 0 < row['size'] <= 2000 and row['ordered'] is True,
                    'invalid observed Path')

    audit = legacy_audit.audit_recording(directory)
    try:
        run = parser.load(directory)
    except AssertionError as exc:
        raise ValueError(f'existing evaluator rejected recording (TF, Path, counts or CSV): {exc}') from exc
    require(run['tf_creations'] == 1, 'expected one TF broadcaster creation')
    filter_log = run['effective_surf_filter'].strip()
    filter_values = re.findall(r'^Double value is:\s*(\S+)\s*$', filter_log, re.MULTILINE)
    require(len(filter_values) == 1 or not filter_values and len(filter_log.splitlines()) == 1,
            'effective surf_filter query did not record a single value')
    run['effective_surf_filter'] = finite(filter_values[0] if filter_values else filter_log, 'surf_filter')
    require(set(valid.values()) == run['frames'].keys() and valid.keys() == run['poses'].keys(),
            'successful scans, parser frame IDs and observed poses disagree')
    require(run['clock_mapping_offset_deviation_ms']['count'] > 0, 'steady clock mapping has no samples')
    require(all(a[0] < b[0] and a[1] <= b[1] for a, b in zip(clock_rows, clock_rows[1:])),
            'IMU callback clock jumps or steady diagnostic order invalid')
    for event, durations in run['stages'].items():
        for fid, value in durations.items():
            if fid in run['frames']:
                finite(value, f'{event}.duration_ms')
    for fid, row in run['frames'].items():
        finite(row['wall_mapped_publish_age_ms'], f'frame {fid} steady-mapped state age')
        row['publish_age_ms'] = 1000 * (float(frames[fid]['odom_publish']['ros_s']) -
                                        float(frames[fid]['odom_publish']['scan_end_s']))

    ages = {}
    for (event, field), values in callback_ages.items():
        good = [value for value in values if math.isfinite(value) and value >= 0]
        ages.setdefault(event, {})[field] = {**parser.stats(good), 'negative': sum(v < 0 for v in values),
                                             'nonfinite': sum(not math.isfinite(v) for v in values)}
    prefix = imu_accepted[:fingerprint_count]
    checks = {
        **{key: run[key] for key in ('counts', 'imu_statuses', 'tf_creations', 'path_messages',
                                     'tf_odom_position_difference_m', 'tf_odom_sign_equivalent_quat_difference',
                                     'clock_mapping_offset_deviation_ms', 'cpu_temperature_c',
                                     'cpu_frequency_mhz_samples', 'effective_surf_filter')},
        'observer_frame_edge': list(FRAME),
        'audit': audit, 'pending_at_eof': pending,
        'expected_raw_inputs': inputs,
        'lidar_input_stamp_sha256': digest_values(lidar_stamps),
        'synchronized_scan_stamp_sha256': digest_values(targets),
        'initialization': {'imu_status_minus_2_count': statuses['imu_callback'][-2],
                           'first_accepted_raw_stamp_s': prefix[0],
                           'accepted_prefix_count': len(prefix),
                           'accepted_prefix_sha256': digest_values(prefix)},
        'callback_statuses': {event: dict(counts) for event, counts in statuses.items()},
        'callback_ages_ms': ages, 'sync_wait_ms': parser.stats(sync_wait), 'wait_reasons': dict(wait_reasons),
        'evidence': (verified_evidence if verified_evidence is not None else
                     {str(path): digest(path) for path in (diagnostics, observed_path, directory / 'returncodes.json')}),
        'evidence_verification': ('verified against frozen replay SHA-256'
                                  if verified_evidence is not None else 'legacy archive, frozen replay hashes unavailable'),
        'diagnostics_path': str(diagnostics), 'observer_path': str(observed_path),
    }
    return run, valid, checks


def difference(parser, runs, keys):
    position, orientation = [], []
    for key in sorted(keys):
        a, b = [runs[v]['poses'][key] for v in VARIANTS]
        position.append(math.dist(a['pos'], b['pos']))
        direct = max(abs(x-y) for x, y in zip(a['quat'], b['quat']))
        opposite = max(abs(x+y) for x, y in zip(a['quat'], b['quat']))
        orientation.append(min(direct, opposite))
    return {'position_m': parser.stats(position),
            'quaternion_component_sign_equivalent': parser.stats(orientation)}


def window_stats(parser, runs, indexes, keys):
    require(keys, 'empty successful scan window')
    window = {}
    for variant, run in runs.items():
        summary = parser.summarize(run, keys)
        duration = summary['stages_ms']['full_cycle']
        require(duration['count'] == len(keys), 'full_cycle statistics are missing successful frames')
        durations = [run['frames'][indexes[variant][key]]['duration_ms'] for key in keys]
        for threshold in (80, 100):
            over = sum(value > threshold for value in durations)
            summary[f'cycles_over_{threshold}ms'] = over
            summary[f'fraction_over_{threshold}ms'] = over / len(keys)
        summary['mean_le_80ms'] = duration['mean'] <= 80
        summary['p95_le_80ms'] = duration['p95'] <= 80
        window[variant] = summary
    window['paired_output_difference'] = difference(parser, runs, keys)
    window['mean_change_ms'] = (window['candidate']['stages_ms']['full_cycle']['mean'] -
                                window['baseline']['stages_ms']['full_cycle']['mean'])
    return window


ABSOLUTE_GATES = {
    'tf_odom_position_max_m': 'tf_odom_position_difference_m',
    'tf_odom_quaternion_component_max': 'tf_odom_sign_equivalent_quat_difference',
    'paired_pose_max_m': 'position_m',
    'paired_quaternion_component_max': 'quaternion_component_sign_equivalent',
}
RELATIVE_GATES = {
    'outdoor_high_load_wall_mapped_age_p95_candidate_max': ('wall_mapped_publish_age_ms', 'p95', 'baseline p95 + 5 ms', 5),
    'outdoor_high_load_lidar_q_max_candidate_max': ('lidar_q', 'max', 'baseline max + 1', 1),
    'wall_output_interval_p95_candidate_max': ('wall_output_interval_ms', 'p95', 'baseline p95 + 5 ms', 5),
}


def gate_limits(contract):
    limits, missing = {}, []
    frozen = contract.get('regression_gates') or {}
    require(isinstance(frozen, dict), 'expected a frozen regression gate object')
    for name in (*ABSOLUTE_GATES, *RELATIVE_GATES):
        value = frozen.get(name)
        if name in RELATIVE_GATES and value == RELATIVE_GATES[name][2]:
            limits[name] = (True, RELATIVE_GATES[name][3])
        elif type(value) in (int, float) and math.isfinite(value) and value >= 0:
            limits[name] = (False, value)
        else:
            missing.append(f'missing or unsupported frozen regression threshold {name}')
    return limits, missing


def regression_checks(limits, checks, windows):
    values = []
    all_window = next(iter(windows.values()))
    high_load = windows.get('high_load_40_100s')
    for name, (relative, limit) in limits.items():
        if name.startswith('tf_odom'):
            field = ABSOLUTE_GATES[name]
            for variant, record in checks.items():
                values.append((name, variant, record[field].get('max'), limit))
        elif name in ABSOLUTE_GATES:
            field = ABSOLUTE_GATES[name]
            values.append((name, 'paired', all_window['paired_output_difference'][field].get('max'), limit))
        else:
            field, statistic, _, _ = RELATIVE_GATES[name]
            window = all_window if name.startswith('wall_output_interval') else high_load
            if window is None:
                continue
            baseline = window['baseline'][field].get(statistic)
            candidate = window['candidate'][field].get(statistic)
            maximum = limit + baseline if relative and baseline is not None else None if relative else limit
            values.append((name, 'candidate', candidate, maximum))
    return [{'gate': name, 'variant': variant, 'value': value, 'limit': limit,
             'pass': value <= limit if value is not None and limit is not None else None}
            for name, variant, value, limit in values]


def collect_pairs(run_dir, manifest, parser, report):
    contract, runs = manifest['contract'], manifest['runs']
    bags = contract['bags']
    require(isinstance(bags, dict) and bags and 'outdoor' in bags, 'missing outdoor bag definition')
    require(isinstance(runs, dict) and runs and all(isinstance(run, dict) for run in runs.values()),
            'no valid completed runs in manifest')
    legacy = contract.get('scheme_id') == '001' and all('variant' not in run for run in runs.values())
    for key, value in [('warmup_seconds', 5), ('high_load_start_s', 40), ('high_load_end_s', 100),
                       ('target_ms', 80)]:
        require(contract.get(key, value if legacy and key != 'target_ms' else None) == value,
                f'frozen contract requires {key}={value}')
    require(contract.get('replay_rate', 1) == 1 and contract.get('clock_hz', 200) == 200,
            'steady clock mapping requires 1x replay at 200 Hz')
    require(type(contract.get('require_p95_target', False)) is bool, 'invalid require_p95_target')
    count_required = contract.get('minimum_successful_pairs', 2)
    require(type(count_required) is int and count_required >= 2, 'minimum_successful_pairs must be at least 2')
    scheduled = contract.get('repetitions')
    require(scheduled is None or type(scheduled) is int and scheduled >= 1,
            'invalid scheduled repeat count')
    limits, blockers = gate_limits(contract)
    require(not manifest.get('failure'), f'pipeline failure recorded: {manifest.get("failure")}')
    if manifest.get('bag_hashes_verified') is True and not legacy:
        report['bag_input_verification'] = 'verified by pipeline'
    else:
        report['bag_input_verification'] = 'recorded metadata only; bag files not verified by report'
        blockers.append('bag file hashes were recorded but not verified for this analysis')
    groups = defaultdict(dict)
    seen_paths = set()
    for name, record in runs.items():
        if legacy:
            variant, dataset = name.split('_', 1)
            repeat, relative = 1, name
            identity = contract.get('frozen_software', {}).get(variant, {}).get('sha256')
        else:
            variant, dataset, repeat, relative = (record[key] for key in ('variant', 'dataset', 'repeat', 'path'))
            identity = record.get('binary_sha256')
        require(dataset in bags and variant in VARIANTS and type(repeat) is int and repeat > 0,
                f'invalid dataset, variant or repeat for {name}')
        directory = (run_dir / relative).resolve()
        require(directory.is_relative_to(run_dir) and directory != run_dir, f'run path escapes evidence directory: {name}')
        require(directory not in seen_paths and variant not in groups[dataset, repeat],
                f'duplicate run, variant or evidence path: {name}')
        seen_paths.add(directory)
        groups[dataset, repeat][variant] = (name, record, directory, identity)

    performance, failures = [], []
    identity_by_variant = defaultdict(set)
    for (dataset, repeat), entries in sorted(groups.items()):
        label = f'{dataset} pair {repeat}'
        block = report['datasets'].setdefault(dataset, {'pairs': {}})
        try:
            require(set(entries) == set(VARIANTS), 'incomplete baseline/candidate pair')
            bag = bags[dataset]
            require(isinstance(bag, dict) and isinstance(bag.get('sha256'), dict), 'invalid frozen bag definition')
            start = finite(bag['start_s'], 'bag.start_s')
            duration = finite(bag['duration_s'], 'bag.duration_s')
            require(duration > (100 if dataset == 'outdoor' else 5) and bag['path'],
                    'bag does not cover the frozen analysis window')
            require(bag['sha256'] and all(valid_hash(value) for value in bag['sha256'].values()),
                    'missing or invalid bag file hashes')
            loaded, indexes, checks = {}, {}, {}
            for variant, (name, record, directory, identity) in entries.items():
                require(type(record.get('exit_code', record.get('exitcode'))) is int and
                        record.get('exit_code', record.get('exitcode')) == 0,
                        f'{name} did not exit successfully: {record.get("exit_code", record.get("exitcode"))}')
                require(valid_hash(identity), f'missing or invalid binary hash: {name}')
                identity_by_variant[variant].add(identity)
                try:
                    frozen_evidence = None if legacy else record.get('evidence_sha256')
                    require(legacy or isinstance(frozen_evidence, dict),
                            'missing frozen evidence SHA-256 mapping')
                    loaded[variant], indexes[variant], checks[variant] = read_recording(
                        directory, bag, start, 5, parser, frozen_evidence=frozen_evidence)
                except ERRORS as exc:
                    raise ValueError(f'{name}: {exc}') from exc
                checks[variant]['binary_sha256'] = identity
            baseline, candidate = (indexes[v] for v in VARIANTS)
            require(baseline.keys() == candidate.keys(),
                    f'successful scan sets differ ({len(baseline.keys()-candidate.keys())} baseline-only, '
                    f'{len(candidate.keys()-baseline.keys())} candidate-only)')
            for key in ('lidar_input_stamp_sha256', 'synchronized_scan_stamp_sha256', 'pending_at_eof', 'effective_surf_filter'):
                require(checks['baseline'][key] == checks['candidate'][key], f'paired identity differs for {key}')
            comparable_initialization = checks['baseline']['initialization'] == checks['candidate']['initialization']
            if not comparable_initialization:
                blockers.append(f'{label}: initialization IMU acceptance fingerprints differ')
            after = {key for key in baseline if key / 1e6 >= start + 5}
            windows = {'all_after_5s': window_stats(parser, loaded, indexes, after)}
            if dataset == 'outdoor':
                high = {key for key in after if start + 40 <= key / 1e6 < start + 100}
                windows['high_load_40_100s'] = window_stats(parser, loaded, indexes, high)
                summary = windows['high_load_40_100s']['candidate']
                performance.append((summary['mean_le_80ms'], summary['p95_le_80ms']))
                if not summary['mean_le_80ms']:
                    failures.append(f'{label}: candidate full_cycle mean exceeds 80 ms')
                if contract.get('require_p95_target') and not summary['p95_le_80ms']:
                    failures.append(f'{label}: candidate full_cycle p95 exceeds 80 ms')
            gates = regression_checks(limits, checks, windows)
            for gate in gates:
                item = f'{label}: {gate["gate"]} ({gate["variant"]})'
                if gate['pass'] is None:
                    blockers.append(item + ' has no valid measurement')
                if gate['pass'] is False:
                    failures.append(item + ' exceeds the frozen regression limit')
            block['pairs'][str(repeat)] = {'windows': windows, 'checks': checks, 'regression_checks': gates,
                                           'initialization_comparable': comparable_initialization,
                                           'common_scans': len(after)}
        except ERRORS as exc:
            blockers.append(f'{label}: {exc}')
            block['pairs'][str(repeat)] = {'invalid_reason': str(exc)}
    for variant, identities in identity_by_variant.items():
        if len(identities) != 1:
            blockers.append(f'{variant} binary identity differs across repeats')
    if all(len(identity_by_variant[v]) == 1 for v in VARIANTS) and identity_by_variant['baseline'] == identity_by_variant['candidate']:
        blockers.append('baseline and candidate recorded the same binary identity')
    for dataset in dict.fromkeys((*REQUIRED_DATASETS, *bags)):
        pairs = report['datasets'].get(dataset, {}).get('pairs', {})
        measured = [pair for pair in pairs.values() if 'windows' in pair]
        count = len(measured)
        if dataset in report['datasets']:
            report['datasets'][dataset]['successful_pairs'] = count
        if count < count_required:
            blockers.append(f'{dataset}: {count} successful paired runs; {count_required} required')
        if scheduled is not None and set(pairs) != {str(i) for i in range(1, scheduled + 1)}:
            blockers.append(f'{dataset}: manifest runs do not match the {scheduled} scheduled repeats')
        for variant in VARIANTS:
            for identity in ('initialization', 'lidar_input_stamp_sha256', 'synchronized_scan_stamp_sha256'):
                if measured and any(pair['checks'][variant][identity] != measured[0]['checks'][variant][identity]
                                    for pair in measured[1:]):
                    blockers.append(f'{dataset}: {variant} {identity} differs between repeats')
    expected_outdoor = scheduled if scheduled is not None else sum(dataset == 'outdoor' for dataset, _ in groups)
    if performance and len(performance) == expected_outdoor:
        report['performance_mean_pass'], report['performance_p95_pass'] = (all(values) for values in zip(*performance))
    report['repeatability_verified'] = not blockers
    report['reasons'] = blockers + failures
    report['verdict'] = 'RETEST' if blockers else 'REJECT' if failures else 'ACCEPT'


def initial_report(run_dir):
    return {'verdict': 'RETEST', 'performance_mean_pass': None, 'performance_p95_pass': None,
            'repeatability_verified': False, 'reasons': [], 'datasets': {},
            'evidence_manifest': str(run_dir / 'manifest.json'),
            'absolute_accuracy': {'availability': 'missing', 'reason': 'no independent trajectory ground truth'},
            'bag_input_verification': 'missing',
            'metric_semantics': {
                'state_age': 'odom_publish ROS time minus successful scan end, before publication',
                'steady_mapped_age': 'existing parser 1x active-playback ROS to steady mapping, extrapolated after EOF',
                'output_period': 'steady interval between matched diagnostic odometry publishes',
                'tf': 'camera_init to aft_mapped, same node-clock header and pose as corresponding odometry',
                'target': 'successful outdoor high-load processing; 80 ms mean and p95 reported independently',
                'quantiles': 'existing parser nearest rank',
            }}


def evaluate(run_dir):
    report = initial_report(run_dir)
    try:
        paths = [Path(__file__).resolve(), Path(legacy_audit.__file__), legacy_audit.PARSER]
        report['evaluators'] = {'report_version': 1, 'sha256': {
            str(path.relative_to(legacy_audit.ROOT)): digest(path) for path in paths}}
        manifest = read_json(run_dir / 'manifest.json')
        contract = manifest['contract']
        require(isinstance(contract, dict), 'expected a manifest contract object')
        report['definition'] = contract
        report['evidence_manifest_sha256'] = digest(run_dir / 'manifest.json')
        frozen = contract.get('frozen_files_sha256', {})
        for relative, actual in report['evaluators']['sha256'].items():
            require(relative not in frozen or frozen[relative] == actual,
                    f'current evaluator differs from frozen hash: {relative}')
        spec = importlib.util.spec_from_file_location('existing_surf_summary', legacy_audit.PARSER)
        parser = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(parser)
        collect_pairs(run_dir, manifest, parser, report)
    except ERRORS as exc:
        report['reasons'].append(f'{type(exc).__name__}: {exc}')
    return report


def metric_rows(report):
    def stats_rows(dataset, pair, variant, event, status, metric, unit, values, path):
        if not values.get('count'):
            yield [dataset, pair, variant, event, status, metric, 'count', '', unit, '', path,
                   'missing', 'no valid samples']
            return
        for statistic, value in values.items():
            yield [dataset, pair, variant, event, status, metric, statistic, value,
                   'samples' if statistic == 'count' else unit, values['count'], path, 'measured', '']

    for dataset, block in report['datasets'].items():
        for pair_id, pair in block['pairs'].items():
            if 'windows' not in pair:
                yield [dataset, pair_id, '', '', '', 'pair_evidence', '', '', '', '',
                       report['evidence_manifest'], 'invalid', pair['invalid_reason']]
                continue
            for variant, check in pair['checks'].items():
                for metric in ('tf_odom_position_difference_m', 'tf_odom_sign_equivalent_quat_difference'):
                    unit = 'm' if metric.endswith('_m') else 'ratio'
                    yield from stats_rows(dataset, pair_id, variant, 'tf', '', metric, unit, check[metric],
                                          check['observer_path'])
            for window, values in pair['windows'].items():
                for variant in VARIANTS:
                    summary = values[variant]
                    path = pair['checks'][variant]['diagnostics_path']
                    for event, stats in summary['stages_ms'].items():
                        yield from stats_rows(dataset, pair_id, variant, event, '1' if event == 'full_cycle' else '',
                                              f'{window}.duration_ms', 'ms', stats, path)
                    for metric in ('publish_age_ms', 'wall_mapped_publish_age_ms', 'wall_output_interval_ms',
                                   'imu_q', 'lidar_q'):
                        yield from stats_rows(dataset, pair_id, variant, 'odom_publish' if metric.endswith('_ms') else 'full_cycle',
                                              '1', f'{window}.{metric}', 'ms' if metric.endswith('_ms') else 'messages',
                                              summary[metric], path)
                    for label in ('cycles_over_80ms', 'cycles_over_100ms', 'fraction_over_80ms', 'fraction_over_100ms'):
                        count = label.startswith('cycles')
                        yield [dataset, pair_id, variant, 'full_cycle', '1', f'{window}.{label}',
                               'count' if count else 'proportion', summary[label], 'frames' if count else 'ratio',
                               summary['scans'], path, 'measured', '']
                pair_path = ' | '.join(pair['checks'][v]['observer_path'] for v in VARIANTS)
                for metric, values_for_metric in values['paired_output_difference'].items():
                    unit = 'm' if metric == 'position_m' else 'ratio'
                    yield from stats_rows(dataset, pair_id, 'candidate', 'paired_odom', '', f'{window}.{metric}',
                                          unit, values_for_metric, pair_path)
    for metric, reason in (
        ('absolute_accuracy', report['absolute_accuracy']['reason']),
        ('dds_stale_drop', 'no DDS or subscriber accounting'),
        ('downstream_receive_latency', 'no downstream clock-linked instrumentation'),
    ):
        yield ['', '', '', '', '', metric, '', '', '', '', report['evidence_manifest'], 'missing', reason]


def markdown(report):
    def flag(value):
        return 'PASS' if value is True else 'FAIL' if value is False else 'missing'

    lines = ['# FAST-LIVO2 offline A/B report', '',
             f'**{report["verdict"]}**', '',
             f'- Outdoor high-load mean ≤80 ms **{flag(report["performance_mean_pass"])}**',
             f'- Outdoor high-load p95 ≤80 ms **{flag(report["performance_p95_pass"])}**',
             f'- Repeatability **{"verified" if report["repeatability_verified"] else "unverified"}**',
             f'- Bag input hashes **{report["bag_input_verification"]}**',
             '- Absolute trajectory accuracy **missing** (independent ground truth unavailable).',
             '- At 10 Hz the expected input interval is 100 ms. Mean processing time does not predict output rate.',
             '- The 197 ms historical measurement and the 145 ms observation use different conditions.',
             '- State age uses the successful scan end. The odometry header and the corresponding '
             'camera_init → aft_mapped TF retain the node clock and matching poses.', '']
    if report['reasons']:
        lines.extend(['## Evidence and decision', '', *(f'- {reason}' for reason in report['reasons']), ''])
    for dataset, group in report['datasets'].items():
        for repeat, pair in group['pairs'].items():
            if 'windows' not in pair:
                continue
            lines.extend([f'## {dataset} pair {repeat}', '',
                          '| Window | Variant | Mean ms | p50 ms | p95 ms | Max ms | >80 ms | >100 ms |',
                          '|---|---|---:|---:|---:|---:|---:|---:|'])
            for window, values in pair['windows'].items():
                for variant in VARIANTS:
                    row = values[variant]
                    stats = row['stages_ms']['full_cycle']
                    n = row['scans']
                    above = [f'{row[f"cycles_over_{threshold}ms"]}/{n} ({row[f"fraction_over_{threshold}ms"]:.2%})'
                             for threshold in (80, 100)]
                    times = [f'{stats[field]:.9f}' for field in ('mean', 'p50', 'p95', 'max')]
                    lines.append('| ' + ' | '.join([window, variant, *times, *above]) + ' |')
            lines.append('')
            for variant in VARIANTS:
                check = pair['checks'][variant]
                init = check['initialization']
                lines.append(f'- {variant} startup IMU status -2 count {init["imu_status_minus_2_count"]}, '
                             f'first accepted raw stamp {init["first_accepted_raw_stamp_s"]}, '
                             f'first {init["accepted_prefix_count"]} accepted stamps SHA-256 '
                             f'`{init["accepted_prefix_sha256"]}`.')
            lines.extend(['', 'Per-window state age, steady-mapped age, output interval, queues and paired pose '
                          'difference are in metrics.csv and comparison.json. The comparison also records '
                          'TF parity, raw scan audits and evaluator hashes.', ''])
    return '\n'.join(lines) + '\n'


def write_report(directory, report):
    targets = [directory / filename for filename in REPORT_FILES]
    require(not any(path.exists() or path.is_symlink() for path in targets),
            f'report output already exists in {directory}; use a new --output directory')
    result = {key: report[key] for key in ('verdict', 'reasons', 'performance_mean_pass', 'performance_p95_pass',
                                           'repeatability_verified')}
    contents = (json.dumps(report, indent=2, allow_nan=False) + '\n', markdown(report),
                json.dumps(result, indent=2, allow_nan=False) + '\n')
    for path, content in zip(targets, contents):
        with path.open('x', encoding='utf-8') as stream:
            stream.write(content)
    with targets[3].open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(METRIC_COLUMNS)
        writer.writerows(metric_rows(report))


def analyze(run_dir: Path, *, write: bool = True) -> dict:
    """Read a run manifest and report only audited pair measurements."""
    run_dir = Path(run_dir).resolve()
    report = evaluate(run_dir)
    if write:
        write_report(run_dir, report)
    return report


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('run_dir', type=Path)
    cli.add_argument('--output', type=Path, help='new directory for reports, without changing the source run')
    args = cli.parse_args()
    try:
        if args.output is None:
            report = analyze(args.run_dir)
        else:
            report = analyze(args.run_dir, write=False)
            output = args.output.resolve()
            require(not output.is_relative_to(args.run_dir.resolve()), '--output must be outside the evidence directory')
            output.mkdir(parents=True, exist_ok=False)
            write_report(output, report)
    except ERRORS as exc:
        cli.exit(1, f'{exc}\n')
    print(json.dumps({name: report[name] for name in ('verdict', 'performance_mean_pass', 'performance_p95_pass')}))


if __name__ == '__main__':
    main()
