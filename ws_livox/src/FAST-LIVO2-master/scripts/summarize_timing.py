#!/usr/bin/env python3
"""Summarize buffered FAST-LIVO2 timing CSVs. P95 is the nearest rank."""
import argparse
from collections import Counter, defaultdict
import csv
import json
import math


def stats(values):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return {'count': 0}
    n = len(values)
    return {'count': n, 'mean': math.fsum(values) / n,
            'p50': values[math.ceil(0.50 * n) - 1],
            'p95': values[math.ceil(0.95 * n) - 1], 'max': values[-1], 'min': values[0]}


def summarize(filename, skip_seconds=0):
    events = defaultdict(lambda: defaultdict(list))
    groups = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    statuses = defaultdict(Counter)
    counters = Counter()
    origin = None
    if skip_seconds > 0:
        # Nested scopes are emitted on completion, not in start-time order.
        with open(filename, newline='') as stream:
            origin = min((float(row['start_steady_s']) for row in csv.DictReader(stream)), default=0)
    first_age = last_age = None
    fields = ('duration_ms', 'age_s', 'header_age_s', 'corrected_header_age_s', 'callback_gap_ms', 'imu_q', 'lidar_q',
              'prop_imu_q', 'input_points', 'down_points', 'effective_points',
              'map_voxels', 'path_poses', 'iterations')
    with open(filename, newline='') as stream:
        for row in csv.DictReader(stream):
            stamp = float(row['start_steady_s'])
            if origin is not None and stamp - origin < skip_seconds:
                continue
            event = row['event']
            counters[event] += int(row['count'])
            statuses[event][row['status']] += int(row['count'])
            for field in fields:
                value = float(row[field])
                if field not in ('age_s', 'header_age_s', 'corrected_header_age_s', 'duration_ms', 'callback_gap_ms') and value < 0:
                    continue
                events[event][field].append(value)
                groups[event][row['status']][field].append(value)
            if event == 'odom_publish' and math.isfinite(float(row['age_s'])):
                point = {'steady_s': stamp, 'age_s': float(row['age_s'])}
                if first_age is None:
                    first_age = point
                last_age = point
    result = {'file': filename, 'skip_seconds': skip_seconds, 'percentile': 'nearest_rank',
              'events': {event: {**{field: stats(values) for field, values in data.items()},
                                 'statuses': dict(statuses[event]), 'represented_count': counters[event],
                                 'by_status': {status: {field: stats(values) for field, values in fields.items()}
                                               for status, fields in groups[event].items()},
                                 'duration_unit': ('sum_ms_per_wait_window' if event in
                                                   ('wait_spin', 'wait_sync', 'wait_sleep') else 'ms_per_record')}
                         for event, data in events.items()}}
    if first_age and last_age:
        elapsed = last_age['steady_s'] - first_age['steady_s']
        result['scan_age_trend'] = {'first': first_age, 'last': last_age,
            'growth_s': last_age['age_s'] - first_age['age_s'],
            'growth_s_per_wall_s': ((last_age['age_s'] - first_age['age_s']) / elapsed) if elapsed > 0 else None}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv')
    parser.add_argument('--skip-seconds', type=float, default=0,
                        help='Discard startup by steady time, relative to first CSV record')
    args = parser.parse_args()
    print(json.dumps(summarize(args.csv, args.skip_seconds), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
