#!/usr/bin/env python3
"""Compare measured paired CPP results. No values are fabricated or inferred."""
import csv, sys, math, statistics
from pathlib import Path

def load(path):
    p=Path(path)
    with p.open(newline='',encoding='utf-8') as f:
        reader=csv.DictReader(f)
        required={'map_id','run_id','region_count','runtime_ms','path_length_m','coverage_ratio'}
        if not required.issubset(set(reader.fieldnames or [])):
            raise ValueError(f'{path}: missing columns {sorted(required-set(reader.fieldnames or []))}')
        data={}
        for row in reader:
            key=(row['map_id'],row['run_id'])
            if key in data:
                raise ValueError(f'{path}: duplicate pair {key}')
            record={k:float(row[k]) for k in ('region_count','runtime_ms','path_length_m','coverage_ratio')}
            if not all(math.isfinite(v) for v in record.values()) or any(v<0 for v in record.values()):
                raise ValueError(f'{path}: invalid measured values for {key}')
            if record['coverage_ratio']>1:
                raise ValueError(f'{path}: coverage_ratio exceeds 1 for {key}')
            data[key]=record
    if not data:
        raise ValueError(f'{path}: no rows')
    return data

def main():
    if len(sys.argv)!=3:
        print('Usage: python3 scripts/compare_metrics.py baseline.csv candidate.csv',file=sys.stderr)
        return 2
    before,after=map(load,sys.argv[1:])
    if set(before)!=set(after):
        missing1=sorted(set(before)-set(after)); missing2=sorted(set(after)-set(before))
        raise ValueError(f'Mismatched map/run pairs; only baseline={missing1}, only candidate={missing2}')
    fields=('region_count','runtime_ms','path_length_m','coverage_ratio')
    print('Paired comparison (same map_id + run_id; negative Δ means lower candidate):')
    print('metric                 mean baseline    mean candidate    mean Δ       median Δ')
    for field in fields:
        b=[before[k][field] for k in sorted(before)]
        a=[after[k][field] for k in sorted(before)]
        diffs=[x-y for x,y in zip(a,b)]
        print(f'{field:21s}{statistics.mean(b):15.4f}{statistics.mean(a):18.4f}{statistics.mean(diffs):12.4f}{statistics.median(diffs):15.4f}')
    print(f'Compared {len(before)} paired observations. Check individual maps, variance and safety separately.')
    return 0

if __name__=='__main__':
    try: sys.exit(main())
    except (ValueError,OSError) as exc:
        print(f'ERROR: {exc}',file=sys.stderr); sys.exit(1)
