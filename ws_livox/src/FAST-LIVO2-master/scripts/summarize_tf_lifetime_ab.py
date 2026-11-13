#!/usr/bin/env python3
"""Compare equal scan timestamps across completed isolated A/B replay runs."""
import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path
import sqlite3


def stats(values):
    a=sorted(float(x) for x in values if math.isfinite(float(x)))
    if not a:return {'count':0}
    n=len(a)
    return dict(count=n,mean=math.fsum(a)/n,p50=a[math.ceil(.50*n)-1],
                p95=a[math.ceil(.95*n)-1],p99=a[math.ceil(.99*n)-1],max=a[-1],min=a[0])


def load(root):
    events=defaultdict(list)
    with next((root/'timing').glob('run_*/events.csv')).open() as f:
        for r in csv.DictReader(f):
            assert None not in r and all(v is not None for v in r.values())
            events[r['event']].append({k:(v if k=='event' else float(v)) for k,v in r.items()})
    outputs=[json.loads(line) for line in (root/'outputs.jsonl').read_text().splitlines()]
    assert json.loads((root/'returncodes.json').read_text())=={'observer':0,'node':0,'player':0}
    return events,outputs


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    p.add_argument('--bag',type=Path,required=True);p.add_argument('--skip-seconds',type=float,default=3)
    args=p.parse_args()
    db=next(args.bag.glob('*.db3'))
    con=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
    lo,hi=con.execute('SELECT min(timestamp)*1e-9,max(timestamp)*1e-9 FROM messages').fetchone()
    expected=dict(con.execute('SELECT t.name,count(m.id) FROM topics t JOIN messages m ON m.topic_id=t.id GROUP BY t.name'))
    con.close()
    names=['A1','B1','B2','A2','A3','B3'];runs={n:load(args.root/n) for n in names}
    key=lambda r:round(r['scan_end_s']*1e6)
    candidates={n:{key(r):r for r in ev['full_cycle'] if r['status']==1 and lo+args.skip_seconds<=r['scan_end_s']<=hi}
                for n,(ev,outputs) in runs.items()}
    common=set.intersection(*(set(c) for c in candidates.values()))
    assert len(common)>200,(len(common),{n:len(c) for n,c in candidates.items()})
    result=dict(root=str(args.root),bag=str(args.bag),bag_start_s=lo,bag_end_s=hi,
                skip_seconds=args.skip_seconds,common_scan_count=len(common),runs={},pooled={})
    pooled=defaultdict(lambda:defaultdict(list));flat=[]
    for n,(ev,outputs) in runs.items():
        selected=[candidates[n][k] for k in sorted(common)];ids={r['frame_id'] for r in selected}
        ts=[r['scan_end_s'] for r in selected]
        stages={event:[r['duration_ms'] for r in ev[event] if r['frame_id'] in ids]
                for event in ['full_cycle','odom_publish','tf_send','odom_send','process_imu','state_estimation']}
        imus=[r for r in ev['imu_callback'] if min(ts)<=r['raw_stamp_s']<=max(ts)]
        stages['imu_callback_age_ms']=[r['header_age_s']*1000 for r in imus]
        stages['imu_callback_gap_ms']=[r['callback_gap_ms'] for r in imus]
        stages['state_age_ms']=[r['age_s']*1000 for r in ev['odom_publish'] if r['frame_id'] in ids]
        counts=json.loads((args.root/n/'observer_counts.json').read_text())
        tf=[r for r in outputs if r['event']=='tf'];odom=[r for r in outputs if r['event']=='odom']
        assert all(r['parent']=='camera_init' and r['child']=='aft_mapped' for r in tf+odom)
        tf_by_header={round(r['stamp_s']*1e6):r for r in tf}
        diffs=[]
        for o in odom:
            t=tf_by_header.get(round(o['stamp_s']*1e6))
            if t:
                diffs.append(max(abs(a-b) for a,b in zip(t['pos']+t['quat'],o['pos']+o['quat'])))
        # tf2 Transform stores a rotation matrix and reconstructs its quaternion;
        # allow only numerical roundoff in that conversion.
        assert diffs and max(diffs)<1e-12
        assert len(tf)==len(odom)==len(ev['odom_publish'])
        creates=len(ev['tf_broadcaster_create'])
        assert len(ev['imu_callback'])==expected['/livox/imu']
        assert len(ev['lidar_callback'])==expected['/livox/lidar']
        assert creates==(len(ev['odom_publish']) if n.startswith('A') else 1)
        assert all(r['status'] in (1,-2) for r in ev['imu_callback'])
        if n.startswith('B'):assert len(counts['tf_endpoint_gids'])==1
        summary=dict(stages={k:stats(v) for k,v in stages.items()},
            tf_creation_count=creates,tf_creation_duration_ms=stats([r['duration_ms'] for r in ev['tf_broadcaster_create']]),
            successful_frames=len(ev['odom_publish']),full_cycles=len(ev['full_cycle']),
            input_imu_count=len(ev['imu_callback']),input_lidar_count=len(ev['lidar_callback']),
            imu_statuses=dict(Counter(str(int(r['status'])) for r in ev['imu_callback'])),
            observed_tf_count=len(tf),observed_odom_count=len(odom),
            observed_tf_endpoint_count=len(counts['tf_endpoint_gids']),
            tf_odom_max_component_difference=max(diffs),
            cycles_over_100ms=sum(x>100 for x in stages['full_cycle']))
        result['runs'][n]=summary
        for event,values in stages.items():pooled[n[0]][event].extend(values)
        for r in selected:
            fid=r['frame_id'];flat.append(dict(run=n,scan_end_s=r['scan_end_s'],frame_id=int(fid),
                **{event:next(s['duration_ms'] for s in ev[event] if s['frame_id']==fid)
                   for event in ['full_cycle','odom_publish','process_imu','state_estimation']}))
    for group,events in pooled.items():
        result['pooled'][group]={k:stats(v) for k,v in events.items()}
        result['pooled'][group]['cycles_over_100ms']=sum(x>100 for x in events['full_cycle'])
    result['changes_percent']={k:{s:(result['pooled']['B'][k][s]/result['pooled']['A'][k][s]-1)*100
                                     for s in ['mean','p95','p99','max']}
                               for k in ['full_cycle','odom_publish','imu_callback_age_ms','state_estimation']}
    (args.root/'comparison.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    with (args.root/'matched_frames.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(flat[0]),lineterminator='\n');w.writeheader();w.writerows(flat)
    print(json.dumps(dict(common_scan_count=len(common),pooled=result['pooled'],changes_percent=result['changes_percent']),indent=2))


if __name__=='__main__':main()
