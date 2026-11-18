#!/usr/bin/env python3
"""Match identical scan timestamps across surf-filter A/B raw-data replays."""
import argparse
from collections import Counter,defaultdict
import csv
import json
import math
from pathlib import Path
import re
import sqlite3

import numpy as np


def stats(values):
    a=sorted(float(v) for v in values if math.isfinite(float(v)))
    if not a:return {'count':0}
    n=len(a)
    return {'count':n,'mean':math.fsum(a)/n,'p50':a[math.ceil(.5*n)-1],
        'p95':a[math.ceil(.95*n)-1],'p99':a[math.ceil(.99*n)-1],'min':a[0],'max':a[-1]}


def load(root):
    frame=defaultdict(dict);odom={};callbacks=defaultdict(list);stages=defaultdict(dict);counts=Counter();statuses=Counter();creates=0
    selected=['scan_end_s','start_steady_s','duration_ms','ros_s','age_s','imu_q','lidar_q','path_poses','down_points','input_points','effective_points','iterations','map_voxels','status']
    with next((root/'timing').glob('run_*/events.csv')).open() as f:
        for r in csv.DictReader(f):
            assert None not in r and all(v is not None for v in r.values())
            e=r['event'];counts[e]+=1;fid=int(r['frame_id'])
            if e in ['imu_callback','lidar_callback']:
                callbacks[e].append({k:float(r[k]) for k in ['start_steady_s','ros_s','raw_stamp_s','header_age_s','callback_gap_ms','status']})
                if e=='imu_callback':statuses[r['status']]+=1
            if e=='tf_broadcaster_create':creates+=1
            if e=='full_cycle' and r['status']=='1':frame[fid]={k:float(r[k]) for k in selected}
            if e=='odom_publish':odom[fid]={k:float(r[k]) for k in selected}
            if fid:stages[e][fid]=float(r['duration_ms'])
    frame=dict(frame)
    cb=callbacks['imu_callback'];maxros=max(r['ros_s'] for r in cb)
    offsets=[r['ros_s']-r['start_steady_s'] for r in cb if 1e9<r['ros_s']<maxros-.5]
    offset=float(np.median(offsets))
    for fid,r in frame.items():
        o=odom[fid];r['publish_age_ms']=o['age_s']*1000
        # /clock stops at EOF. Extend its active-playback clock-to-steady mapping
        # into drain time so a late output cannot appear fresh because /clock froze.
        r['wall_mapped_publish_age_ms']=(o['start_steady_s']+offset-o['scan_end_s'])*1000
        r['publish_steady_s']=o['start_steady_s']
    outputs=[json.loads(line) for line in (root/'outputs.jsonl').read_text().splitlines()]
    observed_odom=[r for r in outputs if r['event']=='odom'];tf=[r for r in outputs if r['event']=='tf']
    assert len(observed_odom)==len(odom)==len(tf)
    ordered=sorted(odom,key=lambda fid:odom[fid]['start_steady_s'])
    poses={round(odom[fid]['scan_end_s']*1e6):out for fid,out in zip(ordered,observed_odom)}
    tf_position=[];tf_orientation=[]
    for o,t in zip(observed_odom,tf):
        assert abs(o['stamp_s']-t['stamp_s'])<2e-6
        tf_position.append(max(abs(np.array(o['pos'])-t['pos'])))
        q1=np.array(o['quat']);q2=np.array(t['quat']);tf_orientation.append(min(max(abs(q1-q2)),max(abs(q1+q2))))
    assert max(tf_position)<1e-12 and max(tf_orientation)<1e-12
    paths=[r for r in outputs if r['event']=='path'];assert all(r['size']<=2000 and r['ordered'] for r in paths)
    telemetry=(root.parent/(root.name+'_tegrastats.log')).read_text()
    temperatures=[float(s) for s in re.findall(r'\bcpu@([\d.]+)C',telemetry)]
    frequencies=[float(s) for s in re.findall(r'%@(\d+)',telemetry)]
    return {'frames':frame,'stages':stages,'callbacks':callbacks,'poses':poses,'counts':dict(counts),'imu_statuses':dict(statuses),
        'tf_creations':creates,'tf_odom_position_difference_m':stats(tf_position),'tf_odom_sign_equivalent_quat_difference':stats(tf_orientation),
        'path_messages':len(paths),'cpu_temperature_c':stats(temperatures),'cpu_frequency_mhz_samples':stats(frequencies),
        'clock_mapping_offset_deviation_ms':stats([(v-offset)*1000 for v in offsets]),
        'returncodes':json.loads((root/'returncodes.json').read_text()),
        'effective_surf_filter':(root/'effective_surf_filter.txt').read_text()}


def summarize(run,keys):
    key=lambda r:round(r['scan_end_s']*1e6)
    rs=[r for r in run['frames'].values() if key(r) in keys];ids={fid for fid,r in run['frames'].items() if key(r) in keys}
    result={'scans':len(rs),'stages_ms':{e:stats(v[fid] for fid in ids if fid in v) for e,v in run['stages'].items()},
        **{k:stats(r[k] for r in rs) for k in ['publish_age_ms','wall_mapped_publish_age_ms','imu_q','lidar_q','down_points','effective_points','iterations','map_voxels','path_poses']},
        'cycles_over_100ms':sum(r['duration_ms']>100 for r in rs)}
    times=sorted((r['publish_steady_s'],r['scan_end_s']) for r in rs)
    result['wall_output_interval_ms']=stats((b[0]-a[0])*1000 for a,b in zip(times,times[1:]))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);args=p.parse_args()
    manifest=json.loads((args.root/'manifest.json').read_text());names=manifest['order']
    runs={name:load(args.root/name) for name in names}
    db=next(Path(manifest['bag']).glob('*.db3'));con=sqlite3.connect(db.as_uri()+'?mode=ro',uri=True)
    lo,hi=con.execute('select min(timestamp)*1e-9,max(timestamp)*1e-9 from messages').fetchone();con.close()
    key=lambda r:round(r['scan_end_s']*1e6)
    common=set.intersection(*({key(r) for r in run['frames'].values() if r['scan_end_s']>=lo+5} for run in runs.values()))
    assert len(common)>1100,len(common)
    high={k for k in common if lo+40<=k/1e6<lo+100}
    result={'manifest':manifest,'bag_start_s':lo,'bag_end_s':hi,'skip_first_source_seconds':5,'common_scan_count':len(common),
        'high_load_definition':'same source scans at bag start +40 through +100 seconds','high_load_scan_count':len(high),'runs':{},'pooled':{},'differences':{}}
    for name,r in runs.items():
        assert r['returncodes']=={'observer':0,'node':0,'player':0},r['returncodes']
        assert r['counts']['imu_callback']==manifest['expected_messages']['/livox/imu']
        assert r['counts']['lidar_callback']==manifest['expected_messages']['/livox/lidar']
        assert r['tf_creations']==1
        result['runs'][name]={'all_common':summarize(r,common),'high_load':summarize(r,high),
            **{k:v for k,v in r.items() if k not in ['frames','stages','callbacks','poses']}}
    for group in ['A','B']:
        members=[r for name,r in runs.items() if name.startswith(group)]
        merged={'frames':{},'stages':defaultdict(dict)}
        for index,r in enumerate(members):
            for fid,v in r['frames'].items():merged['frames'][(index,fid)]=v
            for e,values in r['stages'].items():
                for fid,v in values.items():merged['stages'][e][(index,fid)]=v
        result['pooled'][group]={'all_common':summarize(merged,common),'high_load':summarize(merged,high)}
    for region in ['all_common','high_load']:
        a=result['pooled']['A'][region];b=result['pooled']['B'][region]
        result['differences'][region]={e:{s:(b['stages_ms'][e][s]/a['stages_ms'][e][s]-1)*100 for s in ['mean','p95','max']} for e in ['full_cycle','state_estimation','process_imu','voxel_map_update']}
    # Trajectory disagreement is not accuracy: these recordings have no ground truth.
    trajectory={}
    for n1,n2 in [('A1','B1'),('A2','B2'),('A1','A2'),('B1','B2')]:
        pos=[];angles=[]
        for k in sorted(common):
            a=runs[n1]['poses'][k];b=runs[n2]['poses'][k]
            pos.append(float(np.linalg.norm(np.array(a['pos'])-b['pos'])))
            q1=np.array(a['quat']);q2=np.array(b['quat']);dot=abs(float(np.dot(q1,q2)/(np.linalg.norm(q1)*np.linalg.norm(q2))))
            angles.append(math.degrees(2*math.acos(min(1,dot))))
        trajectory[n1+'_'+n2]={'position_difference_m':stats(pos),'orientation_difference_deg':stats(angles)}
    result['trajectory_comparisons_without_ground_truth']=trajectory
    (args.root/'comparison.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (args.root/'matched_frames.csv').open('w',newline='') as f:
        fields=['run','scan_end_s','relative_source_s','full_cycle_ms','lio_ms','imu_ms','map_update_ms','down_points','age_ms','wall_mapped_age_ms','lidar_q']
        w=csv.DictWriter(f,fieldnames=fields,lineterminator='\n');w.writeheader()
        for name,run in runs.items():
            for fid,r in sorted(run['frames'].items()):
                if key(r) not in common:continue
                w.writerow({'run':name,'scan_end_s':r['scan_end_s'],'relative_source_s':r['scan_end_s']-lo,
                    'full_cycle_ms':r['duration_ms'],'lio_ms':run['stages']['state_estimation'][fid],
                    'imu_ms':run['stages']['process_imu'][fid],'map_update_ms':run['stages']['voxel_map_update'][fid],
                    'down_points':r['down_points'],'age_ms':r['publish_age_ms'],
                    'wall_mapped_age_ms':r['wall_mapped_publish_age_ms'],'lidar_q':r['lidar_q']})
    print(json.dumps({'common_scans':len(common),'pooled':result['pooled'],'differences':result['differences'],'trajectory':trajectory},indent=2))


if __name__=='__main__':main()
