#!/usr/bin/env python3
"""Read-only recorded-data analysis. Does not create ROS nodes or replay data."""
from pathlib import Path
from collections import Counter, defaultdict
import csv, hashlib, importlib.util, json, math, sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

OUT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('prior_analysis', '/home/nvidia/MPPI_3dnavi/docs/analysis/indoor_imu_2026-11-12_210002/analyze.py')
prior = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior)
stats, stamp, custom = prior.stats, prior.stamp, prior.custom

def clean(x):
    if isinstance(x, dict): return {str(k): clean(v) for k,v in x.items()}
    if isinstance(x, (list,tuple)): return [clean(v) for v in x]
    if isinstance(x, float) and not math.isfinite(x): return None
    return x

def summary(events, frames):
    result = {}
    for name, rows in events.items():
        rs = [r for r in rows if r['frame_id'] in frames]
        if not rs: continue
        result[name] = {'duration_ms':stats([r['duration_ms'] for r in rs]), 'statuses':dict(Counter(int(r['status']) for r in rs))}
        for key in ['age_s','callback_gap_ms','imu_q','lidar_q','prop_imu_q','input_points','down_points','effective_points','map_voxels','path_poses','iterations']:
            result[name][key] = stats([r[key] for r in rs if math.isfinite(r[key]) and r[key]>=0])
    return result

def trend(rs):
    rs = sorted(rs, key=lambda r:r['start_steady_s'])
    ts=np.array([r['start_steady_s'] for r in rs]); ages=np.array([r['age_s'] for r in rs])
    return {'count':len(rs), 'elapsed_s':float(ts[-1]-ts[0]), 'first_age_ms':float(ages[0]*1000),
        'last_age_ms':float(ages[-1]*1000), 'age_ms':stats(ages*1000),
        'slope_ms_per_minute':float(np.polyfit(ts-ts[0],ages*1000,1)[0]*60),
        'first_20s_age_ms':stats(ages[ts<ts[0]+20]*1000), 'last_20s_age_ms':stats(ages[ts>ts[-1]-20]*1000),
        'windows_20s':[{'start_elapsed_s':i,'age_ms':stats(ages[(ts-ts[0]>=i)&(ts-ts[0]<i+20)]*1000)} for i in range(0,int(ts[-1]-ts[0])+1,20)]}

def run(label, bagdir, timingpath):
    manifest=[]
    db=next(bagdir.glob('*.db3'))
    for path in [db,bagdir/'metadata.yaml',timingpath]:
        before=path.stat(); h=hashlib.sha256()
        with path.open('rb') as f:
            for block in iter(lambda:f.read(1024*1024),b''): h.update(block)
        after=path.stat(); assert (before.st_size,before.st_mtime_ns)==(after.st_size,after.st_mtime_ns)
        manifest.append({'path':str(path),'size':after.st_size,'mtime_ns':after.st_mtime_ns,'sha256':h.hexdigest()})
    events=prior.timing(timingpath)
    streams=defaultdict(list); transforms=defaultdict(list); paths=[]; logs=[]
    con=sqlite3.connect(db.as_uri()+'?mode=ro',uri=True)
    assert con.execute('pragma quick_check').fetchone()[0]=='ok'
    topics={i:(n,t) for i,n,t in con.execute('select id,name,type from topics')}
    types={n:get_message(t) for n,t in topics.values()}
    validations=0
    for tid,ns,raw in con.execute('select topic_id,timestamp,data from messages order by timestamp,id'):
        name,_=topics[tid]; arrival=ns*1e-9
        if name=='/livox/lidar':
            r=custom(raw);r['arrival']=arrival
            if validations<3:
                m=deserialize_message(raw,types[name]); assert m.point_num==r['points']==len(m.points)
                assert abs(stamp(m.header.stamp)-r['header'])<1e-6
                assert abs(max(p.offset_time for p in m.points)*1e-9-r['span'])<1e-12
                validations+=1
            streams[name].append(r);continue
        m=deserialize_message(raw,types[name])
        if name in ['/livox/imu','/aft_mapped_to_init']:
            r={'arrival':arrival,'header':stamp(m.header.stamp)}
            if name=='/aft_mapped_to_init':
                p=m.pose.pose;r.update(pos=[p.position.x,p.position.y,p.position.z],quat=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
            streams[name].append(r)
        elif name in ['/tf','/tf_static']:
            for t in m.transforms:
                p=t.transform;transforms[(t.header.frame_id,t.child_frame_id)].append({'arrival':arrival,'header':stamp(t.header.stamp),'pos':[p.translation.x,p.translation.y,p.translation.z],'quat':[p.rotation.x,p.rotation.y,p.rotation.z,p.rotation.w]})
        elif name=='/path':
            stamps=[stamp(p.header.stamp) for p in m.poses]
            paths.append({'arrival':arrival,'header':stamp(m.header.stamp),'poses':len(stamps),'ordered':all(a<=b for a,b in zip(stamps,stamps[1:])),'last_header':stamps[-1] if stamps else None})
        elif name=='/rosout': logs.append({'arrival':arrival,'level':m.level,'node':m.name,'msg':m.msg})
    lo,hi=con.execute('select min(timestamp)*1e-9,max(timestamp)*1e-9 from messages').fetchone();con.close()
    pairs={}; match={}
    for topic,event in [('/livox/imu','imu_callback'),('/livox/lidar','lidar_callback'),('/aft_mapped_to_init','odom_publish')]:
        callbacks=sorted(events[event],key=lambda r:r['raw_stamp_s']);stamps=np.array([r['raw_stamp_s'] for r in callbacks]);ps=[]
        for r in streams[topic]:
            j=int(np.searchsorted(stamps,r['header'])); opts=[k for k in [j-1,j] if 0<=k<len(stamps)]
            k=min(opts,key=lambda k:abs(stamps[k]-r['header']))
            if abs(stamps[k]-r['header'])<=2e-6: ps.append((r,callbacks[k]))
        pairs[topic]=ps
        match[topic]={'matched':len(ps),'messages':len(streams[topic]),'unique_callbacks':len({b['start_steady_s'] for a,b in ps}),
            'stamp_error_us':stats([abs(a['header']-b['raw_stamp_s'])*1e6 for a,b in ps]),
            'callback_header_age_ms':stats([b['header_age_s']*1000 for a,b in ps]),'callback_gap_ms':stats([b['callback_gap_ms'] for a,b in ps]),
            'duration_ms':stats([b['duration_ms'] for a,b in ps]),'statuses':dict(Counter(int(b['status']) for a,b in ps))}
    odrows=[b for a,b in pairs['/aft_mapped_to_init']]; frames={r['frame_id'] for r in odrows}
    fullframes={r['frame_id'] for r in events['full_cycle'] if r['status']==1}
    cycles=[r for r in events['full_cycle'] if r['frame_id'] in frames]
    fullcycles=[r for r in events['full_cycle'] if r['status']==1]
    stage=summary(events,frames); allstage=summary(events,fullframes)
    out={'bag':str(bagdir),'csv':str(timingpath),'manifest':manifest,'bag_duration_s':hi-lo,'bag_start_s':lo,'bag_end_s':hi,
        'parser_ros_validations':validations,'streams':{n:prior.stream_summary(rs) for n,rs in streams.items()},
        'matched':match,'bag_stages':stage,'complete_session_stages':allstage,'bag_state_age':trend(odrows),
        'complete_session_state_age':trend(events['odom_publish']),
        'tf_broadcaster_creations':len(events['tf_broadcaster_create']),
        'full_cycle_over_100ms':sum(r['duration_ms']>100 for r in cycles),'full_cycle_over_100ms_complete_session':sum(r['duration_ms']>100 for r in fullcycles),
        'full_cycle_over_100ms_denominator':len(cycles),'complete_session_denominator':len(fullcycles),
        'all_callback_statuses':{e:dict(Counter(int(r['status']) for r in events[e])) for e in ['imu_callback','lidar_callback']},
        'lidar':{'points':stats([r['points'] for r in streams['/livox/lidar']]),'scan_span_ms':stats([r['span']*1000 for r in streams['/livox/lidar']]),
            'recorder_scan_end_age_ms':stats([(r['arrival']-r['header']-r['span'])*1000 for r in streams['/livox/lidar']]),
            'callback_scan_end_age_ms':stats([(b['ros_s']-a['header']-a['span'])*1000 for a,b in pairs['/livox/lidar']])},
        'scan_dt_ms':stats(np.diff([r['scan_end_s'] for r in odrows])*1000),
        'path':{'messages':len(paths),'poses':stats([r['poses'] for r in paths]),'all_ordered':all(r['ordered'] for r in paths),
            'all_header_equals_last_pose':all(abs(r['header']-r['last_header'])<1e-6 for r in paths),
            'header_dt_ms':stats(np.diff([r['header'] for r in paths])*1000),
            'actual_publication_ms':stats([r['duration_ms'] for r in events['path_publish'] if r['frame_id'] in frames and r['status']==1]),
            'complete_actual_publication_ms':stats([r['duration_ms'] for r in events['path_publish'] if r['status']==1])},
        'tf_chains':{f'{a}->{b}':{'count':len(rs),'recorder_header_age_ms':stats([(r['arrival']-r['header'])*1000 for r in rs]),'header_dt_ms':stats(np.diff([r['header'] for r in rs])*1000)} for (a,b),rs in transforms.items()},
        'warning_error_counts':dict(Counter((r['node'],r['msg']) for r in logs if r['level']>=30)),
        'log_node_counts':dict(Counter(r['node'] for r in logs))}
    # The TF timestamp is publication time. Verify it carries the exact odometry pose.
    tf=transforms.get(('camera_init','aft_mapped'),[]);tmap={round(r['header']*1e6):r for r in tf};diffs=[];positions=[];quats=[];sign_flips=0
    for r in streams['/aft_mapped_to_init']:
        t=tmap.get(round(r['header']*1e6))
        if t:
            diffs.append(max(abs(np.array(t['pos']+t['quat'])-np.array(r['pos']+r['quat']))))
            positions.append(max(abs(np.array(t['pos'])-np.array(r['pos']))))
            q1=np.array(t['quat']);q2=np.array(r['quat'])
            same=float(max(abs(q1-q2)));opposite=float(max(abs(q1+q2)))
            quats.append(min(same,opposite));sign_flips+=opposite<same
    out['tf_odom_pose_comparison']={'matched':len(diffs),'position_max_component_difference_m':stats(positions),
        'quaternion_sign_equivalent_max_component_difference':stats(quats),'opposite_quaternion_sign_count':sign_flips,
        'raw_components_before_quaternion_sign_equivalence':stats(diffs)}
    offset=np.median([r['ros_s']-r['start_steady_s'] for r in events['imu_callback']])
    out['diagnostic_flush_ms']=stats([r['duration_ms'] for r in events['diagnostic_flush'] if lo<=r['start_steady_s']+offset<=hi])
    out['queue_windows_20s']=[]
    t0=odrows[0]['start_steady_s']
    for i in range(0,int(odrows[-1]['start_steady_s']-t0)+1,20):
        fs={r['frame_id'] for r in odrows if i<=r['start_steady_s']-t0<i+20}
        out['queue_windows_20s'].append({'elapsed_s':i,**{name:{key:stats([r[key] for r in events[name] if r['frame_id'] in fs]) for key in ['imu_q','lidar_q']} for name in ['sync_before','sync_after','full_cycle']}})
    out['slowest_frames']=[]
    for r in sorted(cycles,key=lambda r:r['duration_ms'],reverse=True)[:12]:
        st={name:next((s['duration_ms'] for s in events[name] if s['frame_id']==r['frame_id']),None) for name in ['spin','sync','process_imu','state_estimation','voxel_map_update','odom_publish','path_publish','post_map_publications','lio_tail_console_and_file_io']}
        out['slowest_frames'].append({'frame_id':int(r['frame_id']),'elapsed_s':r['start_steady_s']-t0,'duration_ms':r['duration_ms'],'state_age_ms':r['age_s']*1000,'lidar_q':r['lidar_q'],'stages_ms':st})
    # A callback gap can contain an entire processing cycle in this serial executor.
    callbacks=sorted(events['imu_callback'],key=lambda r:r['start_steady_s']);gaps=[]
    for a,b in zip(callbacks,callbacks[1:]):
        if not lo<=b['ros_s']<=hi or b['callback_gap_ms']<=30:continue
        overlap=[c for c in fullcycles if c['start_steady_s']<b['start_steady_s'] and c['start_steady_s']+c['duration_ms']/1000>a['start_steady_s']]
        covered=sum(max(0,min(b['start_steady_s'],c['start_steady_s']+c['duration_ms']/1000)-max(a['start_steady_s'],c['start_steady_s']))*1000 for c in overlap)
        gaps.append({'gap_ms':b['callback_gap_ms'],'covered_cycle_ms':covered,'frames':[int(c['frame_id']) for c in overlap]})
    out['imu_long_callback_gaps']={'count':len(gaps),'with_cycle_overlap':sum(bool(r['frames']) for r in gaps),'cycle_covered_fraction':sum(r['covered_cycle_ms'] for r in gaps)/sum(r['gap_ms'] for r in gaps) if gaps else None}
    with (OUT/(label+'_matched_odometry.csv')).open('w') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['frame_id','header_s','scan_end_s','actual_state_age_ms','recorder_arrival_s'])
        for a,b in pairs['/aft_mapped_to_init']:w.writerow([int(b['frame_id']),a['header'],b['scan_end_s'],b['age_s']*1000,a['arrival']])
    (OUT/(label+'_metrics.json')).write_text(json.dumps(clean(out),ensure_ascii=False,indent=2)+'\n')
    print(label,json.dumps(clean({'age':out['bag_state_age'],'cycles':stage['full_cycle']['duration_ms'],'queues_before':{k:stage['sync_before'][k] for k in ['imu_q','lidar_q']},'queues_after':{k:stage['sync_after'][k] for k in ['imu_q','lidar_q']},'full_cycles':len(fullcycles),'slow':out['slowest_frames'][:3],'path':out['path']}),ensure_ascii=False),flush=True)

if __name__=='__main__':
    root=Path('/home/nvidia/fastlivo_timing')
    run('latest',root/'outdoor_bags/handheld_20261115_202612',root/'session_20261115_202458.erzjsp/run_1794745502735169375_9436/events.csv')
    run('earlier',root/'outdoor_bags/handheld_20261115_202254',root/'session_20261115_202200.3tiOmh/run_1794745325066431849_6334/events.csv')
