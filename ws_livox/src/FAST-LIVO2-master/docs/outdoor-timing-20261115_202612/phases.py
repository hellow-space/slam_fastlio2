from pathlib import Path
import importlib.util,json,math
import numpy as np
from collections import Counter
root=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('a',root/'analyze.py');a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
d=json.loads((root/'latest_metrics.json').read_text());ev=a.prior.timing(Path(d['csv']));lo=d['bag_start_s'];hi=d['bag_end_s']
od=[r for r in ev['odom_publish'] if lo<=r['ros_s']<=hi];t0=od[0]['ros_s']
def rows(event,fs):return [r for r in ev[event] if r['frame_id'] in fs]
windows=[]
for i in range(0,121,10):
    os=[r for r in od if i<=r['ros_s']-t0<i+10];fs={r['frame_id'] for r in os}
    if not fs:continue
    cs=rows('full_cycle',fs)
    w={'elapsed_s':i,'frames':len(fs),'age_ms':a.stats([r['age_s']*1000 for r in os]),'lidar_q':a.stats([r['lidar_q'] for r in cs])}
    for e in ['full_cycle','process_imu','state_estimation','voxel_map_update','map_covariance_and_update','spin','sync','post_map_publications','path_publish']:
        w[e+'_ms']=a.stats([r['duration_ms'] for r in rows(e,fs)])
    for key in ['down_points','input_points','effective_points','map_voxels','iterations','path_poses']:
        w[key]=a.stats([r[key] for r in cs])
    w['over100ms']=sum(r['duration_ms']>100 for r in cs)
    for event in ['imu_callback','lidar_callback']:
        rs=[r for r in ev[event] if i<=r['ros_s']-t0<i+10]
        w[event]={'count':len(rs),'age_ms':a.stats([r['header_age_s']*1000 for r in rs]),'dt_correction_ms':a.stats([(r['corrected_stamp_s']-r['raw_stamp_s'])*1000 for r in rs])}
    windows.append(w)
peak=max(od,key=lambda r:r['age_s']);peakfs={peak['frame_id']}
out={'origin_ros_s':t0,'windows_10s':windows,'peak':{e:rows(e,peakfs) for e in ['odom_publish','sync_before','sync_after','full_cycle','process_imu','state_estimation']}}
(root/'phases.json').write_text(json.dumps(a.clean(out),ensure_ascii=False,indent=2)+'\n')
for w in windows:
 print(w['elapsed_s'],'frames',w['frames'],'cycle',round(w['full_cycle_ms']['mean'],2),'lio',round(w['state_estimation_ms']['mean'],2),'map',round(w['voxel_map_update_ms']['mean'],2),'age',round(w['age_ms']['mean'],1),'q',w['lidar_q']['max'],'down',round(w['down_points']['mean']),'eff',round(w['effective_points']['mean']),'iter',round(w['iterations']['mean'],2),'voxels',round(w['map_voxels']['mean']),'imucbs',w['imu_callback']['count'],'imuage',round(w['imu_callback']['age_ms']['mean'],1))
print('PEAK',json.dumps(a.clean(out['peak']),ensure_ascii=False))
# Standalone artifact: actual chronological age/queue and stage timings.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(4,1,figsize=(11,12),sharex=True,constrained_layout=True)
t=np.array([r['ros_s']-t0 for r in od]);fs={r['frame_id'] for r in od};byid={r['frame_id']:r for r in od}
axes[0].plot(t,[r['age_s'] for r in od],lw=1);axes[0].set_ylabel('Actual pose age (s)')
cs=rows('full_cycle',fs)
axes[1].plot([byid[r['frame_id']]['ros_s']-t0 for r in cs],[r['lidar_q'] for r in cs]);axes[1].set_ylabel('Queued LiDAR scans\n(after sync)')
for e in ['full_cycle','state_estimation','process_imu','voxel_map_update']:
 rs=rows(e,fs);axes[2].plot([byid[r['frame_id']]['ros_s']-t0 for r in rs],[r['duration_ms'] for r in rs],lw=.7,label=e)
axes[2].axhline(100,color='red',ls='--',lw=1,label='10 Hz budget');axes[2].set_ylabel('Wall duration (ms)');axes[2].legend(ncol=2,fontsize=8)
rs=[r for r in ev['imu_callback'] if lo<=r['ros_s']<=hi]
axes[3].plot([r['ros_s']-t0 for r in rs],[r['header_age_s'] for r in rs],lw=.6);axes[3].set_ylabel('IMU callback age (s)');axes[3].set_xlabel('Elapsed recording wall time (s)')
for ax in axes:ax.grid(alpha=.25)
fig.suptitle('FAST-LIVO2 handheld recording 2026-11-15 20:26:12\nMeasured scan age and queue backlog (TF headers use publication time)')
fig.savefig(root/'timing_and_backlog.png',dpi=160);plt.close(fig)
