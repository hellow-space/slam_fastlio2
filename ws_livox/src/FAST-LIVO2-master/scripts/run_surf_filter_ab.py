#!/usr/bin/env python3
"""Four sequential, domain-isolated replays; preserve pre-existing source logs."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys
import time


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    pkg=Path(__file__).resolve().parents[1]
    binary=Path('/home/nvidia/.cache/fastlivo_timing_install/on/lib/fast_livo/fastlivo_mapping')
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try: argv=(entry/'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError,PermissionError,ProcessLookupError):continue
        if argv and Path(os.fsdecode(argv[0])).name=='fastlivo_mapping':
            raise RuntimeError('A localization process is already running; no experiment started.')
    args.out.mkdir(parents=True,exist_ok=False)
    backup=args.out/'original_logs';backup.mkdir()
    logs=pkg/'Log'
    files={f.relative_to(logs):digest(f) for f in logs.rglob('*') if f.is_file()}
    for rel in files:
        (backup/rel).parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(logs/rel,backup/rel)
    config=pkg/'config/nav_test.yaml';config_digest=digest(config)
    shutil.copy2(config,args.out/'source_nav_test.yaml')
    shutil.copy2(pkg/'config/camera_pinhole.yaml',args.out/'camera_pinhole.yaml')
    db=next(args.bag.glob('*.db3'))
    con=sqlite3.connect(db.as_uri()+'?mode=ro',uri=True)
    expected=dict(con.execute('select t.name,count(m.id) from topics t join messages m on m.topic_id=t.id group by t.name'))
    duration=con.execute('select (max(timestamp)-min(timestamp))*1e-9 from messages').fetchone()[0];con.close()
    manifest={'bag':str(args.bag),'bag_sha256':digest(db),'binary':str(binary),'binary_sha256':digest(binary),
        'source_config_sha256':config_digest,'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=pkg,text=True).strip(),
        'domain':176,'localhost_only':True,'clock_hz':200,'rate':1,'order':['A1','B1','B2','A2'],
        'A_filter_m':0.1,'B_filter_m':0.15,'bag_duration_s':duration,'expected_messages':expected,
        'original_log_sha256':{str(k):v for k,v in files.items()},'results':{}}
    (args.out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    try:
        for name in manifest['order']:
            value=.1 if name.startswith('A') else .15
            thermal=(args.out/(name+'_tegrastats.log')).open('w')
            telemetry=subprocess.Popen(['tegrastats','--interval','1000'],stdout=thermal,stderr=subprocess.STDOUT)
            started=time.monotonic()
            try:
                cmd=[sys.executable,str(pkg/'scripts/benchmark_tf_lifetime.py'),'run','--binary',str(binary),
                    '--out',str(args.out/name),'--bag',str(args.bag),'--surf-filter',str(value),
                    '--playback-timeout',str(duration+90),'--post-play-seconds','30']
                print('START',name,'filter',value,flush=True)
                subprocess.run(cmd,check=True)
            finally:
                telemetry.terminate();telemetry.wait(timeout=10);thermal.close()
            with next((args.out/name/'timing').glob('run_*/events.csv')).open() as f:
                counts={};status={}
                for row in csv.DictReader(f):
                    event=row['event'];counts[event]=counts.get(event,0)+1
                    if event=='imu_callback':status[row['status']]=status.get(row['status'],0)+1
            assert counts['imu_callback']==expected['/livox/imu'],counts
            assert counts['lidar_callback']==expected['/livox/lidar'],counts
            assert counts['full_cycle']>=expected['/livox/lidar']-1,counts
            assert counts['tf_broadcaster_create']==1,counts
            for rel in files:
                dest=args.out/name/'source_logs'/rel;dest.parent.mkdir(parents=True,exist_ok=True)
                if (logs/rel).exists():shutil.copy2(logs/rel,dest)
            manifest['results'][name]={'filter':value,'elapsed_wall_s':time.monotonic()-started,'counts':counts,'imu_statuses':status}
            (args.out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
            print('DONE',name,json.dumps(manifest['results'][name]),flush=True)
    finally:
        for f in logs.rglob('*'):
            if f.is_file() and f.relative_to(logs) not in files:f.unlink()
        for rel in files:shutil.copy2(backup/rel,logs/rel)
        manifest['original_logs_restored']=all(digest(logs/rel)==h for rel,h in files.items())
        manifest['source_config_unchanged']=digest(config)==config_digest
        manifest['binary_unchanged']=digest(binary)==manifest['binary_sha256']
        (args.out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        assert manifest['original_logs_restored'] and manifest['source_config_unchanged'] and manifest['binary_unchanged']


if __name__=='__main__':main()
