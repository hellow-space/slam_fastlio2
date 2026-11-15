#!/usr/bin/env python3
"""Isolated raw-input replay and lightweight output observer for TF lifetime A/B."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def observe(out):
    import rclpy
    from rclpy.qos import QoSProfile
    from tf2_msgs.msg import TFMessage
    from nav_msgs.msg import Odometry, Path as PathMsg
    from sensor_msgs.msg import PointCloud2
    rclpy.init()
    node = rclpy.create_node('tf_lifetime_benchmark_observer')
    stream = (out/'outputs.jsonl').open('w')
    counts = {'cloud_messages': 0}
    endpoint_gids = set()

    def save_tf(msg):
        for t in msg.transforms:
            p = t.transform
            stream.write(json.dumps(dict(event='tf', steady_s=time.monotonic(),
                stamp_s=t.header.stamp.sec+t.header.stamp.nanosec*1e-9,
                parent=t.header.frame_id, child=t.child_frame_id,
                pos=[p.translation.x,p.translation.y,p.translation.z],
                quat=[p.rotation.x,p.rotation.y,p.rotation.z,p.rotation.w]))+'\n')

    def save_odom(msg):
        p = msg.pose.pose
        stream.write(json.dumps(dict(event='odom',steady_s=time.monotonic(),
            stamp_s=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            parent=msg.header.frame_id,child=msg.child_frame_id,
            pos=[p.position.x,p.position.y,p.position.z],
            quat=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]))+'\n')

    def cloud(raw):
        counts['cloud_messages'] += 1

    def save_path(msg):
        stamps = [p.header.stamp.sec+p.header.stamp.nanosec*1e-9 for p in msg.poses]
        stream.write(json.dumps(dict(event='path',steady_s=time.monotonic(),
            stamp_s=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            size=len(msg.poses),first_pose_s=stamps[0] if stamps else None,
            last_pose_s=stamps[-1] if stamps else None,
            ordered=all(a<=b for a,b in zip(stamps,stamps[1:]))))+'\n')

    def endpoints():
        for publisher in node.get_publishers_info_by_topic('/tf'):
            if publisher.node_name == 'laserMapping':
                endpoint_gids.add(bytes(publisher.endpoint_gid).hex())

    node.create_subscription(TFMessage,'/tf',save_tf,QoSProfile(depth=100))
    node.create_subscription(Odometry,'/aft_mapped_to_init',save_odom,QoSProfile(depth=100))
    node.create_subscription(PointCloud2,'/cloud_registered',cloud,QoSProfile(depth=100),raw=True)
    node.create_subscription(PathMsg,'/path',save_path,QoSProfile(depth=10))
    node.create_timer(.25,endpoints)
    (out/'observer_ready').touch()
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        stream.close()
        counts['tf_endpoint_gids'] = sorted(endpoint_gids)
        (out/'observer_counts.json').write_text(json.dumps(counts,indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def stop(p,timeout=20):
    if p is None or p.poll() is not None:
        return
    p.send_signal(signal.SIGINT)
    p.wait(timeout=timeout)


def run(args):
    args.out.mkdir(parents=True,exist_ok=False)
    env=dict(os.environ,ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1',
             RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',FASTLIVO_TIMING_DIR=str(args.out/'timing'),
             ROS_LOG_DIR=str(args.out/'ros_logs'))
    pkg=Path(__file__).resolve().parents[1]
    cmds={
        'observer':[sys.executable,str(Path(__file__).resolve()),'observe',str(args.out)],
        'node':[str(args.binary),'--ros-args','-r','__node:=laserMapping',
                '--params-file',str(pkg/'config/nav_test.yaml'),
                '--params-file',str(pkg/'config/camera_pinhole.yaml'),'-p','use_sim_time:=true'],
        'player':['ros2','bag','play',str(args.bag),'--topics','/livox/imu','/livox/lidar',
                  '--clock','200','--delay','2','--disable-keyboard-controls',
                  '--wait-for-all-acked','3000']}
    (args.out/'commands.json').write_text(json.dumps(dict(commands=cmds,domain=args.domain,
          wall_start_s=time.time(),rate=1,clock_hz=200),indent=2))
    processes={};logs=[]
    try:
        for name in ['observer','node']:
            log=(args.out/(name+'.log')).open('w');logs.append(log)
            processes[name]=subprocess.Popen(cmds[name],env=env,stdout=log,stderr=subprocess.STDOUT)
            for _ in range(100):
                ready=(args.out/'observer_ready').exists() if name=='observer' else bool(list((args.out/'timing').glob('run_*/events.csv')))
                if ready:break
                if processes[name].poll() is not None:raise RuntimeError(name+' exited during startup')
                time.sleep(.1)
            else:raise RuntimeError(name+' startup timeout')
        time.sleep(2)
        log=(args.out/'player.log').open('w');logs.append(log)
        processes['player']=subprocess.Popen(cmds['player'],env=env,stdout=log,stderr=subprocess.STDOUT)
        processes['player'].wait(timeout=65)
        if processes['player'].returncode:raise RuntimeError('player failed')
        if processes['observer'].poll() is not None:raise RuntimeError('observer failed')
        time.sleep(2)
        stop(processes['node'])
        stop(processes['observer'])
    finally:
        for name in ['player','node','observer']:
            p=processes.get(name)
            if p is not None and p.poll() is None:
                try:stop(p)
                except subprocess.TimeoutExpired:
                    p.terminate();p.wait(timeout=10)
        for log in logs:log.close()
        (args.out/'returncodes.json').write_text(json.dumps({k:p.returncode for k,p in processes.items()},indent=2))
    if processes['node'].returncode != 0:raise RuntimeError('localization node failed; inspect node.log')
    csvfile=next((args.out/'timing').glob('run_*/events.csv'))
    with csvfile.open() as f:rows=list(csv.DictReader(f))
    assert all(None not in r and all(v is not None for v in r.values()) for r in rows)
    count=lambda event:sum(r['event']==event for r in rows)
    summary=dict(csv=str(csvfile),full_cycles=count('full_cycle'),
                 odom_events=count('odom_publish'),tf_creations=count('tf_broadcaster_create'))
    (args.out/'run_summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='mode',required=True)
    obs=sub.add_parser('observe');obs.add_argument('out',type=Path)
    bench=sub.add_parser('run');bench.add_argument('--binary',type=Path,required=True)
    bench.add_argument('--out',type=Path,required=True)
    bench.add_argument('--bag',type=Path,required=True)
    bench.add_argument('--domain',type=int,choices=[176],default=176)
    args=parser.parse_args()
    if args.mode=='observe':observe(args.out)
    else:run(args)


if __name__=='__main__':
    main()
