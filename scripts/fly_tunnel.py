#!/usr/bin/env python3
"""Local-only Gazebo / MID-360 / FAST-LIO2 / ArduCopter 4.6.3 flight test."""
import argparse
import csv
import json
import hashlib
import os
from pathlib import Path
import signal
import shutil
import struct
import subprocess
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'.runtime/python'))
os.environ['MAVLINK20']='1'
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='2'
import numpy as np
from scipy.spatial.transform import Rotation
from pymavlink import mavutil
from prepare_flight import prepare
from mid360_slam import Mid360SLAM,stamp


def record_mavlink(handle,message):
    # mavudp.recv_msg overrides the base implementation and skips setup_logfile.
    handle.write(struct.pack('>Q',(time.time_ns()//1000)&~3)+message.get_msgbuf())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--distance',type=float,default=100)
    parser.add_argument('--name',default=time.strftime('%Y%m%d-%H%M%S'))
    parser.add_argument('--wall-timeout',type=float,default=1800)
    parser.add_argument('--initialize-only',action='store_true')
    parser.add_argument('--backend',choices=['fastlio','kiss'],default='fastlio')
    args=parser.parse_args()
    if not 0<args.distance<=200: parser.error('distance must be within the first straight')
    if Path(args.name).name!=args.name or args.name in ('.','..'):
        parser.error('name must be a single directory name within runs/')
    output=ROOT/'runs'/args.name
    output.mkdir(parents=True,exist_ok=False)
    for name in ['gazebo','cache','tmp','ros','config','Log','PCD']: (output/name).mkdir()
    inputs=['config/flight.parm','config/fastlio_mid360.yaml','scripts/fly_tunnel.py',
            'scripts/prepare_flight.py','scripts/mid360_slam.py','scripts/fastlio_adapter.py']
    hashes={}
    for name in inputs:
        destination=output/'reproducibility'/name
        destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/name,destination)
        hashes[name]=hashlib.sha256(destination.read_bytes()).hexdigest()
    (output/'input_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    provenance=json.loads((ROOT/'config/flight_provenance.json').read_text())
    for name,item in provenance.items():
        if isinstance(item,dict) and 'sha256' in item:
            item['sha256']=hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
    (output/'source_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    env=os.environ.copy()
    env.update(GZ_PARTITION='livo-'+uuid.uuid4().hex,GZ_IP='127.0.0.1',
               GZ_SIM_RESOURCE_PATH=str(ROOT/'models'),GZ_SIM_SYSTEM_PLUGIN_PATH=str(ROOT/'build/gazebo'),
               GZ_LOG_PATH=str(output/'gazebo'),IGN_LOG_PATH=str(output/'gazebo'),
               XDG_CACHE_HOME=str(output/'cache'),XDG_CONFIG_HOME=str(output/'config'),
               ROS_LOG_DIR=str(output/'ros'),ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID='77',TMPDIR=str(output/'tmp'),
               LIBGL_ALWAYS_SOFTWARE='1',LP_NUM_THREADS='4',MESA_SHADER_CACHE_DIR=str(output/'cache/mesa'))
    os.environ.update({k:v for k,v in env.items() if k.startswith(('GZ_','IGN_','ROS_'))})
    from gz.transport13 import Node
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.world_control_pb2 import WorldControl
    from gz.msgs10.boolean_pb2 import Boolean
    node=Node()
    slam=Mid360SLAM(output,node,args.backend)
    truth_file=(output/'truth.csv').open('w',buffering=1)
    truth=csv.writer(truth_file); truth.writerow(['t','x','y','z','qw','qx','qy','qz'])
    imu_file=(output/'imu.csv').open('w',buffering=1)
    imu_writer=csv.writer(imu_file); imu_writer.writerow(['t','gx','gy','gz','ax','ay','az'])
    telemetry=(output/'telemetry.jsonl').open('w',buffering=1)
    odom_file=(output/'mavlink_odometry.csv').open('w',buffering=1)
    odom_writer=csv.writer(odom_file); odom_writer.writerow(['t','x','y','z','qw','qx','qy','qz','body_vx','body_vy','body_vz','quality'])
    state={'sim':0.,'truth':None}
    def truth_cb(message):
        state['sim']=stamp(message)
        for pose in message.pose:
            if pose.name=='test_quad':
                p=pose.position; q=pose.orientation
                state['truth']=[p.x,p.y,p.z]
                truth.writerow([state['sim'],p.x,p.y,p.z,q.w,q.x,q.y,q.z])
    def imu_cb(message):
        a=message.linear_acceleration; w=message.angular_velocity
        imu_writer.writerow([stamp(message),w.x,w.y,w.z,a.x,a.y,a.z])
        if slam.adapter: slam.adapter.publish_imu(message)
    node.subscribe(LaserScan,'/mid360/range_image',slam.receive)
    node.subscribe(Pose_V,'/world/metro_tunnel/dynamic_pose/info',truth_cb)
    node.subscribe(IMU,'/mid360/imu',imu_cb)
    prepare(output/'world.sdf')
    connection=mavutil.mavlink_connection('udpin:127.0.0.1:15760',source_system=255,source_component=190)
    tlog=(output/'telemetry.tlog').open('wb')
    processes=[]; handles=[]
    def start(command,name):
        handle=(output/name).open('w'); handles.append(handle)
        process_env=env.copy()
        if name=='gazebo.log':
            process_env['LD_PRELOAD']=str(ROOT/'build/libgazebo_log_redirect.so')
            process_env['LIVO_OGRE_LOG_PATH']=str(output/'gazebo/ogre2.log')
        process=subprocess.Popen(command,cwd=output,env=process_env,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process)
    eventfile=(output/'events.jsonl').open('w',buffering=1)
    def event(name,**kwargs):
        record={'t':state['sim'],'event':name,**kwargs}
        eventfile.write(json.dumps(record)+'\n'); print(json.dumps(record),flush=True)
    def command(cmd,*values):
        params=list(values)+[0]*(7-len(values))
        connection.mav.command_long_send(1,1,cmd,0,*params)
        event('command',command=cmd,params=params)
    def target(x,z):
        # Position + yaw only in LOCAL_NED; yaw zero follows tunnel +X.
        connection.mav.set_position_target_local_ned_send(0,1,1,mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b100111111000,x,0,z,0,0,0,0,0,0,0,0)
    algorithm='FAST-LIO2 ROS 2' if args.backend=='fastlio' else 'KISS-ICP 1.2.3'
    result={'completed':False,'distance_command_m':args.distance,'algorithm':algorithm,'sensor':'MID-360 pattern resampled from Gazebo','gps':False}
    (output/'run_config.json').write_text(json.dumps({**result,'source':provenance['ardupilot']['source'],'point_rate':200000,'frame_rate':10,'range_image_rate':50,'range_noise_std_m':.02,'range_limit_m':40},indent=2))
    try:
        if args.backend=='fastlio':
            shutil.copy(ROOT/'config/fastlio_mid360.yaml',output/'fastlio_mid360.yaml')
            start([str(ROOT/'lio_ws/install/fast_lio/lib/fast_lio/fastlio_mapping'),
                   '--ros-args','--params-file',str(output/'fastlio_mid360.yaml')],'fastlio.log')
        start(['gz','sim','-s','-r','--headless-rendering','--record-path',str(output/'gazebo/state'),
               '--record-period','0.1','--seed','360','-v','3',str(output/'world.sdf')],'gazebo.log')
        binary=ROOT/'build/apm/sitl/bin/arducopter'
        defaults=','.join(map(str,[ROOT/'dependencies/apm4.6.3/Tools/autotest/default_params/copter.parm',ROOT/'config/flight.parm']))
        start([str(binary),'--model','JSON','--sim-address','127.0.0.1','--sim-port-out','19002',
               '--serial0','udpclient:127.0.0.1:15760','--serial1','none','--serial2','none',
               '--instance','42','--home','31.2304,121.4737,10,0','--defaults',defaults,'--wipe'],'sitl.log')
        phase='initializing'; armed=False; position=None; ekf_flags=0; boot=None
        last_hb=last_origin=last_status=last_command=0.; last_odom=-1.; phase_t=0.
        settled_since=None; deadline=time.monotonic()+args.wall_timeout
        F=np.diag([1.,-1.,-1.])
        while time.monotonic()<deadline:
            now=time.monotonic(); sim=state['sim']
            if any(p.poll() is not None for p in processes): raise RuntimeError('A simulator process exited; inspect gazebo.log / sitl.log')
            if slam.error: raise RuntimeError(slam.error)
            if slam.dropped: raise RuntimeError('Laser processing queue overflow')
            # Fixed reduced wall-clock rate preserves all samples. Abort on
            # overflow or stale estimates rather than dropping sensor frames.
            while True:
                message=connection.recv_match(blocking=False)
                if message is None: break
                kind=message.get_type()
                if kind=='BAD_DATA': continue
                record_mavlink(tlog,message)
                if kind in ['HEARTBEAT','LOCAL_POSITION_NED','EKF_STATUS_REPORT','STATUSTEXT','COMMAND_ACK','GPS_GLOBAL_ORIGIN','HOME_POSITION','ATTITUDE']:
                    telemetry.write(json.dumps({'sim_t':sim,**message.to_dict()})+'\n')
                if kind=='HEARTBEAT':
                    armed=bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if boot is None:
                        boot=sim; event('heartbeat')
                        for mid,hz in [(32,20),(30,20),(193,4)]: command(511,mid,1e6/hz)
                elif kind=='LOCAL_POSITION_NED': position=np.array([message.x,message.y,message.z])
                elif kind=='EKF_STATUS_REPORT': ekf_flags=message.flags
                elif kind=='STATUSTEXT': event('statustext',text=message.text)
                elif kind=='COMMAND_ACK': event('ack',command=message.command,result=message.result)
            if now-last_hb>.5:
                connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
                connection.mav.rc_channels_override_send(1,1,1500,1500,1000,1500,1500,1500,1500,1500)
                last_hb=now
            estimate=slam.snapshot()
            if estimate and estimate['t']>last_odom:
                R=estimate['rotation']; p=F@estimate['position']; body_velocity=F@R.T@estimate['velocity']
                q=np.roll(Rotation.from_matrix(F@R@F).as_quat(),1)
                covariance=np.zeros(21); covariance[[0,6,11]]=.15**2; covariance[[15,18,20]]=.05**2
                connection.mav.odometry_send(int(estimate['t']*1e6),20,12,*p,q,*body_velocity,0,0,0,
                    covariance,[float('nan')]*21,0,mavutil.mavlink.MAV_ESTIMATOR_TYPE_LIDAR,estimate['quality'])
                odom_writer.writerow([estimate['t'],*p,*q,*body_velocity,estimate['quality']])
                last_odom=estimate['t']
            if boot is not None and now-last_origin>3 and phase=='initializing':
                connection.mav.set_gps_global_origin_send(1,312304000,1214737000,10000)
                last_origin=now
            if now-last_status>10:
                event('status',phase=phase,armed=armed,ekf_flags=ekf_flags,position=position.tolist() if position is not None else None,
                      slam=estimate['position'].tolist() if estimate else None,truth=state['truth'],queue=slam.q.qsize())
                last_status=now
            if phase=='initializing' and estimate and position is not None and sim>25 and (ekf_flags & 24)==24:
                if args.initialize_only:
                    result['completed']=True; result['initialization_only']=True; break
                connection.set_mode('GUIDED')
                phase='arming'; phase_t=sim; last_command=0
                event('phase',phase=phase)
            elif phase=='arming':
                if sim-phase_t>20: raise RuntimeError('Pre-arm checks did not pass in 20 seconds')
                if armed:
                    command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,1.5)
                    phase='takeoff'; phase_t=sim; event('phase',phase=phase)
                elif now-last_command>4:
                    command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
                    last_command=now
            elif phase=='takeoff' and position is not None and position[2]<-1.35:
                if sim-phase_t>7:
                    target(args.distance,-1.5)
                    phase='straight'; phase_t=sim; event('phase',phase=phase)
            elif phase=='straight' and position is not None:
                if np.linalg.norm(position-np.array([args.distance,0,-1.5]))<.35:
                    if settled_since is None: settled_since=sim
                    elif sim-settled_since>5:
                        result['endpoint_truth']=state['truth']; result['endpoint_ekf']=position.tolist()
                        result['endpoint_slam']=estimate['position'].tolist()
                        connection.set_mode('LAND'); phase='landing'; phase_t=sim; event('phase',phase=phase)
                else: settled_since=None
            elif phase=='landing' and not armed:
                result['completed']=True; event('flight_complete'); break
            if armed and last_odom>0 and sim-last_odom>1:
                raise RuntimeError('External odometry is more than one simulation second stale')
            if phase in ['takeoff','straight'] and not armed:
                raise RuntimeError('Unexpected disarm before mission completion')
            # Independent test supervisor: truth can only abort an invalid trial,
            # never correct an estimate or generate a flight setpoint.
            if armed and estimate and state['truth'] is not None:
                relative_truth=np.array(state['truth'])-np.array([100.5,0,.195])
                if np.linalg.norm(relative_truth-estimate['position'])>5:
                    raise RuntimeError('Evaluation abort: localization error exceeded 5 m')
                if abs(state['truth'][1])>1.6 or state['truth'][2]>3.8:
                    raise RuntimeError('Evaluation abort: vehicle left the flight corridor')
            if sim>400: raise RuntimeError('Simulation-time mission timeout')
            time.sleep(.005)
        else: raise RuntimeError('Wall-clock timeout')
    except (Exception,KeyboardInterrupt) as exc:
        result['error']=str(exc) or type(exc).__name__; event('failed',error=result['error'])
        try: connection.set_mode('LAND')
        except Exception: pass
    finally:
        result['sim_duration']=state['sim']; result['dropped_scans']=slam.dropped
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try: process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL); process.wait()
        slam.close()
        for topic in ['/mid360/range_image','/world/metro_tunnel/dynamic_pose/info','/mid360/imu']:
            node.unsubscribe(topic)
        connection.close()
        for handle in [truth_file,imu_file,telemetry,tlog,odom_file,eventfile,*handles]: handle.close()
        (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({'directory':str(output),**result},indent=2),flush=True)
    return 0 if result['completed'] else 1


if __name__=='__main__': sys.exit(main())
