#!/usr/bin/env python3
"""Compare independent Gazebo truth with laser odometry (initial alignment only)."""
import argparse
import json
import os
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    args=parser.parse_args(); run=args.run.resolve()
    os.environ['MPLCONFIGDIR']=str(run/'cache/matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    slam=np.genfromtxt(run/'slam.csv',delimiter=',',names=True)
    truth=np.genfromtxt(run/'truth.csv',delimiter=',',names=True)
    result=json.loads((run/'result.json').read_text())
    events=[json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
    if slam.size<2 or truth.size<2: raise RuntimeError('Insufficient trajectory samples')
    world_actual=np.column_stack([np.interp(slam['t'],truth['t'],truth[a]) for a in 'xyz'])
    world_relative=world_actual-world_actual[0]
    actual=world_actual.copy()
    index=np.argmin(abs(truth['t']-slam['t'][0]))
    q=[truth[a][index] for a in ['qx','qy','qz','qw']]
    first_rotation=Rotation.from_quat(q).as_matrix()
    initial_estimate=Rotation.from_quat([slam[a][0] for a in ['qx','qy','qz','qw']]).as_matrix()
    estimate=np.column_stack([slam[a] for a in 'xyz'])
    actual=(actual-actual[0])@first_rotation@initial_estimate.T+estimate[0]
    error=estimate-actual
    error_norm=np.linalg.norm(error,axis=1)
    phases={e['phase']:e['t'] for e in events if e['event']=='phase'}
    airborne=slam['t']>=phases.get('takeoff',slam['t'][0])
    flight=airborne & (slam['t']<=phases.get('landing',slam['t'][-1]))
    metrics={
        'completed':result['completed'],
        'alignment':'First pose only; no scale fitting / trajectory alignment / ground-truth correction',
        'slam_ate_rmse_m':float(np.sqrt(np.mean(error_norm[airborne]**2))),
        'slam_max_position_error_m':float(np.max(error_norm[airborne])),
        'slam_rmse_xyz_m':np.sqrt(np.mean(error[airborne]**2,axis=0)).tolist(),
        'evaluated_interval_s':[float(slam['t'][airborne][0]),float(slam['t'][airborne][-1])],
        'evaluation_segment':'Takeoff through disarm, including landing',
        'through_hover_ate_rmse_m':float(np.sqrt(np.mean(error_norm[flight]**2))),
        'through_hover_max_error_m':float(np.max(error_norm[flight])),
        'actual_max_forward_distance_m':float(actual[airborne,0].max()),
        'actual_max_lateral_deviation_m':float(abs(actual[flight,1]).max()),
        'world_max_lateral_deviation_m':float(abs(world_actual[flight,1]).max()),
        'slam_frame_interval_median_s':float(np.median(np.diff(slam['t']))),
        'slam_frame_interval_max_s':float(np.max(np.diff(slam['t']))),
        'cloud_points_median':float(np.median(slam['points'])),
        'processing_latency_median_wall_s':float(np.median(slam['processing_s'])),
        'samples':int(len(slam)),
    }
    if 'landing' in phases:
        k=np.argmin(abs(slam['t']-phases['landing']))
        metrics['endpoint_actual_xyz_m']=actual[k].tolist()
        metrics['endpoint_slam_xyz_m']=estimate[k].tolist()
        metrics['endpoint_error_xyz_m']=error[k].tolist()
        metrics['endpoint_error_norm_m']=float(error_norm[k])
        metrics['endpoint_world_position_m']=world_actual[k].tolist()
        metrics['endpoint_world_forward_distance_m']=float(world_actual[k,0]-world_actual[0,0])
        metrics['endpoint_world_relative_xyz_m']=world_relative[k].tolist()
        metrics['endpoint_world_target_error_xyz_m']=(world_relative[k]-np.array([result['distance_command_m'],0,1.5])).tolist()
        metrics['endpoint_world_target_error_norm_m']=float(np.linalg.norm(world_relative[k]-np.array([result['distance_command_m'],0,1.5])))
    # Flight-controller logs prove which source and observations were consumed.
    from pymavlink import mavutil
    bins=sorted((run/'logs').glob('*.BIN'))
    counts={}; params={}; failures=[]; last_arm=None; first_arm_us=None; vis_rejected=0
    for binary in bins:
        log=mavutil.mavlink_connection(str(binary))
        while True:
            m=log.recv_match(type=['PARM','VISP','VISV','ERR','ARM','MSG'],blocking=False)
            if m is None: break
            kind=m.get_type(); counts[kind]=counts.get(kind,0)+1
            if kind=='PARM': params[m.Name]=float(m.Value)
            elif kind=='ERR': failures.append(m.to_dict())
            elif kind=='ARM':
                last_arm=m.to_dict()
                if m.ArmState==1 and first_arm_us is None: first_arm_us=m.TimeUS
            elif kind=='VISP': vis_rejected+=int(m.Ign)
        log.close()
    critical={k:v for k,v in params.items() if k.startswith('EK3_SRC1_') or k in ['GPS1_TYPE','GPS2_TYPE','VISO_TYPE','ARMING_CHECK']}
    metrics.update(parameters=critical,log_counts=counts,flight_controller_errors=failures,
                   visual_position_rejected=vis_rejected,last_arm_record=last_arm,
                   errors_after_arming=[e for e in failures if first_arm_us is not None and e['TimeUS']>=first_arm_us])
    ekf=[json.loads(line) for line in (run/'telemetry.jsonl').read_text().splitlines()
         if '"mavpackettype": "LOCAL_POSITION_NED"' in line]
    if ekf:
        et=np.array([e['sim_t'] for e in ekf])
        ep=np.array([[e['x'],-e['y'],-e['z']] for e in ekf])
        tp=np.column_stack([np.interp(et,truth['t'],truth[a]) for a in 'xyz'])
        tp=(tp-world_actual[0])@first_rotation@initial_estimate.T+estimate[0]
        selected=(et>=phases.get('takeoff',et[0]))&(et<=phases.get('landing',et[-1]))
        metrics['ekf_position_rmse_m']=float(np.sqrt(np.mean(np.sum((ep[selected]-tp[selected])**2,axis=1))))
        metrics['ekf_timestamp_note']='Receive-time simulation stamps; approximate EKF comparison'
    (run/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    np.savetxt(run/'trajectory_comparison.csv',np.column_stack([slam['t'],actual,estimate,error]),delimiter=',',
               header='sim_time,truth_x,truth_y,truth_z,slam_x,slam_y,slam_z,error_x,error_y,error_z',comments='')
    fig,axes=plt.subplots(3,1,figsize=(11,10),constrained_layout=True)
    axes[0].plot(actual[:,0],actual[:,1],label='Gazebo ground truth')
    axes[0].plot(estimate[:,0],estimate[:,1],label='MID-360 / '+result['algorithm'])
    axes[0].set(xlabel='Forward (m)',ylabel='Lateral (m)',title=f"{result['distance_command_m']:g} m tunnel test — initial pose alignment only")
    for i,axis in enumerate('xyz'): axes[1].plot(slam['t'],error[:,i],label=axis.upper()+' error')
    axes[1].set(xlabel='Simulation time (s)',ylabel='Position error (m)')
    axes[2].plot(slam['t'],world_relative[:,2],label='Ground truth (world vertical, no rotation)')
    axes[2].plot(slam['t'],estimate[:,2],label='LiDAR relative altitude')
    axes[2].set(xlabel='Simulation time (s)',ylabel='Relative altitude (m)')
    for axis in axes[1:]:
        if 'landing' in phases:
            axis.axvspan(phases['landing'],slam['t'][-1],color='gray',alpha=.12,label='Landing')
    for axis in axes:
        axis.grid(alpha=.25); axis.legend()
    fig.savefig(run/'flight_evaluation.png',dpi=150)
    print(json.dumps(metrics,indent=2))


if __name__=='__main__': main()
