"""MID-360 angular sampling and KISS-ICP. This module has no truth input.

Gazebo samples motion every 20 ms. Each emitted point retains that actual
snapshot time as well as its nominal position in the manufacturer's scan.
"""
import csv
import json
import queue
import threading
import time
import traceback
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from kiss_icp.config import KISSConfig
from kiss_icp.kiss_icp import KissICP
from geometry import ROOT


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nsec * 1e-9


class Mid360SLAM:
    def __init__(self, output, node, backend='kiss'):
        from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
        self.message_type = PointCloudPacked
        self.publisher = node.advertise('/mid360/points', PointCloudPacked)
        self.output=output
        (output/'clouds').mkdir()
        self.q=queue.Queue(maxsize=100)
        self.lock=threading.Lock()
        self.state=None
        self.error=None
        self.dropped=0
        self.running=True
        self.rng=np.random.default_rng(360)
        pattern=np.loadtxt(ROOT/'dependencies/livox_pattern/mid360.csv',delimiter=',',skiprows=1)
        self.azimuth=np.deg2rad((pattern[:,1]+180)%360-180)
        self.elevation=np.deg2rad(np.clip(90-pattern[:,2],-7,52))
        self.cursor=0
        config=KISSConfig()
        config.data.max_range=40
        config.data.min_range=.4
        config.data.deskew=True
        config.mapping.voxel_size=.12
        config.mapping.max_points_per_voxel=12
        config.registration.max_num_iterations=20
        config.registration.convergence_criterion=.001
        config.registration.max_num_threads=2
        config.adaptive_threshold.fixed_threshold=.15
        self.icp=KissICP(config)
        (output/'kiss_icp.json').write_text(config.model_dump_json(indent=2))
        self.file=(output/'slam.csv').open('w',buffering=1)
        self.writer=csv.writer(self.file)
        self.writer.writerow(['t','x','y','z','qw','qx','qy','qz','vx','vy','vz','quality','points','processing_s'])
        self.adapter=None
        if backend=='fastlio':
            from fastlio_adapter import FastLIOAdapter
            self.adapter=FastLIOAdapter(self)
        self.thread=threading.Thread(target=self.work,daemon=True)
        self.thread.start()

    def receive(self, message):
        if stamp(message)<3: return
        try: self.q.put_nowait(message)
        except queue.Full: self.dropped+=1

    def work(self):
        packets=[]
        packet_times=[]
        nominal_times=[]
        frame_index=0
        previous=None
        velocity=np.zeros(3)
        first_t=None
        try:
            while self.running or not self.q.empty():
                try: message=self.q.get(timeout=.2)
                except queue.Empty: continue
                t=stamp(message)
                count=4000 # 50 snapshots/s * 4000 rays = 200,000 points/s.
                ids=(np.arange(count)+self.cursor)%len(self.azimuth)
                self.cursor=(self.cursor+count)%len(self.azimuth)
                az=self.azimuth[ids]; el=self.elevation[ids]
                h=np.rint((az-message.angle_min)/message.angle_step).astype(int)
                v=np.rint((el-message.vertical_angle_min)/message.vertical_angle_step).astype(int)
                h=np.clip(h,0,message.count-1); v=np.clip(v,0,message.vertical_count-1)
                ranges=np.asarray(message.ranges)[v*message.count+h]
                # Use the actual ray direction, not a different direction at its range.
                az=message.angle_min+h*message.angle_step
                el=message.vertical_angle_min+v*message.vertical_angle_step
                valid=np.isfinite(ranges)&(ranges>=.4)&(ranges<=40)
                ranges=ranges[valid]+self.rng.normal(0,.02,np.count_nonzero(valid))
                az=az[valid]; el=el[valid]
                xyz=ranges[:,None]*np.column_stack((np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)))
                packets.append(xyz)
                packet_times.append(np.full(len(xyz),t))
                nominal_times.append((t-.02+(np.arange(count)+.5)/200000)[valid])
                if len(packets)<5: continue
                started=time.monotonic()
                points=np.concatenate(packets)
                times=np.concatenate(packet_times)
                nominal=np.concatenate(nominal_times)
                packets=[]; packet_times=[]; nominal_times=[]
                if first_t is None:
                    first_t=t
                    # Choose the tunnel axis as the map's X direction from LiDAR
                    # geometry alone. No simulated position/orientation is used.
                    body_points=points@np.diag([1.,-1.,-1.])
                    # Side walls, excluding track bed, for initial axis fit.
                    fitting=body_points[(abs(body_points[:,1])>1.5)&(abs(body_points[:,0])<25)][::3]
                    def residual(v):
                        yaw,cy,cz,radius=v
                        lateral=-np.sin(yaw)*fitting[:,0]+np.cos(yaw)*fitting[:,1]
                        return np.sqrt((lateral-cy)**2+(fitting[:,2]-cz)**2)-radius
                    fit=least_squares(residual,[0,0,1.7,2.5],loss='soft_l1',f_scale=.03,
                                      bounds=([-.3,-1,0,2.2],[.3,1,3,2.8]))
                    self.icp.last_pose[:3,:3]=Rotation.from_euler('z',-fit.x[0]).as_matrix()
                    if self.adapter: self.adapter.alignment=self.icp.last_pose[:3,:3].copy()
                    (self.output/'lidar_initial_alignment.json').write_text(json.dumps({
                        'method':'Robust cylinder axis fit to first LiDAR scan, no ground truth',
                        'axis_yaw_in_lidar_rad':fit.x[0],'cylinder_center_yz_m':fit.x[1:3].tolist(),
                        'radius_m':fit.x[3],'median_abs_residual_m':float(np.median(abs(residual(fit.x))))},indent=2))
                if self.adapter:
                    self.adapter.publish_cloud(points,times)
                    np.savez_compressed(self.output/'clouds'/f'{frame_index:06d}.npz',
                                        xyz=points.astype(np.float32),time=times,nominal_time=nominal)
                    self.publish(points,times)
                    frame_index+=1
                    continue
                points=points@np.diag([1.,-1.,-1.])
                normalized=(times-times.min())/max(.001,times.max()-times.min())
                self.icp.register_frame(points,normalized)
                pose=self.icp.last_pose.copy()
                rotation=pose[:3,:3]
                # Correct the LiDAR-to-body lever arm, referenced to the first body pose.
                position=pose[:3,3]+np.array([0,0,.18])-rotation@np.array([0,0,.18])
                if previous is not None:
                    dt=t-previous[0]
                    if dt<=0: raise RuntimeError('Non-monotonic LiDAR time')
                    raw_velocity=(position-previous[1])/dt
                    velocity=.6*velocity+.4*raw_velocity
                    if np.linalg.norm(raw_velocity)>8: raise RuntimeError('ICP translation jumped (>8 m/s)')
                previous=(t,position)
                qxyzw=Rotation.from_matrix(rotation).as_quat()
                qwxyz=np.roll(qxyzw,1)
                quality=80 if len(points)>5000 else 0
                state={'t':t,'position':position,'rotation':rotation,'velocity':velocity.copy(),'quality':quality,'frames':frame_index+1}
                self.writer.writerow([t,*position,*qwxyz,*velocity,quality,len(points),time.monotonic()-started])
                np.savez_compressed(self.output/'clouds'/f'{frame_index:06d}.npz',xyz=points.astype(np.float32),time=times,nominal_time=nominal,pose=pose)
                if frame_index%10==0:
                    np.save(self.output/'local_map.npy',self.icp.local_map.point_cloud().astype(np.float32))
                self.publish(points,times)
                with self.lock: self.state=state
                frame_index+=1
        except Exception:
            self.error=traceback.format_exc()
            (self.output/'slam_error.txt').write_text(self.error)

    def publish(self,points,times):
        dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4'),('timestamp','<f8')])
        data=np.empty(len(points),dtype=dtype)
        for i,axis in enumerate('xyz'): data[axis]=points[:,i]
        data['intensity']=100 # synthetic constant; not a reflectance model
        data['timestamp']=times
        message=self.message_type()
        message.header.stamp.sec=int(times[-1])
        message.header.stamp.nsec=int((times[-1]%1)*1e9)
        header=message.header.data.add(); header.key='frame_id'; header.value.append('mid360_link')
        for name in dtype.names:
            field=message.field.add(); field.name=name; field.offset=dtype.fields[name][1]
            field.datatype=7 if name=='timestamp' else 6; field.count=1
        message.height=1; message.width=len(data); message.point_step=dtype.itemsize
        message.row_step=len(data)*dtype.itemsize; message.is_dense=True; message.data=data.tobytes()
        self.publisher.publish(message)

    def snapshot(self):
        with self.lock: return self.state

    def close(self):
        self.running=False
        self.thread.join(timeout=20)
        if self.adapter: self.adapter.close()
        self.file.close()
