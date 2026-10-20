#!/usr/bin/env python3
"""Reconstruct an estimated map from saved MID-360 scans and SLAM poses only."""
import argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--stride',type=int,default=10)
    parser.add_argument('--voxel',type=float,default=.08)
    args=parser.parse_args(); run=args.run
    pose=np.genfromtxt(run/'slam.csv',delimiter=',',names=True)
    times=pose['t']; xyz=np.column_stack([pose[a] for a in 'xyz'])
    rot=Rotation.from_quat(np.column_stack([pose[a] for a in ['qx','qy','qz','qw']]))
    interpolate=Slerp(times,rot)
    clouds=[]; mount=np.diag([1.,-1.,-1.]); lever=np.array([0,0,.18])
    for file in sorted((run/'clouds').glob('*.npz'))[::args.stride]:
        data=np.load(file); points=data['xyz']; stamps=data['time']
        for t in np.unique(stamps):
            if t<times[0] or t>times[-1]: continue
            R=interpolate(t).as_matrix()
            position=np.array([np.interp(t,times,xyz[:,i]) for i in range(3)])
            # Base relative pose -> lidar optical pose. No truth.csv is read.
            transformed=(points[stamps==t]@mount.T+lever)@R.T+position
            clouds.append(transformed.astype(np.float32))
    cloud=np.concatenate(clouds)
    _,idx=np.unique(np.floor(cloud/args.voxel).astype(np.int32),axis=0,return_index=True)
    cloud=cloud[idx]
    np.save(run/'estimated_map.npy',cloud)
    with (run/'estimated_map.ply').open('wb') as file:
        file.write(('ply\nformat binary_little_endian 1.0\ncomment Estimated SLAM map; no ground truth\n'
                    f'element vertex {len(cloud)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n').encode())
        file.write(cloud.astype('<f4').tobytes())
    print(f'Estimated map: {len(cloud)} points, {run / "estimated_map.ply"}')


if __name__=='__main__': main()
