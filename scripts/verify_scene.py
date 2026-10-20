#!/usr/bin/env python3
"""Render the full Gazebo tunnel and measure straights, both bends and contacts."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from geometry import ROOT, numbers, write_png, write_xml, xml
from alignment import Alignment


def diagnostic_world(config,path):
    a = Alignment(config)
    tree = ET.parse(ROOT/'worlds/metro_tunnel.sdf')
    world = tree.getroot().find('world')
    gui = world.find('gui')
    if gui is not None: world.remove(gui)
    camera_stations = {'left':520.,'right':1480.,'straight':200.}
    scans = {'straight':200.,'left':680.,'right':1640.,'transition_in':440.,'transition_out':1880.}
    for name,station in camera_stations.items():
        model = xml(world,'model',name='inspection_camera_'+name)
        xml(model,'static','true')
        xml(model,'pose',numbers([*a.point(station,0,1.55),0,.035,a.at(station)[2]]))
        sensor = xml(xml(model,'link',name='camera_link'),'sensor',name='camera',type='camera')
        xml(sensor,'topic','/inspection/image/'+name)
        xml(sensor,'always_on','true'); xml(sensor,'update_rate',2)
        camera = xml(sensor,'camera'); xml(camera,'horizontal_fov',1.5)
        image = xml(camera,'image')
        xml(image,'width',1200); xml(image,'height',750); xml(image,'format','R8G8B8')
        clip = xml(camera,'clip'); xml(clip,'near',.05); xml(clip,'far',180)
    for name,station in scans.items():
        model = xml(world,'model',name='inspection_lidar_'+name)
        xml(model,'static','true')
        xml(model,'pose',numbers([*a.point(station,0,config['axis_center_z']),0,0,a.at(station)[2]]))
        sensor = xml(xml(model,'link',name='lidar_link'),'sensor',name='lidar',type='gpu_lidar')
        xml(sensor,'topic','/inspection/scan/'+name)
        xml(sensor,'always_on','true'); xml(sensor,'update_rate',2)
        lidar = xml(sensor,'lidar'); horizontal = xml(xml(lidar,'scan'),'horizontal')
        for tag,value in [('samples',7201),('resolution',1),('min_angle',-math.pi),('max_angle',math.pi)]: xml(horizontal,tag,value)
        limits = xml(lidar,'range')
        for tag,value in [('min',.05),('max',150),('resolution',.005)]: xml(limits,tag,value)
    probes = {}
    radius = .05
    for location,station in [('straight',200.),('left',680.),('right',1640.),('transition',440.)]:
        station = config['sleeper_spacing']*round(station/config['sleeper_spacing'])
        roll = a.at(station)[4]
        for surface,lateral,z in [('floor',0,0),('rail_left',(config['track_gauge']+.07)/2,.25),('rail_right',-(config['track_gauge']+.07)/2,.25)]:
            name = location+'_'+surface+'_probe'
            x,y,height = a.point(station,lateral,z,True)
            probes[name] = [x,y,height+radius/math.cos(roll)]
            add_probe(world,name,x,y,height+.6,radius)
        # Vertical guide prevents a sphere sliding down the steep curved wall.
        name = location+'_wall_probe'
        lateral = 2.35
        x,y,_ = a.point(station,lateral,0)
        zc = config['axis_center_z']
        height = zc-math.sqrt((config['inner_diameter']/2-radius)**2-lateral**2)
        probes[name] = [x,y,height]
        add_probe(world,name,x,y,zc,radius)
    write_xml(tree.getroot(),path)
    return camera_stations,scans,probes


def add_probe(world,name,x,y,z,radius):
    model = xml(world,'model',name=name)
    xml(model,'pose',numbers([x,y,z,0,0,0]))
    link = xml(model,'link',name='ball')
    inertial = xml(link,'inertial'); xml(inertial,'mass',1)
    inertia = xml(inertial,'inertia')
    for axis in ('ixx','iyy','izz'): xml(inertia,axis,.4*radius**2)
    xml(xml(xml(xml(link,'collision',name='ball_collision'),'geometry'),'sphere'),'radius',radius)
    # Tests contacts in Z without rolling off the deliberately superelevated rails.
    joint = xml(model,'joint',name='vertical_guide',type='prismatic')
    xml(joint,'parent','world'); xml(joint,'child','ball')
    axis = xml(joint,'axis'); xml(axis,'xyz','0 0 1')
    limits = xml(axis,'limit'); xml(limits,'lower',-3); xml(limits,'upper',1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'output')
    parser.add_argument('--software',action='store_true')
    parser.add_argument('--timeout',type=float,default=180)
    args = parser.parse_args()
    output = args.output.resolve(); output.mkdir(parents=True,exist_ok=True)
    config = json.loads((ROOT/'config/generated.json').read_text())
    os.environ['GZ_PARTITION'] = 'tunnel-check-'+uuid.uuid4().hex
    os.environ['GZ_SIM_RESOURCE_PATH'] = str(ROOT/'models')+os.pathsep+os.environ.get('GZ_SIM_RESOURCE_PATH','')
    if args.software: os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image, RGB_INT8
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.pose_v_pb2 import Pose_V
    node = Node()
    state,lock = {'images':{},'scans':{},'poses':{},'frames':{},'sim_time':0},threading.Lock()
    callbacks = []
    def image_cb(name):
        def callback(message):
            with lock:
                state['images'][name]=message
                state['frames'][name]=state['frames'].get(name,0)+1
        callbacks.append(callback)
        return callback
    def scan_cb(name):
        def callback(message):
            with lock: state['scans'][name]=message
        callbacks.append(callback)
        return callback
    def pose_cb(message):
        with lock:
            state['sim_time']=message.header.stamp.sec+message.header.stamp.nsec*1e-9
            for pose in message.pose:
                if pose.name.endswith('_probe'):
                    state['poses'][pose.name]=[pose.position.x,pose.position.y,pose.position.z]
    with tempfile.TemporaryDirectory(prefix='metro-tunnel-check-') as directory:
        path = Path(directory)/'inspection.sdf'
        cameras,scans,probes = diagnostic_world(config,path)
        for name in cameras: node.subscribe(Image,'/inspection/image/'+name,image_cb(name))
        for name in scans: node.subscribe(LaserScan,'/inspection/scan/'+name,scan_cb(name))
        node.subscribe(Pose_V,'/world/metro_tunnel/dynamic_pose/info',pose_cb)
        with (output/'gazebo.log').open('w') as log:
            process = subprocess.Popen(['gz','sim','-s','-r','--headless-rendering','-v','3',str(path)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                deadline = time.monotonic()+args.timeout
                while time.monotonic()<deadline:
                    with lock:
                        ready = (all(state['frames'].get(name,0)>=3 for name in cameras)
                                 and len(state['scans'])==len(scans) and state['sim_time']>=3
                                 and len(state['poses'])==len(probes))
                    if ready: break
                    if process.poll() is not None: raise RuntimeError(f'Gazebo exited with {process.returncode}; see {output / "gazebo.log"}')
                    time.sleep(.1)
                else:
                    with lock: details = {k:len(v) if isinstance(v,dict) else v for k,v in state.items()}
                    raise RuntimeError(f'Timed out: {details}; see {output / "gazebo.log"}')
            finally:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
                    try: process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGKILL); process.wait()
    report={'checks':{},'lidar_ranges_m':{},'contacts':{},'scene_length_m':config['length']}
    for name,message in state['images'].items():
        if message.pixel_format_type != RGB_INT8: raise RuntimeError('Expected RGB8 image')
        rgb=b''.join(message.data[y*message.step:y*message.step+message.width*3] for y in range(message.height))
        filename = 'tunnel_preview.png' if name=='left' else 'tunnel_'+name+'.png'
        write_png(output/filename,message.width,message.height,rgb)
        report['checks'][name+'_render']=max(rgb)-min(rgb)>40
    r=config['inner_diameter']/2
    for name,scan in state['scans'].items():
        count=len(scan.ranges)-1
        samples={'right_wall':scan.ranges[count//4],'left_wall':scan.ranges[3*count//4],'forward':scan.ranges[count//2],'back':scan.ranges[0]}
        report['lidar_ranges_m'][name]={key:value if math.isfinite(value) else None for key,value in samples.items()}
        report['checks'][name+'_bore']=all(abs(samples[k]-r)<.04 for k in ('right_wall','left_wall'))
        if name in ('left','right'):
            radius={'left':600,'right':800}[name]
            # Opposite rays through an annulus obey t_forward*t_back=2*R*r+r².
            # This invariant avoids first-order error from GPU ray yaw quantization
            # at a grazing intersection (whose individual range error scales as R).
            measured_radius=(samples['forward']*samples['back']-r*r)/(2*r)
            report.setdefault('lidar_inferred_radius_m',{})[name]=measured_radius
            report['checks'][name+'_curvature_visibility']=abs(measured_radius-radius)<1.0
        elif name=='straight': report['checks']['straight_clear']=samples['forward']>100 and samples['back']>100
    for name,expected in probes.items():
        position=state['poses'][name]
        error=max(abs(x-y) for x,y in zip(position,expected))
        report['contacts'][name]={'expected':expected,'actual':position,'max_error_m':error}
        report['checks'][name]=error<.025
    (output/'verification.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'checks':report['checks'],'lidar_inferred_radius_m':report.get('lidar_inferred_radius_m'),
                      'max_contact_error_m':max(p['max_error_m'] for p in report['contacts'].values())},indent=2))
    if not all(report['checks'].values()): raise SystemExit('Scene checks failed')
    print(f'PASS — full 2200 m scene, previews: {output}')


if __name__ == '__main__': main()
