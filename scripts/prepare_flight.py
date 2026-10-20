#!/usr/bin/env python3
"""Prepare a MID-360-equipped Iris and a tunnel flight world."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from geometry import ROOT, xml, write_xml


def prepare(output, fdm_port=19002):
    tree = ET.parse(ROOT/'worlds/metro_tunnel.sdf')
    world = tree.getroot().find('world')
    gui = world.find('gui')
    if gui is not None:
        world.remove(gui)
    world.find('physics/max_step_size').text = '0.001'
    # Lower wall-clock rate gives the laser estimator time to process each scan.
    world.find('physics/real_time_factor').text = '0.25'
    model = ET.parse(ROOT/'dependencies/ardupilot_gazebo/iris_with_ardupilot.sdf').getroot().find('model')
    model.set('name','test_quad')
    xml(model,'pose','100.5 0 0.205 0 0 0')
    plugin = model.find("plugin[@name='ArduPilotPlugin']")
    plugin.find('fdm_port_in').text = str(fdm_port)
    # World +X is north, +Y west, +Z up; body is forward-left-up.
    plugin.find('gazeboXYZToNED').text = '0 0 0 180 0 0'
    plugin.find('no_time_sync').text = '0'
    link = xml(model,'link',name='mid360_link')
    # Invert the MID-360 so its broad vertical sector observes the track bed.
    # The repetitive sleepers provide axial geometry absent from smooth lining.
    xml(link,'pose','0 0 0.18 3.141592653589793 0 0')
    inertial=xml(link,'inertial'); xml(inertial,'mass',.265)
    inertia=xml(inertial,'inertia')
    for axis in ['ixx','iyy','izz']: xml(inertia,axis,.00018)
    visual=xml(link,'visual',name='mid360_housing')
    xml(xml(xml(visual,'geometry'),'box'),'size','.065 .065 .06')
    material=xml(visual,'material'); xml(material,'diffuse','.15 .15 .17 1')
    joint=xml(model,'joint',name='mid360_mount',type='fixed')
    xml(joint,'parent','iris_with_standoffs::base_link'); xml(joint,'child','mid360_link')
    sensor=xml(link,'sensor',name='mid360_range_image',type='gpu_lidar')
    xml(sensor,'topic','/mid360/range_image'); xml(sensor,'always_on','true'); xml(sensor,'update_rate',50)
    lidar=xml(sensor,'lidar'); scan=xml(lidar,'scan')
    for name,n,lo,hi in [('horizontal',1801,-math.pi,math.pi),('vertical',120,math.radians(-7),math.radians(52))]:
        axis=xml(scan,name)
        for key,value in [('samples',n),('resolution',1),('min_angle',lo),('max_angle',hi)]: xml(axis,key,value)
    limits=xml(lidar,'range')
    for key,value in [('min',.1),('max',40),('resolution',.002)]: xml(limits,key,value)
    # Noise is added after angular resampling, independently for each emitted point.
    imu=xml(link,'sensor',name='mid360_imu',type='imu')
    xml(imu,'topic','/mid360/imu'); xml(imu,'always_on','true'); xml(imu,'update_rate',200)
    world.append(model)
    write_xml(tree.getroot(),output)


if __name__=='__main__':
    prepare(ROOT/'worlds/tunnel_mid360.sdf')
