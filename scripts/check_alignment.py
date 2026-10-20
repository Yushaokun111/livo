#!/usr/bin/env python3
"""Numerically check alignment length, C2 joins, gauge, and cant; export a plan."""
import json
import math
import os
from pathlib import Path
from alignment import Alignment, check_design
from geometry import ROOT


def checks(c):
    a = check_design(c)
    report = {}
    ds = .25
    pts = [a.at(min(i*ds,a.length))[:2] for i in range(math.ceil(a.length/ds)+1)]
    chord_length = sum(math.dist(p,q) for p,q in zip(pts,pts[1:]))
    report['integrated_length_m'] = chord_length
    report['length_error_m'] = abs(chord_length-a.length)
    report['max_join_position_error_m'] = 0.
    report['max_join_heading_error_rad'] = 0.
    report['max_join_curvature_error_1_m'] = 0.
    for before,after in zip(a.segments,a.segments[1:]):
        dx,dy,theta,kappa = a.local(before,before['length'])
        angle = before['yaw0']
        x = before['x0']+math.cos(angle)*dx-math.sin(angle)*dy
        y = before['y0']+math.sin(angle)*dx+math.cos(angle)*dy
        report['max_join_position_error_m'] = max(report['max_join_position_error_m'],math.hypot(x-after['x0'],y-after['y0']))
        report['max_join_heading_error_rad'] = max(report['max_join_heading_error_rad'],abs(angle+theta-after['yaw0']))
        report['max_join_curvature_error_1_m'] = max(report['max_join_curvature_error_1_m'],abs(kappa-a.local(after,0)[3]))
    curvature_errors,gauge_errors,cant_errors = [],[],[]
    for seg in a.segments:
        for fraction in (.1,.5,.9):
            s = seg['s0']+seg['length']*fraction
            delta = .02
            p0,p1,p2 = (a.at(s+t) for t in (-delta,0,delta))
            vx,vy = ((p2[i]-p0[i])/(2*delta) for i in (0,1))
            ax,ay = ((p2[i]-2*p1[i]+p0[i])/(delta*delta) for i in (0,1))
            numerical = (vx*ay-vy*ax)/(vx*vx+vy*vy)**1.5
            curvature_errors.append(abs(numerical-p1[3]))
            left,right = (a.point(s,side*c['track_gauge']/2,.25,True) for side in (1,-1))
            gauge_errors.append(abs(math.dist(left,right)-c['track_gauge']))
            left,right = (a.point(s,side*(c['track_gauge']+.07)/2,.25,True) for side in (1,-1))
            cant_errors.append(abs(right[2]-left[2]-a.cant(seg,seg['length']*fraction)))
    report['max_numerical_curvature_error_1_m'] = max(curvature_errors)
    report['max_gauge_error_m'] = max(gauge_errors)
    report['max_cant_error_m'] = max(cant_errors)
    report['max_cant_ramp_per_mille'] = max(seg.get('cant_mm',0)/seg['length'] for seg in a.segments if seg['type'].startswith('transition'))
    report['checks'] = {
        'length':report['length_error_m']<.001,
        'position_continuity':report['max_join_position_error_m']<1e-8,
        'heading_continuity':report['max_join_heading_error_rad']<1e-9,
        'curvature_continuity':report['max_join_curvature_error_1_m']<1e-10,
        'numerical_curvature':report['max_numerical_curvature_error_1_m']<2e-8,
        'gauge':report['max_gauge_error_m']<1e-8,
        'cant':report['max_cant_error_m']<1e-8,
        'cant_ramp':report['max_cant_ramp_per_mille']<=2,
    }
    # Ensure invalid modifications are rejected rather than silently rescaling radii.
    for change in ('length','speed','radius','short_transition'):
        bad = json.loads(json.dumps(c))
        if change == 'length': bad['length'] += 1
        if change == 'speed': bad['design_speed_kmh'] = 100
        if change == 'radius': bad['alignment'][2]['radius'] = 250
        if change == 'short_transition': bad['alignment'][1]['length'] = 20
        try: check_design(bad)
        except ValueError: report['checks']['reject_'+change] = True
        else: report['checks']['reject_'+change] = False
    return report


def plot(c,output):
    os.environ.setdefault('MPLCONFIGDIR','/tmp/livo-matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    a = Alignment(c)
    fig,(ax,curv) = plt.subplots(2,1,figsize=(13,8),gridspec_kw={'height_ratios':[2.6,1]},layout='constrained')
    colors = {'straight':'#586b7c','arc':'#157e85','transition_in':'#e4a33d','transition_out':'#e4a33d'}
    for index,seg in enumerate(a.segments):
        ss = [seg['s0']+seg['length']*i/100 for i in range(101)]
        states = [a.at(s) for s in ss]
        ax.plot([p[0] for p in states],[p[1] for p in states],color=colors[seg['type']],lw=5)
        curv.plot(ss,[p[3]*1000 for p in states],color=colors[seg['type']],lw=3)
        m = states[50]
        label = f"{seg['length']:g} m"
        if seg['type']=='arc': label += f" / R={seg['radius']} m"
        if not seg['type'].startswith('transition'):
            ax.annotate(label,(m[0],m[1]),xytext=(0,18),textcoords='offset points',ha='center',fontsize=10)
        curv.axvline(seg['s0'],color='#d7dde3',lw=.6)
    ax.scatter([0,a.at(a.length)[0]],[0,a.at(a.length)[1]],color='#162d45',s=45,zorder=3)
    ax.annotate('K0+000',(0,0),xytext=(0,12),textcoords='offset points')
    ax.annotate('K2+200',a.at(a.length)[:2],xytext=(-60,-24),textcoords='offset points')
    ax.set_aspect('equal',adjustable='datalim')
    ax.set_xlabel('World X [m]'); ax.set_ylabel('World Y [m]')
    ax.set_title('METRO TUNNEL / 2200 m centerline / 5 m bore',loc='left',fontsize=17,fontweight='bold',pad=18)
    ax.text(.01,.97,'80 km/h design basis  |  cubic transitions: 4 x 80 m  |  cant: 120 / 95 mm',transform=ax.transAxes,va='top',color='#586b7c',fontsize=10)
    curv.set_xlabel('Chainage s [m]'); curv.set_ylabel('Curvature [1/km]')
    for axes in (ax,curv):
        axes.spines[['top','right']].set_visible(False)
        axes.grid(True,alpha=.12)
    fig.savefig(output/'alignment_plan.png',dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    c = json.loads((ROOT/'config/tunnel.json').read_text())
    output = ROOT/'output'; output.mkdir(exist_ok=True)
    report = checks(c)
    (output/'alignment_checks.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not all(report['checks'].values()): raise SystemExit('Alignment validation failed')
    plot(c,output)
