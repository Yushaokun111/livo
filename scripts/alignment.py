"""Arc-length-parametrized metro alignment, with exact cubic-parabola transitions."""
from bisect import bisect_right
from functools import lru_cache
import math


def rotate(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c*x-s*y, s*x+c*y


def cubic_arc(x, a):
    # Five-point Gauss-Legendre quadrature; slopes here are below 0.1 rad.
    nodes = (0, -.5384693101056831, .5384693101056831, -.9061798459386640, .9061798459386640)
    weights = (.5688888888888889, .4786286704993665, .4786286704993665, .2369268850561891, .2369268850561891)
    return x/2 * sum(w*math.sqrt(1+9*a*a*(x*(u+1)/2)**4) for u,w in zip(nodes,weights))


class CubicTransition:
    """y=a*x^3, solved for exact arc length L and endpoint curvature 1/R."""
    def __init__(self, length, radius):
        lo, hi = 1e-10, .3
        for _ in range(70):
            q = (lo+hi)/2
            x = 2*radius*q/(1+q*q)**1.5
            a = q/(3*x*x)
            if cubic_arc(x,a) < length: lo = q
            else: hi = q
        self.length, self.radius = length,radius
        self.x = 2*radius*q/(1+q*q)**1.5
        self.a = q/(3*self.x*self.x)
        self.angle = math.atan(q)

    def entry(self, s):
        x = self.x*s/self.length
        for _ in range(5):
            x -= (cubic_arc(x,self.a)-s)/math.sqrt(1+9*self.a*self.a*x**4)
        slope = 3*self.a*x*x
        return x,self.a*x**3,math.atan(slope),6*self.a*x/(1+slope*slope)**1.5

    def evaluate(self, s, entering):
        if entering: return self.entry(s)
        x,y,yaw,kappa = self.entry(self.length-s)
        dx,dy = rotate(self.x-x,-(self.a*self.x**3-y),self.angle)
        return dx,dy,self.angle-yaw,kappa


class Alignment:
    def __init__(self, config):
        self.length = config["length"]
        self.gauge_centers = config["track_gauge"]+.07
        self.segments, self.starts = [],[]
        s,x,y,yaw = 0.,0.,0.,0.
        for source in config["alignment"]:
            seg = dict(source,s0=s,x0=x,y0=y,yaw0=yaw)
            length = seg["length"]
            if not math.isfinite(length) or length <= 0: raise ValueError("Segment length must be positive")
            if seg["type"] in ("transition_in","transition_out"):
                seg["cubic"] = CubicTransition(length,seg["radius"])
            elif seg["type"] not in ("straight","arc"):
                raise ValueError("Unknown alignment segment type")
            self.starts.append(s); self.segments.append(seg)
            dx,dy,dyaw,_ = self.local(seg,length)
            dx,dy = rotate(dx,dy,yaw)
            x,y,yaw,s = x+dx,y+dy,yaw+dyaw,s+length
        if not math.isclose(s,self.length,abs_tol=1e-7):
            raise ValueError(f"Alignment sums to {s:g} m, but length is {self.length:g} m")
        for a,b in zip(self.segments,self.segments[1:]):
            if abs(self.local(a,a["length"])[3]-self.local(b,0)[3]) > 1e-10:
                raise ValueError("Curvature must be continuous at every segment boundary")
            if abs(self.cant(a,a["length"])-self.cant(b,0)) > 1e-10:
                raise ValueError("Cant must be continuous at every segment boundary")

    @staticmethod
    def local(seg,s):
        kind = seg["type"]
        if kind == "straight": return s,0.,0.,0.
        sign,r = seg["direction"],seg["radius"]
        if kind == "arc":
            return r*math.sin(s/r),sign*r*(1-math.cos(s/r)),sign*s/r,sign/r
        x,y,yaw,kappa = seg["cubic"].evaluate(s,kind=="transition_in")
        return x,sign*y,sign*yaw,sign*kappa

    @staticmethod
    def cant(seg,s):
        if seg["type"] == "straight": return 0.
        ratio = 1.
        if seg["type"] == "transition_in": ratio = s/seg["length"]
        if seg["type"] == "transition_out": ratio = 1-s/seg["length"]
        return seg["direction"]*seg["cant_mm"]*.001*ratio

    @lru_cache(maxsize=32768)
    def at(self,s):
        if not -1e-6 <= s <= self.length+1e-6: raise ValueError(f"Station {s} outside alignment")
        s = max(0.,min(self.length,s))
        seg = self.segments[max(0,bisect_right(self.starts,s)-1)]
        local_s = s-seg["s0"]
        dx,dy,yaw,kappa = self.local(seg,local_s)
        dx,dy = rotate(dx,dy,seg["yaw0"])
        cant = self.cant(seg,local_s)
        roll = -math.asin(cant/self.gauge_centers)
        return seg["x0"]+dx,seg["y0"]+dy,seg["yaw0"]+yaw,kappa,roll

    def point(self,s,y,z,banked=False):
        x0,y0,yaw,_,roll = self.at(s)
        if banked:
            y,z = rotate(y,z-.25,roll)
            z += .25
        dx,dy = rotate(0,y,yaw)
        return x0+dx,y0+dy,z

    def vector(self,s,normal,banked=False):
        _,_,yaw,_,roll = self.at(s)
        x,y,z = normal
        if banked: y,z = rotate(y,z,roll)
        x,y = rotate(x,y,yaw)
        return x,y,z


def check_design(config):
    """Guard the explicitly verified 80 km/h A/B main-line design envelope."""
    if config["design_speed_kmh"] != 80:
        raise ValueError("This design profile verifies 80 km/h only; recheck GB tables before changing speed")
    table = {600:(70,120),800:(55,95)}
    rows = config["alignment"]
    for i,seg in enumerate(rows):
        if seg["type"] == "straight":
            if 0 < i < len(rows)-1 and seg["length"] < .5*config["design_speed_kmh"]:
                raise ValueError("Intermediate straight must be at least 40 m")
            continue
        if seg["radius"] not in table: raise ValueError("Verified 80 km/h curve radii are 600 m and 800 m")
        minimum_length,cant = table[seg["radius"]]
        if seg["direction"] not in (-1,1): raise ValueError("Curve direction must be +1 or -1")
        if seg["cant_mm"] != cant: raise ValueError("Cant differs from the verified GB 50157 table value")
        if seg["type"] == "arc":
            if seg["length"] < 25: raise ValueError("A-type main-line circular arc must be at least 25 m")
        elif seg["type"] in ("transition_in","transition_out"):
            if not minimum_length <= seg["length"] <= seg["radius"]*.25:
                raise ValueError("Transition length outside validated range")
            if cant*.001/seg["length"] > .002+1e-10: raise ValueError("Cant ramp exceeds 2 per mille")
    return Alignment(config)
