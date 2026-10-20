#!/usr/bin/env python3
"""Generate a self-contained, periodic metro tunnel. Python standard library only."""
import argparse
import json
import math
from pathlib import Path
import random
import struct
import xml.etree.ElementTree as ET
import zlib

ROOT = Path(__file__).resolve().parents[1]


def xml(parent, tag, value=None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if value is not None:
        child.text = str(value)
    return child


def numbers(values):
    return " ".join(f"{v:.7g}" for v in values)


def write_xml(root, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def write_png(path, width, height, rgb):
    def chunk(kind, data):
        return (struct.pack("!I", len(data)) + kind + data
                + struct.pack("!I", zlib.crc32(kind + data) & 0xffffffff))
    scanlines = b"".join(b"\0" + rgb[y * width * 3:(y + 1) * width * 3]
                         for y in range(height))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack("!2I5B", width, height, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(scanlines, 9)) + chunk(b"IEND", b""))


def concrete_texture(path, contrast, seed):
    rng = random.Random(seed)
    size = 256
    coarse = [[rng.uniform(-1, 1) for _ in range(16)] for _ in range(16)]
    pixels = bytearray()
    for y in range(size):
        for x in range(size):
            gx, gy = x / 16, y / 16
            ix, iy = int(gx), int(gy)
            tx, ty = gx - ix, gy - iy
            smooth = sum(coarse[(iy + dy) % 16][(ix + dx) % 16]
                         * (tx if dx else 1 - tx) * (ty if dy else 1 - ty)
                         for dx in (0, 1) for dy in (0, 1))
            value = int(173 + contrast * (45 * smooth + 20 * rng.uniform(-1, 1)))
            pixels.extend((value, value, max(0, value - 3)))
    write_png(path, size, size, pixels)


class Mesh:
    def __init__(self):
        self.vertices, self.normals, self.uvs = [], [], []

    def quad(self, points, normals, uv=None):
        uv = uv or [(0, 0), (1, 0), (1, 1), (0, 1)]
        for index in (0, 1, 2, 0, 2, 3):
            self.vertices.extend(points[index])
            self.normals.extend(normals[index])
            self.uvs.extend(uv[index])

    def box(self, center, size):
        # Counterclockwise faces viewed from outside.
        x, y, z = center
        a, b, c = [v / 2 for v in size]
        points = [(x + dx*a, y + dy*b, z + dz*c) for dx, dy, dz in
                  [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),
                   (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]]
        for face, normal in [((0,3,2,1),(0,0,-1)),((4,5,6,7),(0,0,1)),
                             ((0,1,5,4),(0,-1,0)),((3,7,6,2),(0,1,0)),
                             ((0,4,7,3),(-1,0,0)),((1,2,6,5),(1,0,0))]:
            self.quad([points[i] for i in face], [normal]*4)

    def tube_surface(self, x0, x1, radius, center_z, segments, inward=True,
                     theta0=0, theta1=2*math.pi, uv_period=1.5):
        for i in range(segments):
            a = theta0 + (theta1-theta0)*i/segments
            b = theta0 + (theta1-theta0)*(i+1)/segments
            points = [(x, radius*math.cos(t), center_z+radius*math.sin(t))
                      for x,t in [(x0,a),(x1,a),(x1,b),(x0,b)]]
            sign = -1 if inward else 1
            normals = [(0,sign*math.cos(t),sign*math.sin(t)) for t in (a,a,b,b)]
            uv = [(x/uv_period, radius*t/uv_period) for x,t in [(x0,a),(x1,a),(x1,b),(x0,b)]]
            if not inward:
                points.reverse(); normals.reverse(); uv.reverse()
            self.quad(points, normals, uv)

    def save(self, path):
        root = ET.Element("COLLADA", {"xmlns":"http://www.collada.org/2005/11/COLLADASchema", "version":"1.4.1"})
        asset = xml(root, "asset")
        xml(asset, "created", "2026-10-20T00:00:00Z")
        xml(asset, "modified", "2026-10-20T00:00:00Z")
        xml(asset, "unit", name="meter", meter="1")
        xml(asset, "up_axis", "Z_UP")
        geometry = xml(xml(root,"library_geometries"),"geometry",id="mesh",name="mesh")
        mesh = xml(geometry,"mesh")
        for name, values, fields in [("positions",self.vertices,"XYZ"),("normals",self.normals,"XYZ"),("uv",self.uvs,"ST")]:
            source = xml(mesh,"source",id=name)
            xml(source,"float_array",numbers(values),id=name+"-array",count=str(len(values)))
            accessor = xml(xml(source,"technique_common"),"accessor",source="#"+name+"-array",count=str(len(values)//len(fields)),stride=str(len(fields)))
            for field in fields:
                xml(accessor,"param",name=field,type="float")
        vertices = xml(mesh,"vertices",id="vertices")
        xml(vertices,"input",semantic="POSITION",source="#positions")
        count = len(self.vertices)//3
        triangles = xml(mesh,"triangles",count=str(count//3))
        for offset, semantic, source in [(0,"VERTEX","vertices"),(1,"NORMAL","normals"),(2,"TEXCOORD","uv")]:
            attrs = {"semantic":semantic,"source":"#"+source,"offset":str(offset)}
            if semantic == "TEXCOORD": attrs["set"] = "0"
            xml(triangles,"input",**attrs)
        xml(triangles,"p"," ".join(f"{i} {i} {i}" for i in range(count)))
        scene = xml(xml(root,"library_visual_scenes"),"visual_scene",id="scene")
        xml(xml(scene,"node",id="node"),"instance_geometry",url="#mesh")
        xml(xml(root,"scene"),"instance_visual_scene",url="#scene")
        write_xml(root,path)


def collision(link, name, center, size, roll=0):
    col = xml(link,"collision",name=name+"_collision")
    xml(col,"pose",numbers([*center,roll,0,0]))
    xml(xml(xml(col,"geometry"),"box"),"size",numbers(size))
    return col


