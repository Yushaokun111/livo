#!/usr/bin/env python3
"""Generate a 2200 m metro tunnel along a configurable horizontal alignment."""
import argparse
import json
from pathlib import Path
import math
from geometry import ROOT
from alignment import check_design
from scene_builder import build


def validate(c):
    positive = ["length","inner_diameter","wall_thickness","ring_spacing","sleeper_spacing","track_gauge","light_spacing"]
    for name in positive:
        if not isinstance(c[name], (int,float)) or not math.isfinite(c[name]) or c[name] <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if not 4.5 <= c["inner_diameter"] <= 6.0:
        raise ValueError("inner_diameter must be 4.5–6.0 m for this track/walkway layout")
    if not 12 <= c["length"] <= 10000:
        raise ValueError("length must be 12–10000 m")
    if not 0.8 <= c["track_gauge"] <= 1.6:
        raise ValueError("track_gauge must be 0.8–1.6 m")
    for name in ("ambient","texture_contrast"):
        if not 0 <= c[name] <= 1: raise ValueError(f"{name} must be 0–1")
    if not 0 <= c["light_intensity"] <= 5: raise ValueError("light_intensity must be 0–5")
    if not isinstance(c["radial_segments"],int) or not 64 <= c["radial_segments"] <= 512:
        raise ValueError("radial_segments must be an integer in 64–512")
    if c["sleeper_spacing"] < .3 or c["ring_spacing"] < .1 or c["light_spacing"] < 2:
        raise ValueError("Minimum spacing: sleepers 0.3 m, rings 0.1 m, lights 2 m")



def generate(c, output):
    validate(c)
    if not 10 <= c["chunk_length"] <= 100: raise ValueError("chunk_length must be 10–100 m")
    if not .25 <= c["mesh_step"] <= 3: raise ValueError("mesh_step must be 0.25–3 m")
    alignment = check_design(c)
    build(c,output,alignment)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",type=Path,default=ROOT/"config/tunnel.json")
    parser.add_argument("--output",type=Path,default=ROOT)
    for option in ["inner-diameter","light-spacing","light-intensity","ambient","texture-contrast"]:
        parser.add_argument("--"+option,type=float)
    args = parser.parse_args()
    c = json.loads(args.config.read_text())
    for key in c:
        value = getattr(args,key,None)
        if value is not None: c[key] = value
    try: generate(c,args.output.resolve())
    except ValueError as exc: parser.error(str(exc))


if __name__ == "__main__": main()
