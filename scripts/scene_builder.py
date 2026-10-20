"""Sweep mesh chunks along the track alignment, with banked rails and track slab."""
import csv
import json
import math
import xml.etree.ElementTree as ET
from geometry import Mesh, concrete_texture, numbers, write_xml, xml
from alignment import rotate

MATERIALS = {
    "lining":(.82,.82,.80,0,.96),"joints":(.22,.22,.21,0,1),
    "slab":(.35,.35,.33,0,1),"sleepers":(.50,.50,.47,0,.95),
    "rail_webs":(.24,.20,.17,.55,.65),"rail_heads":(.52,.54,.55,.8,.30),
    "fasteners":(.18,.17,.15,.5,.7),"walkways":(.43,.43,.40,0,.95),
    "cables":(.075,.075,.07,0,.85),"lamps":(.22,.24,.24,.4,.65),
    "diffusers":(.92,.94,.88,0,.5)}
BANKED = {"slab","sleepers","rail_webs","rail_heads","fasteners"}
COLLIDERS = {"lining","slab","sleepers","rail_webs","rail_heads","walkways"}


class RouteMesh(Mesh):
    def __init__(self, alignment, anchor, banked):
        super().__init__()
        self.alignment,self.anchor,self.banked = alignment,anchor,banked

    def quad(self,points,normals,uv=None):
        transformed,normal_vectors = [],[]
        ax,ay,heading = self.anchor
        for point,normal in zip(points,normals):
            station,y,z = point
            x,y,z = self.alignment.point(station,y,z,self.banked)
            x,y = rotate(x-ax,y-ay,-heading)
            transformed.append((x,y,z))
            nx,ny,nz = self.alignment.vector(station,normal,self.banked)
            nx,ny = rotate(nx,ny,-heading)
            normal_vectors.append((nx,ny,nz))
        super().quad(transformed,normal_vectors,uv)


def stations(start,end,spacing):
    first = max(0,math.ceil((start-spacing/2)/spacing-1e-9))
    return [spacing/2+i*spacing for i in range(first,math.ceil((end-spacing/2)/spacing-1e-9))]


def apply_material(visual,name,c):
    material = xml(visual,"material")
    cr,cg,cb,metal,rough = MATERIALS[name]
    for tag in ("ambient","diffuse"): xml(material,tag,numbers([cr,cg,cb,1]))
    xml(material,"specular","0.08 0.08 0.08 1")
    pbr = xml(xml(material,"pbr"),"metal")
    xml(pbr,"metalness",metal); xml(pbr,"roughness",rough)
    if name == "lining": xml(pbr,"albedo_map","model://metro_tunnel/materials/textures/concrete.png")
    if name == "diffusers":
        glow = min(c["light_intensity"],1)
        xml(material,"emissive",numbers([.85*glow,.9*glow,.82*glow,1]))


def build(c,output,alignment):
    model_dir = output/"models/metro_tunnel"
    mesh_dir = model_dir/"meshes"
    mesh_dir.mkdir(parents=True,exist_ok=True)
    concrete_texture(model_dir/"materials/textures/concrete.png",c["texture_contrast"],c["seed"])
    length,r,n = c["length"],c["inner_diameter"]/2,c["radial_segments"]
    zc,t = r-.4,c["wall_thickness"]
    floor_half = math.sqrt(r*r-zc*zc)
    rail_y = (c["track_gauge"]+.07)/2
    walkway_y = math.sqrt(r*r-(zc-.30)**2)-.24
    sdf = ET.Element("sdf",version="1.9")
    model = xml(sdf,"model",name="metro_tunnel")
    xml(model,"static","true")
    chunk_count = math.ceil(length/c["chunk_length"])
    generated_files = set()
    light_count,sleeper_count = 0,0
    for index in range(chunk_count):
        start,end = index*c["chunk_length"],min(length,(index+1)*c["chunk_length"])
        anchor = alignment.at((start+end)/2)[:3]
        layers = {name:RouteMesh(alignment,anchor,name in BANKED) for name in MATERIALS}
        link = xml(model,"link",name=f"section_{index:03d}")
        xml(link,"pose",numbers([anchor[0],anchor[1],0,0,0,anchor[2]]))
        cuts = {start,end}
        cuts.update(s for s in alignment.starts if start < s < end)
        cuts.update(start+i*c["mesh_step"] for i in range(1,math.ceil((end-start)/c["mesh_step"])))
        cuts = sorted(cuts)
        for a,b in zip(cuts,cuts[1:]):
            layers["lining"].tube_surface(a,b,r,zc,n,uv_period=c["ring_spacing"])
            layers["lining"].tube_surface(a,b,r+t,zc,n,inward=False,uv_period=c["ring_spacing"])
            for k in range(6):
                angle = k*math.pi/3
                layers["joints"].tube_surface(a,b,r-.002,zc,1,theta0=angle-.002,theta1=angle+.002)
            center,span = (a+b)/2,b-a
            layers["slab"].box((center,0,-.15),(span,2*floor_half,.3))
            for side in (-1,1):
                y = side*rail_y
                for name,z,w,h in [("rail_webs",.115,.15,.03),("rail_webs",.17,.018,.08),("rail_heads",.23,.07,.04)]:
                    layers[name].box((center,y,z),(span,w,h))
                layers["walkways"].box((center,side*walkway_y,.15),(span,.42,.3))
                for height in (1.05,1.16):
                    y = math.sqrt(r*r-(height-zc)**2)-.045
                    layers["cables"].box((center,side*y,height),(span,.055,.045))
        for s in stations(start,end,c["ring_spacing"]):
            if s+.007 <= length:
                layers["joints"].tube_surface(s-.007,s+.007,r-.001,zc,n)
        for s in stations(start,end,c["sleeper_spacing"]):
            if s+.11 > length: continue
            sleeper_count += 1
            layers["sleepers"].box((s,0,.05),(.22,2.3,.10))
            for y in (-rail_y,rail_y):
                for dy in (-.11,.11): layers["fasteners"].box((s,y+dy,.117),(.14,.05,.034))
        light_stations = [s for s in stations(start,end,c["light_spacing"]) if s+.6 <= length]
        for s in light_stations:
            for side in (-1,1):
                layers["lamps"].box((s,side*r*.62,zc+r*.70),(1.2,.16,.10))
                layers["diffusers"].box((s,side*r*.62,zc+r*.70-.054),(1.10,.125,.016))
                x,y,z = alignment.point(s,side*r*.60,zc+r*.70-.15)
                x,y = rotate(x-anchor[0],y-anchor[1],-anchor[2])
                light = xml(link,"light",name=f"lamp_{light_count}",type="point")
                light_count += 1
                xml(light,"pose",numbers([x,y,z,0,0,0]))
                xml(light,"diffuse","0.86 0.9 0.82 1"); xml(light,"specular","0.10 0.10 0.10 1")
                xml(light,"intensity",c["light_intensity"])
                attenuation = xml(light,"attenuation")
                for tag,value in [("range",10),("constant",1),("linear",.16),("quadratic",.06)]: xml(attenuation,tag,value)
                xml(light,"cast_shadows","false")
        for name,mesh in layers.items():
            if not mesh.vertices: continue
            filename = f"section_{index:03d}_{name}.dae"
            generated_files.add(filename)
            mesh.save(mesh_dir/filename)
            uri = f"model://metro_tunnel/meshes/{filename}"
            visual = xml(link,"visual",name=name)
            xml(xml(xml(visual,"geometry"),"mesh"),"uri",uri)
            xml(visual,"cast_shadows","false")
            apply_material(visual,name,c)
            if name in COLLIDERS:
                col = xml(link,"collision",name=name+"_collision")
                xml(xml(xml(col,"geometry"),"mesh"),"uri",uri)
        if index%10 == 0: print(f"Generated chunk {index+1}/{chunk_count}",flush=True)
    # End planes are normal to the local track tangent and outside [0,L].
    for label,station,sign in [("start",0,-1),("end",length,1)]:
        x,y,yaw,_,_ = alignment.at(station)
        dx,dy = rotate(sign*.1,0,yaw)
        link = xml(model,"link",name=label+"_cap")
        xml(link,"pose",numbers([x+dx,y+dy,zc,0,0,yaw]))
        for kind in ("visual","collision"):
            child = xml(link,kind,name=label+"_"+kind)
            xml(child,"pose",numbers([0,0,0,0,math.pi/2,0]))
            cylinder = xml(xml(child,"geometry"),"cylinder")
            xml(cylinder,"radius",r+t); xml(cylinder,"length",.2)
            if kind == "visual": apply_material(child,"slab",c)
    write_xml(sdf,model_dir/"model.sdf")
    config = ET.Element("model")
    xml(config,"name","2200 m metro alignment"); xml(config,"version","2.0")
    xml(config,"sdf","model.sdf",version="1.9")
    xml(config,"description","Concrete metro tunnel with straight, cubic-transition and circular-arc sections.")
    write_xml(config,model_dir/"model.config")
    # Only remove files owned by the previous generator, after the replacement is complete.
    for file in mesh_dir.glob("*.dae"):
        if file.name not in generated_files and (file.stem in MATERIALS or file.name.startswith("section_")):
            file.unlink()
    write_world(c,output,alignment)
    (output/"config").mkdir(exist_ok=True)
    camera_station = 520.
    if camera_station >= length: camera_station = 6.
    camera = [*alignment.point(camera_station,0,1.55),0,.035,alignment.at(camera_station)[2]]
    generated = {**c,"axis_center_z":zc,"rail_top_z":.25,"camera_station":camera_station,
                 "camera_pose":camera,"light_count":light_count,"sleeper_count":sleeper_count,"chunk_count":chunk_count,
                 "end_pose":[*alignment.point(length,0,0),0,0,alignment.at(length)[2]],
                 "segment_stations":[{"type":seg["type"],"start":seg["s0"],"end":seg["s0"]+seg["length"]} for seg in alignment.segments]}
    (output/"config/generated.json").write_text(json.dumps(generated,indent=2)+"\n")
    with (output/"config/centerline.csv").open("w") as f:
        writer = csv.writer(f)
        writer.writerow(["s_m","x_m","y_m","yaw_rad","curvature_1_m","track_roll_rad"])
        for i in range(math.ceil(length)+1):
            s = min(float(i),length)
            writer.writerow([s,*alignment.at(s)])
    print(f"Generated {length:g} m × Ø{2*r:g} m tunnel: {chunk_count} chunks, {light_count} lights")


def write_world(c,output,alignment):
    sdf = ET.Element("sdf",version="1.9")
    world = xml(sdf,"world",name="metro_tunnel")
    physics = xml(world,"physics",name="physics",type="ignored")
    xml(physics,"max_step_size",.001); xml(physics,"real_time_factor",1)
    xml(world,"gravity","0 0 -9.81")
    for filename,name in [("physics","Physics"),("user-commands","UserCommands"),("scene-broadcaster","SceneBroadcaster"),("sensors","Sensors"),("imu","Imu")]:
        plugin = xml(world,"plugin",filename=f"gz-sim-{filename}-system",name=f"gz::sim::systems::{name}")
        if name == "Sensors": xml(plugin,"render_engine","ogre2")
    scene = xml(world,"scene")
    xml(scene,"ambient",numbers([c["ambient"]]*3+[1]))
    xml(scene,"background","0.015 0.015 0.02 1")
    xml(scene,"shadows","false"); xml(scene,"grid","false")
    xml(xml(world,"include"),"uri","model://metro_tunnel")
    gui = xml(world,"gui",fullscreen="false")
    view = xml(gui,"plugin",filename="MinimalScene",name="3D View")
    gzgui = xml(view,"gz-gui")
    xml(gzgui,"title","Metro tunnel · 2200 m")
    xml(gzgui,"property","docked",type="string",key="state")
    xml(view,"engine","ogre2"); xml(view,"scene","scene")
    station = min(520,c["length"]-6)
    xml(view,"camera_pose",numbers([*alignment.point(station,0,1.55),0,.035,alignment.at(station)[2]]))
    clip = xml(view,"camera_clip"); xml(clip,"near",.05); xml(clip,"far",300)
    for name in ["GzSceneManager","InteractiveViewControl","CameraTracking","EntityContextMenuPlugin","WorldControl","WorldStats","EntityTree"]:
        plugin = xml(gui,"plugin",filename=name,name=name)
        if name == "WorldControl":
            xml(plugin,"play_pause","true"); xml(plugin,"step","true"); xml(plugin,"start_paused","false")
    write_xml(sdf,output/"worlds/metro_tunnel.sdf")
