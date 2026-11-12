"""Mirror a Gazebo SDF world (static boxes) into a standalone Isaac Sim USD world.

    ~/apps/robotics/isaacsim/python.sh scripts/build_world.py \
        [--sdf .../square_room.sdf] [--out usd/worlds/square_room.usda]

The output holds /World with a physics scene, lights, a ground plane and one box per SDF
model, coloured from the SDF's Gazebo/<Colour> material names. It can be opened on its
own in Isaac Sim (File > Open) and is what run_sim.py loads under the robot.
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DESC = os.path.join(os.path.dirname(PKG), "sambot_description")

parser = argparse.ArgumentParser()
parser.add_argument("--sdf", default=os.path.join(DESC, "world", "room_with_walls", "square_room.sdf"))
parser.add_argument("--out", default=None, help="default: usd/worlds/<sdf stem>.usda")
args, _ = parser.parse_known_args()
out = args.out or os.path.join(PKG, "usd", "worlds", os.path.splitext(os.path.basename(args.sdf))[0] + ".usda")

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402

# Gazebo/<name> material -> (base colour, roughness). Unknown names fall back to "Grey".
PALETTE = {
    "Grey": ((0.42, 0.43, 0.45), 0.85),
    "White": ((0.86, 0.83, 0.77), 0.7),
    "Orange": ((0.95, 0.42, 0.08), 0.5),
    "Red": ((0.8, 0.1, 0.08), 0.5),
    "Green": ((0.15, 0.6, 0.2), 0.5),
    "Blue": ((0.1, 0.3, 0.8), 0.5),
    "Yellow": ((0.95, 0.8, 0.1), 0.5),
    "Black": ((0.05, 0.05, 0.05), 0.6),
}

os.makedirs(os.path.dirname(out), exist_ok=True)
stage = Usd.Stage.CreateNew(out)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

# ---- physics scene ------------------------------------------------------------
scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)
PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateTimeStepsPerSecondAttr(60)  # matches run_sim.py

# ---- lights: sun for shadows + dome so nothing facing away from the sun is black --------
sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
sun.CreateIntensityAttr(2500)
sun.CreateAngleAttr(1.0)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-40, 0, 30))
dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
dome.CreateIntensityAttr(600)
dome.CreateColorAttr(Gf.Vec3f(0.95, 0.97, 1.0))

# ---- materials -------------------------------------------------------------------
_materials = {}


def material(name):
    if name not in PALETTE:
        name = "Grey"
    if name not in _materials:
        color, roughness = PALETTE[name]
        mat = UsdShade.Material.Define(stage, f"/World/Looks/{name}")
        shader = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        _materials[name] = mat
    return _materials[name]


def bind(prim, name):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material(name))


def gazebo_colour(model):
    for elem in model.iter("name"):
        if elem.text and elem.text.startswith("Gazebo/"):
            return elem.text.split("/", 1)[1]
    return "Grey"


# ---- ground: visible quad + infinite collision plane ---------------------------------
# (a thin scaled box let the wheels' cylinder colliders sink in, so physics uses a plane)
half = 50.0
quad = UsdGeom.Mesh.Define(stage, "/World/Ground/Visual")
quad.CreatePointsAttr([(-half, -half, 0), (half, -half, 0), (half, half, 0), (-half, half, 0)])
quad.CreateFaceVertexCountsAttr([4])
quad.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
quad.CreateNormalsAttr([(0, 0, 1)] * 4)
quad.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
quad.CreateExtentAttr([(-half, -half, 0), (half, half, 0)])
plane = UsdGeom.Plane.Define(stage, "/World/Ground/Collider")
plane.CreateAxisAttr("Z")
plane.CreatePurposeAttr(UsdGeom.Tokens.guide)
UsdPhysics.CollisionAPI.Apply(plane.GetPrim())

sdf_root = ET.parse(args.sdf).getroot()
ground_model = next((m for m in sdf_root.iter("model") if m.get("name") == "ground_plane"), None)
bind(quad.GetPrim(), gazebo_colour(ground_model) if ground_model is not None else "Grey")

# ---- static boxes from the SDF ----------------------------------------------------
count = 0
for model in sdf_root.iter("model"):
    name = model.get("name")
    box = model.find(".//collision/geometry/box/size")
    pose = model.find("pose")
    if name == "ground_plane" or box is None or pose is None:
        continue
    x, y, z, _, _, yaw = (float(v) for v in pose.text.split())
    cube = UsdGeom.Cube.Define(stage, f"/World/Room/{name}")
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    xf.AddRotateZOp().Set(math.degrees(yaw))
    xf.AddScaleOp().Set(Gf.Vec3f(*(float(v) for v in box.text.split())))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    bind(cube.GetPrim(), gazebo_colour(model))
    count += 1

stage.GetRootLayer().Save()
print(f"[build_world] wrote {out} ({count} boxes from {os.path.basename(args.sdf)})", flush=True)
simulation_app.close()
