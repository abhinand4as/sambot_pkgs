"""Sambot in Isaac Sim: world USD + robot + ROS 2 bridge, matching the Gazebo topic contract.

    ~/apps/robotics/launch_isaacsim.sh is for the plain GUI; this script is the
    scripted equivalent and is what the launch file runs:

    ~/apps/robotics/isaacsim/python.sh scripts/run_sim.py [--headless] [--world WORLD.usda]

Published (ROS 2):  /clock  /demo/odom  /demo/imu  /joint_states  /scan
Subscribed (ROS 2): /isaac/cmd_vel (Twist; fed from /demo/cmd_vel by cmd_vel_relay)
No /tf is published: robot_state_publisher + the EKF own the TF tree, as in Gazebo.
"""
import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--world", default=None, help="world USD (default: usd/worlds/square_room.usda)")
parser.add_argument("--no-lidar", action="store_true")
parser.add_argument("--debug-poses", action="store_true", help="print rigid-body world poses every 2 s")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.usd  # noqa: E402
import usdrt.Sdf  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

for ext in ("isaacsim.ros2.bridge", "isaacsim.sensors.physics", "isaacsim.sensors.experimental.physics",
            "isaacsim.robot.wheeled_robots"):
    app_utils.enable_extension(ext)
simulation_app.update()

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_USD = os.path.abspath(args.world or os.path.join(PKG, "usd", "worlds", "square_room.usda"))

# Robot constants from sambot_description/urdf (keep in sync with the URDF).
WHEEL_RADIUS = 0.10
WHEEL_SEPARATION = 0.36
ROBOT = "/World/sambot"
BASE_LINK = f"{ROBOT}/Geometry/base_link"
LIDAR_LINK = f"{BASE_LINK}/lidar_link"

omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")

print("[run_sim] world", flush=True)
# ---- world (physics scene, lights, ground, room) from scripts/build_world.py --------
if not os.path.isfile(WORLD_USD):
    raise SystemExit(f"[run_sim] {WORLD_USD} not found; build it with scripts/build_world.py")
stage.GetPrimAtPath("/World").GetReferences().AddReference(WORLD_USD)

# The world's physics scene arrives through a reference, which SimulationManager does not
# notice, so on play it would add its own /PhysicsScene. Isaac's simulation time (and thus
# /clock and every message stamp) then advances once per scene per step, i.e. 2x+ faster than
# physics, and anything integrating velocity over stamps (EKF, SLAM) overshoots.
import carb  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

PHYSICS_SCENE = "/World/PhysicsScene"
PHYSICS_HZ = 60
simulation_app.update()
SimulationManager.set_default_physics_scene(PHYSICS_SCENE)
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(PHYSICS_SCENE)).GetTimeStepsPerSecondAttr().Set(PHYSICS_HZ)
# One physics step per app frame, run-loop paced at PHYSICS_HZ so the sim runs at real time.
settings = carb.settings.get_settings()
settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
settings.set_int("/app/runLoops/main/rateLimitFrequency", PHYSICS_HZ)
settings.set_bool("/app/runLoops/main/manualModeEnabled", True)
settings.set_bool("/app/player/useFixedTimeStepping", True)  # timeline += 1/PHYSICS_HZ per frame
timeline = omni.timeline.get_timeline_interface()
timeline.set_time_codes_per_second(PHYSICS_HZ)
timeline.set_target_framerate(PHYSICS_HZ)

print("[run_sim] robot", flush=True)
# ---- robot -------------------------------------------------------------------
prim = stage.DefinePrim(ROBOT)
prim.GetReferences().AddReference(os.path.join(PKG, "usd", "sambot", "sambot.usda"))
UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.17))  # base_footprint ~2 cm above floor

print("[run_sim] Velocity-controlled", flush=True)
# Velocity-controlled wheels: stiffness 0, force drive with bounded torque so a step in cmd_vel
# cannot pop a wheelie (USD angular drive gains are per degree).
for side in ("l", "r"):
    drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(f"{ROBOT}/Physics/drivewhl_{side}_joint"), "angular")
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(0.0)
    drive.CreateDampingAttr().Set(10.0)
    drive.CreateMaxForceAttr().Set(20.0)


def bind_friction(name, mu, combine, colliders):
    mat = UsdShade.Material.Define(stage, f"/World/PhysicsMaterials/{name}")
    phys = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    phys.CreateStaticFrictionAttr(mu)
    phys.CreateDynamicFrictionAttr(mu)
    phys.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr(combine)
    for path in colliders:
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(
            mat, UsdShade.Tokens.weakerThanDescendants, "physics")


# The URDF caster is a fixed sphere; Gazebo makes it slide with mu=0.001 (SDF), do the same here.
bind_friction("caster", 0.001, "min", [f"{BASE_LINK}/front_caster/sphere_1"])
bind_friction("wheel", 1.0, "max", [f"{BASE_LINK}/drivewhl_{s}_link/cylinder_1" for s in ("l", "r")])

print("[run_sim] IMU on base_link", flush=True)
# IMU on base_link
from isaacsim.sensors.experimental.physics import IMU  # noqa: E402

IMU(f"{BASE_LINK}/imu_sensor")
simulation_app.update()

print("[run_sim] RTX 2D lidar", flush=True)
# ---- RTX 2D lidar -> /scan ----------------------------------------------------
if not args.no_lidar:
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

    lidar = Lidar.create(path=f"{LIDAR_LINK}/sensor", config="Example_Rotary_2D", tick_rate=10.0)
    simulation_app.update()
    lp = stage.GetPrimAtPath(lidar.paths[0])
    rate = float(lp.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get() or 0)
    firing = float(lp.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get() or 0)
    sensor = LidarSensor(lidar, annotators=[])
    sensor.attach_writer(
        "RtxLidarROS2PublishLaserScan", topicName="scan", frameId="lidar_link",
        horizontalFov=360.0, horizontalResolution=360.0 * rate / firing,
        depthRange=[0.12, 3.5], rotationRate=rate, azimuthRange=[-180.0, 180.0])

print("[run_sim] OmniGraph", flush=True)
# ---- OmniGraph: clock, cmd_vel -> wheels, odom, imu, joint states --------------
keys = og.Controller.Keys
IMU_PRIM = f"{BASE_LINK}/imu_sensor"
og.Controller.edit(
    {"graph_path": "/World/ROS2Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("Tick", "omni.graph.action.OnPlaybackTick"),
            ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("SubTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("BreakLin", "omni.graph.nodes.BreakVector3"),
            ("BreakAng", "omni.graph.nodes.BreakVector3"),
            ("Diff", "isaacsim.robot.wheeled_robots.DifferentialController"),
            ("Art", "isaacsim.core.nodes.IsaacArticulationController"),
            ("OdomCompute", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("OdomPub", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("ReadImu", "isaacsim.sensors.physics.IsaacReadIMU"),
            ("ImuPub", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ("JointPub", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ],
        keys.SET_VALUES: [
            ("Clock.inputs:topicName", "/clock"),
            ("SubTwist.inputs:topicName", "/isaac/cmd_vel"),
            ("Diff.inputs:wheelRadius", WHEEL_RADIUS),
            ("Diff.inputs:wheelDistance", WHEEL_SEPARATION),
            ("Diff.inputs:maxLinearSpeed", 0.6),
            ("Diff.inputs:maxAngularSpeed", 2.0),
            # same limit as the Gazebo DiffDrive plugin (max_linear_acceleration 1.0)
            ("Diff.inputs:maxAcceleration", 1.0),
            ("Diff.inputs:maxDeceleration", 1.0),
            ("Diff.inputs:maxAngularAcceleration", 2.0),
            ("Art.inputs:targetPrim", [usdrt.Sdf.Path(BASE_LINK)]),
            ("Art.inputs:jointNames", ["drivewhl_l_joint", "drivewhl_r_joint"]),
            ("OdomCompute.inputs:chassisPrim", [usdrt.Sdf.Path(BASE_LINK)]),
            ("OdomPub.inputs:topicName", "/demo/odom"),
            ("OdomPub.inputs:odomFrameId", "odom"),
            ("OdomPub.inputs:chassisFrameId", "base_link"),
            ("ReadImu.inputs:imuPrim", [usdrt.Sdf.Path(IMU_PRIM)]),
            ("ImuPub.inputs:topicName", "/demo/imu"),
            ("ImuPub.inputs:frameId", "imu_link"),
            ("JointPub.inputs:topicName", "/joint_states"),
            ("JointPub.inputs:targetPrim", [usdrt.Sdf.Path(BASE_LINK)]),  # articulation root
        ],
        keys.CONNECT: [
            ("Tick.outputs:tick", "Clock.inputs:execIn"),
            ("Tick.outputs:tick", "SubTwist.inputs:execIn"),
            ("Tick.outputs:tick", "OdomCompute.inputs:execIn"),
            ("Tick.outputs:tick", "ReadImu.inputs:execIn"),
            ("Tick.outputs:tick", "JointPub.inputs:execIn"),
            ("Ctx.outputs:context", "Clock.inputs:context"),
            ("Ctx.outputs:context", "SubTwist.inputs:context"),
            ("Ctx.outputs:context", "OdomPub.inputs:context"),
            ("Ctx.outputs:context", "ImuPub.inputs:context"),
            ("Ctx.outputs:context", "JointPub.inputs:context"),
            ("SimTime.outputs:simulationTime", "Clock.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime", "OdomPub.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime", "ImuPub.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime", "JointPub.inputs:timeStamp"),
            ("SubTwist.outputs:execOut", "Diff.inputs:execIn"),
            # Twist vectors -> scalar v_x / w_z for the diff-drive controller
            ("SubTwist.outputs:linearVelocity", "BreakLin.inputs:tuple"),
            ("SubTwist.outputs:angularVelocity", "BreakAng.inputs:tuple"),
            ("BreakLin.outputs:x", "Diff.inputs:linearVelocity"),
            ("BreakAng.outputs:z", "Diff.inputs:angularVelocity"),
            ("Tick.outputs:deltaSeconds", "Diff.inputs:dt"),  # needed for the acceleration limits
            # Diff has no execOut; drive the wheels every tick with its latest command
            ("Tick.outputs:tick", "Art.inputs:execIn"),
            ("Diff.outputs:velocityCommand", "Art.inputs:velocityCommand"),
            ("OdomCompute.outputs:execOut", "OdomPub.inputs:execIn"),
            ("OdomCompute.outputs:position", "OdomPub.inputs:position"),
            ("OdomCompute.outputs:orientation", "OdomPub.inputs:orientation"),
            ("OdomCompute.outputs:linearVelocity", "OdomPub.inputs:linearVelocity"),
            ("OdomCompute.outputs:angularVelocity", "OdomPub.inputs:angularVelocity"),
            ("ReadImu.outputs:execOut", "ImuPub.inputs:execIn"),
            ("ReadImu.outputs:angVel", "ImuPub.inputs:angularVelocity"),
            ("ReadImu.outputs:linAcc", "ImuPub.inputs:linearAcceleration"),
            ("ReadImu.outputs:orientation", "ImuPub.inputs:orientation"),
        ],
    },
)
simulation_app.update()

print("[run_sim] play", flush=True)
app_utils.play()
frame = 0
while simulation_app.is_running():
    simulation_app.update()
    frame += 1
    if frame == 10:
        scenes = [str(p.GetPath()) for p in stage.Traverse() if p.GetTypeName() == "PhysicsScene"]
        if scenes != [PHYSICS_SCENE]:
            print(f"[run_sim] WARNING: physics scenes {scenes}; /clock will run faster than physics", flush=True)
    if args.debug_poses and frame % 120 == 0:
        from omni.physx import get_physx_interface

        for link in ("", "/drivewhl_l_link", "/drivewhl_r_link", "/front_caster"):
            tf = get_physx_interface().get_rigidbody_transformation(BASE_LINK + link)
            pos = [round(v, 3) for v in tf["position"]] if tf.get("ret_val") else None
            rot = [round(v, 3) for v in tf["rotation"]] if tf.get("ret_val") else None
            print(f"[pose] {link or '/base_link'} pos={pos} quat_xyzw={rot}", flush=True)
app_utils.stop()
simulation_app.close()
