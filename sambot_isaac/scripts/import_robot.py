"""Convert urdf/sambot.urdf into usd/sambot/sambot.usd (run with Isaac's python.sh).

    ~/apps/robotics/isaacsim/python.sh scripts/import_robot.py
"""
import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig  # noqa: E402

pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
config = URDFImporterConfig(
    urdf_path=os.path.join(pkg, "urdf", "sambot.urdf"),
    usd_path=os.path.join(pkg, "usd"),
    merge_fixed_joints=False,   # keep lidar_link/camera_link/base_footprint as separate prims for TF
    merge_mesh=False,
    collision_from_visuals=False,
    fix_base=False,
    joint_drive_type="acceleration",
    joint_target_type="velocity",
    override_joint_stiffness=0.0,
    override_joint_damping=1000.0,
)
out = URDFImporter(config).import_urdf()
print(f"[import_robot] wrote {out}")
app.close()
sys.exit(0)
