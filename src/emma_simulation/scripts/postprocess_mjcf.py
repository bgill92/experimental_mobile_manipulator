#!/usr/bin/env python3
"""Fix up the MJCF written by mujoco_ros2_control's URDF converter.

Usage: postprocess_mjcf.py <mjcf_dir>

Rewrites <mjcf_dir>/mujoco_description_formatted.xml in place. Safe to re-run.
"""

import sys
import xml.etree.ElementTree as ET

import mujoco

# The converter loads .dae files with trimesh, which ignores the COLLADA <unit> tag.
# RViz honours it, so the scale belongs here and not in the URDF.
MESH_SCALES = {
    "myagv_": "0.0254 0.0254 0.0254",  # Inches.
    "gripper_": "0.001 0.001 0.001",  # Millimetres.
}

# ponytail: box stand-in for the myAGV wheels so the base rests on the floor. Replace
# it once the base gets real wheel joints. Sized to the base mesh footprint.
BASE_CONTACT = {
    "name": "base_contact",
    "type": "box",
    "size": "0.155 0.115 0.02",
    "pos": "0.007 -0.004 0.02",
    "contype": "0",
    "conaffinity": "1",
    "condim": "3",
    "group": "3",
}


def fmt(values) -> str:
    return " ".join(f"{v:.6g}" for v in values)


def main() -> None:
    mjcf_dir = sys.argv[1].rstrip("/")
    path = f"{mjcf_dir}/mujoco_description_formatted.xml"
    tree = ET.parse(path)
    root = tree.getroot()

    # The converter drops the <inertial> of bodies whose mass comes only from fused
    # static URDF links (the base). The URDF it compiled still has the right values, so
    # copy them over; otherwise MuJoCo derives mass from the collision meshes.
    reference = mujoco.MjModel.from_xml_path(f"{mjcf_dir}/robot_description_formatted.urdf")
    for body in root.iter("body"):
        if body.find("inertial") is None:
            ref = reference.body(body.get("name"))
            inertial = ET.Element("inertial", {
                "pos": fmt(ref.ipos), "quat": fmt(ref.iquat),
                "mass": fmt(ref.mass), "diaginertia": fmt(ref.inertia),
            })
            # MuJoCo wants <inertial> before any <joint>/<freejoint> in a body.
            body.insert(0, inertial)

    for mesh in root.iter("mesh"):
        name = mesh.get("file", "").rsplit("/", 1)[-1]
        for prefix, scale in MESH_SCALES.items():
            if name.startswith(prefix):
                mesh.set("scale", scale)

    base = root.find("./worldbody/body[@name='base_footprint']")
    if base is None:
        sys.exit("base_footprint body not found; did the converter run with -f?")
    if base.find(f"geom[@name='{BASE_CONTACT['name']}']") is None:
        ET.SubElement(base, "geom", BASE_CONTACT)

    tree.write(path)


if __name__ == "__main__":
    main()
