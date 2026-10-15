#!/usr/bin/env python3
"""Fix up the MJCF written by mujoco_ros2_control's URDF converter.

Usage: postprocess_mjcf.py <mjcf_dir>

Rewrites <mjcf_dir>/mujoco_description_formatted.xml in place. Safe to re-run.
"""

import math
import sys
import xml.etree.ElementTree as ET

import mujoco

# The converter loads .dae files with trimesh, which ignores the COLLADA <unit> tag.
# RViz honours it, so the scale belongs here and not in the URDF.
MESH_SCALES = {
    "myagv_": "0.0254 0.0254 0.0254",  # Inches.
    "gripper_": "0.001 0.001 0.001",  # Millimetres.
}

# Mecanum rollers, ported from JunHeonYoon/mujoco_mecanum (MIT), wheel_code_gen.py. Each
# wheel gets free-spinning spheres on hinges along the roller axis, so the contact friction
# acts like real rollers. Two changes from upstream:
# - Spheres use the roller radius, not the wheel radius (its Summit XL example does the same).
# - Upstream tilts each roller toward the neighbouring spoke. On a wheel this small that comes
#   out at ~14 degrees instead of 45, and the base strafes at a quarter speed. So the axis is
#   set to 45 degrees directly.
# Must match wheel_radius in emma.urdf.xacro.
WHEEL_RADIUS = 0.04
N_ROLLERS = 12
ROLLER_RATIO = 0.08 / 0.127  # Upstream's roller-to-wheel radius ratio.
# Wheel body -> roller tilt (sign of the tangential axis part). Opposite corners match so the
# rollers form the X that mecanum_drive_controller's kinematics assume; check_mecanum.py
# catches a flipped sign.
WHEELS = {
    "front_left_wheel": -1,
    "front_right_wheel": 1,
    "rear_left_wheel": 1,
    "rear_right_wheel": -1,
}


def fmt(values) -> str:
    return " ".join(f"{v:.6g}" for v in values)


def add_rollers(wheel: ET.Element, tilt: int) -> None:
    roller_r = WHEEL_RADIUS * ROLLER_RATIO
    for i in range(N_ROLLERS):
        # Offset half a roller pitch so that at wheel angle 0 two rollers straddle the contact
        # point. With a roller straight down the base starts balanced on it and rolls about
        # 1 cm (half a pitch) in the first seconds of the sim.
        angle = 2 * math.pi * (i + 0.5) / N_ROLLERS
        c, s = math.cos(angle), math.sin(angle)
        body = ET.SubElement(wheel, "body", {
            "name": f"{wheel.get('name')}_roller_{i}",
            # Sphere surface reaches exactly the wheel radius.
            "pos": fmt([(WHEEL_RADIUS - roller_r) * c, 0, (WHEEL_RADIUS - roller_r) * s]),
        })
        ET.SubElement(body, "inertial", {"pos": "0 0 0", "mass": "0.001", "diaginertia": "1e-6 1e-6 1e-6"})
        # 45 degrees between the wheel axle (y) and the rim tangent.
        ET.SubElement(body, "joint", {"name": f"{wheel.get('name')}_roller_{i}_joint", "type": "hinge",
                                      "axis": fmt([-tilt * s, 1, tilt * c]), "damping": "0.0001",
                                      "limited": "false"})
        # Touches the floor (contype 1) and nothing else.
        ET.SubElement(body, "geom", {"type": "sphere", "size": fmt([roller_r]), "contype": "0",
                                     "conaffinity": "1", "condim": "3", "group": "3"})


# Finger contact: the fingers (contype bit 4) touch only graspable objects (bit 2), so they never
# hit each other, the arm or the base. A soft-ish, high-friction contact with condim 4 adds
# torsional friction so a pinched block does not spin out. See the README's collision-bits table.
FINGER_MESHES = ("gripper_left", "gripper_right")
FINGER_CONTACT = {
    "contype": "4",
    "conaffinity": "2",
    "condim": "4",
    "friction": "1.5 0.02 0.0005",
    "solref": "0.005 1",
    "solimp": "0.95 0.99 0.001",
}


def set_finger_collisions(root: ET.Element) -> None:
    for geom in root.iter("geom"):
        if geom.get("class") == "collision" and geom.get("mesh") in FINGER_MESHES:
            geom.attrib.update(FINGER_CONTACT)


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

    for name, tilt in WHEELS.items():
        wheel = root.find(f".//body[@name='{name}']")
        if wheel is None:
            sys.exit(f"{name} body not found; does the URDF still define it?")
        if wheel.find("body") is None:
            add_rollers(wheel, tilt)

    set_finger_collisions(root)

    ET.indent(tree)
    tree.write(path)


if __name__ == "__main__":
    main()
