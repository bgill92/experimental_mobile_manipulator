#!/usr/bin/env python3
"""Put emma in a MolmoSpaces iTHOR room.

Usage: molmospaces_scene.py [plan] [x y yaw] [--freeze-kg KG]
       molmospaces_scene.py FloorPlan201 -4.3 3.0 1.5708 --freeze-kg 4

Downloads the room into mujoco/molmospaces/ (once), then writes
mujoco/molmospaces/scenes/ithor/<plan>_emma.xml with emma spawned at (x, y, yaw) on the floor.
Rerun after `pixi run gen-mjcf` so the scene picks up the new robot model.

Speed: the room's loose bodies rest on the floor and each other, so a bare room solves ~240
contacts a step and runs at ~2x real time. Bodies of at least --freeze-kg (default 4: furniture,
which emma cannot move anyway) lose their free joint and become static, and MuJoCo sleep is on so
resting small objects stop costing anything until something touches them. FloorPlan201 then runs
at ~15x real time. Drawers and doors keep their own joints.

Only the scene archive and the THOR object files it references are downloaded, with HTTP range
requests into the dataset's shard tars: a room costs a few hundred MB of download instead of the
~2 GB full object set. Living rooms are FloorPlan201-230; the default spawn is an open spot in
FloorPlan201, so pass a pose for any other room. Data is CC BY 4.0, from
https://huggingface.co/datasets/allenai/molmospaces.
"""

import argparse
import json
import re
import sys
import tarfile
import urllib.parse
import urllib.request
from collections import defaultdict
from compression import zstd
from pathlib import Path

import mujoco
import numpy as np

DATASET = "allenai/molmospaces"
SCENES = "mujoco/scenes/ithor/20251217_with_occupancy"
OBJECTS = "mujoco/objects/thor/20251117"
MJCF = Path(__file__).resolve().parent.parent / "mujoco"
DEST = MJCF / "molmospaces"

# MolmoSpaces collision bits: structure (floor, walls) has contype 8, loose objects contype 1, and
# both accept every bit in 15. Emma's rollers only accept bit 1 and its body meshes collide with
# nothing, so the rollers would fall through the floor and the body would drive through walls.
ROLLER_CONAFFINITY = 1 | 8
# Hit the room but not emma's own geoms (conaffinity 0 or 1). The chassis hull reaches the
# ground, so the floor drops the chassis bit; otherwise it drags and the base cannot drive.
ARM_CONTYPE = 2
CHASSIS_CONTYPE = 4


def index(source: str) -> dict[str, dict]:
    """Archive name -> {shard_id, offset, size}, from the dataset's per-source parquet table."""
    rows, offset = {}, 0
    while True:
        query = urllib.parse.urlencode({"dataset": DATASET, "config": source.replace("/", "__"),
                                        "split": "pkgs", "offset": offset, "length": 100})
        with urllib.request.urlopen(f"https://datasets-server.huggingface.co/rows?{query}") as r:
            page = json.load(r)["rows"]
        rows.update({row["row"]["path"]: row["row"] for row in page})
        if len(page) < 100:
            return rows
        offset += 100


def stream(source: str, entry: dict) -> tarfile.TarFile:
    """Open one .tar.zst archive inside a shard tar without downloading the whole shard."""
    url = f"https://huggingface.co/datasets/{DATASET}/resolve/main/{source}/shards/{entry['shard_id']:05d}.tar"
    end = entry["offset"] + entry["size"] - 1
    response = urllib.request.urlopen(urllib.request.Request(url, headers={"Range": f"bytes={entry['offset']}-{end}"}))
    return tarfile.open(fileobj=zstd.ZstdFile(response), mode="r|")


def fetch(plan: str) -> Path:
    scene_dir, objects_dir = DEST / "scenes/ithor", DEST / "objects/thor"
    scene_xml = scene_dir / f"{plan}_physics.xml"

    if not scene_xml.exists():
        print(f"Downloading {plan}")
        with stream(SCENES, index(SCENES)[f"ithor_{plan}_physics.tar.zst"]) as tar:
            tar.extractall(scene_dir, filter="data")

    # The scene references object meshes and textures as ../../objects/thor/<package>/...
    needed = defaultdict(set)
    for ref in re.findall(r'file="\.\./\.\./objects/thor/([^"]+)"', scene_xml.read_text()):
        if not (objects_dir / ref).exists():
            needed[ref.split("/")[0]].add(ref)

    objects = index(OBJECTS) if needed else {}
    for package, refs in needed.items():
        print(f"Extracting {len(refs)} files from '{package}'")
        with stream(OBJECTS, objects[f"thor_{package}.tar.zst"]) as tar:
            for member in tar:
                if member.name in refs:
                    tar.extract(member, objects_dir, filter="data")
                    refs.discard(member.name)
        if refs:
            sys.exit(f"Missing from '{package}': {sorted(refs)[:5]}")
    return scene_xml


def absolute_assets(spec: mujoco.MjSpec, xml: Path) -> None:
    """Resolve asset paths now: after attaching, one meshdir/texturedir would apply to both models."""
    for assets, subdir in ((spec.meshes, spec.meshdir), (spec.textures, spec.texturedir)):
        for asset in assets:
            if asset.file:
                asset.file = str((xml.parent / subdir / asset.file).resolve())
    spec.meshdir = spec.texturedir = ""


def freeze(scene: mujoco.MjSpec, min_kg: float) -> None:
    """Weld bodies of at least min_kg to the world; MuJoCo skips contacts between static bodies."""
    model = scene.compile()
    for joint in [j for j in scene.joints if j.type == mujoco.mjtJoint.mjJNT_FREE]:
        if model.body_subtreemass[model.joint(joint.name).bodyid[0]] >= min_kg:
            scene.delete(joint)


def compose(scene_xml: Path, x: float, y: float, yaw: float, freeze_kg: float) -> Path:
    robot_xml = MJCF / "mujoco_description_formatted.xml"
    scene, robot = mujoco.MjSpec.from_file(str(scene_xml)), mujoco.MjSpec.from_file(str(robot_xml))
    freeze(scene, freeze_kg)
    scene.option.enableflags |= mujoco.mjtEnableBit.mjENBL_SLEEP
    absolute_assets(scene, scene_xml)
    absolute_assets(robot, robot_xml)

    base = robot.body("base_footprint")
    for geom in robot.geoms:
        if geom.conaffinity == 1:
            geom.conaffinity = ROLLER_CONAFFINITY
        elif geom.group == 3:
            geom.contype = CHASSIS_CONTYPE if geom.parent == base else ARM_CONTYPE
    scene.geom("floor").conaffinity &= ~CHASSIS_CONTYPE

    base.pos = [x, y, 0]
    base.quat = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
    # No prefix: mujoco_ros2_control finds joints and actuators by their URDF names. Without one,
    # emma's root default class ("main") is written nameless, which the parser rejects.
    robot.default.name = "emma"
    scene.attach(robot, frame=scene.worldbody.add_frame(), prefix="")
    # Point the viewer's default camera at emma instead of the room centre.
    scene.stat.center = [x, y, 0.3]
    scene.stat.extent = 1.5

    out = scene_xml.with_name(scene_xml.name.replace("_physics.xml", "_emma.xml"))
    out.write_text(scene.to_xml())
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("plan", nargs="?", default="FloorPlan201")
    parser.add_argument("pose", nargs="*", type=float, default=[-4.3, 3.0, 1.5708], metavar="x y yaw",
                        help="spawn pose in the room's world frame (m, m, rad)")
    parser.add_argument("--freeze-kg", type=float, default=4.0,
                        help="make loose bodies at least this heavy static (default 4; inf keeps all loose)")
    args = parser.parse_args()
    if len(args.pose) != 3:
        parser.error("pose needs x y yaw")
    print(f"Scene ready: {compose(fetch(args.plan), *args.pose, args.freeze_kg)}")


if __name__ == "__main__":
    main()
