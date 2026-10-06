#!/usr/bin/env bash
# Regenerates the MuJoCo model from emma.urdf.xacro into src/emma_simulation/mujoco/,
# next to the scene.xml that includes it. Run after any URDF or mujoco_inputs.xml change.
set -eo pipefail  # No -u: colcon setup scripts read unset variables.

source install/setup.bash
pkg="$(cd "$(dirname "$0")/.." && pwd)"
dest="${pkg}/mujoco"
work="$(mktemp -d)"
trap 'rm -rf "${work}"' EXIT

xacro "${pkg}/../emma_description/urdf/emma.urdf.xacro" > "${work}/emma.urdf"
# The converter resolves relative paths against the cwd, so run it from the work dir.
(cd "${work}" && ros2 run mujoco_ros2_control make_mjcf_from_robot_description.py \
  --urdf emma.urdf \
  --mujoco_inputs "${dest}/mujoco_inputs.xml" \
  --scene "${dest}/scene.xml" \
  --add_free_joint --save_only --output out)
python "${pkg}/scripts/postprocess_mjcf.py" "${work}/out"

# Keep only the model and the asset files it references; the converter leaves
# intermediates (unsplit OBJs, MTL folders) that roughly double the size.
out="${work}/out"
grep -o 'file="[^"]*"' "${out}/mujoco_description_formatted.xml" | cut -d'"' -f2 | sort -u > "${work}/used"
(cd "${out}/assets" && find . -type f | sed 's#^\./##' | sort | comm -23 - "${work}/used" | xargs -r rm)
find "${out}/assets" -type d -empty -delete
rm -rf "${dest}/assets"
mv "${out}/assets" "${out}/mujoco_description_formatted.xml" "${dest}/"
echo "MJCF written to ${dest}"
