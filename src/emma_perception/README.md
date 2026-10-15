# emma_perception

Finds the pick-and-place block in emma's wrist camera and publishes its pose. Classical
computer vision only: a colour threshold picks out the red block and the depth image places it
in 3D. A learned detector can replace it later behind the same `/block_pose` topic.

## Contents

| Path | What it is |
|---|---|
| `emma_perception/block_detector.py` | The `block_detector` node and the pure image functions it uses. |
| `test/test_block_detector.py` | The image functions on synthetic images and point clouds, without ROS. |

## Running

Start the sim with the pick scene, the detector, and move the arm to the look pose, from which the
wrist camera sees the block on the table (from the home pose it does not):

```bash
pixi run sim mujoco_model:=$PWD/install/emma_simulation/share/emma_simulation/mujoco/pick_scene.xml
pixi run bash -c "source install/setup.bash && ros2 run emma_perception block_detector --ros-args -p use_sim_time:=true"
pixi run bash -c "source install/setup.bash && ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{trajectory: {joint_names: [joint2_to_joint1, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6], points: [{positions: [0.862, 0.771, -1.533, -0.695, -0.133, 0.855], time_from_start: {sec: 3}}]}}\""
pixi run bash -c "source install/setup.bash && ros2 topic echo /block_pose"   # about (0.25, 0, 0.155) in base_link
```

To see what the detector sees, add `-p debug:=true` (and `--log-level block_detector:=debug` for
the point counts in the log). Watch `/block_detector/debug_image` and `/block_detector/debug_mask`
in `rqt_image_view`, and add two PointCloud2 displays in RViz (fixed frame `base_link`, different
colours) for `/block_detector/debug_points` and `/block_detector/debug_top_points`. The top-face
cloud includes the top rows of the visible side faces that fall inside the 2 mm band.

| Topic | Direction | Type |
|---|---|---|
| `/wrist_camera/color/image_raw` | in | `sensor_msgs/Image` (`rgb8` or `bgr8`) |
| `/wrist_camera/depth/image_raw` | in | `sensor_msgs/Image` (`32FC1` metres, or `16UC1` millimetres) |
| `/wrist_camera/color/camera_info` | in | `sensor_msgs/CameraInfo` |
| `/tf`, `/tf_static` | in | camera pose at the image time |
| `/block_pose` | out | `geometry_msgs/PoseStamped` in `target_frame`, stamped with the image time |
| `~/debug_image` | out, with `debug` | colour image with the red outlines, the blob centre and its depth, and the blob / top-face point counts |
| `~/debug_mask` | out, with `debug` | `mono8` mask of every pixel inside the HSV range, before the largest blob is picked |
| `~/debug_points` | out, with `debug` | `PointCloud2` in `target_frame`: every blob pixel with a valid depth |
| `~/debug_top_points` | out, with `debug` | `PointCloud2` in `target_frame`: the points taken as the top face |

| Parameter | Default | Meaning |
|---|---|---|
| `hsv_lower`, `hsv_upper` | `[0, 120, 70]`, `[10, 255, 255]` | OpenCV HSV range (hue 0–180). A range starting at hue 0 also takes 170–180, since red wraps. |
| `min_area_px` | `30` | Smallest blob that counts as the block. |
| `target_frame` | `base_link` | Frame of the published pose; z must point up. |
| `cube_size` | `0.01` | Block edge length, m. |
| `debug` | `false` | Publish the `~/debug_*` topics and log the point counts at debug level. |
| `color_topic`, `depth_topic`, `info_topic` | the wrist camera topics above | Inputs. |

## How it works

For every colour and depth pair (`ApproximateTimeSynchronizer`, 50 ms slop):

1. `segment`: HSV threshold to a mask.
2. `locate`: the largest blob of at least `min_area_px`, with its median depth in 0.04–0.6 m.
   No blob or no valid depth means no message.
3. `blob_points`: every blob pixel with a valid depth, backprojected through the intrinsics
   (`backproject`) into the optical frame.
4. The points go into `target_frame` with the TF at the image's stamp (0.2 s timeout).
5. `top_face`: looking down at an angle, the camera sees the top face and one or two sides.
   The points within 2 mm of the top (95th percentile of z) are the top face. The smallest
   rectangle around them in x, y gives the centre and the yaw, and the block centre is half a
   cube below the top. Yaw is wrapped to ±45°, since a cube looks the same every 90°. Roll and
   pitch are 0: the block is assumed to rest flat.

The pose is published every frame; there is no filtering or debouncing. A consumer should take a
sample after the arm has settled at the look pose.

The node spins on a multi-threaded executor: the TF listener's callbacks are reentrant, so they
keep arriving while an image callback waits in `lookup_transform`.

## Assumptions and caveats

- **One red object**: the largest red blob is taken to be the block. Anything else red in view
  (or a red-brown table under different lighting) confuses it. The sim table's hue is about 15
  on OpenCV's scale, above the default upper bound of 10.
- **A cube of known size resting flat**: the centre comes from the top face and `cube_size`.
- **Accuracy**: in the sim, at the look pose (about 0.2 m away), the published position is
  within about 1 mm of MuJoCo's ground truth for the block at its start (0.25, 0, 0.155) and at
  the place target (0.25, 0.08, 0.155). Taking the top face matters: the first version backprojected the blob centre at the median depth,
  which mixes in the side face and came out about 2 mm low and 2 mm short.
- **Sim camera only so far**: the topic names follow the Orbbec ROS 2 driver, so a real
  Gemini 305 should drop in, but its colour and depth need to be registered (aligned) and the
  thresholds re-tuned for real lighting.
