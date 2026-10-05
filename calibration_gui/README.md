# ZED 2i × Livox Mid-360S Calibration GUI

An offline Python application for refining the rigid transform from Livox points to the ZED 2i left-camera optical frame. The package lives separately from the experiment recording/tracking scripts and does not require ROS 2 for PCD/PLY plus image or SVO work.

## Start

Use Python 3.10 or newer. Install the application dependencies from the repository root:

```bash
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install -r calibration_gui/requirements.txt
python3 -m calibration_gui --demo
```

The demo creates a deterministic RGB image and a synthetic hallway/box cloud using the packaged initial transform. It can be used without ZED, Livox, ROS 2, or Open3D.

Load an SVO2 and a point-cloud file at launch:

```bash
python3 -m calibration_gui --svo hallway.svo2 --frame 0 --cloud hallway.pcd
```

Or use an image and its matching intrinsics, including OpenCV YAML or ROS `CameraInfo` YAML:

```bash
python3 -m calibration_gui --image left.png --intrinsics left_camera.yaml --cloud hallway.ply
```

For a ROS 2 bag, source the ROS 2 Jazzy environment and install/build the message package used by the bag:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m calibration_gui --svo hallway.svo2 --bag recordings/livox/run_001 --topic /livox/lidar --bag-frame 0
```

Inputs can also be opened from the GUI toolbar. `--initial` selects a JSON initial transform. The included [`default_extrinsic.json`](default_extrinsic.json) captures the current nominal measurement supplied for this rig: LiDAR origin at `(X,Y,Z) = (60,-80,20) mm` in the ZED left optical axes, LiDAR `+Y` toward ZED `−X`, LiDAR `+X` 25° below camera forward, and LiDAR `+Z` 65° from camera `+Z`. These are starting values to check against CAD and hallway observations.

## Frames and units

The transform is always

```text
p_zed_left_optical = R_zed_from_livox * p_livox + t_zed_from_livox
```

- Target frame: ZED **left eye optical frame**, with `+X` right, `+Y` down, `+Z` forward. This is the projection frame at the left camera lens center, not the midpoint between the two ZED lenses or the housing top.
- Source frame: point cloud frame, initially named `livox_frame`. The rosbag adapter displays and warns about a different `frame_id`.
- Translation is the LiDAR origin expressed in the ZED target axes. The transform maps source points to target points; it is not the inverse transform.
- Internal units are metres/radians. GUI fields show mm/degrees. GUI RPY is an active source-to-target rotation composed as `Rz(yaw) @ Ry(pitch) @ Rx(roll)`.
- The ZED image pixel projection always uses optical axes. `transform.sdk_to_optical_matrix()` provides explicit conversion helpers for supported ZED SDK pose coordinate systems when using SDK pose data. SVO image projection itself uses the matching left-camera rectified parameters.
- PCD/PLY/NPY/NPZ point coordinates are treated as metres. Convert files to metres before loading if their source uses millimetres; unit autodetection is intentionally not used.

The initial translation is the latest stated relative position. Verify that it is measured from the ZED left optical origin. A measurement from the camera housing surface must be converted using the lens-to-housing CAD offset before treating it as this transform. The supplied ZED 2i datasheet gives the housing envelope as `175.3 × 30.3 × 43.1 mm`, a `120 mm` stereo baseline, and drawings that distinguish the two image planes; it does not make a housing-top measurement interchangeable with the left-eye SDK optical origin. The Livox manual defines its point-cloud origin `O` and `O-XYZ` axes separately from the mounting-hole pattern. For the final CAD transform, place the LiDAR model's origin at Livox `O`, then measure that origin from the ZED left optical frame.

With the supplied Livox axis alignment, the initial matrix displays as GUI RPY approximately `(roll, pitch, yaw) = (0°, -65°, 90°)`. The mechanical 25° downward tilt is the LiDAR `+X` forward-axis angle; it is not the standalone GUI pitch value because the two devices' axis orientations also differ.

## GUI workflow

1. Load one static left image/SVO frame and the matching static LiDAR cloud.
2. Use translation and RPY spin boxes or sliders to align wall edges, corners, poles, and boxes. Changes refresh the 2D overlay and 3D camera-colored cloud.
3. The 3D panel shows the ZED optical axes at the origin and Livox axes at its transformed origin. Drag to rotate, right/middle-drag to pan, and scroll to zoom. The optional Open3D window is available when Open3D is installed.
4. For a numeric metric, pick the same physical feature in the 3D cloud and image, add at least six distributed correspondences, then run **Refine from correspondences**. The metric is pixel reprojection error. Color similarity is not presented as a calibration score.
5. Save `calibration.yaml`; it includes initial and calibrated transforms, quaternion (`xyzw`), rotation and homogeneous matrices, frame definitions, intrinsics, input paths, timestamp, reprojection metrics, and a ROS 2 parent/child static-transform representation.

RGB colorization samples the nearest projected LiDAR return at each image pixel. Camera-behind points, invalid points, distance-filtered points, and points outside the camera FOV remain gray. This point-to-point Z-buffer removes LiDAR collisions in image pixels; it cannot infer occlusion by scene surfaces that have no LiDAR return. RGB mode uses the camera image; Intensity mode is offered when that field exists; Depth mode uses camera-frame Z.

The overlay toggle, point radius, display distance limits, image zoom, color mode, 3D visibility, optional image plane, reset, undo, calibration YAML load/save, and a full-window PNG screenshot are available in the GUI.

## Inputs and format guarantees

- SVO/SVO2 input is lazy-loaded through `pyzed.sl`; the adapter retrieves `VIEW.LEFT` and uses the matching SDK `calibration_parameters.left_cam` intrinsics. It does not produce or consume a depth map. Raw still images require a matching K/distortion YAML or JSON file.
- PCD and PLY ASCII and scalar-preserving binary readers support XYZ and intensity/reflectivity. Binary compressed PCD is delegated to Open3D only when no intensity field would be lost; unsupported or ambiguous layouts produce an error. NPY expects an `N×3` array; NPZ accepts `points` or `xyz`, and optional `intensity` and `colors_rgb` arrays.
- rosbag2 support is isolated in `io/rosbag2.py`. It checks the topic message type, then explicitly decodes `sensor_msgs/msg/PointCloud2` or `livox_ros_driver2/msg/CustomMsg`. Other message types fail with an explanation rather than being interpreted as PointCloud2. PointCloud2 intensity/reflectivity is retained when present.
- The project environment contains a real `/livox/lidar` PointCloud2 recording. It was inspected as `sensor_msgs/msg/PointCloud2`, with `frame_id=livox_frame`, XYZ and intensity fields. Test the adapter locally with that available recording using `--bag`.

## Development and verification

Run the pure/offline tests:

```bash
python3 -m pytest -q calibration_gui/tests
```

The tests cover transform direction/units, reported mount axes, ZED SDK-to-optical convention mapping, projection rejection, RGB Z-buffer behavior, synthetic reprojection refinement, YAML/TF round-trip, ASCII and binary PCD/PLY scalar fields, and camera intrinsics YAML formats.

Check an installed rosbag recording (requires the ROS 2 Jazzy Python environment and its message types):

```bash
python3 - <<'PY'
from pathlib import Path
from calibration_gui.io.rosbag2 import read_bag_pointcloud
frame = read_bag_pointcloud(Path("recordings/livox/test_001_20261005_121841"))
print(frame.message_type, frame.frame_id, frame.cloud.points_m.shape, frame.cloud.intensity is not None)
PY
```

## Current environment limitations

The core synthetic pipeline can be exercised without the devices. This development environment does not currently import `pyzed.sl` because its NVIDIA runtime is missing `libnvidia-encode.so.1`, so SVO playback and runtime ZED intrinsics have not been exercised here. The physical camera/LiDAR alignment has not been tested. Install ZED SDK 5.4.0 and its matching Python binding on the target machine to use SVO input. Open3D is optional; the embedded interactive 3D view is used if it is absent. ROS bag loading requires ROS 2 Python packages and the message definition package recorded in the bag. The system Python in this workspace also has a user-installed NumPy 2.5 paired with system SciPy 1.11; those two are ABI-incompatible. The test run used the compatible system package set with `PYTHONNOUSERSITE=1`; an isolated virtual environment avoids this workspace-specific issue.

The current build does not synchronize moving camera and LiDAR streams, estimate time offset, or compensate motion. Use static scenes or properly synchronized frames for calibration. Livox CustomMsg parsing is implemented against its standard `header` and `points` fields; a custom fork with a different schema will be rejected or require a matching adapter.

Reference material: [attached ZED 2i datasheet](../ZED%202i%20-%20Datasheet%20-%20V1.5%20-%202026-09-22.pdf), [attached Livox Mid-360S manual](../Livox_Mid-360s_User_Manual_en.pdf), [Stereolabs coordinate frames](https://docs.stereolabs.com/docs/development/zed-sdk/modules/positional-tracking/coordinate-frames/), and [camera calibration parameters](https://docs.stereolabs.com/docs/development/zed-sdk/modules/camera/camera-calibration).
