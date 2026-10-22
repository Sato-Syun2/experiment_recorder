# ZED 2i × Livox Mid-360S calibration — preliminary analysis

Source: `tmp.zip`, 2026-10-21.

**Status: preliminary. Do not treat this matrix as a final independently validated extrinsic.**

## Data checked

- ZED: 468 rectified 1280×720 left images, 468 NEURAL depth `.npz` arrays, camera intrinsics and IMU.
- Livox: 972 PointCloud2 messages over ~97.1 seconds, extracted by a lightweight reader directly from MCAP.
- ZED↔Livox clock: median nearest-neighbor image/pointcloud gap ~28 ms (maximum ~54 ms).

## Seven static pole observations

|   station |   t_start |   t_end |   points |   forward_m |   side_m |   hmedian_m |   fwd_mad_m |   image_frame |   image_u_px |   image_v_px |   predicted_u_px |   predicted_v_px |   fit_error_px |   leave_one_out_error_px |
|----------:|----------:|--------:|---------:|------------:|---------:|------------:|------------:|--------------:|-------------:|-------------:|-----------------:|-----------------:|---------------:|-------------------------:|
|     1.000 |     8.000 |  13.000 |  571.000 |       2.870 |    0.052 |       0.513 |       0.004 |        53.000 |      643.000 |      521.000 |          642.122 |          521.600 |          1.063 |                    6.585 |
|     2.000 |    18.000 |  22.000 |  201.000 |       4.831 |    0.149 |       0.552 |       0.006 |       100.000 |      635.000 |      404.000 |          636.727 |          403.324 |          1.855 |                    2.325 |
|     3.000 |    29.000 |  33.000 |  138.000 |       6.811 |    0.213 |       0.541 |       0.008 |       155.000 |      636.000 |      350.000 |          636.710 |          348.176 |          1.958 |                    2.284 |
|     4.000 |    41.000 |  45.000 |  102.000 |       8.888 |    0.382 |       0.514 |       0.009 |       215.000 |      631.000 |      316.000 |          630.833 |          315.279 |          0.740 |                    0.894 |
|     5.000 |    52.000 |  56.000 |   55.000 |      10.850 |    0.556 |       0.450 |       0.013 |       262.000 |      629.000 |      295.000 |          626.584 |          295.245 |          2.429 |                    3.040 |
|     6.000 |    63.000 |  67.000 |   34.000 |      12.826 |    0.565 |       0.731 |       0.010 |       316.000 |      630.000 |      280.000 |          630.333 |          281.048 |          1.100 |                    1.403 |
|     7.000 |    74.000 |  77.000 |   49.000 |      14.862 |    0.645 |       0.524 |       0.010 |       369.000 |      630.000 |      269.000 |          630.688 |          270.232 |          1.411 |                    1.852 |

Consecutive pole-position separations: mean 1.999 m, SD 0.047 m.

## Geometric fitting

- Livox floor plane normal: `[-0.56331  -0.021705  0.82596 ]`; estimated plane-distance to LiDAR ~2.065 m.
- ZED floor plane normal: `[-0.006652  0.967274  0.253645]`; plane-distance to camera ~1.926 m.
- Fitted camera down pitch using pole pixels: 15.855°. Pitch from NEURAL stereo floor alone: 14.693°.
- Image-fitting reprojection RMSE: 1.14 px per coordinate.
- Leave-one-out reprojection Euclidean mean: 2.63 px; maximum: 6.59 px.

### Preliminary transform `T_camera_left_rectified_optical_from_livox_frame`

```text
[[ 0.011349 -0.999764 -0.018532 -0.017218]
 [ 0.31616   0.02117  -0.94847  -0.124808]
 [ 0.948638  0.004905  0.316326 -0.070462]
 [ 0.        0.        0.        1.      ]]
```

Rotation quaternion (x, y, z, w): `[0.4104428, -0.4163819, 0.5665259, 0.5806989]`
Camera origin expressed in Livox coordinates [m]: `[0.1065, -0.01423, -0.09641]`.

## Independent depth consistency test

Eight ZED frames and nearest-in-time LiDAR scans, 34,926 depth pixel comparisons on projected cloud points:
- Overall median absolute depth discrepancy: 0.113 m.
- 90th percentile: 0.395 m.
- Within ±0.5 m: 93.2%.
- This is **not** a direct estimate of 3D tracking accuracy. Stereo depth on thin poles often returns background; points beyond 7m become less reliable.

## Constraints

The pole centers are almost collinear. We constrained floor normals and camera height, and fitted a 4-parameter planar pose. The observed pixel alignment does **not** establish a unique, independently verified unconstrained 6-DoF extrinsic. Roll and vertical translation depend on estimated floor planes; small pitch errors produce large long-range depth errors. Validate using door recess corners and at least one off-axis reference location.

Files: `pole_7_stations.csv`, `calibration_preliminary.json`, `lidar_pole_trajectory.png`, `zed_pole_reprojection.png`, `depth_consistency_check.png`.
