"""Recording-specific offline geometry; no device or original-data writes."""
from pathlib import Path
import csv
import hashlib
import json
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from track_people import (_iter_bag_clouds, _pointcloud2_to_numpy,
                          _message_timestamp_ns, _rotation_from_up_vector)
from audit_recordings import audit
from calibrate_zed_lidar_planes import _fit_plane, _angle_between
from corridor_frame import estimate_corridor_frame

REVIEW = Path('/data/experiment_data/20261022_group003_recording_002_calibration_review')
BAG = Path('/data/experiment_data/recordings/20261022_group003/recording_002/lidar')
SEED = 20261022


def save(name, data):
    with (OUT / name).open('x', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def main():
    if (OUT / 'geometry.json').exists():
        raise FileExistsError('Choose a fresh output folder')
    camera = json.loads((REVIEW / 'camera.json').read_text())
    rows = list(csv.DictReader((REVIEW / 'frames.csv').open()))
    imus = list(csv.DictReader((REVIEW / 'imu.csv').open()))
    first = int(camera['first_image_timestamp_ns'])
    old_config = json.loads((ROOT / 'config.json').read_text())
    old_up = np.array(old_config['tracking']['gravity_alignment']['up_vector'])
    old_up /= np.linalg.norm(old_up)
    lidar_audit = audit(BAG, old_up)
    up = np.array(lidar_audit['imu_up'])
    zacc = np.array([[float(r['accel_' + a + '_m_s2']) for a in 'xyz'] for r in imus])
    zup = Rotation.from_quat(camera['camera_imu_rotation_imu_to_left_camera_xyzw']).apply(zacc.mean(axis=0))
    zup /= np.linalg.norm(zup)
    print('Lidar IMU up', up, 'ZED IMU up', zup, flush=True)
    zparts = []
    for r in rows:
        im = cv2.imread(str(REVIEW / r['image_file']))
        if im is None or im.shape[:2] != (camera['height'], camera['width']):
            raise ValueError(f'Invalid image {r}')
        with np.load(REVIEW / r['depth_file']) as archive:
            depth = archive['depth_m']
        if depth.shape != im.shape[:2]:
            raise ValueError('Depth/image dimensions differ')
        if float(r['elapsed_s']) <= 2.5:
            v, u = np.mgrid[0:depth.shape[0]:6, 0:depth.shape[1]:6]
            d = depth[::6, ::6]
            points = np.stack(((u-camera['cx'])*d/camera['fx'],
                               (v-camera['cy'])*d/camera['fy'], d), axis=-1).reshape(-1, 3)
            keep = np.isfinite(points).all(axis=1) & (points[:, 2] > .4) & (points[:, 2] < 18)
            zparts.append(points[keep])
    lparts = []
    for msg, fb, _ in _iter_bag_clouds(BAG, '/livox/lidar', 'pointcloud2', 'mcap'):
        t = (_message_timestamp_ns(msg, fb) - first) / 1e9
        if t > 2.5:
            break
        if t < 0:
            continue
        p = _pointcloud2_to_numpy(msg)
        keep = np.isfinite(p).all(axis=1) & (np.linalg.norm(p, axis=1) < 20) & (p[:, 0] > 0)
        lparts.append(p[keep])
    geometries = {}
    for kind, points, imu_up in [('lidar', np.concatenate(lparts).astype(float), up),
                                 ('zed', np.concatenate(zparts).astype(float), zup)]:
        rng = np.random.default_rng(SEED)
        if len(points) > 60000:
            points = points[rng.choice(len(points), 60000, replace=False)]
        heights = points @ imu_up
        counts, edges = np.histogram(heights, bins=np.arange(-3.5, -.5, .025))
        prior_d = -float(edges[np.argmax(counts)] + .0125)
        print(kind, 'floor prior from histogram', prior_d, flush=True)
        floor = _fit_plane(points, imu_up, prior_d, .45, angle_tolerance_deg=6,
                           point_tolerance_m=.025 if kind == 'lidar' else .04,
                           iterations=1400, seed=SEED)
        level = _rotation_from_up_vector(floor['normal'])
        shift = np.array([0., 0., floor['d_m']])
        lp = points @ level.T + shift
        # Both sensors are fitted independently; orient the horizontal working
        # frame toward the sensor's forward axis before using the wall fitter.
        forward = level @ np.array([1., 0., 0.] if kind == 'lidar' else [0., 0., 1.])
        forward[2] = 0
        forward /= np.linalg.norm(forward)
        lateral = np.cross([0., 0., 1.], forward)
        heading = np.stack([forward, lateral, [0., 0., 1.]])
        lp = lp @ heading.T
        corr = estimate_corridor_frame(lp, x_min=.5, x_max=16, y_min=-4, y_max=4,
                    z_min=.3, z_max=2.3, voxel_size=.08, line_tolerance=.06,
                    ransac_iterations=1800, seed=SEED)
        cd = corr.to_dict()
        cd['source_frame'] = 'livox_level_frame' if kind == 'lidar' else 'zed_heading_level_frame'
        # LiDAR's forward heading is very close to +X, but include the heading
        # algebraically in the corridor axes used by the existing tracker.
        if kind == 'lidar':
            cd['longitudinal_xy'] = (heading.T @ np.r_[corr.longitudinal_xy, 0.])[:2].tolist()
            cd['lateral_xy'] = (heading.T @ np.r_[corr.lateral_xy, 0.])[:2].tolist()
            cd['origin_xy'] = (heading.T @ np.r_[corr.origin_xy, 0.])[:2].tolist()
        basis = np.array([[*corr.longitudinal_xy, 0.], [*corr.lateral_xy, 0.], [0., 0., 1.]])
        R = basis @ heading @ level
        t = basis @ (heading @ shift - np.r_[corr.origin_xy, 0.])
        cp = points @ R.T + t
        np.savez_compressed(OUT / (kind + '_geometry.npz'), raw=points, corridor=cp)
        geometries[kind] = dict(floor=floor, floor_vs_imu_deg=_angle_between(imu_up, np.array(floor['normal'])),
            rotation_raw_to_level=level.tolist(), translation_raw_to_level=shift.tolist(),
            corridor=cd, rotation_raw_to_corridor=R.tolist(), translation_raw_to_corridor=t.tolist(),
            origin_note='Projection of this sensor origin onto floor/centerline; not the surveyed corridor end.')
        print(kind, 'floor', floor, 'wall gap', corr.wall_gap, flush=True)
    save('corridor_candidate.json', geometries['lidar']['corridor'])
    cfg = json.loads(json.dumps(old_config))
    cfg['tracking']['gravity_alignment'] = dict(enabled=True,
        up_vector=geometries['lidar']['floor']['normal'],
        ground_height_along_up_m=-geometries['lidar']['floor']['d_m'])
    cfg['tracking']['corridor_frame_file'] = str(OUT / 'corridor_candidate.json')
    # Never load the previous installation's external transform implicitly.
    cfg['tracking']['lidar_to_zed_frame_file'] = str(OUT / 'not_yet_calibrated.json')
    save('analysis_config.json', cfg)
    save('geometry.json', dict(camera=camera, checked_images=len(rows), checked_depths=len(rows),
         zed_imu_up=zup.tolist(), zed_acceleration_std=zacc.std(axis=0).tolist(), lidar_audit=lidar_audit,
         geometries=geometries, calibration_interval_zed_s=[0., 2.5], seed=SEED,
         clock_note='Raw absolute timestamps. No estimated clock correction.',
         source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
            [Path(__file__), ROOT/'config.json', ROOT/'scripts/corridor_frame.py',
             ROOT/'scripts/calibrate_zed_lidar_planes.py', REVIEW/'frames.csv', REVIEW/'camera.json',
             REVIEW/'imu.csv', BAG/'metadata.yaml']}))


if __name__ == '__main__':
    main()
