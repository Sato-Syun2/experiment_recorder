"""Offline recording-specific analysis; never modifies input data or tracker settings.

Run with repository .venv/bin/python. Tracking must already have completed using
analysis_config.json. No live ROS node, camera, SDK, or hardware is started.
"""
from pathlib import Path
import csv
import hashlib
import json
import sys

import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from track_people import load_settings, _confirmation_point_threshold

REVIEW = Path('/data/experiment_data/20261021_group002_recording_003_walk_review')
CAL = OUT.parent / '20261021_group002_pole_calibration'


def read_csv(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def save_json(name, data):
    with (OUT / name).open('x', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def stats(values):
    a = np.asarray(values, dtype=float)
    if not len(a):
        return {'n': 0}
    return dict(n=len(a), median=float(np.median(a)), p10=float(np.percentile(a, 10)),
                p90=float(np.percentile(a, 90)), min=float(a.min()), max=float(a.max()))


def main():
    for name in ['metrics.json', 'dashboard.json', 'index.html', 'review_frames', 'image_measurements.csv']:
        if (OUT / name).exists():
            raise FileExistsError(f'Choose a fresh analysis output: {OUT / name}')
    meta = json.loads((OUT / 'tracks.metadata.json').read_text())
    if not meta['completed'] or meta['frames'] != 318:
        raise RuntimeError('Expected the complete 318-frame recording')
    frames = read_csv(REVIEW / 'frames.csv')
    tracks = read_csv(OUT / 'tracks.csv')
    ledger = read_csv(OUT / 'tracks.frames.csv')
    camera = json.loads((REVIEW / 'camera.json').read_text())
    first = int(camera['first_image_timestamp_ns'])
    settings = load_settings(OUT / 'analysis_config.json')
    prep = json.loads((CAL / 'preparation.json').read_text())
    pole = json.loads((CAL / 'pole_fit.json').read_text())
    cache = np.load(CAL / 'central_cloud_3.npz')
    # Cached cloud uses the same floor and axes, but precedes the pole-origin shift.
    cloud = cache['points'].copy()
    cloud[:, 0] -= pole['corridor_longitudinal_origin_offset_m']
    starts, stamps = cache['starts'], cache['stamps']
    by_frame = {}
    for r in tracks:
        item = dict(id=int(r['track_id']), status=r['status'], detected=r['detected'] == '1',
                    x=float(r['corridor_x']), y=float(r['corridor_y']),
                    z=float(r['corridor_z']), points=int(r['point_count']))
        by_frame.setdefault(int(r['frame']), []).append(item)
    # These IDs were selected from the one-walker recording and reviewed against
    # image overlays; they remain original tentative/confirmed states throughout.
    ids = {17, 60}
    person = [r for r in tracks if int(r['track_id']) in ids]
    observed = [r for r in person if r['detected'] == '1']
    confirmed = [r for r in observed if r['status'] == 'confirmed']
    selected_stamps = {int(r['timestamp_ns']) for r in confirmed}
    all_observed_stamps = {int(r['timestamp_ns']) for r in observed}
    # Broad image-review interval including entry/exit and short empty margins.
    # It is not an exhaustive human-presence label or surveyed 3--15 m gate.
    visible = [r for r in ledger if 2.0 <= (int(r['timestamp_ns']) - first) / 1e9 <= 29.8]
    cover = sum(int(r['timestamp_ns']) in selected_stamps for r in visible)
    cand_cover = sum(int(r['timestamp_ns']) in all_observed_stamps for r in visible)
    times = np.array([(int(r['timestamp_ns']) - first) / 1e9 for r in observed])
    xy = np.array([[float(r['corridor_x']), float(r['corridor_y'])] for r in observed])
    dt = np.diff(times)
    speed = np.linalg.norm(np.diff(xy, axis=0), axis=1) / dt
    adjacent = dt < .25
    # Local straight-line residual measures internal smoothness, never accuracy.
    residuals = []
    for i in range(2, len(times) - 2):
        t = times[i-2:i+3]
        if np.max(np.diff(t)) > .25:
            continue
        fit = np.column_stack([np.ones(5), t - times[i]])
        pred = np.linalg.lstsq(fit, xy[i-2:i+3], rcond=None)[0][0]
        residuals.append(float(np.linalg.norm(pred - xy[i])))
    gaps = []
    ct = np.array([(int(r['timestamp_ns']) - first) / 1e9 for r in confirmed])
    for a, b in zip(ct[:-1], ct[1:]):
        if b-a > .25:
            gaps.append(dict(last_observation_s=float(a), next_observation_s=float(b),
                             observation_interval_s=float(b-a)))
    turn = int(np.argmax(xy[:, 0]))
    # Reconstruct consecutive-observation window used for tentative promotion.
    diagnostic = []
    for track_id in sorted(ids):
        seq = [r for r in person if int(r['track_id']) == track_id]
        point_sum = 0
        hits = 0
        recent = []
        last_frame = None
        for r in seq:
            if r['detected'] != '1':
                continue
            f = int(r['frame'])
            recent = recent + [np.array([float(r['level_x']), float(r['level_y'])])] if last_frame == f-1 else [np.array([float(r['level_x']), float(r['level_y'])])]
            recent = recent[-15:]
            hits += 1
            point_sum += int(r['point_count'])
            last_frame = f
            dist = np.linalg.norm([float(r['level_x']), float(r['level_y'])])
            diagnostic.append(dict(track_id=track_id, camera_elapsed_s=(int(r['timestamp_ns'])-first)/1e9,
                                   status=r['status'], consecutive_window_length=len(recent),
                                   mean_points=point_sum/hits,
                                   confirmation_point_threshold=_confirmation_point_threshold(dist, settings)))
    with (OUT / 'confirmation_diagnostic.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(diagnostic[0]))
        writer.writeheader(); writer.writerows(diagnostic)

    # Independent image foreground selection. No LiDAR position, depth, or ID is
    # used to select the image component. This is NOT semantic body tracking.
    backgrounds = [cv2.imread(str(REVIEW / r['image_file'])) for r in frames[:12]]
    gray_bg = np.median([cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) for im in backgrounds], axis=0).astype('uint8')
    image_data = []
    montage = []
    (OUT / 'review_frames').mkdir()
    for i, r in enumerate(frames):
        image = cv2.imread(str(REVIEW / r['image_file']))
        if image is None or image.shape != (720, 1280, 3):
            raise RuntimeError(f'Bad image {r}')
        with np.load(REVIEW / r['depth_file']) as arc:
            depth = arc['depth_m']
        if depth.shape != image.shape[:2]:
            raise RuntimeError('Image/depth shape mismatch')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        mask = (cv2.absdiff(gray, gray_bg) > 24).astype('uint8')
        mask[:, :480] = 0; mask[:, 820:] = 0
        mask[:120] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), 'uint8'))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), 'uint8'))
        n, labels, boxes, _ = cv2.connectedComponentsWithStats(mask, 8)
        options = []
        for label in range(1, n):
            x, y, w, h, area = boxes[label]
            if area > 70 and h >= 25 and w >= 5 and h/w >= 1.1:
                options.append((int(area), label))
        record = dict(sample_index=i, camera_elapsed_s=float(r['elapsed_s']),
                      image_timestamp_ns=int(r['image_timestamp_ns']), bbox=None,
                      foreground_depth_m=None, foreground_x=None, foreground_y=None,
                      valid_depth_pixels=0, torso_iqr_m=None, paired_lidar_id=None,
                      paired_lidar_status=None, depth_difference_m=None,
                      longitudinal_difference_m=None, nearest_header_residual_ms=None)
        if options:
            _, label = max(options)
            x, y, w, h, area = map(int, boxes[label])
            # Inner upper/middle body patch excludes feet and reflected shadows.
            inner = cv2.erode((labels == label).astype('uint8'), np.ones((3, 3), 'uint8')) > 0
            roi = np.zeros_like(inner)
            roi[y+int(.15*h):y+int(.65*h), x+int(.18*w):x+int(.82*w)+1] = True
            good = inner & roi & np.isfinite(depth) & (depth > .2) & (depth < 40)
            yy, xx = np.where(good)
            record['bbox'] = [x, y, w, h]
            record['valid_depth_pixels'] = len(xx)
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 190, 255), 2)
            if len(xx) >= 15:
                z = float(np.median(depth[good]))
                u, v = float(np.median(xx)), float(np.median(yy))
                zed = np.array([(u-camera['cx'])*z/camera['fx'], (v-camera['cy'])*z/camera['fy'], z])
                # settings transform accepts level coordinates; invert that same
                # frozen transform, then use the frozen corridor transform.
                level = (zed-settings.lidar_to_zed_frame.translation) @ settings.lidar_to_zed_frame.rotation
                c = settings.corridor_frame.transform_position(level)
                record.update(foreground_depth_m=z, foreground_x=float(c[0]), foreground_y=float(c[1]),
                              torso_iqr_m=float(np.percentile(depth[good], 75)-np.percentile(depth[good], 25)))
            cv2.circle(image, (x+w//2, y+int(.4*h)), 4, (0, 190, 255), -1)
        stamp = int(r['image_timestamp_ns'])
        j = int(np.argmin(np.abs(stamps-stamp)))
        record['nearest_header_residual_ms'] = float((int(stamps[j])-stamp)/1e6)
        # Raw absolute timestamps, NO clock correction or positional fit.
        candidates = [s for s in person if int(s['frame']) == j+1 and s['detected'] == '1']
        for s in candidates:
            z = float(s['zed_z'])
            u = round(camera['fx']*float(s['zed_x'])/z+camera['cx'])
            v = round(camera['fy']*float(s['zed_y'])/z+camera['cy'])
            col = (255, 150, 40) if s['status'] == 'confirmed' else (210, 60, 210)
            cv2.drawMarker(image, (u, v), col, cv2.MARKER_CROSS, 18, 2)
            if abs(record['nearest_header_residual_ms']) <= 60:
                record.update(paired_lidar_id=int(s['track_id']), paired_lidar_status=s['status'])
                if record['foreground_depth_m'] is not None:
                    record['depth_difference_m'] = record['foreground_depth_m']-z
                    record['longitudinal_difference_m'] = record['foreground_x']-float(s['corridor_x'])
        cv2.putText(image, f"ZED {float(r['elapsed_s']):.2f}s | raw timestamps, sync unverified", (15, 30), 0, .65, (255, 255, 255), 2)
        cv2.imwrite(str(OUT / 'review_frames' / f'{i:04d}.jpg'), cv2.resize(image, (768, 432)), [cv2.IMWRITE_JPEG_QUALITY, 85])
        if i % 15 == 0 or i == 299:
            crop = image[140:720, 470:830]
            crop = cv2.resize(crop, (180, 290))
            cv2.putText(crop, f'{i}: {float(r["elapsed_s"]):.1f}s', (4, 20), 0, .45, (255, 255, 255), 1)
            montage.append(crop)
        image_data.append(record)
    canvas = np.zeros((4*290, 6*180, 3), 'uint8')
    for k, im in enumerate(montage):
        canvas[(k//6)*290:(k//6+1)*290, (k%6)*180:(k%6+1)*180] = im
    cv2.imwrite(str(OUT / 'correspondence_review.jpg'), canvas)
    with (OUT / 'image_measurements.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(image_data[0]))
        writer.writeheader(); writer.writerows(image_data)

    metrics = dict(recording='20261021_group002_recording_003', processed_clouds=len(ledger),
                   corridor_width_m=settings.corridor_frame.wall_gap,
                   review_images=len(frames), image_duration_s=float(frames[-1]['elapsed_s']),
                   selected_ids=sorted(ids), selection_note='One walker, offline visual correspondence review; tentative status is not promoted.',
                   visible_window_s=[2., 29.8], visibility_note='Broad image-review window including entry/exit and short empty margins; not exhaustive person-presence labels or surveyed 3--15m gate. Fractions are frame coverage, not recall.',
                   visible_clouds=len(visible), confirmed_frames_in_visible_window=cover,
                   confirmed_coverage=cover/len(visible),
                   selected_candidate_observed_frames_in_visible_window=cand_cover,
                   candidate_observation_coverage=cand_cover/len(visible),
                   confirmed_rows=len(confirmed), selected_observed_rows=len(observed),
                   confirmed_x_range_m=[min(float(r['corridor_x']) for r in confirmed), max(float(r['corridor_x']) for r in confirmed)],
                   all_selected_x_range_m=[float(xy[:, 0].min()), float(xy[:, 0].max())],
                   selected_y_range_m=[float(xy[:, 1].min()), float(xy[:, 1].max())],
                   observed_turn_time_s=float(times[turn]), observed_turn_x_m=float(xy[turn, 0]),
                   confirmed_observation_gaps_over_250ms=gaps,
                   selected_observation_intervals_s=stats(dt),
                   selected_gaps_over_250ms=[dict(start_s=float(a), end_s=float(b), interval_s=float(b-a)) for a,b in zip(times[:-1],times[1:]) if b-a > .25],
                   selected_adjacent_speed_mps=stats(speed[adjacent]),
                   local_linear_residual_m=stats(residuals),
                   local_residual_note='5 consecutive observed samples, <=250ms gaps; internal smoothness, NOT true position error.',
                   pole_distance_residual_m=pole['pole_distance_residual_m'],
                   calibration_note='Frozen recording_002 floor, wall axes, nominal-pole origin offset; same installation assumed. No scale tuning on walk.',
                   installation_up_change_deg=prep['installation_up_change_deg'],
                   clock_note='Raw absolute timestamps only. Nearest-frame residual is NOT synchronization accuracy.',
                   nearest_image_cloud_residual_ms=stats([r['nearest_header_residual_ms'] for r in image_data]),
                   zed_lidar_depth_difference_m=stats([r['depth_difference_m'] for r in image_data if r['depth_difference_m'] is not None]),
                   comparison_note='Independent RGB foreground/depth median vs LiDAR cluster median; different body surfaces, extrinsic candidate, unverified sync. Not ground truth.',
                   absolute_position_error_m=None, absolute_error_note='No independent time-series position reference. Endpoint declaration is not a surveyed body-center time series.')
    # Comparisons by LiDAR depth range, retaining separate candidate/confirmed counts.
    ranges = []
    for lo, hi in [(3, 5), (5, 7), (7, 9), (9, 11), (11, 13), (13, 15), (15, 17)]:
        a = [r for r in observed if lo <= float(r['corridor_x']) < hi]
        b = [r for r in a if r['status'] == 'confirmed']
        ranges.append(dict(lo_m=lo, hi_m=hi, selected_observed=len(a), confirmed=len(b),
                           note='Counts of selected observations, not a distance-specific detection rate.'))
    metrics['range_counts'] = ranges
    save_json('metrics.json', metrics)
    # Sensitivity to a plausible timestamp shift is descriptive, not an estimate.
    sensitivity = []
    for delta in [-.2, -.1, 0, .1, .2]:
        diffs = []
        for r in image_data:
            if r['foreground_depth_m'] is None:
                continue
            target = r['camera_elapsed_s']+delta
            j = int(np.argmin(abs(times-target)))
            if abs(times[j]-target) <= .06:
                diffs.append(r['foreground_depth_m']-float(observed[j]['zed_z']))
        sensitivity.append(dict(lidar_target_minus_image_s=delta, signed_difference_m=stats(diffs),
                                abs_difference_m=stats(np.abs(diffs))))
    save_json('clock_sensitivity.json', sensitivity)

    plot_trajectories(observed, confirmed, first, settings.corridor_frame.wall_gap)
    dashboard = dict(metrics=metrics, images=image_data, frames=[])
    for i, r in enumerate(ledger):
        pts = cloud[starts[i]:starts[i+1]]
        dashboard['frames'].append(dict(t=(int(r['timestamp_ns'])-first)/1e9,
                                        points=np.round(pts[:, :2], 3).tolist(),
                                        tracks=by_frame.get(int(r['frame']), [])))
    # Embed data so file:// works and the page needs no server or network.
    packed = json.dumps(dashboard, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
    template = (OUT / 'dashboard_template.html').read_text()
    with (OUT / 'index.html').open('x', encoding='utf-8') as stream:
        stream.write(template.replace('__DATA__', packed.replace('</', '<\\/')))
    save_json('dashboard.json', dashboard)
    sources = [REVIEW/'camera.json', REVIEW/'frames.csv', REVIEW/'imu.csv', OUT/'analysis_config.json',
               OUT/'corridor_candidate.json', OUT/'tracks.csv', CAL/'preparation.json', CAL/'pole_fit.json',
               CAL/'candidate.json', CAL/'central_cloud_3.npz', Path(__file__), OUT/'dashboard_template.html',
               ROOT/'scripts/track_people.py']
    save_json('manifest.json', dict(inputs_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                                    python=sys.version, numpy=np.__version__, opencv=cv2.__version__,
                                    original_files_modified=False, live_devices_started=False))
    print(json.dumps(metrics, ensure_ascii=False, indent=2))


def plot_trajectories(observed, confirmed, first, width):
    fig, (ax, time_ax) = plt.subplots(2, 1, figsize=(14, 7), gridspec_kw={'height_ratios':[1, 2]})
    for track_id, col in [(17, '#1686c9'), (60, '#d26c0c')]:
        rows = [r for r in observed if int(r['track_id']) == track_id]
        x = np.array([float(r['corridor_x']) for r in rows]); y = np.array([float(r['corridor_y']) for r in rows])
        t = np.array([(int(r['timestamp_ns'])-first)/1e9 for r in rows])
        # Dots never bridge missing observations.
        ax.scatter(x, y, s=9, c=col, alpha=.7, label=f'ID {track_id}: selected observations incl. tentative')
        time_ax.scatter(t, x, s=10, c=col, alpha=.7)
    cx = [float(r['corridor_x']) for r in confirmed]; cy = [float(r['corridor_y']) for r in confirmed]
    ax.scatter(cx, cy, s=12, color='#1645a5', label='Confirmed + detected only')
    time_ax.scatter([(int(r['timestamp_ns'])-first)/1e9 for r in confirmed], cx, s=13, color='#1645a5', label='Confirmed + detected')
    for yy in [-width/2, width/2]:
        ax.axhline(yy, color='#63717d')
    ax.axhline(0, color='gray', ls=':', lw=.7)
    ax.set(xlim=(0, 18), ylim=(-1.6, 1.6), xlabel='Corridor longitudinal X (m)', ylabel='Lateral Y (m)', title='Top view / equal horizontal scale; origin from nominal pole positions')
    ax.set_aspect('equal'); ax.legend(loc='upper right', fontsize=7)
    time_ax.set(xlabel='ZED image elapsed time (s), raw absolute timestamps', ylabel='Corridor X (m)', ylim=(0, 18), title='Solid blue observations are confirmed. Other dots are candidates, not successful detections.')
    time_ax.axhline(3, color='gray', ls=':', lw=.7); time_ax.axhline(15, color='gray', ls=':', lw=.7)
    for a in [ax, time_ax]:
        a.grid(alpha=.15)
    fig.tight_layout(); fig.savefig(OUT/'trajectory.png', dpi=160); plt.close(fig)


if __name__ == '__main__':
    main()
