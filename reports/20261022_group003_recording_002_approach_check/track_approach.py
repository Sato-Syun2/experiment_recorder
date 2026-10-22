"""Background preparation only before approach; new tracks at 7.2s."""
from pathlib import Path
import csv
import hashlib
import json
import sys
import time
import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from track_people import (PeopleTracker, CsvWriter, load_settings, detect_clusters,
    _iter_bag_clouds, _pointcloud2_to_numpy, _message_timestamp_ns)
from prepare import BAG, REVIEW, save

START, END = 7.2, 21.0


def main():
    if (OUT / 'tracks.csv').exists():
        raise FileExistsError('Choose fresh output')
    cfg = OUT / 'analysis_config.json'
    settings = load_settings(cfg)
    tracker = PeopleTracker(settings)
    camera = json.loads((REVIEW / 'camera.json').read_text())
    origin = int(camera['first_image_timestamp_ns'])
    writer = CsvWriter(OUT / 'tracks.csv', settings.corridor_frame, None,
                       settings.gravity_alignment_rotation, settings.gravity_alignment_translation)
    ledger, cloud_parts, cloud_starts, stamps, raw_background = [], [], [0], [], []
    bg_count, n, done = 0, 0, False
    started = time.monotonic()
    try:
        for source_frame, (msg, fb, _) in enumerate(_iter_bag_clouds(BAG, '/livox/lidar', 'pointcloud2', 'mcap'), 1):
            stamp = _message_timestamp_ns(msg, fb)
            t = (stamp - origin) / 1e9
            if t > END:
                break
            p = _pointcloud2_to_numpy(msg)
            # Learn the two existing background models from the first 30 bag
            # frames. Do not call update(), assign IDs or keep outbound tracks.
            if bg_count < settings.background_learning_frames:
                processed = tracker.preprocess(p)
                tracker.cluster_background.update(detect_clusters(processed, settings))
                bg_count += 1
                raw_background.append(dict(source_frame=source_frame, elapsed_s=t))
                continue
            if t < START:
                continue
            snapshots = tracker.update(p, stamp)
            n += 1
            writer.write(n, stamp, snapshots, origin)
            ledger.append(dict(frame=n, source_bag_frame=source_frame, timestamp_ns=stamp,
                bag_timestamp_ns=fb, time_sec=t,
                confirmed_detections=sum(s.confirmed and s.detected for s in snapshots),
                predicted_tracks=sum(s.confirmed and not s.detected for s in snapshots)))
            raw = p[np.isfinite(p).all(axis=1)]
            level = raw @ settings.gravity_alignment_rotation.T + settings.gravity_alignment_translation
            c = settings.corridor_frame
            xy = level[:, :2] - c.origin_xy
            cp = np.column_stack([xy @ c.longitudinal_xy, xy @ c.lateral_xy, level[:, 2]])
            keep = (cp[:, 0] > -.5) & (cp[:, 0] < 19) & (abs(cp[:, 1]) < 1.4) & (cp[:, 2] > -.15) & (cp[:, 2] < 2.4)
            cloud_parts.append(cp[keep].astype(np.float32))
            cloud_starts.append(cloud_starts[-1] + int(keep.sum()))
            stamps.append(stamp)
            if n % 25 == 0:
                print('approach frames', n, 'elapsed', round(t, 3), flush=True)
        done = True
    finally:
        writer.close()
        if ledger:
            with (OUT / 'tracks.frames.csv').open('x', newline='') as stream:
                w = csv.DictWriter(stream, fieldnames=list(ledger[0]))
                w.writeheader()
                w.writerows(ledger)
        save('tracks.metadata.json', dict(completed=done, frames=n, background_frames=raw_background,
            background_note='Only preprocess and cluster-background learning on first 30 bag clouds; no outbound tracking.',
            window_zed_s=[START, END], track_history='fresh; IDs start at 1 at approach',
            settings_changes='new installation geometry only; confirmation/detection thresholds unchanged',
            wall_time_s=time.monotonic()-started, config_sha256=hashlib.sha256(cfg.read_bytes()).hexdigest(),
            source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                [Path(__file__), ROOT/'scripts/track_people.py', ROOT/'scripts/assignment.py']}))
    if cloud_parts:
        np.savez_compressed(OUT/'approach_clouds.npz', points=np.concatenate(cloud_parts),
                            starts=cloud_starts, stamps=stamps)
    print('finished', n, 'frames', tracker.next_id-1, 'IDs', flush=True)


if __name__ == '__main__':
    main()
