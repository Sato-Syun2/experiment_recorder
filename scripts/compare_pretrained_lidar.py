#!/usr/bin/env python3
"""Offline PointPillars inference and unchanged-rule baseline on the same bag.

Uses a KITTI pretrained detector without fitting to this recording. The model
estimates person boxes, not gait or joints. No live ROS nodes are started.
Install torch-pointcloud in an isolated Python environment; see the run report.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.metadata
import json
from pathlib import Path
import time

import numpy as np

import track_people as legacy


def sha256(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def voxelize(points, max_points=32, max_voxels=40000):
    """CPU equivalent of first-encounter hard pillars; retain input point order.

    Returns z,y,x coordinates. Avoids a spconv/CUDA dependency without changing
    the pretrained network's 0.16 x 0.16 x 4 m pillar layout.
    """
    lo = np.array([0., -39.68, -3.], np.float32)
    hi = np.array([69.12, 39.68, 1.], np.float32)
    p = points[np.isfinite(points).all(axis=1) &
               (points[:, :3] >= lo).all(axis=1) &
               (points[:, :3] < hi).all(axis=1)]
    keys = np.floor((p[:, :3] - lo) / np.array([.16, .16, 4.], np.float32)).astype(np.int32)
    voxels = np.zeros((min(max_voxels, len(p)), max_points, 4), np.float32)
    coords = np.zeros((len(voxels), 3), np.int32)
    counts = np.zeros(len(voxels), np.int32)
    table = {}
    for point, key in zip(p, keys):
        key = tuple(int(v) for v in key)
        idx = table.get(key)
        if idx is None:
            if len(table) >= max_voxels:
                continue
            idx = len(table)
            table[key] = idx
            coords[idx] = key[::-1]
        if counts[idx] < max_points:
            voxels[idx, counts[idx]] = point
            counts[idx] += 1
    n = len(table)
    return voxels[:n], coords[:n], counts[:n]


def read_cloud(message, actual_type):
    if actual_type == 'custom':
        a = np.array([(p.x, p.y, p.z, p.reflectivity) for p in message.points], np.float32)
    else:
        from sensor_msgs_py import point_cloud2
        names = [f.name for f in message.fields]
        intensity = next((n for n in ('intensity', 'reflectivity') if n in names), None)
        if intensity is None:
            raise ValueError('Intensity field required; refusing to fabricate reflectance.')
        cloud = point_cloud2.read_points(message, field_names=('x', 'y', 'z', intensity), skip_nans=True)
        a = np.column_stack([cloud[n] for n in ('x', 'y', 'z', intensity)]).astype(np.float32)
    if a.size == 0:
        return np.empty((0, 4), np.float32)
    return a.reshape(-1, 4)


def write_rows(path, rows, fields):
    with path.open('x', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--max-frames', type=int)
    parser.add_argument('--step', type=int, default=1)
    parser.add_argument('--model-height-m', type=float, default=1.73)
    parser.add_argument('--threads', type=int, default=4)
    args = parser.parse_args()
    if args.step < 1 or args.threads < 1 or (args.max_frames is not None and args.max_frames < 1):
        parser.error('step, threads and max-frames must be positive')
    if not np.isfinite(args.model_height_m) or args.model_height_m <= 0:
        parser.error('model-height-m must be finite and positive')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    settings = legacy.load_settings(args.config)
    if not settings.gravity_alignment_enabled or settings.corridor_frame is None:
        raise ValueError('A recording-specific gravity and corridor calibration is required.')

    import torch
    import torch_pointcloud as tp
    from torch_pointcloud.models import model_info
    from torch_pointcloud.ops.box3d import nms3d
    torch.set_num_threads(args.threads)
    torch.manual_seed(0)
    model = tp.create_model('pointpillars.kitti.openpcdet', pretrained=True).cpu().eval()
    baseline = legacy.PeopleTracker(settings)
    baseline_path = args.output_dir / 'baseline_tracks.csv'
    writer = legacy.CsvWriter(baseline_path, settings.corridor_frame, settings.lidar_to_zed_frame,
                             settings.gravity_alignment_rotation, settings.gravity_alignment_translation)
    records, ledger, review_clouds = [], [], []
    start_ns, processed = None, 0
    begin = time.monotonic()
    try:
        for index, (message, bag_stamp, kind) in enumerate(legacy._iter_bag_clouds(
                args.bag, settings.topic, 'auto', 'mcap'), 1):
            if (index-1) % args.step:
                continue
            stamp = legacy._message_timestamp_ns(message, bag_stamp)
            if start_ns is None:
                start_ns = stamp
            raw = read_cloud(message, kind)
            level = raw[:, :3] @ settings.gravity_alignment_rotation.T + settings.gravity_alignment_translation
            # No background removal, point downsampling, temporal accumulation,
            # or human-specific ROI before learned inference.
            model_points = np.column_stack([level, np.clip(raw[:, 3], 0, 255)/255.]).astype(np.float32)
            model_points[:, 2] -= args.model_height_m
            v, c, n = voxelize(model_points)
            inference_begin = time.monotonic()
            if len(v):
                with torch.inference_mode():
                    out = model(torch.from_numpy(v), torch.from_numpy(c),
                                torch.from_numpy(n), torch.zeros(len(n), dtype=torch.long))
                    decoded = model.decode(out)
                    keep = decoded['scores'] >= .1  # OpenPCDet KITTI score threshold
                    boxes, scores = decoded['boxes'][keep], decoded['scores'][keep]
                    labels = decoded['labels'][keep]
                    # Official OpenPCDet class-agnostic rotated BEV NMS, IoU .01.
                    order = scores.argsort(descending=True)[:4096]
                    boxes, scores, labels = boxes[order], scores[order], labels[order]
                    kept = nms3d(boxes, scores, .01, rotated=True, max_keep=500)
                    boxes, scores, labels = boxes[kept].numpy(), scores[kept].numpy(), labels[kept].numpy()
            else:
                boxes, scores, labels = np.empty((0, 7)), np.empty(0), np.empty(0, int)
            inference_s = time.monotonic() - inference_begin
            for box, score, label in zip(boxes, scores, labels):
                pos = box[:3].copy()
                pos[2] += args.model_height_m
                corridor = settings.corridor_frame.transform_position(pos)
                records.append(dict(frame=index, timestamp_ns=stamp, label=int(label),
                                    class_name=('Car', 'Pedestrian', 'Cyclist')[int(label)], score=float(score),
                                    level_x=float(pos[0]), level_y=float(pos[1]), level_z=float(pos[2]),
                                    corridor_x=float(corridor[0]), corridor_y=float(corridor[1]),
                                    corridor_z=float(corridor[2]), length=float(box[3]), width=float(box[4]),
                                    height=float(box[5]), yaw=float(box[6])))
            snapshots = baseline.update(raw[:, :3], stamp)
            writer.write(index, stamp, snapshots, start_ns)
            ledger.append(dict(frame=index, timestamp_ns=stamp, bag_timestamp_ns=bag_stamp,
                               input_points=len(raw), input_intensity_min=float(raw[:, 3].min()) if len(raw) else None,
                               input_intensity_max=float(raw[:, 3].max()) if len(raw) else None,
                               pillars=len(v), model_inference_s=inference_s,
                               model_pedestrians=int((labels == 1).sum()),
                               baseline_confirmed=sum(s.detected and s.confirmed for s in snapshots)))
            # Small corridor crop for interactive visual evidence, not network input.
            xy = level[:, :2] - settings.corridor_frame.origin_xy
            cx = xy @ settings.corridor_frame.longitudinal_xy
            cy = xy @ settings.corridor_frame.lateral_xy
            mask = (cx > 0) & (cx < 20) & (abs(cy) < 1.6) & (level[:, 2] > .1) & (level[:, 2] < 2.3)
            cloud = np.column_stack([cx[mask], cy[mask], level[mask, 2]])
            if len(cloud) > 1800:
                cloud = cloud[np.linspace(0, len(cloud)-1, 1800).astype(int)]
            review_clouds.append(dict(frame=index, timestamp_ns=str(stamp), points=np.round(cloud, 3).tolist()))
            processed += 1
            if processed == 1 or processed % 10 == 0:
                print(f'processed={processed} source_frame={index} pedestrians={int((labels==1).sum())} inference_s={inference_s:.3f}', flush=True)
            if args.max_frames is not None and processed >= args.max_frames:
                break
    finally:
        writer.close()
    write_rows(args.output_dir/'detections.csv', records,
               ['frame','timestamp_ns','label','class_name','score','level_x','level_y','level_z',
                'corridor_x','corridor_y','corridor_z','length','width','height','yaw'])
    write_rows(args.output_dir/'frames.csv', ledger, list(ledger[0]) if ledger else ['frame','timestamp_ns'])
    (args.output_dir/'review_clouds.json').write_text(json.dumps(review_clouds, separators=(',', ':')))
    # Record the downloaded, pinned weight artifact as well as source/config hashes.
    info = model_info('pointpillars.kitti.openpcdet')
    from torch_pointcloud.config import MODELS_DIR
    cache = MODELS_DIR
    weights = [p for p in Path(cache).rglob('*.safetensors') if 'pointpillars' in str(p)]
    manifest = dict(completed=True, processed_frames=processed, source_frames_step=args.step,
                    bag=str(args.bag.resolve()), config=str(args.config.resolve()),
                    model=info['name'], pretrained=info['weights'], model_hparams=info['hparams'],
                    packages={n: importlib.metadata.version(n) for n in
                              ['torch','torch-pointcloud','numpy','scipy','safetensors']},
                    device='cpu', threads=args.threads, model_height_m=args.model_height_m,
                    intensity_transform='clip raw Livox reflectivity to [0,255] then divide by 255; no cross-sensor reflectance calibration',
                    thresholds=dict(score=.1, nms_bev_iou=.01, nms_pre_max=4096, nms_post_max=500),
                    seconds=time.monotonic()-begin,
                    sha256={str(p.resolve()):sha256(p) for p in
                            [Path(__file__), Path(legacy.__file__), args.config,
                             settings.corridor_frame_file, args.bag/'metadata.yaml', *weights]})
    (args.output_dir/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__':
    main()
