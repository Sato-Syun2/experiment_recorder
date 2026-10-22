"""Read-only review audit; manual stem ROIs are diagnostics, not calibration."""
from pathlib import Path
import csv
import hashlib
import json
import cv2
import numpy as np

ROOT = Path('/data/experiment_data/20261021_group002_recording_002_pole_review')
OUT = Path(__file__).resolve().parent
rows = list(csv.DictReader((ROOT / 'frames.csv').open()))
imu = list(csv.DictReader((ROOT / 'imu.csv').open()))
cam = json.loads((ROOT / 'camera.json').read_text())
errors, stats = [], []
for r in rows:
    item = {'sample_index': int(r['sample_index']), 'elapsed_s': float(r['elapsed_s'])}
    try:
        im = cv2.imread(str(ROOT / r['image_file']))
        if im is None or im.shape[:2] != (cam['height'], cam['width']):
            raise ValueError('Image missing, unreadable, or unexpected dimensions')
        with np.load(ROOT / r['depth_file']) as a:
            d = a['depth_m'].astype(np.float32)
        if d.shape != im.shape[:2]:
            raise ValueError('Depth dimensions differ from image')
        good = np.isfinite(d) & (d > 0)
        item.update(valid_depth_fraction=float(good.mean()),
                    depth_gt_15m_pixels=int(np.sum(good & (d > 15))))
    except Exception as e:
        errors.append({'sample_index': r['sample_index'], 'error': str(e)})
    stats.append(item)
stamps = np.array([int(r['image_timestamp_ns']) for r in rows], dtype=np.int64)
dt = np.diff(stamps) / 1e9
imu_stamps = {int(r['image_timestamp_ns']) for r in imu}
imu_lag = [(int(r['imu_timestamp_ns']) - int(r['image_timestamp_ns'])) / 1e6 for r in imu]
acc = np.array([[float(r['accel_'+axis+'_m_s2']) for axis in 'xyz'] for r in imu])
# Manually selected interior stem pixels from pole_samples.jpg; x/y in full image.
rois = [(10,646,450), (20,638,360), (30,638,310), (40,633,285),
        (53,630,270), (63,631,260), (73,631,250)]
pole_stats = []
for t,x,y in rois:
    r = min(rows, key=lambda r: abs(float(r['elapsed_s'])-t))
    with np.load(ROOT / r['depth_file']) as a:
        d = a['depth_m'].astype(np.float32)
    roi = d[y-5:y+6, x-1:x+2]
    v = roi[np.isfinite(roi) & (roi>0)]
    pole_stats.append(dict(sample_index=int(r['sample_index']), elapsed_s=float(r['elapsed_s']),
                           roi_xyxy=[x-1,y-5,x+2,y+6], valid_pixels=len(v), total_pixels=roi.size,
                           depth_m_p10_p50_p90=np.percentile(v,[10,50,90]).tolist() if len(v) else None))
summary = dict(input=str(ROOT), script=str(Path(__file__).resolve()), camera=cam,
               metadata_sha256={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest()
                                for n in ['camera.json','frames.csv','imu.csv']},
               image_rows=len(rows), imu_rows=len(imu), errors=errors,
               distinct_image_files=len({r['image_file'] for r in rows}),
               distinct_depth_files=len({r['depth_file'] for r in rows}),
               total_image_files=len(list((ROOT/'frames').glob('*.jpg'))),
               total_depth_files=len(list((ROOT/'depth').glob('*.npz'))),
               imu_missing_image_timestamps=[int(s) for s in stamps if int(s) not in imu_stamps],
               timestamp_nonmonotonic_count=int(np.sum(dt<=0)),
               sample_interval_s_min_median_max=[float(dt.min()),float(np.median(dt)),float(dt.max())],
               gaps_over_0_3s=[dict(after_sample=int(rows[i]['sample_index']),
                                   elapsed_s=float(rows[i]['elapsed_s']), gap_s=float(v))
                               for i,v in enumerate(dt) if v>0.3],
               image_imu_lag_ms_min_median_max=[float(min(imu_lag)),float(np.median(imu_lag)),float(max(imu_lag))],
               imu_acceleration_finite=bool(np.isfinite(acc).all()),
               mean_imu_acceleration_native_m_s2=acc.mean(axis=0).tolist(),
               manual_stem_roi_diagnostics=pole_stats,
               limitations=['ROI depths are optical-axis depths, not measured corridor distances.',
                            'Manual small ROIs can include background; no pole segmentation or calibration fitted.',
                            'No LiDAR clock offset assumed or estimated by this script.'])
with (OUT/'export_check.json').open('x') as f:
    json.dump(summary,f,ensure_ascii=False,indent=2,allow_nan=False)
with (OUT/'frame_depth_check.csv').open('x') as f:
    w=csv.DictWriter(f,fieldnames=['sample_index','elapsed_s','valid_depth_fraction','depth_gt_15m_pixels'])
    w.writeheader();w.writerows(stats)
print(json.dumps({k:v for k,v in summary.items() if k not in ['camera','metadata_sha256']},ensure_ascii=False,indent=2))
