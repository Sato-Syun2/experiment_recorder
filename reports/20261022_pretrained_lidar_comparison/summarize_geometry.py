"""Supplemental, descriptive geometry statistics; no metric ground truth."""
from pathlib import Path
import csv
import hashlib
import json
import numpy as np

p=Path(__file__).resolve().parent
source=p/'full/detections.csv'
with source.open(newline='') as f:
    rows=[r for r in csv.DictReader(f) if r['class_name']=='Pedestrian' and
          0<float(r['corridor_x'])<20 and abs(float(r['corridor_y']))<1.2]
def stats(a):
    return dict(n=len(a),median=float(np.median(a)),p10=float(np.percentile(a,10)),
                p90=float(np.percentile(a,90)),minimum=float(min(a)),maximum=float(max(a)))
data=dict(model_box_height_m=stats([float(r['height']) for r in rows]),
          model_box_bottom_relative_to_calibration_floor_m=stats([float(r['level_z'])-float(r['height'])/2 for r in rows]),
          model_score=stats([float(r['score']) for r in rows]),
          visual_review=dict(method='Assistant inspection of saved image gallery and original-size 8s/24s overlays',
                             checked=['image_gallery.jpg','overlay_08s.jpg','overlay_24s.jpg'],
                             note='Near/middle range boxes follow the visible walker; missing far/turn interval and below-floor box bottoms remain. Not exhaustive frame-level ground truth.'),
          note='Estimated person boxes and partial installation calibration, not anatomical dimensions or independent accuracy',
          sha256={str(f.resolve()):hashlib.sha256(f.read_bytes()).hexdigest() for f in [source,Path(__file__),p/'analyze.py']})
with (p/'comparison_v2/supplementary_metrics.json').open('x') as f:
    json.dump(data,f,ensure_ascii=False,indent=2)
    f.write('\n')
