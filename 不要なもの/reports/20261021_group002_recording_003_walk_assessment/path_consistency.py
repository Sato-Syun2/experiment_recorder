"""Descriptive out/back consistency; candidate observations are not true labels."""
from pathlib import Path
import csv
import json
import numpy as np

OUT = Path(__file__).resolve().parent
with (OUT / 'tracks.csv').open() as stream:
    rows = [r for r in csv.DictReader(stream) if r['track_id'] in {'17', '60'} and r['detected'] == '1']
bins = []
for lo in range(3, 15):
    a = [float(r['corridor_y']) for r in rows if r['track_id'] == '17' and lo <= float(r['corridor_x']) < lo+1]
    b = [float(r['corridor_y']) for r in rows if r['track_id'] == '60' and lo <= float(r['corridor_x']) < lo+1]
    if a and b:
        bins.append(dict(x_lo_m=lo, x_hi_m=lo+1, outgoing_n=len(a), returning_n=len(b),
                         outgoing_median_y_m=float(np.median(a)), returning_median_y_m=float(np.median(b)),
                         abs_median_y_difference_m=float(abs(np.median(a)-np.median(b)))))
legs = []
for track_id, sign in [('17', 1), ('60', -1)]:
    a = [r for r in rows if r['track_id'] == track_id and 3 <= float(r['corridor_x']) <= 14]
    x = np.array([float(r['corridor_x']) for r in a])
    t = np.array([(int(r['timestamp_ns'])-1792563646895296000)/1e9 for r in a])
    use = np.diff(t) <= .25
    legs.append(dict(track_id=int(track_id), observations=len(a), adjacent_pairs=int(use.sum()),
                     expected_direction_fraction=float(np.mean(sign*np.diff(x)[use] > 0)),
                     camera_time_range_s=[float(t[0]), float(t[-1])]))
result = dict(bins=bins, legs=legs,
              median_bin_lateral_difference_m=float(np.median([b['abs_median_y_difference_m'] for b in bins])),
              max_bin_lateral_difference_m=float(max(b['abs_median_y_difference_m'] for b in bins)),
              note='Descriptive consistency only. Includes tentative observations; real walking lines can differ. Not true position error.')
with (OUT / 'path_consistency.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result, indent=2))
