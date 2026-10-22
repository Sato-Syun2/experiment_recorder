#!/usr/bin/env python3
"""Build portable recording-003 skeleton playback using the Python standard library.

The bundle must contain bundle.json, skeleton_replay_template.html, and assets/.
SDK export is optional: without it the page explicitly shows that skeleton data
is absent, and can import the actual export folder later in the browser.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import collections
from pathlib import Path


def load_export(root: Path, bundle: dict) -> dict:
    camera = json.loads((root / 'camera.json').read_text(encoding='utf-8-sig'))
    if camera.get('body_format') != 'BODY_34':
        raise ValueError('Expected a BODY_34 SDK export.')
    if camera.get('coordinate_units') != 'meter' or not str(camera.get('coordinate_system', '')).startswith('IMAGE'):
        raise ValueError('Expected metres in the ZED IMAGE coordinate system.')
    if camera.get('serial_number') != bundle['camera']['serial_number']:
        raise ValueError('Camera serial does not match recording 003.')
    with (root / 'frames.csv').open(encoding='utf-8-sig', newline='') as stream:
        frames = list(csv.DictReader(stream))
    if not frames:
        raise ValueError('No SDK frames were exported.')
    origin = int(bundle['origin_ns'])
    lo, hi = (int(frames[i]['image_timestamp_ns']) for i in [0, -1])
    if abs(lo-origin) > 200_000_000 or hi > int(bundle['last_image_ns'])+200_000_000:
        raise ValueError('Export timestamps do not match recording 003; another recording cannot be used.')
    if any(int(a['image_timestamp_ns']) >= int(b['image_timestamp_ns']) for a,b in zip(frames, frames[1:])):
        raise ValueError('SDK frame timestamps must increase strictly.')
    valid_stamps = {int(r['image_timestamp_ns']) for r in frames}
    if len(camera.get('keypoint_names', [])) != 34:
        raise ValueError('Expected 34 SDK keypoint names.')
    bodies = []
    body_counts = collections.Counter()
    seen = set()
    with (root / 'bodies.csv').open(encoding='utf-8-sig', newline='') as stream:
        for r in csv.DictReader(stream):
            stamp = int(r['image_timestamp_ns'])
            if stamp not in valid_stamps:
                raise ValueError('Body timestamp is absent from the SDK frame ledger.')
            key = (stamp, str(r['body_id']))
            if key in seen:
                raise ValueError('Duplicate body ID in an SDK frame.')
            seen.add(key)
            body_counts[stamp] += 1
            def cell(name):
                return json.loads(r.get(name) or '[]')
            joints, pixels, confidence = (cell(name) for name in (
                'keypoints_3d_camera_json', 'keypoints_2d_json', 'keypoint_confidence_json'))
            if any(not isinstance(a, list) or len(a) != 34 for a in [joints, pixels, confidence]):
                raise ValueError('Expected 34 joints, pixels, and joint confidence values.')
            bodies.append(dict(t=(stamp-origin)/1e9, id=str(r['body_id']),
                               state=r['tracking_state'], confidence=float(r['confidence']),
                               joints=joints, pixels=pixels, jointConfidence=confidence))
    if any(body_counts[int(r['image_timestamp_ns'])] != int(r['body_count']) for r in frames):
        raise ValueError('Body observation counts differ from the SDK frame ledger.')
    return dict(camera=camera, frames=[dict(t=(int(r['image_timestamp_ns'])-origin)/1e9,
                                          count=int(r['body_count'])) for r in frames],
                bodies=bodies, source=str(root),
                complete_frame_count=len(frames) == bundle['camera']['total_svo_frames'],
                note='SDK positions remain independent of LiDAR markers. No translation fitting or invented animation.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle-dir', type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument('--body-dir', type=Path)
    parser.add_argument('--quality-json', type=Path, help='optional saved recording-specific segment-length review flags')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = args.bundle_dir.resolve()
    if args.output.exists() or args.output.with_suffix('.manifest.json').exists():
        parser.error(f'Choose a new output name: {args.output}')
    bundle = json.loads((root / 'bundle.json').read_text(encoding='utf-8'))
    body = load_export(args.body_dir.resolve(), bundle) if args.body_dir else None
    if body is not None:
        # The last SDK frame may follow the last 10 Hz review image by one frame.
        bundle['duration'] = max(bundle['duration'], body['frames'][-1]['t'])
        if args.quality_json:
            quality = json.loads(args.quality_json.read_text(encoding='utf-8'))
            body['quality_reference'] = quality['reference']
    # Use a relative asset path on both Windows (UNC) and Linux.
    import os
    bundle['asset_base'] = Path(os.path.relpath(root/'assets', args.output.resolve().parent)).as_posix()
    readme = args.output.resolve().parent/'README.md'
    if not readme.is_file():
        readme = root/'README.md'
    bundle['readme_path'] = Path(os.path.relpath(readme, args.output.resolve().parent)).as_posix()
    data = json.dumps(dict(bundle=bundle, body=body), ensure_ascii=False, allow_nan=False,
                      separators=(',', ':')).replace('</', '<\\/')
    template = (root / 'skeleton_replay_template.html').read_text(encoding='utf-8')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        stream.write(template.replace('__REPLAY_DATA__', data))
    if args.body_dir:
        sources = [args.body_dir/'camera.json', args.body_dir/'frames.csv',
                   args.body_dir/'bodies.csv', root/'bundle.json',
                   root/'skeleton_replay_template.html', Path(__file__)]
        if args.quality_json:
            sources.append(args.quality_json)
        manifest = dict(body_dir=str(args.body_dir.resolve()), body_rows=len(body['bodies']),
                        sdk_frames=len(body['frames']), complete_frame_count=body['complete_frame_count'],
                        source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
        with args.output.with_suffix('.manifest.json').open('x', encoding='utf-8') as stream:
            json.dump(manifest, stream, ensure_ascii=False, indent=2)
    print(f'Playback: {args.output.resolve()}')
    if body is None:
        print('Skeleton data is ABSENT. Import the real recording-003 SDK export before viewing bones.')
    else:
        print(f"SDK frames: {len(body['frames'])}; body rows: {len(body['bodies'])}")


if __name__ == '__main__':
    main()
