#!/usr/bin/env python3
"""保存済みの録画003用SDK骨格を読み、台帳と形状の整合性を調べる。

同じ録画の4.5〜10.5秒における骨格の節長を基準に、極端な伸縮を確認する。
この判定は見直す箇所の目印であり、独立した真値に対する精度評価ではない。
SDKや実機は起動しない。別録画へ使う場合は参照区間・bundleの前提を見直す。
"""

from pathlib import Path
import argparse
import collections
import csv
import hashlib
import json
import math

import numpy as np
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from build_walk_skeleton_replay import load_export

SEGMENTS = {
    'torso': (0, 3),
    'left_thigh': (18, 19),
    'left_shin': (19, 20),
    'right_thigh': (22, 23),
    'right_shin': (23, 24),
}


def summary(values):
    """有限な標本だけから件数・中央値・分位点・最小最大を求める。"""

    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return {'n': 0}
    return dict(
        n=len(a),
        min=float(a.min()),
        median=float(np.median(a)),
        p10=float(np.percentile(a, 10)),
        p90=float(np.percentile(a, 90)),
        max=float(a.max()),
    )


def main():
    """台帳・骨格形状・関節信頼度を確認し、新規フォルダーへ結果を保存する。"""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--body-dir', type=Path, required=True)
    parser.add_argument('--bundle-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=False)
    bundle = json.loads((args.bundle_dir / 'bundle.json').read_text())
    data = load_export(args.body_dir, bundle)
    frames, bodies = data['frames'], data['bodies']
    cam = data['camera']
    raw_frames = list(csv.DictReader((args.body_dir / 'frames.csv').open(encoding='utf-8-sig')))
    raw_bodies = list(csv.DictReader((args.body_dir / 'bodies.csv').open(encoding='utf-8-sig')))
    counts = collections.Counter(int(r['image_timestamp_ns']) for r in raw_bodies)
    for r in raw_frames:
        if counts[int(r['image_timestamp_ns'])] != int(r['body_count']):
            raise ValueError('SDK body count differs from frame ledger')
    if len({(r['image_timestamp_ns'], r['body_id']) for r in raw_bodies}) != len(raw_bodies):
        raise ValueError('Duplicate body/frame observation')
    state_counts = collections.Counter(r['state'] for r in bodies)
    names = cam['keypoint_names']
    # この録画の早い区間を基準にする。実測した身体寸法ではなく、録画固有の目安。
    ref = {}
    for name, (a, b) in SEGMENTS.items():
        ref[name] = float(
            np.median(
                [
                    np.linalg.norm(
                        np.asarray(r['joints'][a], float) - np.asarray(r['joints'][b], float)
                    )
                    for r in bodies
                    if r['state'] == 'OK' and 4.5 <= r['t'] <= 10.5
                ]
            )
        )
    ext = bundle['extrinsic']
    R = np.array(ext['rotation_source_to_target'])
    tr = np.array(ext['translation_m'])
    levelR = np.array(bundle['level_rotation'])
    shift = np.array(bundle['level_translation'])
    corr = bundle['corridor']
    origin = np.array(corr['origin_xy'])
    axes = np.array([corr['longitudinal_xy'], corr['lateral_xy']])
    rows = []
    for raw, r in zip(raw_bodies, bodies):
        p = np.asarray(r['joints'], dtype=float)
        if p.shape != (34, 3) or len(r['jointConfidence']) != 34 or len(r['pixels']) != 34:
            raise ValueError('Expected 34 joints, pixels, and confidence values')
        level = ((p - tr) @ R) @ levelR.T + shift
        c = np.column_stack([(level[:, :2] - origin) @ axes.T, level[:, 2]])
        lengths = {k: float(np.linalg.norm(p[a] - p[b])) for k, (a, b) in SEGMENTS.items()}
        ratios = {k: v / ref[k] for k, v in lengths.items()}
        # 0.5〜1.8倍を外れた節を確認対象とする。元の関節座標は補正しない。
        issues = [k for k, v in ratios.items() if not math.isfinite(v) or v < 0.5 or v > 1.8]
        row = dict(
            frame_index=int(raw['frame_index']),
            t=r['t'],
            id=r['id'],
            state=r['state'],
            confidence=r['confidence'],
            shape_ok=not issues,
            shape_issues=issues,
            pelvis=c[0].tolist(),
            head_height_m=float(c[26, 2]),
            left_ankle_height_m=float(c[20, 2]),
            right_ankle_height_m=float(c[24, 2]),
            lengths_m=lengths,
            length_ratios=ratios,
        )
        rows.append(row)
    ok = [r for r in rows if r['state'] == 'OK']
    coherent = [r for r in ok if r['shape_ok']]
    inconsistent = [r for r in ok if not r['shape_ok']]
    # Windowsでの書き出し・再生作成時に記録された入力ハッシュと照合する。
    source_manifest = (
        args.body_dir.parent
        / f"walk_skeleton_{args.body_dir.name.removeprefix('body_')}.manifest.json"
    )
    manifest_check = []
    if source_manifest.is_file():
        source = json.loads(source_manifest.read_text(encoding='utf-8-sig'))
        for p in [
            args.body_dir / 'camera.json',
            args.body_dir / 'frames.csv',
            args.body_dir / 'bodies.csv',
        ]:
            matches = [
                v
                for k, v in source['source_sha256'].items()
                if k.replace('\\', '/').endswith('/' + p.name)
            ]
            manifest_check.append(
                dict(
                    file=p.name,
                    matches_recorded_sha256=len(matches) == 1
                    and matches[0] == hashlib.sha256(p.read_bytes()).hexdigest(),
                )
            )
    confidence_by_joint = {}
    for name in ['PELVIS', 'NECK', 'LEFT_KNEE', 'RIGHT_KNEE', 'LEFT_ANKLE', 'RIGHT_ANKLE']:
        idx = names.index(name)
        confidence_by_joint[name] = {}
        for label, subset in [
            ('sdk_OK', [r for r in bodies if r['state'] == 'OK']),
            (
                'shape_coherent_OK',
                [r for r, q in zip(bodies, rows) if q['state'] == 'OK' and q['shape_ok']],
            ),
            (
                'shape_inconsistent_OK',
                [r for r, q in zip(bodies, rows) if q['state'] == 'OK' and not q['shape_ok']],
            ),
        ]:
            vals = [r['jointConfidence'][idx] for r in subset]
            confidence_by_joint[name][label] = summary(vals)
    near = []
    # ZED画像とSDK出力は同じ画像時刻を参照する。
    # 以下のLiDAR比較は元の時計の近傍比較であり、同期補正済みとは扱わない。
    image_times = np.array([r['t'] for r in bundle['images']])
    body_times = np.array([r['t'] for r in frames])
    for at in image_times:
        near.append(float(np.min(abs(body_times - at))))
    # ここは既存候補も含めた診断比較。確定人数・検出成功率の集計には使わない。
    selected = [
        r
        for r in bundle['lidar']
        if any(x['id'] in bundle['selected_ids'] and x['detected'] for x in r['tracks'])
    ]
    paired = []
    for q in rows:
        if q['state'] != 'OK':
            continue
        f = min(selected, key=lambda f: abs(f['t'] - q['t']))
        if abs(f['t'] - q['t']) > 0.06:
            continue
        candidates = [r for r in f['tracks'] if r['id'] in bundle['selected_ids'] and r['detected']]
        for r in candidates:
            paired.append(
                dict(
                    t=q['t'],
                    shape_ok=q['shape_ok'],
                    lidar_status=r['status'],
                    dx_m=q['pelvis'][0] - r['x'],
                    dy_m=q['pelvis'][1] - r['y'],
                    raw_clock_difference_ms=(f['t'] - q['t']) * 1e3,
                )
            )
    stats = dict(
        sdk_frames=len(frames),
        expected_frames=bundle['camera']['total_svo_frames'],
        body_rows=len(bodies),
        state_counts=dict(state_counts),
        ids=dict(collections.Counter(r['id'] for r in bodies)),
        duration_s=frames[-1]['t'],
        timestamp_monotonic=True,
        counts_match_frame_ledger=True,
        exported_source_hash_checks=manifest_check,
        sdk_OK_time_s=[ok[0]['t'], ok[-1]['t']],
        shape_coherent_OK=len(coherent),
        shape_inconsistent_OK=len(inconsistent),
        shape_coherent_OK_time_s=[coherent[0]['t'], coherent[-1]['t']],
        shape_inconsistent_OK_time_s=[inconsistent[0]['t'], inconsistent[-1]['t']],
        joint_confidence=confidence_by_joint,
        image_to_nearest_sdk_frame_time_difference_s=summary(near),
        shape_coherent_pelvis_x_m=summary([r['pelvis'][0] for r in coherent]),
        shape_inconsistent_confidence=summary([r['confidence'] for r in inconsistent]),
        shape_inconsistent_left_shin_m=summary([r['lengths_m']['left_shin'] for r in inconsistent]),
        shape_inconsistent_left_ankle_height_m=summary(
            [r['left_ankle_height_m'] for r in inconsistent]
        ),
        sdk_OK_confidence=summary([r['confidence'] for r in ok]),
        body_abs_position_error_m=None,
        accuracy_note='No independent joint/position ground truth. Internal segment-length consistency is not accuracy.',
        coordinate_note='Frozen recording_002 candidate. No visual pose fitting, scale correction, translation snapping, or clock correction applied.',
    )
    pairs_summary = {}
    for key, subset in [
        ('shape_coherent', [r for r in paired if r['shape_ok']]),
        ('shape_inconsistent', [r for r in paired if not r['shape_ok']]),
    ]:
        pairs_summary[key] = dict(
            n=len(subset),
            dx_m=summary([r['dx_m'] for r in subset]),
            dy_m=summary([r['dy_m'] for r in subset]),
        )
    stats['pelvis_minus_selected_lidar'] = pairs_summary
    stats['pairing_note'] = (
        'Raw clocks; includes original tentative LiDAR observations; pelvis and cluster median differ. Not true position error.'
    )
    rules = dict(
        reference_time_s=[4.5, 10.5],
        reference_segment_m=ref,
        ratio_limits=[0.5, 1.8],
        note='Recording-specific review screen: compare SDK segment lengths with early same-person observations. Not anatomical truth or corrected skeleton.',
        segments={k: list(v) for k, v in SEGMENTS.items()},
    )
    for name, value in [
        ('metrics.json', stats),
        ('quality.json', dict(reference=rules, rows=rows)),
        ('lidar_pairs.json', paired),
    ]:
        with (out / name).open('x') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
    with (out / 'quality.csv').open('x', newline='') as stream:
        flat = [
            dict(
                frame_index=r['frame_index'],
                time_s=r['t'],
                body_id=r['id'],
                tracking_state=r['state'],
                confidence=r['confidence'],
                shape_ok=int(r['shape_ok']),
                shape_issues=';'.join(r['shape_issues']),
                pelvis_x_m=r['pelvis'][0],
                pelvis_y_m=r['pelvis'][1],
                pelvis_z_m=r['pelvis'][2],
                head_height_m=r['head_height_m'],
                left_ankle_height_m=r['left_ankle_height_m'],
                **{k + '_m': v for k, v in r['lengths_m'].items()},
            )
            for r in rows
        ]
        writer = csv.DictWriter(stream, fieldnames=list(flat[0]))
        writer.writeheader()
        writer.writerows(flat)
    fig, axs = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    tt = np.array([r['t'] for r in ok])
    cols = ['#168660' if r['shape_ok'] else '#ad4b91' for r in ok]
    axs[0].scatter(
        tt,
        [r['pelvis'][0] for r in ok],
        c=cols,
        s=8,
        label='SDK pelvis: green coherent / purple inconsistent',
    )
    for id in bundle['selected_ids']:
        trks = [
            (f['t'], r)
            for f in bundle['lidar']
            for r in f['tracks']
            if r['id'] == id and r['detected']
        ]
        axs[0].scatter(
            [t for t, r in trks],
            [r['x'] for t, r in trks],
            c='#e3a85e',
            s=5,
            label=f'LiDAR ID{id}: incl. tentative',
        )
    axs[0].set_ylabel('Corridor X (m)')
    axs[0].legend(fontsize=8)
    for name, col in [('torso', '#2477af'), ('left_thigh', '#d28233'), ('left_shin', '#a7478b')]:
        axs[1].plot(tt, [r['lengths_m'][name] for r in ok], color=col, label=name, lw=1)
    axs[1].set_ylabel('SDK segment length (m)')
    axs[1].legend(fontsize=8)
    axs[2].plot(tt, [r['head_height_m'] for r in ok], label='head height', lw=1)
    axs[2].plot(tt, [r['left_ankle_height_m'] for r in ok], label='left ankle height', lw=1)
    axs[2].axhline(0, color='gray', ls=':')
    axs[2].set_ylabel('Floor-relative Z (m)')
    axs[2].set_xlabel('ZED elapsed seconds')
    axs[2].legend(fontsize=8)
    for ax in axs:
        ax.grid(alpha=0.15)
        ax.axvline(inconsistent[0]['t'], color='#ad4b91', ls=':', lw=1)
    fig.suptitle('Recording 003: original SDK observations, no pose correction applied')
    fig.tight_layout()
    fig.savefig(out / 'quality_timeline.png', dpi=150)
    plt.close(fig)
    inputs = [args.body_dir / n for n in ['camera.json', 'frames.csv', 'bodies.csv']] + [
        args.bundle_dir / 'bundle.json',
        Path(__file__),
    ]
    with (out / 'analysis_manifest.json').open('x') as stream:
        json.dump(
            dict(
                body_dir=str(args.body_dir.resolve()),
                input_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
                originals_modified=False,
                live_devices_started=False,
            ),
            stream,
            indent=2,
        )
    print(json.dumps(stats, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
