#!/usr/bin/env python3
"""廊下座標の追跡CSVを集計し、独立した参照ラベルがある場合に精度を評価する。

参照CSV: timestamp_ns,person_id,x_m,y_m[,vx_mps,vy_mps]。
各時刻には評価範囲の全人物を記し、person_id空欄は明示的な無人場面を表す。
位置の真値には測量等による廊下座標のm値を使う。画像クリックだけをmの真値とはしない。
時刻間の補間は行わず、近傍フレームがない場合は未検出として数える。
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import html
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np

from assignment import gated_assignment
from experiment_plan import canonical_plan_hash


def read_csv(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def number(value):
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def observed(row):
    """人物として数える確定実観測（confirmedかつdetected=1）かを判定する。"""

    return row.get("status") == "confirmed" and str(row.get("detected")) == "1"


def load_tracks(path):
    """時刻・ID重複と確定行の廊下座標を検証して追跡CSVを読む。"""

    rows = read_csv(path)
    seen = set()
    for row in rows:
        row["timestamp_ns"] = int(row["timestamp_ns"])
        key = (row["timestamp_ns"], row["track_id"])
        if key in seen:
            raise ValueError(f"Duplicate timestamp/track ID: {key}")
        seen.add(key)
        for field in ("corridor_x", "corridor_y", "corridor_vx", "corridor_vy", "confidence"):
            row[field] = number(row.get(field))
        if observed(row) and any(row[k] is None for k in ("corridor_x", "corridor_y")):
            raise ValueError("Confirmed detections require finite corridor coordinates.")
    return rows


def evaluate_reference(rows, frame_stamps, labels, gate_m, tolerance_s, switch_gap_s):
    """参照フレームごとに有効対応数最大の割当を求め、未記録時刻も未検出に数える。"""

    grouped, reference = defaultdict(list), defaultdict(list)
    for row in rows:
        if observed(row):
            grouped[row["timestamp_ns"]].append(row)
    seen = set()
    for label in labels:
        stamp = int(label["timestamp_ns"])
        reference[stamp]  # 無人と明示された時刻も評価対象に残す。
        pid = label["person_id"].strip()
        if not pid:
            continue
        key = (stamp, pid)
        if key in seen:
            raise ValueError(f"Duplicate reference timestamp/person: {key}")
        seen.add(key)
        xy = [number(label.get(k)) for k in ("x_m", "y_m")]
        if any(v is None for v in xy):
            raise ValueError("Reference labels require finite independent metric positions.")
        reference[stamp].append(dict(person_id=pid, xy=xy, source=label))
    stamps = np.array(sorted(set(frame_stamps)), dtype=np.int64)
    matched, missed, false_positive, switches = 0, 0, 0, 0
    errors, velocity_errors, details = [], [], []
    previous = {}
    used_source_stamps = set()
    for stamp, truth in sorted(reference.items()):
        index = int(np.searchsorted(stamps, stamp))
        choices = [i for i in (index - 1, index) if 0 <= i < len(stamps)]
        nearest = min(choices, key=lambda i: abs(int(stamps[i]) - stamp)) if choices else None
        source_stamp = None if nearest is None else int(stamps[nearest])
        if source_stamp is None or abs(source_stamp - stamp) / 1e9 > tolerance_s:
            predictions = []
        else:
            if source_stamp in used_source_stamps:
                raise ValueError(
                    "Multiple reference frames map to one sensor frame; reduce label frequency/tolerance."
                )
            used_source_stamps.add(source_stamp)
            predictions = grouped[source_stamp]
        a = np.asarray([g["xy"] for g in truth], float).reshape(-1, 2)
        b = np.asarray([[r["corridor_x"], r["corridor_y"]] for r in predictions], float).reshape(
            -1, 2
        )
        distance = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
        # 距離ゲート内の1対1対応を求め、未対応の真値は見逃し、予測側は偽陽性に数える。
        pairs = gated_assignment(distance**2, distance <= gate_m)
        matched += len(pairs)
        missed += len(truth) - len(pairs)
        false_positive += len(predictions) - len(pairs)
        pair_lookup = dict(pairs)
        for i, g in enumerate(truth):
            p = predictions[pair_lookup[i]] if i in pair_lookup else None
            error = None if p is None else float(distance[i, pair_lookup[i]])
            switch = 0
            if p is not None:
                errors.append(error)
                old = previous.get(g["person_id"])
                if old and (stamp - old[0]) / 1e9 <= switch_gap_s and old[1] != p["track_id"]:
                    switches += 1
                    switch = 1
                previous[g["person_id"]] = stamp, p["track_id"]
                actual = [number(g["source"].get(k)) for k in ("vx_mps", "vy_mps")]
                measured = [p[k] for k in ("corridor_vx", "corridor_vy")]
                if all(v is not None for v in actual + measured):
                    velocity_errors.append(float(np.linalg.norm(np.asarray(actual) - measured)))
            details.append(
                dict(
                    timestamp_ns=stamp,
                    source_timestamp_ns=source_stamp,
                    person_id=g["person_id"],
                    track_id="" if p is None else p["track_id"],
                    matched=int(p is not None),
                    position_error_m=error,
                    id_switch=switch,
                )
            )
    total = matched + missed
    return (
        dict(
            evaluated_reference_frames=len(reference),
            reference_person_samples=total,
            matched_samples=matched,
            missed_samples=missed,
            false_positive_samples=false_positive,
            recall=matched / total if total else None,
            missing_rate=missed / total if total else None,
            precision=matched / (matched + false_positive) if matched + false_positive else None,
            id_switches=switches,
            id_switch_max_gap_s=switch_gap_s,
            position_mae_m=float(np.mean(errors)) if errors else None,
            position_rmse_m=float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
            velocity_rmse_mps=(
                float(np.sqrt(np.mean(np.square(velocity_errors)))) if velocity_errors else None
            ),
            velocity_reference_samples=len(velocity_errors),
            note="Errors conditional on matched samples; always interpret with recall and gate. Sample recall is not time-weighted success. Predicted/tentative tracks excluded.",
        ),
        details,
    )


def quality_summary(rows, frame_stamps):
    """真値なしで分かる確定観測数・ID数・観測間隔を記述的に集計する。"""

    good = [r for r in rows if observed(r)]
    by_id = defaultdict(list)
    for r in good:
        by_id[r["track_id"]].append(r)
    tracks = []
    for tid, sequence in sorted(by_id.items()):
        sequence.sort(key=lambda r: r["timestamp_ns"])
        times = np.array([r["timestamp_ns"] for r in sequence], np.int64)
        gaps = np.diff(times) / 1e9
        tracks.append(
            dict(
                track_id=tid,
                confirmed_observations=len(sequence),
                first_timestamp_ns=int(times[0]),
                last_timestamp_ns=int(times[-1]),
                gaps_over_0_25s=int(np.sum(gaps > 0.25)),
                max_observed_gap_s=float(max(gaps)) if len(gaps) else None,
                lateral_range_m=float(np.ptp([r["corridor_y"] for r in sequence])),
            )
        )
    return dict(
        all_frames=len(set(frame_stamps)),
        all_candidate_ids=len({r["track_id"] for r in rows}),
        confirmed_ids=len(by_id),
        confirmed_observations=len(good),
        frames_with_confirmed_detections=len({r["timestamp_ns"] for r in good}),
        predicted_rows=sum(r["status"] == "predicted" for r in rows),
        tracks=tracks,
        accuracy={
            "position_rmse_m": None,
            "id_switches": None,
            "missing_rate": None,
            "note": "Unevaluated without independent reference labels; IDs/gaps are descriptive only.",
        },
    )


def pair_metrics(rows, roles):
    """同時に確定検出された2人の前後差・横差・距離・相対速度を求める。"""

    by_time, baseline = defaultdict(list), {}
    for row in sorted(rows, key=lambda r: r["timestamp_ns"]):
        if observed(row):
            by_time[row["timestamp_ns"]].append(row)
            baseline.setdefault(row["track_id"], row["corridor_y"])
    pairs = []
    for stamp, people in sorted(by_time.items()):
        people.sort(key=lambda r: r["track_id"])
        for i, first in enumerate(people):
            for second in people[i + 1 :]:
                dx = second["corridor_x"] - first["corridor_x"]
                dy = second["corridor_y"] - first["corridor_y"]
                velocities = [r[k] for r in (first, second) for k in ("corridor_vx", "corridor_vy")]
                dvx = (
                    second["corridor_vx"] - first["corridor_vx"]
                    if all(v is not None for v in velocities)
                    else None
                )
                dvy = (
                    second["corridor_vy"] - first["corridor_vy"]
                    if all(v is not None for v in velocities)
                    else None
                )
                pairs.append(
                    dict(
                        timestamp_ns=stamp,
                        first_id=first["track_id"],
                        second_id=second["track_id"],
                        first_role=roles.get(first["track_id"], {}).get("role", ""),
                        second_role=roles.get(second["track_id"], {}).get("role", ""),
                        dx_m=dx,
                        dy_m=dy,
                        distance_m=math.hypot(dx, dy),
                        relative_vx_mps=dvx,
                        relative_vy_mps=dvy,
                        first_lateral_from_first_observation_m=first["corridor_y"]
                        - baseline[first["track_id"]],
                        second_lateral_from_first_observation_m=second["corridor_y"]
                        - baseline[second["track_id"]],
                    )
                )
    return pairs


def resolve_roles(mapping_path, recording_path, plan_path):
    """人手の参加者とIDの対応を検証し、役割はrun_order.jsonから取得する。"""

    if mapping_path is None:
        return {}
    if recording_path is None or plan_path is None:
        raise ValueError("--roles requires --recording-json and --run-order.")
    annotation = json.loads(mapping_path.read_text())
    recording = json.loads(recording_path.read_text())
    plan = json.loads(plan_path.read_text())
    if (
        plan.get("plan_sha256") != canonical_plan_hash(plan)
        or recording.get("plan_sha256") != plan["plan_sha256"]
    ):
        raise ValueError("Recording and validated run_order plan hashes must agree.")
    if annotation.get("recording") != str(recording_path.parent.resolve()):
        raise ValueError("Participant annotation must explicitly identify the recording directory.")
    if annotation.get("actual_behavior_verified") is not True:
        raise ValueError("Verify actual behavior against the planned trial before assigning roles.")
    group = plan["groups"][f"G{int(recording['group_number']):02d}"]
    trial = next(
        t for t in group["trials"] if t["global_trial_number"] == recording["global_trial_number"]
    )
    result = {}
    for track, participant in annotation["track_participants"].items():
        if participant not in ("subject_1", "subject_2"):
            raise ValueError("Participant keys must be subject_1 or subject_2.")
        result[str(track)] = {"participant_id": participant, "role": trial[participant]["role"]}
    return result


def kinematics(rows, max_gap_s=0.25):
    """連続した確定実観測3点を実時刻で微分し、速度と加速度を求める。

    平滑化・穴埋めは行わないため、微分は測定雑音を増幅する。
    端点・予測を含む窓・大きな欠落の微分は空欄を保ち、0で埋めない。
    """

    grouped = defaultdict(list)
    for row in rows:
        if row["status"] in ("confirmed", "predicted"):
            grouped[row["track_id"]].append(row)
    output = []
    for tid, seq in grouped.items():
        seq.sort(key=lambda r: r["timestamp_ns"])
        for i, row in enumerate(seq):
            result = dict(
                timestamp_ns=row["timestamp_ns"],
                track_id=tid,
                status=row["status"],
                x_m=row["corridor_x"],
                y_m=row["corridor_y"],
                raw_derivative_vx_mps=None,
                raw_derivative_vy_mps=None,
                raw_derivative_ax_mps2=None,
                raw_derivative_ay_mps2=None,
                derivative_valid=0,
            )
            if 0 < i < len(seq) - 1:
                triple = seq[i - 1 : i + 2]
                dt1 = (row["timestamp_ns"] - triple[0]["timestamp_ns"]) / 1e9
                dt2 = (triple[2]["timestamp_ns"] - row["timestamp_ns"]) / 1e9
                if (
                    all(observed(r) for r in triple)
                    and 0 < dt1 <= max_gap_s
                    and 0 < dt2 <= max_gap_s
                ):
                    p = np.array([[r["corridor_x"], r["corridor_y"]] for r in triple])
                    velocity = np.gradient(p, [-dt1, 0, dt2], axis=0)[1]
                    acceleration = 2 * ((p[2] - p[1]) / dt2 - (p[1] - p[0]) / dt1) / (dt1 + dt2)
                    result.update(
                        raw_derivative_vx_mps=float(velocity[0]),
                        raw_derivative_vy_mps=float(velocity[1]),
                        raw_derivative_ax_mps2=float(acceleration[0]),
                        raw_derivative_ay_mps2=float(acceleration[1]),
                        derivative_valid=1,
                    )
            output.append(result)
    return sorted(output, key=lambda r: (r["timestamp_ns"], r["track_id"]))


def write_csv(path, rows, fields=None):
    fields = fields or (list(rows[0]) if rows else [])
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_dashboard(path, rows, frames, summary):
    """集計結果と参照評価の有無を、確認用HTMLにまとめる。"""

    start = min(frames) if frames else 0
    data = [
        dict(
            t=(r["timestamp_ns"] - start) / 1e9,
            id=r["track_id"],
            x=r["corridor_x"],
            y=r["corridor_y"],
            vx=r["corridor_vx"],
            vy=r["corridor_vy"],
            c=r["confidence"],
            status=r["status"],
            detected=r["detected"],
        )
        for r in rows
        if r["status"] in ("confirmed", "predicted")
    ]
    payload = json.dumps(
        dict(rows=data, times=[(t - start) / 1e9 for t in sorted(set(frames))]), allow_nan=False
    ).replace("<", "\\u003c")
    document = """<!doctype html><html lang="ja"><meta charset="utf-8">
<title>追跡品質レビュー</title><style>
body{font:16px system-ui;margin:24px;color:#172b3a}canvas{border:1px solid #bbb;width:100%;max-width:1100px}
input{width:80%}pre{white-space:pre-wrap;background:#f3f5f7;padding:12px}
</style><h1>廊下座標の追跡品質</h1>
<p>実線は確定検出。赤い×は予測のみ、薄い点は低い点数信頼度。0.25秒超の欠測は線で接続しません。
信頼度は点数の指標で、精度保証ではありません。距離はクラスタ代表点間であり身体表面間ではありません。</p>
<button id="play">再生 / 停止</button><input id="time" type="range" min="0" value="0" step="1"><span id="label"></span>
<h2>平面軌跡・現在のID（X: 廊下方向 m、Y: 横方向 m、等縮尺）</h2><canvas id="path" width="1100" height="400"></canvas>
<h2>横位置 y [m] ／ 時刻 [s]</h2><canvas id="lateral" width="1100" height="220"></canvas>
<h2>水平速度 [m/s] ／ 時刻 [s]</h2><canvas id="speed" width="1100" height="220"></canvas>
<h2>現在の検出と品質</h2><pre id="current"></pre><h2>集計</h2><pre>__SUMMARY__</pre>
<script>const data=__DATA__,colors=['#1678aa','#bb6514','#8061b7','#25834f'];
const slider=document.getElementById('time');slider.max=Math.max(0,data.times.length-1);
const ids=[...new Set(data.rows.map(r=>r.id))], color=id=>colors[ids.indexOf(id)%colors.length];
const finite=v=>v!==null&&Number.isFinite(v);
function draw(){
const t=data.times[+slider.value]??0;document.getElementById('label').textContent=t.toFixed(2)+' s';
const current=data.rows.filter(r=>Math.abs(r.t-t)<1e-7);
document.getElementById('current').textContent=current.length?current.map(r=>`ID ${r.id}: ${r.status}, x=${r.x?.toFixed(3)}, y=${r.y?.toFixed(3)}, confidence=${r.c?.toFixed(2)}`).join('\\n'):'検出なし（位置を補間していません）';
for(const name of ['path','lateral','speed']){
const c=document.getElementById(name),ctx=c.getContext('2d');ctx.clearRect(0,0,c.width,c.height);
const value=r=>name==='speed'?(finite(r.vx)&&finite(r.vy)?Math.hypot(r.vx,r.vy):null):r.y;
const valid=data.rows.filter(r=>finite(r.x)&&finite(value(r)));
if(!valid.length){ctx.fillText('有効値なし',30,30);continue;}
let xs=valid.map(r=>name==='path'?r.x:r.t),ys=valid.map(value);
let xmin=Math.min(...xs),xmax=Math.max(...xs),ymin=Math.min(...ys),ymax=Math.max(...ys);
if(xmax-xmin<.01){xmin-=.5;xmax+=.5;}if(ymax-ymin<.01){ymin-=.5;ymax+=.5;}
if(name==='path'){let scale=Math.max((xmax-xmin)/1000,(ymax-ymin)/(c.height-70));let xm=(xmin+xmax)/2,ym=(ymin+ymax)/2;xmin=xm-scale*500;xmax=xm+scale*500;ymin=ym-scale*(c.height-70)/2;ymax=ym+scale*(c.height-70)/2;}
const px=x=>65+(x-xmin)/(xmax-xmin)*1000,py=y=>25+(ymax-y)/(ymax-ymin)*(c.height-70);
ctx.font='13px sans-serif';ctx.fillStyle='#333';
for(let i=0;i<=4;i++){let x=xmin+(xmax-xmin)*i/4,y=ymin+(ymax-ymin)*i/4;ctx.fillText(x.toFixed(2),px(x)-10,c.height-15);ctx.fillText(y.toFixed(2),5,py(y));ctx.strokeStyle='#ddd';ctx.beginPath();ctx.moveTo(65,py(y));ctx.lineTo(1065,py(y));ctx.stroke();}
for(const id of ids){let last=null;ctx.strokeStyle=color(id);ctx.fillStyle=color(id);ctx.lineWidth=1.5;
for(const r of data.rows.filter(r=>r.id===id).sort((a,b)=>a.t-b.t)){if(!finite(r.x)||!finite(value(r))){last=null;continue;}
let x=px(name==='path'?r.x:r.t),y=py(value(r)),ok=r.status==='confirmed'&&r.detected==='1';
ctx.globalAlpha=(r.c??0)<.5?.35:1;
if(ok){if(last&&r.t-last.t<=.25){ctx.beginPath();ctx.moveTo(last.x,last.y);ctx.lineTo(x,y);ctx.stroke();}ctx.beginPath();ctx.arc(x,y,2,0,7);ctx.fill();last={t:r.t,x,y};}
else{ctx.strokeStyle='#c33';ctx.beginPath();ctx.moveTo(x-3,y-3);ctx.lineTo(x+3,y+3);ctx.moveTo(x-3,y+3);ctx.lineTo(x+3,y-3);ctx.stroke();ctx.strokeStyle=color(id);last=null;}
if(name==='path'&&Math.abs(r.t-t)<1e-7){ctx.globalAlpha=1;ctx.beginPath();ctx.arc(x,y,6,0,7);ctx.stroke();ctx.fillText('ID '+id,x+8,y-8);}
}ctx.globalAlpha=1;}
if(name!=='path'){ctx.strokeStyle='#111';ctx.beginPath();ctx.moveTo(px(t),20);ctx.lineTo(px(t),c.height-40);ctx.stroke();}
}}
slider.oninput=draw;let timer=null;document.getElementById('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;}else timer=setInterval(()=>{slider.value=(+slider.value+1)%Math.max(1,data.times.length);draw();},100);};draw();
</script></html>"""
    path.write_text(
        document.replace(
            "__SUMMARY__", html.escape(json.dumps(summary, ensure_ascii=False, indent=2))
        ).replace("__DATA__", payload),
        encoding="utf-8",
    )


def main():
    """追跡集計・2人の相対量・任意の真値評価を行い、新規出力先へ保存する。"""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tracks", type=Path, required=True)
    parser.add_argument(
        "--frames",
        type=Path,
        required=True,
        help="All processed frames including no-detection frames",
    )
    parser.add_argument("--labels", type=Path)
    parser.add_argument(
        "--reference-note",
        help="Independent reference method, installation/axes, and annotation split",
    )
    parser.add_argument(
        "--roles",
        type=Path,
        help="Reviewed recording-specific track_participants JSON; roles are read from run_order",
    )
    parser.add_argument("--recording-json", type=Path)
    parser.add_argument("--run-order", type=Path)
    parser.add_argument("--gate-m", type=float, default=0.75)
    parser.add_argument("--time-tolerance-s", type=float, default=0.049)
    parser.add_argument("--id-switch-max-gap-s", type=float, default=1.0)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.labels and not args.reference_note:
        parser.error("--labels requires --reference-note (no estimated sensor output as truth)")
    if any(
        not math.isfinite(v) or v <= 0
        for v in (args.gate_m, args.time_tolerance_s, args.id_switch_max_gap_s)
    ):
        parser.error("Thresholds must be finite and positive.")
    if args.output_dir.exists():
        parser.error("Choose a new output directory.")
    rows = load_tracks(args.tracks)
    frame_rows = read_csv(args.frames)
    frames = [int(r["timestamp_ns"]) for r in frame_rows]
    if len(set(frames)) != len(frames) or any(b <= a for a, b in zip(frames, frames[1:])):
        parser.error("Frame timestamps must be unique and increasing.")
    if not set(r["timestamp_ns"] for r in rows).issubset(set(frames)):
        parser.error("Tracks contain timestamps absent from --frames.")
    roles = resolve_roles(args.roles, args.recording_json, args.run_order)
    summary = quality_summary(rows, frames)
    summary["reference_note"] = args.reference_note
    summary["parameters"] = {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}
    summary["source_sha256"] = {
        str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (
            args.tracks,
            args.frames,
            args.labels,
            args.roles,
            args.recording_json,
            args.run_order,
            Path(__file__),
        )
        if p is not None
    }
    details = []
    if args.labels:
        summary["accuracy"], details = evaluate_reference(
            rows,
            frames,
            read_csv(args.labels),
            args.gate_m,
            args.time_tolerance_s,
            args.id_switch_max_gap_s,
        )
    pairs = pair_metrics(rows, roles)
    summary["minimum_observed_pair_distance_m"] = min(
        (r["distance_m"] for r in pairs), default=None
    )
    summary["pair_note"] = (
        "Simultaneously confirmed cluster representatives only; no interpolation. Minimum across observed samples is not the continuous-time closest approach or body clearance."
    )
    summary["roles"] = roles
    summary["derivative_note"] = (
        "kinematics.csv contains unsmoothed raw three-point derivatives on contiguous detections only. Acceleration amplifies noise; uncertainty and behavioral onset are not validated."
    )
    args.output_dir.mkdir(parents=True)
    (args.output_dir / "metrics.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    )
    write_csv(
        args.output_dir / "pairs.csv",
        pairs,
        fields=None if pairs else ["timestamp_ns", "first_id", "second_id", "distance_m"],
    )
    write_csv(
        args.output_dir / "kinematics.csv",
        kinematics(rows),
        fields=(
            None
            if any(r["status"] in ("confirmed", "predicted") for r in rows)
            else ["timestamp_ns", "track_id", "derivative_valid"]
        ),
    )
    if args.labels:
        write_csv(args.output_dir / "reference_matches.csv", details)
    write_dashboard(args.output_dir / "index.html", rows, frames, summary)
    # 真値テンプレートは空のまま保存し、推測した人物IDや位置は記入しない。
    write_csv(
        args.output_dir / "reference_template.csv",
        [],
        ["timestamp_ns", "person_id", "x_m", "y_m", "vx_mps", "vy_mps"],
    )
    print(args.output_dir / "index.html")


if __name__ == "__main__":
    main()
