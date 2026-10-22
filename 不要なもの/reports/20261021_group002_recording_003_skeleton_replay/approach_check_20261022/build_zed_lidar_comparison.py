#!/usr/bin/env python3
"""Build a far-to-near ZED-only / ZED-plus-LiDAR comparison replay.

This consumes the already exported, fresh-SDK replay and frozen recording-003
LiDAR analysis. It never reads or changes the source SVO2 or rosbag.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
import shutil
from collections import Counter
from pathlib import Path

import numpy as np


START_S = 17.0
END_S = 26.5
LIDAR_TRACK_ID = 60
PAIR_TOLERANCE_S = 0.065


def load_embedded_replay(path: Path) -> tuple[str, dict]:
    html = path.read_text(encoding="utf-8")
    marker = "const INITIAL="
    if marker not in html:
        raise ValueError(f"Cannot locate replay data in {path}")
    begin = html.index(marker) + len(marker)
    end = html.index(", B=INITIAL.bundle", begin)
    return html, json.loads(html[begin:end])


def camera_to_corridor(point: list[float], bundle: dict) -> np.ndarray | None:
    if len(point) < 3 or any(v is None or not math.isfinite(float(v)) for v in point[:3]):
        return None
    extrinsic = bundle["extrinsic"]
    raw = np.asarray(extrinsic["rotation_source_to_target"], dtype=float).T @ (
        np.asarray(point[:3], dtype=float) - np.asarray(extrinsic["translation_m"], dtype=float)
    )
    level = np.asarray(bundle["level_rotation"], dtype=float) @ raw
    level += np.asarray(bundle["level_translation"], dtype=float)
    corridor = bundle["corridor"]
    xy = level[:2] - np.asarray(corridor["origin_xy"], dtype=float)
    return np.asarray([
        np.dot(xy, corridor["longitudinal_xy"]),
        np.dot(xy, corridor["lateral_xy"]),
        level[2],
    ])


def summarize(source_data: dict, tracks_path: Path, sdk_comparison_path: Path) -> dict:
    bundle, body = source_data["bundle"], source_data["body"]
    origin_ns = int(bundle["origin_ns"])
    lidar = []
    with tracks_path.open(encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            if row.get("track_id") != str(LIDAR_TRACK_ID) or row.get("detected") != "1":
                continue
            timestamp = int(row["timestamp_ns"])
            t = (timestamp - origin_ns) / 1e9
            if START_S <= t <= END_S:
                lidar.append({
                    "timestamp_ns": timestamp,
                    "t": t,
                    "status": row["status"],
                    "position": np.asarray([
                        float(row["corridor_x"]), float(row["corridor_y"]),
                        float(row["corridor_z"]),
                    ]),
                    "point_count": int(float(row["point_count"])),
                })
    lidar.sort(key=lambda item: item["timestamp_ns"])
    if not lidar:
        raise ValueError(f"No detected LiDAR ID {LIDAR_TRACK_ID} samples in approach window")

    zed = []
    pelvis_i = bundle["camera"].get("keypoint_names", []).index("PELVIS") if "PELVIS" in bundle["camera"].get("keypoint_names", []) else 0
    for row in body["bodies"]:
        if row.get("state") != "OK" or not START_S <= float(row["t"]) <= END_S:
            continue
        position = camera_to_corridor(row["joints"][pelvis_i], bundle)
        if position is not None:
            zed.append({"t": float(row["t"]), "position": position})
    zed.sort(key=lambda item: item["t"])
    lidar_times = [item["t"] for item in lidar]

    pairs = []
    for item in zed:
        i = bisect.bisect_left(lidar_times, item["t"])
        choices = [j for j in (i - 1, i) if 0 <= j < len(lidar)]
        nearest = min((lidar[j] for j in choices), key=lambda value: abs(value["t"] - item["t"]))
        dt = abs(nearest["t"] - item["t"])
        if dt <= PAIR_TOLERANCE_S:
            delta = nearest["position"] - item["position"]
            pairs.append({
                "t": item["t"], "lidar_t": nearest["t"], "status": nearest["status"],
                "lidar_x": float(nearest["position"][0]),
                "dx": float(delta[0]), "dy": float(delta[1]),
                "horizontal_distance": float(np.linalg.norm(delta[:2])),
                "nearest_sample_delta_ms": dt * 1000,
            })

    def quantiles(values: list[float]) -> dict:
        if not values:
            return {"n": 0}
        return {
            "n": len(values),
            "median": float(np.median(values)),
            "p90": float(np.percentile(values, 90)),
            "max": float(np.max(values)),
        }

    lidar_status = Counter(item["status"] for item in lidar)
    distance_bands = []
    for label, predicate in [
        ("12m以上", lambda x: x >= 12),
        ("8〜12m", lambda x: 8 <= x < 12),
        ("4〜8m", lambda x: 4 <= x < 8),
        ("4m未満", lambda x: x < 4),
    ]:
        band = [item for item in pairs if predicate(item["lidar_x"])]
        distance_bands.append({
            "label": label,
            "paired_frames": len(band),
            "lidar_status_in_pairs": dict(Counter(item["status"] for item in band)),
            "horizontal_difference_m": quantiles([item["horizontal_distance"] for item in band]),
            "absolute_longitudinal_difference_m": quantiles([abs(item["dx"]) for item in band]),
        })

    sdk_comparison = json.loads(sdk_comparison_path.read_text(encoding="utf-8"))
    case = next(item for item in sdk_comparison["cases"] if item["id"] == "far_15m")
    zed_x = np.asarray([float(item["position"][0]) for item in zed])
    pair_zed_x = []
    pair_lidar_x = []
    for item in pairs:
        nearest_zed = min(zed, key=lambda value: abs(value["t"] - item["t"]))
        pair_zed_x.append(float(nearest_zed["position"][0]))
        pair_lidar_x.append(item["lidar_x"])

    return {
        "recording": "20261021_group002_recording_003",
        "scope": "接近のみ",
        "window_camera_elapsed_s": [START_S, END_S],
        "source_sdk_window_s": case["fresh_approach"]["camera_time_window_s"],
        "lidar_track_id": LIDAR_TRACK_ID,
        "lidar_unique_detected_samples": len(lidar),
        "lidar_unique_status_counts": dict(lidar_status),
        "lidar_x_corridor_m": [float(min(x["position"][0] for x in lidar)), float(max(x["position"][0] for x in lidar))],
        "zed_sdk": {
            "processed_frames": case["fresh_approach"]["processed_frames"],
            "sdk_ok_frames": case["fresh_approach"]["sdk_OK_frames"],
            "sdk_ok_fraction": case["fresh_approach"]["sdk_OK_fraction"],
            "shape_coherent_frames": case["fresh_approach"]["shape_coherent_OK_frames"],
            "shape_coherent_fraction": case["fresh_approach"]["shape_coherent_OK_fraction"],
            "both_legs_confidence25_frames": case["fresh_approach"]["both_legs_confidence25_frames"],
            "both_legs_confidence25_fraction": case["fresh_approach"]["both_legs_confidence25_fraction"],
            "pelvis_x_corridor_m": [float(np.min(zed_x)), float(np.max(zed_x))],
        },
        "comparison": {
            "pairing": "同じepoch由来の相対時刻で最も近いLiDAR ID 60の実観測を照合。補間なし。",
            "maximum_nearest_sample_delta_ms": PAIR_TOLERANCE_S * 1000,
            "paired_sdk_frames": len(pairs),
            "sdk_body_observations_in_window": len(zed),
            "nearest_sample_delta_ms": quantiles([item["nearest_sample_delta_ms"] for item in pairs]),
            "horizontal_pelvis_to_lidar_cluster_difference_m": quantiles([item["horizontal_distance"] for item in pairs]),
            "absolute_longitudinal_difference_m": quantiles([abs(item["dx"]) for item in pairs]),
            "longitudinal_pearson_r": float(np.corrcoef(pair_zed_x, pair_lidar_x)[0, 1]) if len(pairs) > 1 else None,
            "distance_bands": distance_bands,
        },
        "limits": [
            "LiDAR ID 60は人手で同一人物らしい軌跡として選定済み。観測の大部分は未確定候補で、confirmed実検出への昇格はしていない。",
            "骨盤と点群クラスタ中心は同じ解剖学的点ではない。絶対位置の独立真値がなく、この差は測位誤差や精度保証ではない。",
            "ZED骨格の廊下配置は部分的なLiDAR-ZED外部変換候補を使う。ZED単独モードはLiDAR観測を非表示にする比較であり、座標変換の前提まで独立ではない。",
            "時刻は元の絶対タイムスタンプから同一の録画原点に対する相対時刻へ変換した。専用の同期offset検定ではない。",
        ],
    }


def patch_template(html: str) -> str:
    def replace_once(old: str, new: str, expected: int = 1) -> None:
        nonlocal html
        count = html.count(old)
        if count != expected:
            raise ValueError(f"Expected {expected} occurrences of {old[:90]!r}, found {count}")
        html = html.replace(old, new)

    replace_once("<title>録画003｜骨格と廊下の3D再生</title>", "<title>録画003｜接近区間 ZED / LiDAR 比較</title>")
    replace_once("<h1>廊下内の歩行：骨格・位置・経路を再生</h1><p>2026-10-21 / group002 / recording003 · ZED BODY_34 / LiDAR</p>",
                 "<h1>奥から近づく区間：ZED SDK / LiDARを比較</h1><p>録画003 · 17.00〜26.50秒 · 往路・折返しを除外</p>")
    replace_once('<select id="rate"><option value="0.25">0.25倍</option><option value="0.5">0.5倍</option><option value="1" selected>1倍</option><option value="2">2倍</option></select><strong id="clock"></strong>',
                 '<select id="rate"><option value="0.25">0.25倍</option><option value="0.5">0.5倍</option><option value="1" selected>1倍</option><option value="2">2倍</option></select><label>比較表示 <select id="dataMode"><option value="zed">ZED SDKのみ</option><option value="zed_lidar" selected>ZED SDK + LiDAR経路</option></select></label><strong id="clock"></strong>')
    replace_once('<input id="slider" type="range" min="0" max="30" step="0.01" value="10">',
                 '<input id="slider" type="range" min="17" max="26.5" step="0.01" value="17">')
    replace_once('<p class="small">ドラッグで回転、ホイールで拡大・縮小。床と壁の線は校正候補による廊下の概略表示です。高さ方向のガイドは壁高の実測値ではありません。</p>',
                 '<p class="notice small" id="modeNote">比較表示を切り替えると、同じ時刻の同じZED骨格を保ったままLiDAR ID 60の軌跡・観測点だけを表示／非表示にします。上面図を初期表示します。</p><p class="small">ドラッグで回転、ホイールで拡大・縮小。床と壁の線は校正候補による廊下の概略表示です。高さ方向のガイドは壁高の実測値ではありません。</p>')
    replace_once('<p class="notice small">骨格はZED観測位置を廊下座標へ変換したものです。LiDARマーカーへ移動・吸着させません。時計差は未補正、外部変換は校正候補のため、双方の位置のずれが残ることがあります。</p>',
                 '<p class="notice small">ZED骨格を廊下座標へ置く変換は部分校正候補です。LiDARの表示はID 60の観測位置で、骨格へ吸着・融合しません。遠距離の点は未確定候補が中心です。時間・外部校正・身体中心の定義による位置差を精度とはみなしません。</p>')
    replace_once("let body=null, bodyGroups=[], edges=[], names=[], t=B.initial_time_s??10, playing=false, last=0, photoImage=new Image(), currentImage=-1;",
                 "let body=null, bodyGroups=[], edges=[], names=[], t=B.initial_time_s??17, playing=false, last=0, photoImage=new Image(), currentImage=-1;")
    replace_once("let orbit={yaw:-1.9,pitch:.16,distance:3.5},drag=null;",
                 "let orbit={yaw:-Math.PI/2,pitch:Math.PI/2-.001,distance:22},drag=null;")
    replace_once("const known=new Set(B.selected_ids), path=[];B.lidar.forEach(f=>f.tracks.filter(r=>known.has(r.id)&&r.detected).forEach(r=>path.push({...r,t:f.t})));",
                 "const known=new Set(B.selected_ids), path=[];B.lidar.forEach(f=>f.tracks.filter(r=>known.has(r.id)&&r.detected).forEach(r=>path.push({...r,t:f.t})));const showLidar=()=>$('dataMode').value==='zed_lidar';")
    replace_once("for(const r of path){if(!$('trajectory').checked&&r.t>t)continue;",
                 "for(const r of path){if(!showLidar())continue;if(!$('trajectory').checked&&r.t>t)continue;", expected=2)
    replace_once("if(Math.abs(f.t-t)<=.065)", "if(showLidar()&&Math.abs(f.t-t)<=.065)", expected=2)
    replace_once("if(Math.abs(lidar.t-t)<=.065)desc.push(", "if(showLidar()&&Math.abs(lidar.t-t)<=.065)desc.push(")
    replace_once("ctx.fillText(`LiDAR ${r.id}`,q[0]+8,q[1]+5)",
                 "ctx.fillText(`LiDAR ${r.id}`,q[0]+8,q[1]+25)")
    replace_once("if(pelvis){\n  const axis=[pelvis[0],pelvis[1]-.7,0],top=",
                 "if(pelvis&&orbit.pitch<1.4){\n  const axis=[pelvis[0],pelvis[1]-.7,0],top=")
    replace_once("t=Math.max(B.playback_start_s??0,Math.min(B.duration,t));$('slider').value=t;$('clock').textContent=`${t.toFixed(2)} / ${B.duration.toFixed(2)} 秒`;",
                 "t=Math.max(B.comparison_window.start_s,Math.min(B.comparison_window.end_s,t));$('slider').value=t;$('clock').textContent=`${t.toFixed(2)} 秒（接近区間）`;")
    replace_once("$('slider').max=B.duration;$('slider').oninput=e=>{t=Number(e.target.value);render()};",
                 "$('slider').min=B.comparison_window.start_s;$('slider').max=B.comparison_window.end_s;$('dataMode').onchange=()=>{const u=new URL(location.href);u.searchParams.set('mode',$('dataMode').value);history.replaceState(null,'',u);render()};$('slider').oninput=e=>{t=Number(e.target.value);render()};")
    replace_once("if($('loop').checked)t=B.playback_start_s??0;else{t=B.duration;playing=false;",
                 "if($('loop').checked)t=B.comparison_window.start_s;else{t=B.comparison_window.end_s;playing=false;")
    replace_once("$('overview').onclick=()=>{$('follow').checked=false;orbit={yaw:-2.05,pitch:.52,distance:25};render()};",
                 "$('overview').onclick=()=>{$('follow').checked=false;orbit={yaw:-Math.PI/2,pitch:Math.PI/2-.001,distance:22};render()};")
    replace_once("const start=new URLSearchParams(location.search).get('t');if(start!==null)t=Number(start)||0;render();requestAnimationFrame(tick);",
                 "const query=new URLSearchParams(location.search),start=query.get('t'),mode=query.get('mode');if(start!==null)t=Number(start)||START_S;if(mode==='zed'||mode==='zed_lidar')$('dataMode').value=mode;render();requestAnimationFrame(tick);")
    # The constant is inserted next to the other playback configuration.
    replace_once("const INITIAL=", "const START_S=17;\nconst INITIAL=")
    return html


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-replay", type=Path, required=True)
    parser.add_argument("--tracks-csv", type=Path, required=True)
    parser.add_argument("--sdk-comparison", type=Path, required=True)
    parser.add_argument("--assets", type=Path, required=True, help="shared recording-003 skeleton assets folder")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    out = args.output_dir.resolve()
    if out.exists():
        parser.error(f"Output folder already exists; choose a new folder: {out}")
    html, data = load_embedded_replay(args.source_replay.resolve())
    bundle = data["bundle"]
    bundle["selected_ids"] = [LIDAR_TRACK_ID]
    bundle["duration"] = END_S
    bundle["comparison_window"] = {"start_s": START_S, "end_s": END_S}
    bundle["initial_time_s"] = 17.3
    bundle["playback_start_s"] = START_S
    bundle["lidar"] = [
        {**frame, "tracks": [row for row in frame["tracks"] if row["id"] == LIDAR_TRACK_ID and row["detected"]]}
        for frame in bundle["lidar"] if START_S <= float(frame["t"]) <= END_S
    ]
    bundle["readme_path"] = "README.md"
    bundle["asset_base"] = Path(__import__("os").path.relpath(args.assets.resolve(), out)).as_posix()

    patched = patch_template(html)
    data_start = patched.index("const INITIAL=") + len("const INITIAL=")
    data_end = patched.index(", B=INITIAL.bundle", data_start)
    # patch_template adds START_S before the embedded replay constant.
    compact = json.dumps(data, ensure_ascii=False, allow_nan=False, separators=(",", ":")).replace("</", "<\\/")
    patched = patched[:data_start] + compact + patched[data_end:]
    metrics = summarize(data, args.tracks_csv.resolve(), args.sdk_comparison.resolve())
    manifest_sources = [args.source_replay.resolve(), args.tracks_csv.resolve(), args.sdk_comparison.resolve(), Path(__file__).resolve()]
    metrics["input_sha256"] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in manifest_sources}

    out.mkdir(parents=True)
    (out / "replay_approach.html").write_text(patched, encoding="utf-8")
    (out / "comparison_metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    shutil.copy2(Path(__file__).resolve(), out / "build_zed_lidar_comparison.py")
    (out / "README.md").write_text(readme_text(metrics), encoding="utf-8")
    (out / "index.html").write_text(index_text(metrics), encoding="utf-8")
    print(f"Created {out}")


def readme_text(metrics: dict) -> str:
    c = metrics["comparison"]
    return f"""# 録画003：接近区間のZED / LiDAR比較

17.00〜26.50秒の「画面奥から近づく」区間だけを再生します。往路や折返しは含みません。

## 表示

`index.html` を開き、「ZED SDKのみ」と「ZED SDK + LiDAR経路」を切り替えます。骨格・時刻・映像は同じで、切り替わるのは廊下の3D表示と上面図のLiDAR ID 60の観測・経路です。初期視点は上面図です。映像パネルにはZED骨格が重なります。

## この区間で確認できたこと

- ZED SDKは {metrics['zed_sdk']['processed_frames']}フレーム中 {metrics['zed_sdk']['sdk_ok_frames']}フレームで骨格を出力し、固定した骨長スクリーニングは {metrics['zed_sdk']['shape_coherent_frames']}フレームで通過しました。
- LiDAR ID 60は {metrics['lidar_unique_detected_samples']}観測（確定 {metrics['lidar_unique_status_counts'].get('confirmed', 0)}、未確定候補 {metrics['lidar_unique_status_counts'].get('tentative', 0)}）。候補は確定検出として数えていません。
- ZED骨盤とLiDARクラスタ中心の水平距離差は、時刻の近い {c['paired_sdk_frames']}フレームの比較で中央値 {c['horizontal_pelvis_to_lidar_cluster_difference_m']['median']:.2f} m、90百分位 {c['horizontal_pelvis_to_lidar_cluster_difference_m']['p90']:.2f} m。奥行き位置の相関は r={c['longitudinal_pearson_r']:.3f} でした。

この距離差は精度や位置誤差ではありません。比較対象は骨盤とLiDAR点群クラスタ中心で一致する物理点ではなく、遠距離のLiDAR観測は主に未確定候補です。ZED骨格の廊下配置にも部分校正候補を使い、独立測量真値がありません。詳しい条件と距離帯別の数値は `comparison_metrics.json` を参照してください。

実機、SVO2、rosbagは起動・変更していません。画像は親の `assets/` を参照します。

`browser_check/` にZED単独表示とLiDAR重畳表示の画面確認画像を保存しています。
"""


def index_text(metrics: dict) -> str:
    c = metrics["comparison"]
    confirmed = metrics["lidar_unique_status_counts"].get("confirmed", 0)
    tentative = metrics["lidar_unique_status_counts"].get("tentative", 0)
    bands = "".join(
        f"<tr><td>{band['label']}</td><td>{band['paired_frames']}</td><td>{band['horizontal_difference_m'].get('median', float('nan')):.2f} m</td><td>{band['horizontal_difference_m'].get('p90', float('nan')):.2f} m</td></tr>"
        for band in c["distance_bands"] if band["paired_frames"]
    )
    return f"""<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>録画003 接近区間 ZED / LiDAR比較</title>
<style>body{{margin:0;background:#edf2f6;color:#172e41;font:16px system-ui,sans-serif}}main{{max-width:1000px;margin:32px auto;padding:0 18px}}section{{background:white;border:1px solid #d6e1e9;border-radius:12px;padding:22px;margin:18px 0}}h1{{font-size:26px}}h2{{font-size:19px}}p{{line-height:1.7}}a.button{{display:inline-block;background:#155b90;color:white;padding:11px 16px;margin:6px;border-radius:7px;text-decoration:none}}table{{border-collapse:collapse;width:100%}}th,td{{padding:9px;border-bottom:1px solid #d6e1e9;text-align:right}}th:first-child,td:first-child{{text-align:left}}.note{{background:#fff1d4;padding:12px;border-left:4px solid #d09327}}small{{color:#526c7f}}</style>
<main><h1>奥から近づく区間：ZED SDK / LiDAR比較</h1><p>録画003の17.00〜26.50秒だけを、同じZED骨格・同じ時刻で切り替えて確認できます。</p>
<section><h2>再生を開く</h2><a class="button" href="replay_approach.html?mode=zed">ZED SDKのみ</a><a class="button" href="replay_approach.html?mode=zed_lidar">ZED SDK + LiDAR経路</a><p>プレイヤー上部の「比較表示」メニューでも切り替えられます。初期視点は上面図です。</p></section>
<section><h2>区間の結果</h2><p>ZED骨格は {metrics['zed_sdk']['sdk_ok_frames']}/{metrics['zed_sdk']['processed_frames']} フレームで出力。LiDAR ID 60は {metrics['lidar_unique_detected_samples']} 観測（確定 {confirmed}、未確定候補 {tentative}）でした。両方の時刻が近い {c['paired_sdk_frames']} フレームについて、ZED骨盤とLiDARクラスタ中心の廊下水平距離差は中央値 <b>{c['horizontal_pelvis_to_lidar_cluster_difference_m']['median']:.2f} m</b>、90百分位 <b>{c['horizontal_pelvis_to_lidar_cluster_difference_m']['p90']:.2f} m</b>。奥行き方向は r={c['longitudinal_pearson_r']:.3f} で一緒に近づく傾向でした。</p><table><thead><tr><th>LiDAR奥行き帯</th><th>対応フレーム</th><th>水平距離差 中央値</th><th>90百分位</th></tr></thead><tbody>{bands}</tbody></table></section>
<section class="note"><b>精度の読み方:</b> 距離差は独立真値に対する誤差ではありません。骨盤と点群クラスタ中心の差、部分的な外部校正、未確定LiDAR候補、同期の不確かさを含みます。遠距離ではLiDAR ID 60が未確定候補のため、補助的な経路比較です。詳細は <a href="README.md">README</a> と <a href="comparison_metrics.json">数値データ</a> を参照。</section></main></html>"""


if __name__ == "__main__":
    main()
