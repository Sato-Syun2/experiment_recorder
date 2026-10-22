#!/usr/bin/env python3
"""Compare fresh approaching-only SDK runs with the saved full-sequence result.

Standard library only. Shape consistency and SDK availability are screening
measures, not accuracy against independent joint/position ground truth.
"""
from pathlib import Path
import argparse
import collections
import hashlib
import html
import json
import math
import statistics

from build_walk_skeleton_replay import load_export


def vector(value):
    return isinstance(value,list) and len(value)>=3 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in value[:3])


def segment_length(points,a,b):
    if len(points)<=max(a,b) or not vector(points[a]) or not vector(points[b]):
        return None
    return math.dist(points[a][:3],points[b][:3])


def review_metrics(data,window,reference):
    first=int(window['expected_image_timestamps_ns'][0]);origin=int(window['source_origin_ns'])
    start=(first-origin)/1e9
    end=(int(window['expected_image_timestamps_ns'][-1])-origin)/1e9
    frames=[f for f in data['frames'] if start-1e-7<=f['t']<=end+1e-7]
    rows=[r for r in data['bodies'] if start-1e-7<=r['t']<=end+1e-7]
    expected=len(window['expected_image_timestamps_ns'])
    if len(frames)!=expected:
        raise ValueError('Comparison needs every declared SDK frame, including no-body frames.')
    ok=[r for r in rows if r['state']=='OK']
    coherent=[];lengths={k:[] for k in reference['segments']}
    for r in ok:
        valid=True
        for name,(a,b) in reference['segments'].items():
            length=segment_length(r['joints'],a,b)
            if length is not None:lengths[name].append(length)
            ratio=length/reference['reference_segment_m'][name] if length is not None else None
            if ratio is None or not .5<=ratio<=1.8:
                valid=False
        if valid:coherent.append(r)
    key=lambda t:round(t,7)
    ok_times={key(r['t']) for r in ok};coherent_times={key(r['t']) for r in coherent}
    legs=[r for r in coherent if all(i<len(r['jointConfidence']) and isinstance(r['jointConfidence'][i],(int,float)) and r['jointConfidence'][i]>=25
                                    and vector(r['joints'][i]) for i in [18,19,20,22,23,24])]
    leg_times={key(r['t']) for r in legs}
    longest=0.;gap=None
    for i,f in enumerate(frames):
        if key(f['t']) not in coherent_times:
            if gap is None:gap=f['t']
        elif gap is not None:
            longest=max(longest,f['t']-gap);gap=None
    if gap is not None:
        longest=max(longest,window['requested_end_s']-gap)
    def ranges(values):
        return dict(n=len(values),min=min(values),median=statistics.median(values),max=max(values)) if values else {'n':0}
    return dict(expected_frames=expected,processed_frames=len(frames),body_observations=len(rows),
                sdk_state_counts=dict(collections.Counter(r['state'] for r in rows)),
                sdk_OK_frames=len(ok_times),sdk_OK_fraction=len(ok_times)/expected,
                shape_coherent_OK_frames=len(coherent_times),shape_coherent_OK_fraction=len(coherent_times)/expected,
                both_legs_confidence25_frames=len(leg_times),both_legs_confidence25_fraction=len(leg_times)/expected,
                first_shape_coherent_time_s=min((r['t'] for r in coherent),default=None),
                first_shape_coherent_latency_s=min((r['t']-start for r in coherent),default=None),
                longest_shape_coherent_absence_s=longest,
                sdk_OK_ids=sorted({r['id'] for r in ok}),shape_coherent_ids=sorted({r['id'] for r in coherent}),
                segment_lengths_m={k:ranges(v) for k,v in lengths.items()},
                camera_time_window_s=[start,end],
                interpretation='SDK availability and same-person shape consistency only. Confidence is not joint accuracy.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle-dir',type=Path,required=True)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--baseline-dir',type=Path,required=True)
    args=parser.parse_args()
    plan=json.loads((args.bundle_dir/'approach_plan.json').read_text(encoding='utf-8'))
    bundle=json.loads((args.bundle_dir/'bundle.json').read_text(encoding='utf-8'))
    reference=json.loads((args.bundle_dir/'reference_quality.json').read_text())['reference']
    baseline=load_export(args.baseline_dir,bundle)
    results=[]
    errors_path=args.run_dir/'case_errors.json'
    errors=json.loads(errors_path.read_text()) if errors_path.exists() else {}
    for case in plan['cases']:
        window=json.loads((args.bundle_dir/case['window_file']).read_text())
        before=review_metrics(baseline,window,reference)
        root=args.run_dir/case['id']/'sdk'
        after=review_metrics(load_export(root,bundle),window,reference) if (root/'camera.json').is_file() else None
        results.append(dict(id=case['id'],label=case['label'],window=window,
                            baseline_full_sequence=before,fresh_approach=after,error=errors.get(case['id'])))
    summary=dict(status='completed' if all(r['fresh_approach'] for r in results) and not errors else 'incomplete',
                 cases=results,reference=reference,
                 note='Windows/SDK cold-start experiment; reference segment lengths frozen from original 4.5--10.5s outbound observations, not true body dimensions.',
                 accuracy='No independent joint/position ground truth. Review the actual image overlay before concluding that tracking works.')
    with (args.run_dir/'comparison.json').open('x',encoding='utf-8') as stream:
        json.dump(summary,stream,ensure_ascii=False,indent=2,allow_nan=False)
    def pct(v):return f'{100*v:.1f}%'
    def safe(x):return html.escape(str(x))
    sections=[]
    for r in results:
        lines=[]
        for label,key in [('以前の往復全体から継続','baseline_full_sequence'),('復路のみ・SDK新規起動','fresh_approach')]:
            m=r[key]
            if m is None:
                lines.append(f'<tr><th>{label}</th><td colspan="5">未完了：{safe(r["error"] or "SDK出力なし")}</td></tr>');continue
            latency='なし' if m['first_shape_coherent_latency_s'] is None else f"{m['first_shape_coherent_latency_s']:.3f}秒"
            lines.append(f'<tr><th>{label}</th><td>{m["sdk_OK_frames"]}/{m["processed_frames"]} ({pct(m["sdk_OK_fraction"])})</td>'
                         f'<td>{m["shape_coherent_OK_frames"]}/{m["processed_frames"]} ({pct(m["shape_coherent_OK_fraction"])})</td>'
                         f'<td>{pct(m["both_legs_confidence25_fraction"])}</td><td>{latency}</td><td>{m["longest_shape_coherent_absence_s"]:.3f}秒</td></tr>')
        replay_path=args.run_dir/f'replay_{r["id"]}.html'
        replay=(f'<p><a href="{safe(replay_path.name)}">この条件を再生</a>　'
                f'<a href="{safe(replay_path.name)}?view=top">上面図で再生</a></p>') if replay_path.is_file() else ''
        sections.append(f'<section><h2>{safe(r["label"])}</h2><p>原録画 {r["window"]["requested_start_s"]}〜{r["window"]["requested_end_s"]}秒。</p>'
                        '<table><thead><tr><th>条件</th><th>SDK OK</th><th>形状整合かつOK</th><th>両脚の関節confidence≥25</th><th>最初の形状整合まで</th><th>形状整合の最大欠落</th></tr></thead><tbody>'
                        +''.join(lines)+'</tbody></table>'+replay+'</section>')
    document='''<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>接近のみの追跡比較</title>
<style>body{font:15px system-ui;background:#edf2f6;color:#183447;margin:24px auto;max-width:1250px;padding:16px}section{padding:20px;background:white;border-radius:10px;margin:20px 0;overflow:auto}table{border-collapse:collapse;width:100%}td,th{padding:10px;border-bottom:1px solid #d9e3eb;text-align:left}p{line-height:1.7}a{color:#155b90}.notice{padding:15px;background:#fff0ce}</style>
<h1>奥から手前へ近づく人物：SDK新規追跡の比較</h1>
<p>同じ録画003の復路を、各条件ごとに別のSDKプロセスで推定しました。往路・折返しのBody Tracking履歴を引き継いでいません。</p>
<p class="notice">割合は人物が映像上で確認できる選定区間の処理フレーム数を分母にしています。形状チェックは同じ人の往路の骨長と比較した補助判定です。関節の正解位置・歩幅・関節角の精度を保証しません。映像との重なりを再生で確認してください。</p>'''+''.join(sections)+'''
<p>本番の「空の場面で待機し、遠方から人物が進入する」状況と完全に同一ではありません。今回の録画では遠方に人物が見えている時点からSDKを新規起動します。LiDARは既存観測の参照表示で、今回新規追跡したのはZED骨格です。</p><p><a href="comparison.json">全数値</a></p></html>'''
    with (args.run_dir/'index.html').open('x',encoding='utf-8') as stream:stream.write(document)
    paths=[args.bundle_dir/'approach_plan.json',args.bundle_dir/'reference_quality.json',args.baseline_dir/'camera.json',args.baseline_dir/'frames.csv',args.baseline_dir/'bodies.csv',Path(__file__)]
    with (args.run_dir/'comparison.manifest.json').open('x') as stream:
        json.dump(dict(source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}),stream,indent=2)
    print(f'Comparison report: {args.run_dir/"index.html"}')


if __name__=='__main__':
    main()
