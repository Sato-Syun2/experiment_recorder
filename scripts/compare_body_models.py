#!/usr/bin/env python3
"""Compare immutable BODY_34 exports at identical original image timestamps.

SDK availability, relative segment stability and host SDK latency are descriptive
metrics. They do not establish target identity or absolute joint accuracy.
"""
from pathlib import Path
import argparse,collections,csv,hashlib,html,json,math,os,statistics
import numpy as np

SEGMENTS=dict(torso=(0,3),left_thigh=(18,19),left_shin=(19,20),right_thigh=(22,23),right_shin=(23,24))
LEGS=[18,19,20,22,23,24]


def read(path):
    return list(csv.DictReader(path.open(encoding='utf-8-sig',newline='')))


def stats(values):
    a=np.asarray(values,float).reshape(-1);a=a[np.isfinite(a)]
    return dict(n=0) if not len(a) else dict(n=len(a),min=float(a.min()),median=float(np.median(a)),
        p10=float(np.percentile(a,10)),p90=float(np.percentile(a,90)),max=float(a.max()))


def assess_export(body_dir,window,reference_ns,geometry=None):
    camera=json.loads((body_dir/'camera.json').read_text(encoding='utf-8-sig'))
    frames,rows=read(body_dir/'frames.csv'),read(body_dir/'bodies.csv')
    if camera.get('body_format')!='BODY_34' or camera.get('coordinate_units')!='meter' or not camera.get('coordinate_system','').startswith('IMAGE'):
        raise ValueError('Expected BODY_34 / IMAGE / meter export.')
    expected=[int(x) for x in window['expected_image_timestamps_ns']]
    if [int(r['image_timestamp_ns']) for r in frames]!=expected or [int(r['frame_index']) for r in frames]!=list(range(window['start_frame'],window['end_frame_exclusive'])):
        raise ValueError('Every declared source frame/timestamp must match exactly.')
    counts=collections.Counter(int(r['image_timestamp_ns']) for r in rows)
    if len({(r['image_timestamp_ns'],r['body_id']) for r in rows})!=len(rows) or any(counts[int(r['image_timestamp_ns'])]!=int(r['body_count']) for r in frames):
        raise ValueError('Body rows do not match the SDK frame ledger.')
    origin=int(window['source_origin_ns']);times=np.array([(t-origin)/1e9 for t in expected])
    good=[r for r in rows if r['tracking_state']=='OK']
    ids=collections.Counter(r['body_id'] for r in good)
    dominant=ids.most_common(1)[0][0] if ids else None
    selected=[r for r in good if r['body_id']==dominant]
    bystamp={}
    for r in good:bystamp.setdefault(int(r['image_timestamp_ns']),[]).append(r)
    switches=0;previous=None
    for t,items in sorted(bystamp.items()):
        if len(items)!=1:previous=None;continue
        if previous and t-previous[0]<=150000000 and previous[1]!=items[0]['body_id']:switches+=1
        previous=(t,items[0]['body_id'])
    result=dict(sdk_version_field=camera.get('sdk_version'),pyzed_version=camera.get('pyzed_package_version'),
        model=camera.get('body_tracking_model'),generation_requested=camera.get('body_model_generation_requested'),
        generation_effective=camera.get('body_model_generation_effective'),generation_verified=camera.get('body_model_generation_readback_verified',False),
        frames=len(frames),window_s=[float(times[0]),float(times[-1])],sdk_OK_any_frames=len(bystamp),
        sdk_OK_any_fraction=len(bystamp)/len(frames),sdk_OK_ids=dict(ids),dominant_id=dominant,
        dominant_OK_frames=len(selected),dominant_OK_fraction=len(selected)/len(frames),
        multiple_OK_ID_frames=sum(len(v)>1 for v in bystamp.values()),single_ID_adjacency_changes=switches,
        SDK_OK_no_body_frames=len(frames)-len(bystamp),
        latency=camera.get('sdk_grab_and_bodies_ms_after_warmup'),model_enable_seconds=camera.get('model_enable_seconds'),
        processing_wall_s=camera.get('processing_wall_seconds_including_export_io'),
        depth_precision=camera.get('depth_precision_effective'),positional_mode=camera.get('positional_tracking_requested_mode'),
        prediction_timeout_s=camera.get('sdk_prediction_timeout_s'),
        identity_note='Dominant ID selected only by observation count; not visually verified person identity. SDK OK can include SDK predictions.',
        timestamp_count_verified=True,first_OK_latency_s=min(((t-expected[0])/1e9 for t in bystamp),default=None))
    missing_start=None;longest=0.
    spacing=float(np.median(np.diff(times))) if len(times)>1 else 0.
    for i,t in enumerate(expected):
        if t not in bystamp and missing_start is None:missing_start=times[i]
        if t in bystamp and missing_start is not None:longest=max(longest,float(times[i]-missing_start));missing_start=None
    if missing_start is not None:longest=max(longest,float(times[-1]+spacing-missing_start))
    result['max_no_OK_gap_s']=longest
    if not selected:
        result.update(both_legs_conf25_inside_frames=0,both_legs_conf25_inside_fraction=0.,segment_ratio_abs_deviation_p90=None)
        return result,[]
    stamps=np.array([int(r['image_timestamp_ns']) for r in selected],dtype=np.int64)
    t=(stamps-origin)/1e9
    joints=np.array([json.loads(r['keypoints_3d_camera_json']) for r in selected],float)
    pixels=np.array([json.loads(r['keypoints_2d_json']) for r in selected],float)
    confidence=np.array([json.loads(r['keypoint_confidence_json']) for r in selected],float)
    if joints.shape[1:]!=(34,3) or pixels.shape[1:]!=(34,2) or confidence.shape[1:]!=(34,):raise ValueError('Unexpected joint topology.')
    inside=np.isfinite(pixels).all(axis=2)&(pixels[:,:,0]>=0)&(pixels[:,:,0]<camera['width'])&(pixels[:,:,1]>=0)&(pixels[:,:,1]<camera['height'])
    leg=np.isfinite(confidence[:,LEGS]).all(axis=1)&(confidence[:,LEGS]>=25).all(axis=1)&inside[:,LEGS].all(axis=1)&np.isfinite(joints[:,LEGS]).all(axis=(1,2))
    refmask=(stamps>=int(reference_ns[0]))&(stamps<=int(reference_ns[1]))
    lengths={key:np.linalg.norm(joints[:,a]-joints[:,b],axis=1) for key,(a,b) in SEGMENTS.items()}
    refs={key:stats(v[refmask]).get('median') for key,v in lengths.items()}
    ratios={key:v/refs[key] for key,v in lengths.items() if refs[key] is not None and refs[key]>0}
    deviations=np.concatenate([abs(v-1) for v in ratios.values()]) if ratios else []
    result.update(both_legs_conf25_inside_frames=int(leg.sum()),both_legs_conf25_inside_fraction=float(leg.sum()/len(frames)),
        reference_rows=int(refmask.sum()),reference_segment_m=refs,segment_lengths_m={k:stats(v) for k,v in lengths.items()},
        segment_ratios_to_same_model_near_reference={k:stats(v) for k,v in ratios.items()},
        segment_ratio_abs_deviation_p90=stats(deviations).get('p90'),
        reference_note='Each model uses its own median within the identical fixed near interval. Relative shape variation only; uniform size bias will not be detected.')
    corridor=None
    if geometry:
        corridor=joints@np.array(geometry['rotation_raw_to_corridor']).T+np.array(geometry['translation_raw_to_corridor'])
        supported=(confidence[:,[32,33]]>=25)&inside[:,[32,33]]&np.isfinite(corridor[:,[32,33]]).all(axis=2)
        z=corridor[:,[32,33],2]
        result.update(heel_z_m=stats(z[supported]),supported_heel_below_minus10cm_frames=int(np.any((z<-.1)&supported,axis=1).sum()),
            supported_heel_below_minus10cm_fraction=float(np.any((z<-.1)&supported,axis=1).sum()/len(frames)),
            pelvis_x_m=stats(corridor[:,0,0]),pelvis_y_m=stats(corridor[:,0,1]),
            floor_note='Frozen recording002 floor candidate, not surveyed ground truth. Never refitted per model.')
    else:result['floor_note']='No validated current-recording ZED floor transform provided; floor metrics unavailable.'
    series=[]
    for i in range(len(t)):
        series.append(dict(t=float(t[i]),pelvis_x=None if corridor is None or not np.isfinite(corridor[i,0,0]) else float(corridor[i,0,0]),
            leg_conf25=bool(leg[i]),shin_ratio=float(ratios['left_shin'][i]) if 'left_shin' in ratios and np.isfinite(ratios['left_shin'][i]) else None))
    return result,series


def chart(series_by_model,field,title):
    valid=[(key,s) for key,s in series_by_model.items() if any(r.get(field) is not None for r in s)]
    if not valid:return ''
    points=[(r['t'],r[field]) for _,seq in valid for r in seq if isinstance(r.get(field),(int,float)) and not isinstance(r.get(field),bool)]
    if not points:return ''
    xmin,xmax=min(x for x,y in points),max(x for x,y in points);ymin,ymax=min(y for x,y in points),max(y for x,y in points)
    if ymax==ymin:ymax+=1
    colors=['#00865b','#315cce','#b65718','#a535a7','#b7a000','#888']
    def p(x,y):return 70+(x-xmin)/(xmax-xmin or 1)*970,250-(y-ymin)/(ymax-ymin)*175
    parts=[f'<h3>{html.escape(title)}</h3><svg viewBox="0 0 1100 310" role="img" aria-label="{html.escape(title)}">']
    for j in range(5):
        y=ymin+j*(ymax-ymin)/4;_,yy=p(xmin,y);parts.append(f'<path d="M70 {yy}H1040" stroke="#dce3eb"/><text x="8" y="{yy+4}">{y:.2f}</text>')
    for j in range(6):
        x=xmin+j*(xmax-xmin)/5;xx,_=p(x,ymin);parts.append(f'<text x="{xx-15}" y="280">{x:.1f}s</text>')
    for i,(label,seq) in enumerate(valid):
        paths=[];last=None
        for r in seq:
            y=r.get(field)
            if y is None: last=None;continue
            x,y=p(r['t'],y);command='M' if last is None or r['t']-last>.15 else 'L';paths.append(f'{command}{x:.2f},{y:.2f}');last=r['t']
        color=colors[i%len(colors)];parts.append(f'<path d="{" ".join(paths)}" fill="none" stroke="{color}" stroke-width="1.4"/><text x="{70+(i%3)*330}" y="{23+(i//3)*22}" fill="{color}">{html.escape(label)}</text>')
    parts.append('</svg>');return ''.join(parts)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package-dir',type=Path,required=True);parser.add_argument('--run-dir',type=Path,required=True)
    args=parser.parse_args();package=args.package_dir.resolve();run=args.run_dir.resolve()
    plan=json.loads((package/'comparison_plan.json').read_text(encoding='utf-8'))
    raw=json.loads((run/'case_results.json').read_text(encoding='utf-8'));summaries=[];series={};groups={}
    for entry in raw['cases']:
        record,scope=entry['recording'],entry['scope'];label=entry['configuration']['id'];key=f'{record}/{scope}'
        row=dict(recording=record,scope=scope,label=label,status=entry['status'])
        if entry['status']=='completed':
            try:
                window=json.loads((run/entry['window']).read_text(encoding='utf-8'))
                body=run/entry['body_dir'];info=plan['recordings'][record]
                geometry=json.loads((package/info['geometry_file']).read_text()) if info.get('geometry_file') else None
                metrics,sequence=assess_export(body,window,info['reference_header_ns'],geometry)
                if not metrics['generation_verified'] or metrics['generation_effective']!=entry['configuration']['generation']:
                    raise ValueError('The effective model generation is unverified or different.')
                row.update(metrics=metrics,body_dir=entry['body_dir']);series.setdefault(key,{})[label]=sequence
            except Exception as exc:row.update(status='invalid',error=str(exc))
        else:row['error']=entry.get('error')
        summaries.append(row);groups.setdefault(key,[]).append(row)
    historical=plan.get('historical_baseline')
    if historical:
        body=package/historical['body_dir_relative_to_package']
        window=json.loads((package/'reference_window_002.json').read_text())
        metrics,seq=assess_export(body,window,plan['recordings']['002']['reference_header_ns'],json.loads((package/'geometry_002.json').read_text()))
        row=dict(recording='002',scope='approach',label='historical_5_4_accurate',status='historical',metrics=metrics,
            note='Historical sdk_version field recorded pyzed package version only; runtime SDK patch and generation were not read back. SDK/model changes are confounded.')
        summaries.append(row);groups.setdefault('002/approach',[]).append(row);series.setdefault('002/approach',{})[row['label']]=seq
    payload=dict(cases=summaries,execution_errors=raw['errors'],ranking_status='pending_visual_person_pole_association',
        interpretation='Compare metrics within each identical recording/window. No absolute accuracy ground truth. Review RGB and ID association before choosing a model.')
    with (run/'comparison.json').open('x',encoding='utf-8') as f:json.dump(payload,f,ensure_ascii=False,indent=2,allow_nan=False)
    def cell(v,percent=False):return '—' if v is None else f'{100*v:.1f}%' if percent else f'{v:.3f}' if isinstance(v,float) else html.escape(str(v))
    sections=[]
    for key,rows in sorted(groups.items()):
        content=[]
        for r in rows:
            m=r.get('metrics',{});lat=m.get('latency') or {};heel=m.get('heel_z_m') or {}
            source=run/r['body_dir'] if r.get('body_dir') else body if r['status']=='historical' else None
            contact=(source/'contact_sheet.jpg') if source else None
            link=(f'<a href="{html.escape(Path(os.path.relpath(contact,run)).as_posix())}">画像一覧</a>' if contact and contact.is_file() else '')
            content.append(f'<tr><th>{html.escape(r["label"])}<br>{link}</th><td>{html.escape(r["status"])}</td><td>{cell(m.get("sdk_OK_any_fraction"),True)}</td><td>{cell(m.get("both_legs_conf25_inside_fraction"),True)}</td><td>{cell(len(m.get("sdk_OK_ids",{})))}</td><td>{cell(m.get("max_no_OK_gap_s"))}</td><td>{cell(m.get("segment_ratio_abs_deviation_p90"))}</td><td>{cell(heel.get("min"))}</td><td>{cell(lat.get("median"))}</td></tr>')
        charts=chart(series.get(key,{}),'shin_ratio','下腿長 / 同モデルの固定近距離区間中央値')+chart(series.get(key,{}),'pelvis_x','固定した廊下座標での骨盤X [m]')
        sections.append(f'<section><h2>録画・範囲：{html.escape(key)}</h2><div class="scroll"><table><tr><th>設定</th><th>状態</th><th>SDK OK率</th><th>両脚confidence≥25・画像内</th><th>OK ID数</th><th>最大OK欠落秒</th><th>骨長比の偏差p90</th><th>かかと最小高m</th><th>SDK処理ms中央値</th></tr>{"".join(content)}</table></div>{charts}</section>')
    errors=''.join(f'<li>{html.escape(k)}: {html.escape(v)}</li>' for k,v in raw['errors'].items())
    page='''<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SDK 5.5.1 モデル比較</title><style>body{font:15px/1.8 system-ui;background:#f1f5fa;color:#1c344e;margin:24px auto;padding:0 18px;max-width:1450px}h1{font-size:26px}section{background:white;padding:20px;border-radius:12px;margin:20px 0}.notice{padding:18px;background:#fff1d4}.scroll{overflow:auto}table{border-collapse:collapse;width:100%;font-size:13px}td,th{padding:9px;border-bottom:1px solid #d7e0e9;text-align:left}svg{width:100%;background:#fafcff}a{color:#1457a1}</style><h1>NEURAL / BODY_34：モデル・世代の比較</h1><p class="notice">MEDIUM/GEN_2が指定の第一候補です。全設定は同じ元画像時刻でSDKを新規開始します。表のSDK OK率は人物の実検出率ではありません。画像で人物とポールの対応を確認してから、骨長・高さ・継続性・速度を総合して選びます。自動的に精度の勝者とは判定しません。</p><p>骨長比は各モデル自身の同一の近距離時間窓を基準にした相対変動です。一様な身体サイズの誤りは検出しません。かかと高は録画002の固定した床面候補に対する値で、003の床高は未検証のため表示しません。SDK処理時間は最初の30フレームを除いたgrab＋retrieve_bodiesのホスト時間で、GPU単体の推論速度ではありません。初回モデル最適化時間は別に保存しています。</p><p>historical行は以前の出力です。旧版で実SDKパッチ・有効な世代を記録していなかったため、SDK更新とモデル変更の影響を完全には分離できません。新SDKのaccurate_gen1を対照にして世代・モデルの差を確認してください。全区間には空の区間が含まれ得るため、SDK OK率を接近区間と直接比較しないでください。</p><p><a href="comparison.json">全数値・設定</a></p>'''+''.join(sections)+(f'<section><h2>未完了・失敗</h2><ul>{errors}</ul></section>' if errors else '')+'</html>'
    (run/'index.html').write_text(page,encoding='utf-8')
    with (run/'comparison_manifest.json').open('x',encoding='utf-8') as f:
        files=[Path(__file__),package/'comparison_plan.json',run/'case_results.json']
        files += [run/r['body_dir']/'camera.json' for r in summaries if r.get('body_dir')]
        json.dump(dict(source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}),f,indent=2)
    print(f'Comparison: {run/"index.html"}')


if __name__=='__main__':main()
