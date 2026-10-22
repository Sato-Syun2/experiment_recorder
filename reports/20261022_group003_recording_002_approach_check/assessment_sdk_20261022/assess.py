"""Assess current recording's immutable SDK export; do not refit calibration."""
from pathlib import Path
import collections
import csv
import hashlib
import json
import sys
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
BASE = OUT.parent
ROOT = BASE.parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from build_walk_skeleton_replay import load_export
from analyze_svo_body_tracking import BODY_34_EDGES

RUN = Path('/data/experiment_data/20261022_group003_recording_002_approach_check/sdk_runs/20261022_150530_153836')
SEGMENTS = dict(torso=(0,3),left_thigh=(18,19),left_shin=(19,20),right_thigh=(22,23),right_shin=(23,24))
LEGS = [18,19,20,22,23,24]
REFERENCE_INTERVAL = [17.,19.]


def read(p):
    return list(csv.DictReader(p.open(encoding='utf-8-sig',newline='')))


def save(name,data):
    with (OUT/name).open('x',encoding='utf-8') as stream:
        json.dump(data,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')


def stats(values):
    a = np.asarray(values,float).reshape(-1)
    a = a[np.isfinite(a)]
    return dict(n=0) if not len(a) else dict(n=len(a),min=float(a.min()),
        p10=float(np.percentile(a,10)),median=float(np.median(a)),p90=float(np.percentile(a,90)),max=float(a.max()))


def main():
    if (OUT/'metrics.json').exists():
        raise FileExistsError('Choose a fresh assessment output')
    bundle=json.loads((BASE/'bundle.json').read_text())
    geo=json.loads((BASE/'geometry.json').read_text())
    body=load_export(RUN/'sdk',bundle)
    camera=body['camera']
    frames,rows=read(RUN/'sdk/frames.csv'),read(RUN/'sdk/bodies.csv')
    exported=read(RUN/'bodies_corridor.csv')
    window=json.loads((RUN/'window.json').read_text())
    if len(frames)!=414 or len(exported)!=len(rows):
        raise ValueError('Incomplete SDK/corridor export')
    if json.loads((RUN/'run_complete.json').read_text())['status']!='sdk_export_and_replay_completed':
        raise ValueError('SDK runner did not complete')
    source=json.loads((RUN/'replay.manifest.json').read_text())
    hashes={}
    for name in ['camera.json','frames.csv','bodies.csv']:
        prior=[v for k,v in source['source_sha256'].items() if k.replace('\\','/').endswith('/sdk/'+name)]
        hashes[name]=len(prior)==1 and prior[0]==hashlib.sha256((RUN/'sdk'/name).read_bytes()).hexdigest()
    if not all(hashes.values()):
        raise ValueError('SDK replay source hash mismatch')
    g=geo['geometries']['zed'];R=np.array(g['rotation_raw_to_corridor']);tr=np.array(g['translation_raw_to_corridor'])
    times=np.array([float(r['elapsed_s']) for r in rows])
    joints=np.array([json.loads(r['keypoints_3d_camera_json']) for r in rows],float)
    pixels=np.array([json.loads(r['keypoints_2d_json']) for r in rows],float)
    conf=np.array([json.loads(r['keypoint_confidence_json']) for r in rows],float)
    c=joints@R.T+tr
    exp=np.array([json.loads(r['keypoints_3d_corridor_json']) for r in exported],float)
    if [(r['image_timestamp_ns'],r['body_id']) for r in rows]!=[(r['image_timestamp_ns'],r['body_id']) for r in exported]:
        raise ValueError('Corridor export body keys differ from original SDK rows')
    ext=bundle['extrinsic'];lev=(joints-np.array(ext['translation_m']))@np.array(ext['rotation_source_to_target'])
    lev=lev@np.array(bundle['level_rotation']).T+np.array(bundle['level_translation'])
    corr=bundle['corridor'];alt=np.concatenate([(lev[:,:,:2]-np.array(corr['origin_xy']))@np.array([corr['longitudinal_xy'],corr['lateral_xy']]).T,lev[:,:,2:]],axis=2)
    lengths={k:np.linalg.norm(joints[:,a]-joints[:,b],axis=1) for k,(a,b) in SEGMENTS.items()}
    refmask=(times>=REFERENCE_INTERVAL[0])&(times<=REFERENCE_INTERVAL[1])
    reference={k:float(np.median(v[refmask])) for k,v in lengths.items()}
    ratios={k:v/reference[k] for k,v in lengths.items()}
    save('reference_quality.json',dict(reference=dict(segments=SEGMENTS,reference_segment_m=reference,
        source_interval_s=REFERENCE_INTERVAL,note='Same current subject, selected near-camera interval. Relative shape consistency only; not measured body dimensions. Never changes original joints.')))
    leg_good=np.isfinite(conf[:,LEGS]).all(axis=1)&(conf[:,LEGS]>=25).all(axis=1)
    inimage=(pixels[:,:,0]>=0)&(pixels[:,:,0]<camera['width'])&(pixels[:,:,1]>=0)&(pixels[:,:,1]<camera['height'])&np.isfinite(pixels).all(axis=2)
    legs_inimage=inimage[:,LEGS].all(axis=1)
    heels_good=np.isfinite(conf[:,[32,33]])&(conf[:,[32,33]]>=25)&inimage[:,[32,33]]
    projected=joints[:,:,:2]/joints[:,:,2:3]*np.array([camera['fx'],camera['fy']])+np.array([camera['cx'],camera['cy']])
    reprojection=np.linalg.norm(projected-pixels,axis=2)
    valid_reproj=(conf>=25)&inimage&np.isfinite(joints).all(axis=2)&(joints[:,:,2]>.2)
    formula=dict(rotation_orthogonality_max=float(abs(R@R.T-np.eye(3)).max()),determinant=float(np.linalg.det(R)),
        runner_corridor_csv_max_abs_difference_m=float(np.nanmax(abs(c-exp))),
        independent_composed_transform_max_abs_difference_m=float(np.nanmax(abs(c-alt))),
        roundtrip_max_abs_difference_m=float(np.nanmax(abs((c-tr)@R-joints))),
        bone_length_change_max_m=max(float(np.nanmax(abs(np.linalg.norm(c[:,a]-c[:,b],axis=1)-v))) for (a,b),v in zip(SEGMENTS.values(),lengths.values())),
        note='Floating-point consistency only; does not validate estimated joint positions or absolute calibration.')
    periods=[]
    for lo,hi in [(7.199002,10),(10,14),(14,17),(17,19),(19,21.01)]:
        m=(times>=lo)&(times<hi)
        periods.append(dict(interval_s=[lo,hi],sdk_rows=int(m.sum()),legs_confidence25_rows=int((m&leg_good).sum()),
            legs_confidence25_and_pixels_inside_rows=int((m&leg_good&legs_inimage).sum()),
            pelvis_x_m=stats(c[m,0,0]),pelvis_y_m=stats(c[m,0,1]),head_z_m=stats(c[m,26,2]),
            heel_z_m=stats(c[m][:,[32,33],2]),supported_inimage_heel_z_m=stats(c[m][:,[32,33],2][heels_good[m]]),
            segment_lengths_m={k:stats(v[m]) for k,v in lengths.items()}))
    # One SDK observation per LiDAR observation: no tripling the evidence by
    # pairing all 30Hz SDK rows to each 10Hz LiDAR point.
    lidar=[r for r in read(BASE/'selected_observations.csv') if r['detected']=='1']
    pairs=[]
    for r in lidar:
        dt=times-float(r['time_sec']);i=int(np.argmin(abs(dt)))
        if abs(dt[i])>.06:continue
        pairs.append(dict(lidar_timestamp_ns=r['timestamp_ns'],sdk_timestamp_ns=rows[i]['image_timestamp_ns'],
            t_lidar_s=float(r['time_sec']),t_sdk_s=float(times[i]),lidar_id=int(r['track_id']),
            lidar_status=r['status'],lidar_x_m=float(r['corridor_x']),lidar_y_m=float(r['corridor_y']),
            sdk_pelvis_x_m=float(c[i,0,0]),sdk_pelvis_y_m=float(c[i,0,1]),
            dx_m=float(c[i,0,0]-float(r['corridor_x'])),dy_m=float(c[i,0,1]-float(r['corridor_y'])),
            sdk_minus_lidar_ms=float(dt[i]*1000)))
    with (OUT/'sensor_pairs.csv').open('x',newline='') as stream:
        w=csv.DictWriter(stream,fieldnames=list(pairs[0]));w.writeheader();w.writerows(pairs)
    pair_metrics={}
    for label,subset in [('including_tentative',pairs),('confirmed_only',[r for r in pairs if r['lidar_status']=='confirmed'])]:
        pair_metrics[label]=dict(n=len(subset),dx_m=stats([r['dx_m'] for r in subset]),dy_m=stats([r['dy_m'] for r in subset]),
            abs_dx_m=stats([abs(r['dx_m']) for r in subset]),abs_dy_m=stats([abs(r['dy_m']) for r in subset]),
            raw_clock_residual_ms=stats([r['sdk_minus_lidar_ms'] for r in subset]))
    flags=[]
    for i,r in enumerate(rows):
        flags.append(dict(frame_index=int(r['frame_index']),t_s=float(times[i]),body_id=r['body_id'],
            sdk_state=r['tracking_state'],legs_confidence25=bool(leg_good[i]),legs_pixels_inside=bool(legs_inimage[i]),
            pelvis_corridor=c[i,0].tolist(),head_z_m=float(c[i,26,2]),left_heel_z_m=float(c[i,32,2]),
            right_heel_z_m=float(c[i,33,2]),segment_ratio_to_current_near_reference={k:float(v[i]) for k,v in ratios.items()}))
    save('observations.json',flags)
    metrics=dict(source_run=str(RUN),processed_frames=len(frames),expected_frames=414,body_rows=len(rows),
        frames_with_sdk_OK=len({r['image_timestamp_ns'] for r in rows if r['tracking_state']=='OK'}),
        sdk_OK_fraction=len({r['image_timestamp_ns'] for r in rows if r['tracking_state']=='OK'})/len(frames),
        states=dict(collections.Counter(r['tracking_state'] for r in rows)),ids=dict(collections.Counter(r['body_id'] for r in rows)),
        no_body_source_frames=[int(r['frame_index']) for r in frames if int(r['body_count'])==0],
        first_body_time_s=float(times[0]),window_time_s=[float(frames[0]['elapsed_s']),float(frames[-1]['elapsed_s'])],
        both_legs_confidence25_frames=int(leg_good.sum()),both_legs_confidence25_fraction=float(leg_good.sum()/len(frames)),
        both_legs_confidence25_and_inside_frames=int((leg_good&legs_inimage).sum()),
        pelvis_x_m=stats(c[:,0,0]),pelvis_y_m=stats(c[:,0,1]),head_z_m=stats(c[:,26,2]),
        heels_z_m=stats(c[:,[32,33],2]),supported_inimage_heels_z_m=stats(c[:,[32,33],2][heels_good]),
        heels_below_minus_10cm_with_confidence25_inside_rows=int(np.any((c[:,[32,33],2]<-.10)&heels_good,axis=1).sum()),
        reference_interval_s=REFERENCE_INTERVAL,reference_segment_m=reference,
        segment_lengths_m={k:stats(v) for k,v in lengths.items()},segment_ratio_to_near_reference={k:stats(v) for k,v in ratios.items()},
        reprojection_3d_vs_sdk_2d_px=stats(reprojection[valid_reproj]),periods=periods,
        transform_arithmetic=formula,sensor_pairs=pair_metrics,source_hash_match=hashes,
        calibration_status='frozen current floor/wall candidate, longitudinal origins provisionally equated',
        clock_status='unestimated; nearest raw absolute timestamps only, no offset fitted',
        position_error_to_ground_truth_m=None,
        interpretation='SDK output availability is high. Corridor approach path broadly agrees. Absolute joint heights and distance-dependent segment lengths are not yet validated; do not call all 15m skeleton measurements normal.')
    save('metrics.json',metrics)

    fig,axs=plt.subplots(5,1,figsize=(12,14),layout='constrained',height_ratios=[1.2,2,1.7,1.5,1.2])
    gap=corr['wall_gap']
    axs[0].axhline(-gap/2,color='gray');axs[0].axhline(gap/2,color='gray');axs[0].plot(c[:,0,0],c[:,0,1],color='#16845b',lw=1,label='ZED pelvis')
    for status,color,label in [('tentative','#c47b11','LiDAR tentative'),('confirmed','#2563eb','LiDAR confirmed')]:
        a=[r for r in lidar if r['status']==status]
        axs[0].scatter([float(r['corridor_x']) for r in a],[float(r['corridor_y']) for r in a],s=12,c=color,label=label)
        axs[1].scatter([float(r['time_sec']) for r in a],[float(r['corridor_x']) for r in a],s=12,c=color)
    axs[0].set(xlim=(0,17),ylim=(-1.4,1.4),xlabel='Corridor X [m]',ylabel='Y [m]',title='Approach: independent ZED pelvis and LiDAR cluster median');axs[0].set_aspect('equal',adjustable='box');axs[0].legend(fontsize=8,loc='upper left')
    axs[1].plot(times,c[:,0,0],color='#16845b',lw=1,label='ZED pelvis');axs[1].set(ylabel='Corridor X [m]')
    axs[2].plot(times,c[:,26,2],color='#555',lw=1,label='HEAD joint')
    for j,color,label in [(32,'#9333ea','Left heel'),(33,'#ce5493','Right heel')]:
        axs[2].scatter(times,c[:,j,2],s=3,c=color,alpha=.3)
        supported=(conf[:,j]>=25)&inimage[:,j];axs[2].scatter(times[supported],c[supported,j,2],s=5,c=color,label=label+' conf>=25, in image')
    axs[2].axhline(0,color='black',ls='--',lw=.8);axs[2].set(ylabel='Height above fitted floor [m]');axs[2].legend(fontsize=8,ncol=3)
    for key in ['left_thigh','left_shin','torso']:axs[3].plot(times,ratios[key],label=key)
    axs[3].axvspan(*REFERENCE_INTERVAL,color='#cddfcc',alpha=.5,label='Same-person near reference');axs[3].axhline(1,color='gray',ls=':');axs[3].set(ylabel='Segment / near median');axs[3].legend(fontsize=8,ncol=4)
    axs[4].plot(times,np.nanmin(conf[:,LEGS],axis=1),color='#16845b',lw=1);axs[4].axhline(25,color='gray',ls='--');axs[4].set(ylabel='Minimum leg confidence',xlabel='Original ZED elapsed [s]',ylim=(-2,102))
    for ax in axs[1:]:ax.set_xlim(7.199,21.0)
    for ax in axs:ax.grid(alpha=.2)
    fig.savefig(OUT/'sdk_diagnostics.png',dpi=140);plt.close(fig)

    # Crop real exported images around the person. Red joints have confidence
    # below 25; they remain visible for diagnosis, without inventing positions.
    assets=OUT/'image_checks';assets.mkdir(exist_ok=False)
    image_rows=read(BASE/'review_frame_timestamps.csv')
    panels=[]
    for target in [7.4,8,9,10.6,12,14,16,18,19.4]:
        fr=min(image_rows,key=lambda r:abs(float(r['elapsed_s'])-target))
        candidates=np.where(np.array([int(r['image_timestamp_ns']) for r in rows])==int(fr['image_timestamp_ns']))[0]
        if not len(candidates):continue
        i=int(candidates[0]);name=f"{int(fr['sample_index']):04d}.jpg"
        im=cv2.imread(str(BASE/'assets'/name))
        if im is None:raise ValueError(name)
        uv=pixels[i]
        for a,b in BODY_34_EDGES:
            if np.isfinite(uv[[a,b]]).all() and inimage[i,a] and inimage[i,b]:
                col=(40,195,30) if min(conf[i,a],conf[i,b])>=25 else (70,70,230)
                cv2.line(im,tuple(np.rint(uv[a]).astype(int)),tuple(np.rint(uv[b]).astype(int)),col,2,cv2.LINE_AA)
        bb=json.loads(rows[i]['bounding_box_2d_json']);x0,y0,x1,y1=bb
        pad=max(25,(x1-x0)*.5)
        left=max(0,int(x0-pad));right=min(im.shape[1],int(x1+pad));top=max(0,int(y0-pad*.5));bottom=min(im.shape[0],int(y1+pad*.5))
        crop=im[top:bottom,left:right]
        ratio=min(390/crop.shape[1],350/crop.shape[0]);tile=cv2.resize(crop,None,fx=ratio,fy=ratio)
        panel=np.full((440,410,3),248,np.uint8);yy=42;xx=(410-tile.shape[1])//2;panel[yy:yy+tile.shape[0],xx:xx+tile.shape[1]]=tile
        cv2.putText(panel,f't={times[i]:.2f}s X={c[i,0,0]:.2f}m',(10,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(25,25,25),1,cv2.LINE_AA)
        cv2.putText(panel,f'Heel Z L/R: {c[i,32,2]:+.2f}/{c[i,33,2]:+.2f}m',(10,412),cv2.FONT_HERSHEY_SIMPLEX,.52,(25,25,25),1,cv2.LINE_AA)
        cv2.putText(panel,'Green conf>=25; red conf<25',(10,432),cv2.FONT_HERSHEY_SIMPLEX,.48,(25,25,25),1,cv2.LINE_AA)
        cv2.imwrite(str(assets/f'{len(panels):02d}.jpg'),panel);panels.append(panel)
    montage=np.full((3*440,3*410,3),248,np.uint8)
    for i,panel in enumerate(panels):montage[i//3*440:(i//3+1)*440,i%3*410:(i%3+1)*410]=panel
    cv2.imwrite(str(OUT/'image_correspondence.jpg'),montage)
    # A fresh replay uses a declared reference from this recording, rather than
    # the older template's implicit early/far interval. Coordinates stay fixed.
    bundle['asset_directory']='../assets'
    save('bundle.json',bundle)
    template=(BASE/'skeleton_replay_template.html').read_text()
    template=template.replace('<main><p','<main><p style="padding:12px;background:#ffe6df">SDK確認済み：414フレーム中411で同じID。接近経路は概ね整合しますが、遠方のかかとは推定床面より最大約28 cm下に入り、脚長は近距離の約1.5倍になる区間があります。形状フラグやSDK OKは精度合格ではありません。<a href="index.html">確認結果</a></p><p',1)
    (OUT/'skeleton_replay_template.html').write_text(template)
    save('manifest.json',dict(source_run=str(RUN),source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        [Path(__file__),RUN/'sdk/camera.json',RUN/'sdk/frames.csv',RUN/'sdk/bodies.csv',RUN/'bodies_corridor.csv',
         RUN/'window.json',RUN/'replay.manifest.json',BASE/'geometry.json',BASE/'bundle.json',BASE/'selected_observations.csv']},
        hardware='None; immutable SDK exports only.',tests='No unit tests requested or run.'))
    print(json.dumps({k:v for k,v in metrics.items() if k not in ['periods','segment_lengths_m','segment_ratio_to_near_reference','source_run']},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
