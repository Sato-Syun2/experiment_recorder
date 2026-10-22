"""Build an auditable comparison from completed offline inference, no devices."""
from pathlib import Path
import argparse
import base64
import csv
import hashlib
import json
import sys
from collections import Counter, defaultdict

import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
from track_people import load_settings
from assignment import gated_assignment

OUT = Path(__file__).resolve().parent
RUN = OUT/'full'
REVIEW = Path('/data/experiment_data/20261021_group002_recording_003_walk_review')
OLD = ROOT/'不要なもの/reports/20261021_group002_recording_003_walk_assessment'


def read(path):
    with path.open(newline='') as f:
        return list(csv.DictReader(f))


def stats(a):
    a = np.asarray(a, dtype=float)
    return dict(n=len(a), median=float(np.median(a)), p90=float(np.percentile(a,90)),
                max=float(np.max(a))) if len(a) else dict(n=0)


def associate(rows, frames):
    """Same .55 m constant-velocity gate, 1.2 s lifetime, .65 velocity update.

    These IDs are descriptive associations, not verified participant identities.
    No fabricated positions or interpolation; all exported rows are detector outputs.
    """
    by_frame=defaultdict(list)
    for r in rows: by_frame[int(r['frame'])].append(r)
    tracks={}; next_id=1
    for f in frames:
        stamp=int(f['timestamp_ns'])
        tracks={k:v for k,v in tracks.items() if (stamp-v['stamp'])/1e9<=1.2}
        current=by_frame[int(f['frame'])]
        ids=list(tracks)
        pp=np.array([tracks[k]['p']+tracks[k]['v']*((stamp-tracks[k]['stamp'])/1e9) for k in ids]).reshape(-1,2)
        xy=np.array([[float(r['level_x']),float(r['level_y'])] for r in current]).reshape(-1,2)
        distances=np.linalg.norm(pp[:,None,:]-xy[None,:,:],axis=2)
        matched={j:ids[i] for i,j in gated_assignment(distances**2,distances<=.55)}
        for j,r in enumerate(current):
            tid=matched.get(j)
            if tid is None:
                tid=next_id;next_id+=1
                velocity=np.zeros(2); hits=1;consecutive=1
            else:
                tr=tracks[tid]
                velocity=.35*tr['v']+.65*(xy[j]-tr['p'])/((stamp-tr['stamp'])/1e9)
                speed=np.linalg.norm(velocity)
                if speed>3: velocity*=3/speed
                hits=tr['hits']+1
                consecutive=tr['consecutive']+1 if tr['last_frame']==int(f['frame'])-1 else 1
            tracks[tid]=dict(p=xy[j],v=velocity,stamp=stamp,hits=hits,consecutive=consecutive,last_frame=int(f['frame']))
            r['track_id']=str(tid)
            r['two_consecutive_outputs']=int(consecutive>=2)
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,default=OUT/'comparison')
    result=parser.parse_args().output_dir
    result.mkdir(exist_ok=False)
    manifest = json.loads((RUN/'manifest.json').read_text())
    if not manifest['completed'] or manifest['processed_frames'] != 318:
        raise ValueError('Require all 318 source clouds')
    frames, baseline, detections = read(RUN/'frames.csv'), read(RUN/'baseline_tracks.csv'), read(RUN/'detections.csv')
    camera = json.loads((REVIEW/'camera.json').read_text())
    origin = int(camera['first_image_timestamp_ns'])
    t = lambda stamp: (int(stamp)-origin)/1e9
    visible = {int(r['frame']) for r in frames if 2 <= t(r['timestamp_ns']) <= 29.8}
    learned = [r for r in detections if r['class_name']=='Pedestrian']
    associate(learned,frames)
    if learned:
        with (result/'model_associated_observations.csv').open('x',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(learned[0]));w.writeheader();w.writerows(learned)
    # Shared display/evaluation spatial gate; no learned-model-specific tuning.
    corridor = lambda r: 0 < float(r['corridor_x']) < 20 and abs(float(r['corridor_y'])) < 1.2
    learned_in = [r for r in learned if corridor(r)]
    old_confirmed = [r for r in baseline if r['status']=='confirmed' and r['detected']=='1' and corridor(r)]
    old_selected = [r for r in baseline if int(r['track_id']) in (17,60) and r['detected']=='1']
    def cover(rows):
        ids={int(r['frame']) for r in rows} & visible
        return dict(frames=len(ids), denominator=len(visible), fraction=len(ids)/len(visible))
    def motion(rows):
        # Rows selected for a descriptive path only; never promote candidates.
        rows=sorted(rows,key=lambda r:int(r['timestamp_ns']))
        tt=np.array([t(r['timestamp_ns']) for r in rows])
        xy=np.array([[float(r['corridor_x']),float(r['corridor_y'])] for r in rows]).reshape(-1,2)
        if len(rows)<2: return dict(observations=len(rows), adjacent_motion=None)
        dt=np.diff(tt)
        valid=(dt>0)&(dt<=.25)
        v=np.linalg.norm(np.diff(xy,axis=0)[valid],axis=1)/dt[valid]
        residuals=[]
        for i in range(2,len(tt)-2):
            ts=tt[i-2:i+3]
            if (np.diff(ts)<=0).any() or np.max(np.diff(ts))>.25: continue
            fit=np.column_stack([np.ones(5),ts-tt[i]])
            estimate=np.linalg.lstsq(fit,xy[i-2:i+3],rcond=None)[0][0]
            residuals.append(np.linalg.norm(estimate-xy[i]))
        return dict(observations=len(rows),max_observation_interval_s=float(dt.max()),
                    adjacent_speed_mps=stats(v),local_linear_residual_m=stats(residuals),
                    note='Internal plausibility only, not true position/velocity error')
    same_bytes=(RUN/'baseline_tracks.csv').read_bytes()==(OLD/'tracks.csv').read_bytes()
    models_by_id=defaultdict(list)
    model_counts=Counter(int(r['frame']) for r in learned_in)
    for r in learned_in: models_by_id[r['track_id']].append(r)
    model_id_summary=[]
    for tid,rows in models_by_id.items():
        rows.sort(key=lambda r:int(r['timestamp_ns']))
        xs=[float(r['corridor_x']) for r in rows];ys=[float(r['corridor_y']) for r in rows]
        model_id_summary.append(dict(track_id=tid,rows=len(rows),start_s=t(rows[0]['timestamp_ns']),
                                     end_s=t(rows[-1]['timestamp_ns']),x_min=min(xs),x_max=max(xs),
                                     y_min=min(ys),y_max=max(ys),motion=motion(rows)))
    metrics=dict(recording='20261021_group002/recording_003',source_frames=len(frames),
                 baseline_identical_to_archived_csv=same_bytes,
                 coverage_window_note='Broad ZED 2.0--29.8 s window incl. entry/exit, not person recall',
                 baseline_confirmed_coverage=cover(old_confirmed),
                 baseline_reviewed_candidates_coverage=cover(old_selected),
                 pointpillars_person_output_coverage=cover(learned_in),
                 model_class_counts=dict(Counter(r['class_name'] for r in detections)),
                 model_person_boxes_all=len(learned),model_person_boxes_in_corridor=len(learned_in),
                 model_person_associated_ids=len({r['track_id'] for r in learned_in}),
                 model_multiple_person_output_frames=sum(v>1 for k,v in model_counts.items() if k in visible),
                 model_associated_track_summaries=model_id_summary,
                 model_two_consecutive_output_coverage=cover([r for r in learned_in if r['two_consecutive_outputs']]),
                 model_association_note='Descriptive IDs, not verified people; two outputs is not equivalent to legacy motion-confirmation',
                 inference_seconds=stats([float(r['model_inference_s']) for r in frames]),
                 model_raw_intensity_max=stats([float(r['input_intensity_max']) for r in frames]),
                 baseline_reviewed_path=motion(old_selected),
                 metric_accuracy=None,metric_accuracy_note='No independent metric trajectory or joint ground truth')
    common_old={int(r['frame']):r for r in old_selected}
    common_new={int(r['frame']):r for r in learned_in}
    paired=sorted(common_old.keys() & common_new.keys())
    a=[common_old[k] for k in paired];b=[common_new[k] for k in paired]
    delta=np.array([[float(q['corridor_x'])-float(p['corridor_x']),float(q['corridor_y'])-float(p['corridor_y'])]
                    for p,q in zip(a,b)]).reshape(-1,2)
    all_model_times=sorted({int(r['timestamp_ns']) for r in learned_in})
    metrics['pointpillars_max_observation_interval_s']=float(np.diff(all_model_times).max()/1e9) if len(all_model_times)>1 else None
    metrics['common_observations']=dict(n=len(paired),baseline_motion=motion(a),pointpillars_motion=motion(b),
                                        horizontal_difference_m=stats(np.linalg.norm(delta,axis=1)),
                                        lateral_abs_difference_m=stats(abs(delta[:,1])),
                                        note='Same clouds, different physical body representative points; differences are NOT errors')
    alt=read(OUT/'height_measured_stride10/detections.csv')
    alt_people=[r for r in alt if r['class_name']=='Pedestrian' and corridor(r)]
    subset={int(r['frame']) for r in read(OUT/'height_measured_stride10/frames.csv')} & visible
    metrics['height_sensitivity']=dict(sampled_visible_frames=len(subset),
                                       main_height_m=1.73,alternative_height_m=2.0506525225917787,
                                       main_person_frames=len({int(r['frame']) for r in learned_in}&subset),
                                       alternative_person_frames=len({int(r['frame']) for r in alt_people}&subset),
                                       note='Stride-10 detector outputs only; alternative baseline tracker is not compared')
    (result/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n')

    colors={'confirmed':'#1764c0','tentative':'#d58915','learned':'#d63274'}
    fig,axes=plt.subplots(2,1,figsize=(12,7),constrained_layout=True)
    for rows,color,label in [(old_confirmed,colors['confirmed'],'Rule: confirmed observation'),
                             (old_selected,colors['tentative'],'Rule: reviewed candidates incl. tentative'),
                             (learned_in,colors['learned'],'PointPillars: person output')]:
        tt=[t(r['timestamp_ns']) for r in rows]
        axes[0].scatter(tt,[float(r['corridor_x']) for r in rows],s=10,c=color,label=label,alpha=.8)
        axes[1].scatter(tt,[float(r['corridor_y']) for r in rows],s=10,c=color,alpha=.8)
    for ax in axes:
        ax.set_xlim(0,30); ax.grid(alpha=.25); ax.set_xlabel('ZED elapsed time [s]')
    axes[0].set_ylabel('Corridor longitudinal X [m]'); axes[1].set_ylabel('Corridor lateral Y [m]')
    axes[0].legend(fontsize=8);axes[1].set_ylim(-1.3,1.3)
    fig.savefig(result/'trajectory_comparison.png',dpi=170);plt.close(fig)

    settings=load_settings(OUT/'analysis_config.json')
    images=read(REVIEW/'frames.csv')
    by_base,by_model=defaultdict(list),defaultdict(list)
    for r in baseline: by_base[int(r['frame'])].append(r)
    for r in learned_in: by_model[int(r['frame'])].append(r)
    clouds=json.loads((RUN/'review_clouds.json').read_text())
    cloud_by_frame={r['frame']:r for r in clouds}
    fields=[]; panels=[]
    for request in np.arange(0,30,.5):
        imgrow=min(images,key=lambda r:abs(float(r['elapsed_s'])-request))
        image_stamp=int(imgrow['image_timestamp_ns'])
        frame=min(frames,key=lambda r:abs(int(r['timestamp_ns'])-image_stamp))
        fi=int(frame['frame'])
        image=cv2.imread(str(REVIEW/imgrow['image_file']))
        def project(pos):
            v=settings.lidar_to_zed_frame.transform_position(pos)
            if v[2]<=.05: return None
            return [float(camera['fx'])*v[0]/v[2]+float(camera['cx']),
                    float(camera['fy'])*v[1]/v[2]+float(camera['cy'])]
        def cross(pos,color,text):
            pix=project(pos)
            if pix is None or not (-100<pix[0]<1380 and -100<pix[1]<820): return
            px,py=map(lambda q:int(round(q)),pix)
            cv2.drawMarker(image,(px,py),color,cv2.MARKER_CROSS,18,2)
            cv2.putText(image,text,(px+10,py-7),cv2.FONT_HERSHEY_SIMPLEX,.45,color,1,cv2.LINE_AA)
        for r in by_base[fi]:
            if r['detected']!='1': continue
            if r['status']!='confirmed' and int(r['track_id']) not in (17,60): continue
            cross([float(r['level_x']),float(r['level_y']),float(r['level_z'])],
                  (205,85,20) if r['status']=='confirmed' else (10,155,235),
                  'R'+r['track_id']+(' C' if r['status']=='confirmed' else ' T'))
        for r in by_model[fi]:
            pos=np.array([float(r['level_x']),float(r['level_y']),float(r['level_z'])])
            cross(pos,(150,30,220),'PP'+r['track_id']+' '+r['score'][:4])
            # Learned box corners, no skeleton inferred.
            length,width,height=(float(r[k]) for k in ('length','width','height'))
            yaw=float(r['yaw']); c,s=np.cos(yaw),np.sin(yaw)
            verts=[]
            for z in [-height/2,height/2]:
                for x,y in [(-length/2,-width/2),(length/2,-width/2),(length/2,width/2),(-length/2,width/2)]:
                    verts.append(project(pos+np.array([c*x-s*y,s*x+c*y,z])))
            for a,b in [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]:
                if verts[a] and verts[b] and max(abs(v) for v in verts[a]+verts[b])<1e5:
                    cv2.line(image,tuple(map(int,verts[a])),tuple(map(int,verts[b])),(150,30,220),2)
        dt=(int(frame['timestamp_ns'])-image_stamp)/1e6
        cv2.putText(image,f'ZED {float(imgrow["elapsed_s"]):.2f}s | LiDAR frame {fi} dt={dt:+.1f}ms',
                    (16,28),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,240,240),2)
        encoded=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,78])[1].tobytes()
        if request in (4,8,12,16,20,24,28):
            (result/f'overlay_{int(request):02d}s.jpg').write_bytes(encoded)
        fields.append(dict(frame=fi,t=t(frame['timestamp_ns']),image_t=float(imgrow['elapsed_s']),
                           image='data:image/jpeg;base64,'+base64.b64encode(encoded).decode(),
                           points=cloud_by_frame[fi]['points'],baseline=by_base[fi],model=by_model[fi]))
        if int(round(request*2)) % 4==0:
            panels.append(cv2.resize(image,(640,360)))
    montage=np.vstack([np.hstack(panels[i:i+3]+[np.zeros_like(panels[0])]*(3-len(panels[i:i+3])))
                       for i in range(0,len(panels),3)])
    cv2.imwrite(str(result/'image_gallery.jpg'),montage)
    payload=json.dumps(fields,separators=(',',':'))
    html='''<!doctype html><html lang="ja"><meta charset="utf-8"><title>LiDAR学習済みモデル比較</title>
<style>body{font:16px system-ui;margin:24px auto;max-width:1200px;color:#18324a;background:#f5f7fa}img{width:100%}canvas{width:100%;background:white}button{padding:9px;font:inherit}input{width:100%}.note{padding:14px;background:#fff0cb}#details{white-space:pre-wrap}a{color:#1260a0}</style>
<h1>録画003：現在の方法とPointPillars</h1>
<p class="note">1人往復の保存データ。青＝現行の確定実検出、橙＝映像対応済み未確定候補、桃＝PointPillars人物出力。候補・予測を確定検出に数えません。点群と画像は元時計の近傍対応で、同期精度・絶対測位精度は未検証です。PointPillarsは骨格・歩幅を推定しません。</p>
<p><a href="../README.md">結果と制約</a> · <a href="metrics.json">指標JSON</a> · <a href="trajectory_comparison.png">軌跡グラフ</a></p>
<button id="play">▶ 再生</button><button id="prev">前へ</button><button id="next">次へ</button><span id="time"></span>
<input id="slider" type="range" min="0" value="20" step="1"><img id="image"><canvas id="top" width="1200" height="300"></canvas><p id="details"></p>
<script>const D=PAYLOAD;const $=x=>document.getElementById(x);let index=20,playing=false,last=0;const p=(x,y)=>[40+x*56,150-y*90];
function draw(){const d=D[index];$('slider').value=index;$('time').textContent=` 映像 ${d.image_t.toFixed(2)}秒 / LiDAR ${d.t.toFixed(2)}秒`;$('image').src=d.image;const c=$('top').getContext('2d');c.clearRect(0,0,1200,300);c.font='14px system-ui';for(let x=0;x<=20;x+=2){c.strokeStyle='#e3e9ef';c.beginPath();c.moveTo(...p(x,-1.3));c.lineTo(...p(x,1.3));c.stroke();c.fillStyle='#345';c.fillText(x+'m',p(x,0)[0]-8,290)}for(const y of [-1.2,1.2]){c.strokeStyle='#667';c.beginPath();c.moveTo(...p(0,y));c.lineTo(...p(20,y));c.stroke()}c.fillStyle='#aab4be';for(const q of d.points)c.fillRect(...p(q[0],q[1]),2,2);
for(const r of d.baseline){if(r.detected!=='1'||r.status!=='confirmed'&&![17,60].includes(+r.track_id))continue;const q=p(+r.corridor_x,+r.corridor_y);c.fillStyle=r.status==='confirmed'?'#1764c0':'#d58915';c.beginPath();c.arc(...q,6,0,Math.PI*2);c.fill();c.fillText('R'+r.track_id+' '+r.status,q[0]+8,q[1]-8)}for(const r of d.model){const q=p(+r.corridor_x,+r.corridor_y);c.strokeStyle='#d63274';c.lineWidth=3;c.strokeRect(q[0]-8,q[1]-8,16,16);c.fillStyle='#d63274';c.fillText('PP '+Number(r.score).toFixed(2),q[0]+10,q[1]+20)}$('details').textContent=`この時刻の廊下内PointPillars人物出力：${d.model.length}件。上面図のXは廊下奥行き、Yは横位置[m]。`;}
$('slider').max=D.length-1;$('slider').oninput=e=>{index=+e.target.value;draw()};$('prev').onclick=()=>{index=Math.max(0,index-1);draw()};$('next').onclick=()=>{index=Math.min(D.length-1,index+1);draw()};$('play').onclick=()=>{playing=!playing;$('play').textContent=playing?'Ⅱ 停止':'▶ 再生'};function tick(now){if(playing&&now-last>500){last=now;index=(index+1)%D.length;draw()}requestAnimationFrame(tick)}index=Math.min(D.length-1,Math.max(0,Number(new URLSearchParams(location.search).get('i')??20)));draw();requestAnimationFrame(tick);</script></html>'''
    (result/'index.html').write_text(html.replace('PAYLOAD',payload))
    print(json.dumps(metrics,ensure_ascii=False,indent=2))


if __name__=='__main__': main()
