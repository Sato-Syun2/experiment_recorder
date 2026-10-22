"""Build offline approach review from immutable exports and new geometry."""
from pathlib import Path
import base64
import csv
import hashlib
import html
import json
import sys
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from prepare import REVIEW, save

IDS = {9, 53}  # Image overlays reviewed separately; preserve original states.


def read(path):
    return list(csv.DictReader(path.open(encoding='utf-8-sig')))


def stats(values):
    a = np.array(values, float)
    return {} if not len(a) else dict(n=len(a), min=float(a.min()), median=float(np.median(a)),
        p90=float(np.percentile(a,90)), max=float(a.max()))


def main():
    if (OUT/'index.html').exists():
        raise FileExistsError('Choose fresh output')
    geo = json.loads((OUT/'geometry.json').read_text())
    cam = geo['camera']
    held = json.loads((OUT/'heldout_geometry.json').read_text())
    rows = read(OUT/'tracks.csv')
    ledger = read(OUT/'tracks.frames.csv')
    frames = [r for r in read(REVIEW/'frames.csv') if 7.15 <= float(r['elapsed_s']) <= 21.05]
    selected = [r for r in rows if int(r['track_id']) in IDS]
    obs = [r for r in selected if r['detected']=='1']
    conf = [r for r in obs if r['status']=='confirmed']
    t = np.array([float(r['time_sec']) for r in obs])
    xyz = np.array([[float(r['corridor_'+a]) for a in 'xyz'] for r in obs])
    dt = np.diff(t)
    same = np.array([a['track_id']==b['track_id'] for a,b in zip(obs[:-1],obs[1:])])
    adjacent = (dt<=.25) & same
    dx = np.diff(xyz[:,0])
    regression = np.polyfit(xyz[:,0], xyz[:,1], 1)
    fit_y = np.polyval(regression, xyz[:,0])
    residual = xyz[:,1]-fit_y
    metrics = dict(window_zed_s=[7.2,21.0], processed_approach_clouds=len(ledger),
        selected_ids=sorted(IDS), selected_observed=len(obs), confirmed_observed=len(conf),
        confirmed_frame_coverage=len(conf)/len(ledger), candidate_inclusive_frame_coverage=len(obs)/len(ledger),
        coverage_note='Broad chosen interval, not an exhaustive person-presence or surveyed distance gate. Candidates are not confirmed detections.',
        selected_x_m=stats(xyz[:,0]), selected_y_m=stats(xyz[:,1]), selected_z_m=stats(xyz[:,2]),
        confirmed_x_m=stats([float(r['corridor_x']) for r in conf]),
        first_observed=dict(t_s=float(t[0]),x_m=float(xyz[0,0])),last_observed=dict(t_s=float(t[-1]),x_m=float(xyz[-1,0])),
        adjacent_observation_pairs=int(adjacent.sum()), approaching_pair_fraction=float(np.mean(dx[adjacent]<0)),
        observed_time_intervals_s=stats(dt), maximum_interval_s=float(dt.max()),
        gaps_over_250ms=[dict(start_s=float(a),end_s=float(b),interval_s=float(b-a)) for a,b in zip(t[:-1],t[1:]) if b-a>.25],
        lateral_linear_fit=dict(slope_m_per_m=float(regression[0]),intercept_m=float(regression[1]),residual_abs_m=stats(abs(residual)),
            note='Describes this actual walking path; not a position-error estimate and not used to fit geometry.'),
        geometry_heldout=held, clock_offset_status='unestimated; raw absolute timestamps used',
        absolute_longitudinal_origin='sensor projection, not surveyed corridor end',
        zed_body_status='not exported; review RGB/depth is not body tracking',
        ground_truth_error_m=None)
    save('metrics.json',metrics)
    with (OUT/'selected_observations.csv').open('x',newline='') as stream:
        w=csv.DictWriter(stream,fieldnames=list(obs[0]));w.writeheader();w.writerows(obs)

    fig, axes=plt.subplots(3,1,figsize=(10,8),layout='constrained',height_ratios=[1,2,1.3])
    gap=geo['geometries']['lidar']['corridor']['wall_gap']
    axes[0].axhline(-gap/2,color='gray');axes[0].axhline(gap/2,color='gray')
    axes[0].axhline(0,color='gray',ls=':',lw=.7)
    for status,color,label in [('tentative','#d97706','Tentative observation'),('confirmed','#2563eb','Confirmed observation')]:
        a=[r for r in obs if r['status']==status]
        x=[float(r['corridor_x']) for r in a];y=[float(r['corridor_y']) for r in a]
        tt=[float(r['time_sec']) for r in a]
        axes[0].scatter(x,y,s=12,c=color,label=label)
        axes[1].scatter(tt,x,s=12,c=color)
        axes[2].scatter(tt,y,s=12,c=color)
    axes[0].set(xlim=(0,17),ylim=(-1.5,1.5),xlabel='Corridor X [m]',ylabel='Y [m]',title='Approach only: sensor-projected longitudinal origin')
    axes[0].set_aspect('equal',adjustable='box');axes[0].legend(loc='upper right',fontsize=8)
    axes[1].set(xlim=(7.2,21),ylim=(0,17),ylabel='Corridor X [m]',xlabel='ZED elapsed [s]')
    axes[2].set(xlim=(7.2,21),ylim=(-.5,.5),ylabel='Corridor Y [m]',xlabel='ZED elapsed [s]')
    for ax in axes:ax.grid(alpha=.2)
    fig.savefig(OUT/'trajectory.png',dpi=150);plt.close(fig)

    cloud=np.load(OUT/'approach_clouds.npz')
    stamps=cloud['stamps'];starts=cloud['starts'];points=cloud['points']
    Rz=np.array(geo['geometries']['zed']['rotation_raw_to_corridor'])
    tz=np.array(geo['geometries']['zed']['translation_raw_to_corridor'])
    # Independent sensor-projected longitudinal origins provisionally coincide.
    # The projection below is descriptive, not full 6DoF calibration validation.
    def project(c):
        z=(np.asarray(c)-tz)@Rz
        if z[2]<.2:return None
        p=(round(cam['fx']*z[0]/z[2]+cam['cx']),round(cam['fy']*z[1]/z[2]+cam['cy']))
        return p if max(abs(p[0]),abs(p[1]))<10000 else None

    assets=OUT/'review_frames';assets.mkdir(exist_ok=False)
    panels=[];by_frame={}
    for r in selected:by_frame.setdefault(int(r['frame']),[]).append(r)
    image_residuals=[]
    for i,fr in enumerate(frames):
        im=cv2.imread(str(REVIEW/fr['image_file']))
        for y in [-gap/2,0,gap/2]:
            for a,b in zip(np.arange(.5,18,.25),np.arange(.75,18.25,.25)):
                pa,pb=project([a,y,0]),project([b,y,0])
                if pa and pb:cv2.line(im,pa,pb,(180,170,80),1,cv2.LINE_AA)
        for x in [3,5,7,9,11,13,15]:
            pa,pb=project([x,-gap/2,0]),project([x,gap/2,0])
            if pa and pb:
                cv2.line(im,pa,pb,(180,170,80),1,cv2.LINE_AA)
                mid=project([x,0,0])
                if mid:cv2.putText(im,str(x)+'m',mid,cv2.FONT_HERSHEY_SIMPLEX,.4,(230,220,80),1,cv2.LINE_AA)
        j=int(np.argmin(abs(stamps-int(fr['image_timestamp_ns']))))
        residual_ms=(int(stamps[j])-int(fr['image_timestamp_ns']))/1e6
        valid=abs(residual_ms)<=65
        current=by_frame.get(int(ledger[j]['frame']),[]) if valid else []
        items=[]
        for r in current:
            if r['detected']!='1':continue
            cp=[float(r['corridor_'+a]) for a in 'xyz']
            confirmed=r['status']=='confirmed'
            color=(220,110,20) if confirmed else (0,160,245)
            uv=project(cp)
            if uv:
                cv2.drawMarker(im,uv,color,cv2.MARKER_CROSS,22,2)
                cv2.putText(im,f"ID{r['track_id']} {r['status']} X={cp[0]:.2f} Y={cp[1]:+.2f}",
                    (max(5,uv[0]-240),min(im.shape[0]-10,max(55,uv[1]-15))),cv2.FONT_HERSHEY_SIMPLEX,.55,color,2,cv2.LINE_AA)
            items.append(dict(id=int(r['track_id']),x=cp[0],y=cp[1],z=cp[2],confirmed=confirmed,points=int(r['point_count'])))
        cv2.rectangle(im,(0,0),(1280,31),(20,20,20),-1)
        cv2.putText(im,f"t={float(fr['elapsed_s']):.3f}s | clock UNVERIFIED | provisional X origin | nearest cloud {residual_ms:+.1f}ms",
            (8,22),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1,cv2.LINE_AA)
        cv2.imwrite(str(assets/f'{i:03d}.jpg'),im,[cv2.IMWRITE_JPEG_QUALITY,86])
        a=points[starts[j]:starts[j+1]] if valid else np.empty((0,3))
        a=a[(a[:,2]>.12)&(a[:,2]<2.2)][::4]
        panels.append(dict(t=float(fr['elapsed_s']),file=f'review_frames/{i:03d}.jpg',tracks=items,
            cloud=np.round(a[:,:2],3).tolist(),residual_ms=residual_ms if valid else None))
        if valid:image_residuals.append(residual_ms)
    save('image_clock_residuals.json',dict(residual_ms=stats(image_residuals),
        note='Nearest raw timestamp residual; not a synchronization-accuracy measurement.'))
    montage=np.full((3*300,3*640,3),248,np.uint8)
    for i,target in enumerate([7.8,9,10.5,12,13.4,14.8,16.4,18,19.6]):
        j=min(range(len(panels)),key=lambda j:abs(panels[j]['t']-target))
        im=cv2.imread(str(assets/f'{j:03d}.jpg'))
        # Show the whole frame, retaining the floor grid and the person marker.
        tile=cv2.resize(im,(640,300));x=i%3*640;y=i//3*300
        montage[y:y+300,x:x+640]=tile
    cv2.imwrite(str(OUT/'correspondence_review.jpg'),montage)
    data=dict(frames=panels,path=[dict(t=float(r['time_sec']),id=int(r['track_id']),x=float(r['corridor_x']),
            y=float(r['corridor_y']),confirmed=r['status']=='confirmed') for r in obs],wall_gap=gap)
    page=(OUT/'review_template.html').read_text()
    page=page.replace('__DATA__',json.dumps(data,ensure_ascii=False,allow_nan=False,separators=(',',':')))
    page=page.replace('__METRICS__',f"確定実検出 {len(conf)}/{len(ledger)} フレーム（{100*len(conf)/len(ledger):.1f}%）、候補を含む実観測 {len(obs)}/{len(ledger)}（{100*len(obs)/len(ledger):.1f}%）。")
    (OUT/'index.html').write_text(page,encoding='utf-8')
    save('report_manifest.json',dict(source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [Path(__file__),OUT/'review_template.html',OUT/'geometry.json',OUT/'tracks.csv',OUT/'heldout_geometry.json']},
        note='No SDK or hardware used. No original files overwritten. No unit tests requested or run.'))
    print(json.dumps({k:v for k,v in metrics.items() if k!='geometry_heldout'},ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
