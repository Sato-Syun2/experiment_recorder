"""LiDAR-only inspection; preserve tentative states and unverified semantics."""
from pathlib import Path
import csv,hashlib,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm

OUT=Path(__file__).resolve().parent


def read(path):
    return list(csv.DictReader(path.open(encoding='utf-8-sig',newline='')))


def main():
    if (OUT/'metrics_preliminary.json').exists():raise FileExistsError('Choose fresh output')
    rows=read(OUT/'tracks.csv');ledger=read(OUT/'tracks.frames.csv')
    geo=json.loads((OUT/'geometry.json').read_text())
    archive=np.load(OUT/'corridor_clouds.npz');points,starts,stamps=archive['points'],archive['starts'],archive['stamps']
    times=(stamps-stamps[0])/1e9
    diagnostics=[]
    for tid in sorted({int(r['track_id']) for r in rows}):
        seq=[r for r in rows if int(r['track_id'])==tid and r['detected']=='1']
        if not seq:continue
        run,best,previous=0,0,None
        for r in seq:
            f=int(r['frame']);run=run+1 if previous==f-1 else 1;best=max(best,run);previous=f
        xy=np.array([[float(r['corridor_x']),float(r['corridor_y'])] for r in seq])
        diagnostics.append(dict(id=tid,observed=len(seq),confirmed=sum(r['status']=='confirmed' for r in seq),
            first_s=float(seq[0]['time_sec']),last_s=float(seq[-1]['time_sec']),
            x_min_m=float(xy[:,0].min()),x_max_m=float(xy[:,0].max()),y_min_m=float(xy[:,1].min()),y_max_m=float(xy[:,1].max()),
            max_consecutive_observed=best,points_median=float(np.median([int(r['point_count']) for r in seq]))))
    with (OUT/'candidate_diagnostics.csv').open('x',newline='') as stream:
        w=csv.DictWriter(stream,fieldnames=list(diagnostics[0]));w.writeheader();w.writerows(diagnostics)
    edges=np.arange(0,19.01,.1);bands=[]
    for lo,hi in [(.15,.95),(.95,2.2)]:
        counts=[]
        for i in range(len(stamps)):
            q=points[starts[i]:starts[i+1]];q=q[(abs(q[:,1])<.65)&(q[:,2]>=lo)&(q[:,2]<hi)]
            counts.append(np.histogram(q[:,0],edges)[0])
        bands.append(np.array(counts))
    fig,axs=plt.subplots(3,1,figsize=(12,10),layout='constrained')
    for ax,counts,title in zip(axs[:2],bands,['Central points: height 0.15--0.95m','Central points: height 0.95--2.2m']):
        im=ax.imshow(counts.T,origin='lower',extent=[times[0],times[-1],0,19],aspect='auto',norm=LogNorm(vmin=1,vmax=max(2,int(np.max(counts)))),cmap='magma')
        ax.set(xlabel='LiDAR elapsed [s]',ylabel='Corridor X [m]',title=title);fig.colorbar(im,ax=ax,label='points / 0.1m bin')
    observed=[r for r in rows if r['detected']=='1' and abs(float(r['corridor_y']))<1.1 and .3<float(r['corridor_x'])<12]
    axs[2].scatter([float(r['time_sec']) for r in observed],[float(r['corridor_x']) for r in observed],s=7,c='#d28b28',alpha=.65)
    axs[2].set(xlabel='LiDAR elapsed [s]',ylabel='Corridor X [m]',title='All central tentative observations (NOT confirmed people)',ylim=(0,12),xlim=(0,times[-1]));axs[2].grid(alpha=.2)
    fig.savefig(OUT/'lidar_preliminary.png',dpi=140);plt.close(fig)
    byframe={}
    for r in observed:
        byframe.setdefault(int(r['frame']),[]).append(dict(id=int(r['track_id']),x=float(r['corridor_x']),
            y=float(r['corridor_y']),z=float(r['corridor_z']),points=int(r['point_count'])))
    panels=[]
    for i,t in enumerate(times):
        q=points[starts[i]:starts[i+1]];q=q[(q[:,2]>.12)&(q[:,2]<2.2)][::25]
        panels.append(dict(t=float(t),cloud=np.round(q[:,:2],3).tolist(),tracks=byframe.get(i+1,[])))
    cfg=json.loads((OUT/'analysis_config.json').read_text())
    result=dict(status='lidar_only_preliminary_RGB_and_SDK_not_yet_exported',processed_frames=len(ledger),
        confirmed_detection_frames=sum(int(r['confirmed_detections'])>0 for r in ledger),
        confirmed_observed_rows=sum(r['status']=='confirmed' and r['detected']=='1' for r in rows),
        total_ids_created=len(diagnostics),unverified_stationary_x_peaks_m=[2.875,4.825],
        moving_cloud_note='High-band raw points trace an outward/return path to roughly 9m. No visual human/pole identity or occlusion label yet.',
        principal_candidate_ids_for_review=[8,38,61,64,100,109],
        principal_candidate_diagnostics=[r for r in diagnostics if r['id'] in [8,38,61,64,100,109]],
        confirmation_motion_window=cfg['tracking']['motion_consistency_window'],
        installation_change=geo['installation_change_from_recording002'],
        comparison_note='No empty/no-pole matched recording or independent visibility labels; cannot attribute failure causally to poles.',
        hardware='Offline ROS bag reading only; no devices, ROS nodes or RViz started.',tests='No unit tests requested or run.')
    with (OUT/'metrics_preliminary.json').open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
    page=(OUT/'preliminary_template.html').read_text().replace('__DATA__',json.dumps(dict(frames=panels,gap=geo['corridor']['wall_gap']),separators=(',',':'),allow_nan=False))
    (OUT/'index_preliminary.html').write_text(page)
    with (OUT/'preliminary_manifest.json').open('x') as f:
        json.dump(dict(source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),OUT/'tracks.csv',OUT/'tracks.frames.csv',OUT/'geometry.json',OUT/'analysis_config.json']}),f,indent=2)
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
