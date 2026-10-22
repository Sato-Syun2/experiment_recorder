"""Review completed SDK 5.5.1 cases without changing SDK outputs/calibration."""
from pathlib import Path
import collections,csv,hashlib,json,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent
PACKAGE=OUT.parent
ROOT=PACKAGE.parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from compare_body_models import stats

RUN=Path('/data/experiment_data/20261022_sdk551_body_model_comparison/runs/20261022_160153_425608')
MODELS=['medium_gen2','accurate_gen2','medium_gen1','accurate_gen1','fast_gen1']
COLORS=['#16815f','#2355c7','#d27718','#9a47ad','#8b862b']


def read(p):return list(csv.DictReader(p.open(encoding='utf-8-sig',newline='')))


def save(name,data):
    with (OUT/name).open('x',encoding='utf-8') as f:
        json.dump(data,f,ensure_ascii=False,indent=2,allow_nan=False)
        f.write('\n')


def main():
    if (OUT/'review_metrics.json').exists():raise FileExistsError('Choose a fresh review output')
    completed=json.loads((RUN/'run_complete.json').read_text())
    if completed['completed_cases']!=15 or completed['failed_cases']!=0:raise ValueError('Expected all 15 completed cases')
    cases=json.loads((RUN/'case_results.json').read_text())['cases']
    compilation=json.loads((RUN/'comparison.json').read_text())
    plan=json.loads((PACKAGE/'comparison_plan.json').read_text())
    geometry=json.loads((PACKAGE/'geometry_002.json').read_text());R=np.array(geometry['rotation_raw_to_corridor']);tr=np.array(geometry['translation_raw_to_corridor'])
    far=[];common_support=[];raw_by_model={};settings=[]
    fig,axs=plt.subplots(3,1,figsize=(12,10),layout='constrained')
    for model,color in zip(MODELS,COLORS):
        body=RUN/'recording_002/approach'/model
        camera=json.loads((body/'camera.json').read_text());frames=read(body/'frames.csv');rows=read(body/'bodies.csv')
        stamps=[int(r['image_timestamp_ns']) for r in frames]
        expected=json.loads((RUN/'recording_002/window_approach.json').read_text())['expected_image_timestamps_ns']
        if stamps!=expected:raise ValueError('Model timestamps differ')
        ok=[r for r in rows if r['tracking_state']=='OK']
        times=np.array([float(r['elapsed_s']) for r in ok]);joints=np.array([json.loads(r['keypoints_3d_camera_json']) for r in ok],float)
        confidence=np.array([json.loads(r['keypoint_confidence_json']) for r in ok],float);pixels=np.array([json.loads(r['keypoints_2d_json']) for r in ok],float)
        c=joints@R.T+tr
        inside=(pixels[:,:,0]>=0)&(pixels[:,:,0]<1280)&(pixels[:,:,1]>=0)&(pixels[:,:,1]<720)&np.isfinite(pixels).all(axis=2)
        support=(confidence[:,[32,33]]>=25)&inside[:,[32,33]]&np.isfinite(c[:,[32,33]]).all(axis=2)
        isfar=(times>=7.199002)&(times<=10.5);ref=(times>=17)&(times<=19)
        reference=np.median(np.linalg.norm(joints[ref,19]-joints[ref,20],axis=1)) if ref.any() else None
        ratio=np.linalg.norm(joints[:,19]-joints[:,20],axis=1)/reference if reference else np.full(len(times),np.nan)
        legs=(confidence[:,[18,19,20,22,23,24]]>=25).all(axis=1)&inside[:,[18,19,20,22,23,24]].all(axis=1)
        far_ledger=[r for r in frames if 7.199002<=float(r['elapsed_s'])<=10.5]
        far.append(dict(model=model,far_frame_count=len(far_ledger),far_OK_frames=int(isfar.sum()),
            far_both_legs_conf25_inside_frames=int((isfar&legs).sum()),
            far_heel_z_m=stats(c[isfar][:,[32,33],2][support[isfar]]),
            far_shin_ratio=stats(ratio[isfar]),note='Same declared time interval, roughly far approach. Not a surveyed true-distance gate.'))
        sample={int(r['image_timestamp_ns']):c[i,[32,33],2] for i,r in enumerate(ok) if isfar[i] and support[i].all()}
        raw_by_model[model]=sample
        if model in ['medium_gen2','accurate_gen2','accurate_gen1']:common_support.append(set(sample))
        for i in [32,33]:
            s=(confidence[:,i]>=25)&inside[:,i]
            axs[0].scatter(times[s],c[s,i,2],s=3,c=color,alpha=.45,label=model if i==32 else None)
        axs[1].plot(times,ratio,color=color,lw=1,label=model)
        axs[2].plot(times,c[:,0,0],color=color,lw=1,label=model)
    for ax in axs:ax.grid(alpha=.2);ax.set_xlim(7.199002,21.0)
    axs[0].axhline(0,c='black',ls='--',lw=.7);axs[0].set(ylabel='Supported heel height [m]',title='Recording002: fixed same floor candidate for all models');axs[0].legend(fontsize=8,ncol=3)
    axs[1].axhline(1,c='gray',ls=':');axs[1].axvspan(17,19,color='#d9e7dc',alpha=.5);axs[1].set(ylabel='Shin / own near median')
    axs[2].set(xlabel='Original ZED elapsed [s]',ylabel='Pelvis corridor X [m]')
    fig.savefig(OUT/'recording002_far_comparison.png',dpi=140);plt.close(fig)
    common=sorted(set.intersection(*common_support))
    same_frame={model:stats(np.concatenate([raw_by_model[model][t] for t in common])) for model in ['medium_gen2','accurate_gen2','accurate_gen1']}
    all_id_full=[]
    for model in MODELS:
        body=RUN/'recording_003/full'/model;frames=read(body/'frames.csv');rows=read(body/'bodies.csv')
        selected=[r for r in rows if r['tracking_state']=='OK' and 8.0<=float(r['elapsed_s'])<=20.5]
        window=[r for r in frames if 8.0<=float(r['elapsed_s'])<=20.5]
        good_times=set();by_t=collections.defaultdict(list)
        for r in selected:
            by_t[r['image_timestamp_ns']].append(r)
            conf=np.array(json.loads(r['keypoint_confidence_json']),float);uv=np.array(json.loads(r['keypoints_2d_json']),float)
            jj=np.array(json.loads(r['keypoints_3d_camera_json']),float);ids=[18,19,20,22,23,24]
            if (conf[ids]>=25).all() and np.isfinite(jj[ids]).all() and (uv[ids,0]>=0).all() and (uv[ids,0]<1280).all() and (uv[ids,1]>=0).all() and (uv[ids,1]<720).all():good_times.add(r['image_timestamp_ns'])
        all_id_full.append(dict(model=model,interval_s=[8.0,20.5],frames=len(window),OK_frames=len(by_t),
            both_legs_conf25_inside_frames_any_id=len(good_times),ambiguous_multiple_OK_frames=sum(len(v)>1 for v in by_t.values()),
            ids=dict(collections.Counter(r['body_id'] for r in selected)),
            note='Broad image-reviewed walking interval. Union over SDK IDs, counting each time once; not a pose accuracy or exhaustive occlusion label.'))
    sources=[]
    for case in cases:
        body=RUN/Path(case['body_dir'].replace('\\','/'));window=RUN/Path(case['window'].replace('\\','/'))
        camera=json.loads((body/'camera.json').read_text());frame_rows=read(body/'frames.csv');body_rows=read(body/'bodies.csv');w=json.loads(window.read_text())
        if camera['sdk_version']!='5.5.1' or camera['body_tracking_model']!=case['configuration']['body_model'] or camera['body_model_generation_effective']!=case['configuration']['generation'] or camera['depth_mode']!='NEURAL':raise ValueError('Effective setting differs')
        if [int(r['image_timestamp_ns']) for r in frame_rows]!=w['expected_image_timestamps_ns']:raise ValueError('Timestamp mismatch')
        counts=collections.Counter(int(r['image_timestamp_ns']) for r in body_rows)
        if any(counts[int(r['image_timestamp_ns'])]!=int(r['body_count']) for r in frame_rows):raise ValueError('Body ledger count mismatch')
        settings.append(dict(recording=case['recording'],scope=case['scope'],model_id=case['configuration']['id'],frames=len(frame_rows),
            runtime_sdk=camera['sdk_version'],generation=camera['body_model_generation_effective'],source_svo=camera['source_svo']))
        sources.extend([body/'camera.json',body/'frames.csv',body/'bodies.csv'])
    old=read(PACKAGE/'historical/recording002_sdk/bodies.csv');new=read(RUN/'recording_002/approach/accurate_gen1/bodies.csv')
    if [(r['image_timestamp_ns'],r['body_id']) for r in old]!=[(r['image_timestamp_ns'],r['body_id']) for r in new]:raise ValueError('Historical keys differ')
    oldj=np.array([json.loads(r['keypoints_3d_camera_json']) for r in old],float);newj=np.array([json.loads(r['keypoints_3d_camera_json']) for r in new],float)
    result=dict(source_run=str(RUN),completed=completed,effective_settings_and_counts=settings,
        recording002_far_window=far,common_far_heel_frames=len(common),common_supported_far_heel_z_m=same_frame,
        recording003_full_common_image_review_window=all_id_full,
        historical_vs_current_accurate_gen1=dict(joint_rows=len(old),joint_max_abs_difference_m=float(np.nanmax(abs(oldj-newj)))),
        recommendation=dict(far_range_first_candidate='accurate_gen2',moderate_range_speed_candidate='medium_gen2',
            scope='These single-person recordings only. Relative skeleton stability and runtime, not independent absolute joint accuracy.'),
        caveats=['Recording003 full medium_gen2 switches IDs 0 to 2; do not interpret dominant-ID-only quality as whole-walk quality.',
            'Early recording003 contains a partly visible head near the camera; high full-run SDK OK is not evidence of pole false detections or a human recall rate.',
            'Recording003 ZED floor calibration remains unverified. No recording002 floor transform applied to recording003.',
            'Recording002 fixed floor is a candidate; accurate_gen2 still has an early heel minimum near -0.38m. Not all joints are validated.',
            'LiDAR recording003 had no confirmed detections with current settings; SDK success does not establish LiDAR success.'])
    save('review_metrics.json',result)
    save('manifest.json',dict(source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),RUN/'run_start.json',RUN/'run_complete.json',RUN/'case_results.json',RUN/'comparison.json',PACKAGE/'geometry_002.json',*sources]},
        operation='Offline analysis of completed SDK outputs. No SDK/hardware rerun, no calibration/coordinate edits, no unit tests requested or run.'))
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
