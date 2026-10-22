"""Recording-specific pole axis / tip calibration, in metres and rectified pixels.

Manual tip rows were read from magnified, background-difference review images.
The radius correction is approximate; image and point uncertainty is reported.
"""
from pathlib import Path
import sys,json,csv,hashlib
import numpy as np
import cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
prep=json.loads((OUT/'preparation.json').read_text())
rev=Path(prep['inputs']['2']['review_dir']);cam=prep['inputs']['2']['camera']
rows=list(csv.DictReader((rev/'frames.csv').open()))
a=np.load(OUT/'central_cloud_2.npz');pts=a['points'];starts=a['starts'];times=a['times']
level=np.array(prep['level_rotation']);shift=np.array(prep['level_translation'])
corr=prep['corridor_initial'];axes=np.array([corr['longitudinal_xy'],corr['lateral_xy']]);origin=np.array(corr['origin_xy'])
up_cam=np.array(prep['inputs']['2']['zed_up_camera'])
up_raw=np.array(prep['inputs']['2']['lidar']['imu_up'])
initial=json.loads((ROOT/'tracking/lidar_to_zed_left_eye.json').read_text())
R0=np.array(initial['rotation_source_to_target']);t0=np.array(initial['translation_m'])
fx,fy,cx,cy=[cam[k] for k in ['fx','fy','cx','cy']]
bg=np.median([cv2.imread(str(rev/r['image_file'])).astype(float) for r in rows[:10]],axis=0)
spec=[(3,50,646,396.5),(5,100,638,321),(7,150,638,286),(9,200,633,266),
      (11,257,630,253),(13,306,631,245),(15,356,631,238)]
obs=[]
for nominal,s,x,tip_y in spec:
    t=float(rows[s]['elapsed_s']);idx=np.flatnonzero(abs(times-t)<=1.5)
    q=np.concatenate([pts[starts[i]:starts[i+1]] for i in idx])
    q=q[(q[:,2]>.15)&(q[:,2]<.82)&(abs(q[:,1])<.45)]
    counts,edges=np.histogram(q[:,0],bins=np.arange(0,19,.1));peak=edges[np.argmax(counts)]+.05
    q=q[abs(q[:,0]-peak)<.16];my=np.median(q[:,1]);q=q[abs(q[:,1]-my)<.12]
    med=np.median(q[:,:2],axis=0)
    # Front-visible cylindrical surface to axis: 25 mm along horizontal viewing ray.
    pole_xy=med+.025*med/np.linalg.norm(med)
    pole_level_xy=pole_xy@axes+origin
    foot_level=np.r_[pole_level_xy,0.]
    foot_raw=(foot_level-shift)@level
    # Pole upright along current LiDAR gravity; intersection of base with fitted floor.
    top_raw=foot_raw+.863*up_raw
    im=np.median([cv2.imread(str(rev/rows[j]['image_file'])).astype(float) for j in range(s-3,s+4)],axis=0)
    diff=np.mean(abs(im-bg),axis=2)
    def center_x(y1,y2):
        val=diff[y1:y2,x-6:x+7]
        w=np.maximum(val-8,0)**2
        return float(((np.arange(x-6,x+7)+.0)*w.sum(axis=0)).sum()/w.sum())
    tip_x=center_x(int(tip_y)+1,int(tip_y)+6)
    # Lower stem point used only as a line constraint, not a known-height point.
    lower_y=int(tip_y)+int(max(12,220/nominal))
    lower_x=center_x(lower_y-2,lower_y+3)
    obs.append(dict(nominal_distance_m=nominal,sample_index=s,elapsed_s=t,window_s=[t-1.5,t+1.5],
                    points=len(q),lidar_xy_iqr_m=(np.percentile(q[:,:2],75,axis=0)-np.percentile(q[:,:2],25,axis=0)).tolist(),
                    corridor_before_origin=pole_xy.tolist(),foot_raw=foot_raw.tolist(),top_raw=top_raw.tolist(),
                    tip_pixel=[tip_x,tip_y],lower_line_pixel=[lower_x,lower_y]))
    print('pole',obs[-1],flush=True)
P=np.array([o['top_raw'] for o in obs]);F=np.array([o['foot_raw'] for o in obs]);uv=np.array([o['tip_pixel'] for o in obs]);low=np.array([o['lower_line_pixel'] for o in obs])
def project(p,R,t):
    v=p@R.T+t
    return np.column_stack([fx*v[:,0]/v[:,2]+cx,fy*v[:,1]/v[:,2]+cy])
def residual(v,ids,tip_pixels=uv,gravity_sigma_deg=.5):
    R=Rotation.from_rotvec(v[:3]).as_matrix();t=v[3:]
    top=project(P[ids],R,t);bottom=project(F[ids],R,t)
    direction=bottom-top
    off=low[ids]-top
    dist=(direction[:,0]*off[:,1]-direction[:,1]*off[:,0])/np.linalg.norm(direction,axis=1)
    # 1 pixel manual tip/line localization scale, 0.5 degree gravity prior scale.
    gravity=np.cross(R@up_raw,up_cam)/np.deg2rad(gravity_sigma_deg)
    return np.r_[(top-tip_pixels[ids]).ravel(),dist,gravity]
v0=np.r_[Rotation.from_matrix(R0).as_rotvec(),t0]
def solve(ids,tip_pixels=uv,gravity_sigma_deg=.5):
    return least_squares(residual,v0,args=(ids,tip_pixels,gravity_sigma_deg),loss='soft_l1',f_scale=1,max_nfev=1500)
sol=solve(np.arange(7));R=Rotation.from_rotvec(sol.x[:3]).as_matrix();t=sol.x[3:]
pred=project(P,R,t);errors=np.linalg.norm(pred-uv,axis=1)
old_errors=np.linalg.norm(project(P,R0,t0)-uv,axis=1)
hold=[]
for i in range(7):
    fit=solve(np.array([j for j in range(7) if j!=i]));rr=Rotation.from_rotvec(fit.x[:3]).as_matrix()
    e=project(P[i:i+1],rr,fit.x[3:])[0]-uv[i]
    hold.append(dict(heldout_distance_m=obs[i]['nominal_distance_m'],error_px=float(np.linalg.norm(e)),
                     translation_m=fit.x[3:].tolist()))
# Sensitivity is not a statistical accuracy confidence interval.
sens=[]
for pix in [-1.,1.]:
    perturb=uv.copy();perturb[:,1]+=pix
    s=solve(np.arange(7),perturb);sens.append(dict(kind=f'all_tip_rows_{pix:+.0f}px',translation_m=s.x[3:].tolist(),
        rotation_change_deg=float(np.rad2deg((Rotation.from_rotvec(s.x[:3])*Rotation.from_matrix(R).inv()).magnitude()))))
for sigma in [.2,1.]:
    s=solve(np.arange(7),gravity_sigma_deg=sigma);sens.append(dict(kind=f'gravity_sigma_{sigma}deg',translation_m=s.x[3:].tolist(),
        rotation_change_deg=float(np.rad2deg((Rotation.from_rotvec(s.x[:3])*Rotation.from_matrix(R).inv()).magnitude()))))
x=np.array([o['corridor_before_origin'][0] for o in obs]);nom=np.array([o['nominal_distance_m'] for o in obs]);offset=float(np.median(x-nom))
candidate={'version':1,'source_frame':'livox_frame','target_frame':'zed_left_eye','coordinate_system':'IMAGE','units':'m',
           'calibration_status':'pole_image_lidar_gravity_candidate','rotation_source_to_target':R.tolist(),'translation_m':t.tolist(),
           'method':'Seven stationary pole tips and vertical image axes, known height 0.863m, LiDAR front-surface radius correction 0.025m, IMU direction prior.',
           'scope':'20261021_group002 recording_002; validated separately against recording_003',
           'notes':'Conditional on upright pole, nominal height, manually annotated tip rows and approximate cylinder-axis estimates. No guarantee of absolute positioning accuracy.'}
result={'observations':obs,'candidate':candidate,'initial_transform':initial,'optimizer_success':bool(sol.success),
        'tip_errors_px':errors.tolist(),'initial_tip_errors_px':old_errors.tolist(),'leave_one_position_out':hold,
        'sensitivity':sens,'gravity_residual_deg':float(np.rad2deg(np.arccos(np.clip((R@up_raw)@up_cam,-1,1)))),
        'tip_rmse_px':float(np.sqrt(np.mean(errors**2))),'initial_tip_rmse_px':float(np.sqrt(np.mean(old_errors**2))),
        'jacobian_singular_values':np.linalg.svd(sol.jac,compute_uv=False).tolist(),
        'corridor_longitudinal_origin_offset_m':offset,'pole_distance_residual_m':(x-offset-nom).tolist(),
        'settings':{'pole_height_m':.863,'pole_radius_m':.025,'gravity_sigma_deg':.5,'tip_sigma_px':1.,'pose_window_half_s':1.5},
        'timestamp_note':'Static ±1.5s windows use raw absolute timestamps. Clock offset has not been estimated; membership stability must be checked.'}
with (OUT/'pole_fit.json').open('x') as f:json.dump(result,f,indent=2,ensure_ascii=False)
with (OUT/'candidate.json').open('x') as f:json.dump(candidate,f,indent=2,ensure_ascii=False)
print(json.dumps({k:v for k,v in result.items() if k not in ['observations','candidate','initial_transform']},indent=2),flush=True)
# Reprojection overlay for every stationary pose, original vs candidate.
canvas=np.zeros((450,7*200,3),np.uint8)
for i,o in enumerate(obs):
    im=cv2.imread(str(rev/rows[o['sample_index']]['image_file']))
    for RR,tt,col in [(R0,t0,(0,0,255)),(R,t,(0,255,0))]:
        pp=project(np.stack([P[i],F[i]]),RR,tt)
        cv2.line(im,tuple(np.round(pp[0]).astype(int)),tuple(np.round(pp[1]).astype(int)),col,1)
        cv2.circle(im,tuple(np.round(pp[0]).astype(int)),3,col,1)
    cv2.circle(im,tuple(np.round(uv[i]).astype(int)),2,(255,255,0),1)
    canvas[:400,i*200:(i+1)*200]=im[150:550,550:750]
    cv2.putText(canvas,f"{o['nominal_distance_m']}m / {errors[i]:.1f}px",(i*200+5,428),0,.5,(255,255,255),1)
cv2.imwrite(str(OUT/'pole_projection_comparison.jpg'),canvas)
