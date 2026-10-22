"""Recording-specific offline preparation. No live devices or original edits."""
from pathlib import Path
import sys, csv, json, hashlib
import numpy as np
from scipy.spatial.transform import Rotation

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from track_people import _iter_bag_clouds, _pointcloud2_to_numpy, _message_timestamp_ns, _rotation_from_up_vector
from audit_recordings import audit
from calibrate_zed_lidar_planes import _fit_plane, _angle_between
from corridor_frame import estimate_corridor_frame

DATA=Path('/data/experiment_data')
REV={2:DATA/'20261021_group002_recording_002_pole_review',3:DATA/'20261021_group002_recording_003_walk_review'}
BAG={i:DATA/f'recordings/20261021_group002/recording_{i:03d}/lidar' for i in [2,3]}
cfg=json.loads((ROOT/'config.json').read_text())
initial_up=np.array(cfg['tracking']['gravity_alignment']['up_vector']);initial_up/=np.linalg.norm(initial_up)
result={'inputs':{},'seed':20261021,'time_offset_status':'not yet estimated; raw timestamps used for discovery'}
for i in [2,3]:
    cam=json.loads((REV[i]/'camera.json').read_text())
    imu=list(csv.DictReader((REV[i]/'imu.csv').open()))
    acc=np.array([[float(r['accel_'+k+'_m_s2']) for k in 'xyz'] for r in imu])
    up=Rotation.from_quat(cam['camera_imu_rotation_imu_to_left_camera_xyzw']).apply(acc.mean(axis=0));up/=np.linalg.norm(up)
    result['inputs'][str(i)]={'review_dir':str(REV[i]),'camera':cam,'zed_up_camera':up.tolist(),
        'zed_acceleration_std':acc.std(axis=0).tolist(),'lidar':audit(BAG[i],initial_up)}
print('IMU read',flush=True)
up=np.array(result['inputs']['2']['lidar']['imu_up'])
parts=[]
first_ns=result['inputs']['2']['camera']['first_image_timestamp_ns']
for msg,fb,_ in _iter_bag_clouds(BAG[2],'/livox/lidar','pointcloud2','mcap'):
    t=(_message_timestamp_ns(msg,fb)-first_ns)/1e9
    if t>2.5: break
    if t<0: continue
    p=_pointcloud2_to_numpy(msg);p=p[np.isfinite(p).all(axis=1)]
    p=p[(np.linalg.norm(p,axis=1)<20)&(p[:,0]>0)]
    parts.append(p)
p=np.concatenate(parts).astype(float)
rng=np.random.default_rng(20261021)
if len(p)>60000:p=p[rng.choice(len(p),60000,replace=False)]
floor=_fit_plane(p,up,2.1,.6,angle_tolerance_deg=5,point_tolerance_m=.025,iterations=1400,seed=20261021)
level=_rotation_from_up_vector(floor['normal']);shift=np.array([0,0,floor['d_m']])
lp=p@level.T+shift
corr=estimate_corridor_frame(lp,x_min=.5,x_max=18,y_min=-3,y_max=3,z_min=.3,z_max=2.3,
                            voxel_size=.08,line_tolerance=.06,ransac_iterations=1800,seed=20261021)
cd=corr.to_dict();cd['source_frame']='livox_level_frame'
result.update(floor=floor,level_rotation=level.tolist(),level_translation=shift.tolist(),corridor_initial=cd,
              floor_vs_imu_deg=_angle_between(up,np.array(floor['normal'])))
result['installation_up_change_deg']={
    'lidar':_angle_between(np.array(result['inputs']['2']['lidar']['imu_up']),np.array(result['inputs']['3']['lidar']['imu_up'])),
    'zed':_angle_between(np.array(result['inputs']['2']['zed_up_camera']),np.array(result['inputs']['3']['zed_up_camera']))}
np.savez_compressed(OUT/'static_geometry.npz',raw=p,level=lp)
print('floor',floor,'corridor',cd,flush=True)
for i in [2,3]:
    first=result['inputs'][str(i)]['camera']['first_image_timestamp_ns']
    chunks=[];starts=[0];times=[];stamps=[]
    for msg,fb,_ in _iter_bag_clouds(BAG[i],'/livox/lidar','pointcloud2','mcap'):
        stamp=_message_timestamp_ns(msg,fb)
        p=_pointcloud2_to_numpy(msg);p=p[np.isfinite(p).all(axis=1)]
        lev=p@level.T+shift
        xy=lev[:,:2]-corr.origin_xy
        c=np.column_stack([xy@corr.longitudinal_xy,xy@corr.lateral_xy,lev[:,2]])
        keep=(c[:,0]>.3)&(c[:,0]<19)&(abs(c[:,1])<1.1)&(c[:,2]>.06)&(c[:,2]<2.3)
        chunks.append(c[keep].astype(np.float32));starts.append(starts[-1]+int(keep.sum()));times.append((stamp-first)/1e9);stamps.append(stamp)
    np.savez_compressed(OUT/f'central_cloud_{i}.npz',points=np.concatenate(chunks),starts=starts,times=times,stamps=stamps)
    print('cached',i,starts[-1],flush=True)
result['code_sha256']={n:hashlib.sha256((ROOT/'scripts'/n).read_bytes()).hexdigest() for n in ['track_people.py','audit_recordings.py','corridor_frame.py','calibrate_zed_lidar_planes.py']}
with (OUT/'preparation.json').open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2)
