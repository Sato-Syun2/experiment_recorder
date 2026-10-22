"""Offline recording003 floor/wall and occupancy preparation; no hardware."""
from pathlib import Path
import hashlib,json,sys
import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
BASE=OUT.parent/'20261022_group003_recording_002_approach_check'
BAG=Path('/data/experiment_data/recordings/20261022_group003/recording_003/lidar')
sys.path.insert(0,str(ROOT/'scripts'))
from audit_recordings import audit
from track_people import _iter_bag_clouds,_pointcloud2_to_numpy,_message_timestamp_ns,_rotation_from_up_vector
from calibrate_zed_lidar_planes import _fit_plane,_angle_between
from corridor_frame import estimate_corridor_frame


def save(name,value):
    with (OUT/name).open('x') as f:json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False)


def main():
    if (OUT/'geometry.json').exists():raise FileExistsError('Choose fresh output')
    previous=json.loads((BASE/'geometry.json').read_text())
    prev=previous['geometries']['lidar']
    up_ref=np.array(previous['lidar_audit']['imu_up'])
    aud=audit(BAG,up_ref);up=np.array(aud['imu_up'])
    parts=[]
    for i,(msg,fb,_) in enumerate(_iter_bag_clouds(BAG,'/livox/lidar','pointcloud2','mcap')):
        if i>=30:break
        p=_pointcloud2_to_numpy(msg);keep=np.isfinite(p).all(axis=1)&(p[:,0]>0)&(np.linalg.norm(p,axis=1)<20)
        parts.append(p[keep])
    p=np.concatenate(parts).astype(float);rng=np.random.default_rng(20261023)
    if len(p)>60000:p=p[rng.choice(len(p),60000,replace=False)]
    heights=p@up;counts,edges=np.histogram(heights,bins=np.arange(-3.5,-.5,.025))
    prior=-float(edges[np.argmax(counts)]+.0125)
    floor=_fit_plane(p,up,prior,.45,angle_tolerance_deg=6,point_tolerance_m=.025,iterations=1400,seed=20261023)
    R=_rotation_from_up_vector(floor['normal']);shift=np.array([0,0,floor['d_m']])
    lev=p@R.T+shift
    corr=estimate_corridor_frame(lev,x_min=.5,x_max=16,y_min=-4,y_max=4,z_min=.3,z_max=2.3,
        voxel_size=.08,line_tolerance=.06,ransac_iterations=1800,seed=20261023)
    cd=corr.to_dict();cd['source_frame']='livox_level_frame'
    previous_axis=np.array(prev['corridor']['longitudinal_xy']);current_axis=corr.longitudinal_xy
    Rprev=np.array(prev['rotation_raw_to_corridor'])
    basis=np.array([[*corr.longitudinal_xy,0],[*corr.lateral_xy,0],[0,0,1]])
    raw_R=basis@R;raw_t=basis@(shift-np.r_[corr.origin_xy,0])
    save('corridor_candidate.json',cd)
    cfg=json.loads((ROOT/'config.json').read_text())
    cfg['tracking']['gravity_alignment']=dict(enabled=True,up_vector=floor['normal'],ground_height_along_up_m=-floor['d_m'])
    cfg['tracking']['corridor_frame_file']=str(OUT/'corridor_candidate.json')
    cfg['tracking']['lidar_to_zed_frame_file']=str(OUT/'no_external_calibration_loaded.json')
    save('analysis_config.json',cfg)
    changes=dict(imu_up_deg=aud['up_difference_from_config_deg'],floor_normal_deg=_angle_between(np.array(prev['floor']['normal']),np.array(floor['normal'])),
        floor_distance_m=floor['d_m']-prev['floor']['d_m'],wall_gap_m=corr.wall_gap-prev['corridor']['wall_gap'],
        raw_corridor_longitudinal_axis_deg=_angle_between(Rprev[0],raw_R[0]),
        corridor_center_origin_change_in_sensor_level_m=float(np.linalg.norm(corr.origin_xy-np.array(prev['corridor']['origin_xy']))),
        note='Independent LiDAR fit. Stability of IMU/walls does not prove no longitudinal sensor translation or unchanged camera orientation.')
    save('geometry.json',dict(lidar_audit=aud,floor=floor,corridor=cd,rotation_raw_to_level=R.tolist(),
        translation_raw_to_level=shift.tolist(),rotation_raw_to_corridor=raw_R.tolist(),translation_raw_to_corridor=raw_t.tolist(),
        installation_change_from_recording002=changes,zed_geometry_status='Not yet measured in recording003',
        source_sha256={str(q):hashlib.sha256(q.read_bytes()).hexdigest() for q in [Path(__file__),BAG/'metadata.yaml',BASE/'geometry.json',ROOT/'config.json']}))
    chunks=[];starts=[0];stamps=[]
    for msg,fb,_ in _iter_bag_clouds(BAG,'/livox/lidar','pointcloud2','mcap'):
        stamp=_message_timestamp_ns(msg,fb);p=_pointcloud2_to_numpy(msg);p=p[np.isfinite(p).all(axis=1)]
        cp=p@raw_R.T+raw_t
        keep=(cp[:,0]>-.5)&(cp[:,0]<19)&(abs(cp[:,1])<1.4)&(cp[:,2]>.06)&(cp[:,2]<2.4)
        chunks.append(cp[keep].astype(np.float32));starts.append(starts[-1]+int(keep.sum()));stamps.append(stamp)
    np.savez_compressed(OUT/'corridor_clouds.npz',points=np.concatenate(chunks),starts=starts,stamps=stamps)
    print('geometry',json.dumps(changes),flush=True)
    print('cached',len(stamps),'clouds',starts[-1],'points',flush=True)


if __name__=='__main__':main()
