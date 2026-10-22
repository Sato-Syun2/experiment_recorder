#!/usr/bin/env python3
"""Windows SDK 5.5.1: fixed-window BODY_34 / NEURAL model comparison."""
from pathlib import Path
from datetime import datetime
import argparse,csv,hashlib,json,os,subprocess,sys


def save(path,data):
    with path.open('x',encoding='utf-8') as f:
        json.dump(data,f,ensure_ascii=False,indent=2,allow_nan=False)
        f.write('\n')


def scan_svo(svo):
    import pyzed.sl as sl
    cam=sl.Camera();init=sl.InitParameters();init.set_from_svo_file(str(svo))
    init.svo_real_time_mode=False;init.depth_mode=sl.DEPTH_MODE.NONE
    status=cam.open(init)
    if status!=sl.ERROR_CODE.SUCCESS:
        cam.close();raise RuntimeError(f'Could not open {svo}: {status}')
    try:
        info=cam.get_camera_information();total=int(cam.get_svo_number_of_frames());stamps=[]
        while True:
            status=cam.grab(sl.RuntimeParameters())
            if status==sl.ERROR_CODE.END_OF_SVOFILE_REACHED:break
            if status!=sl.ERROR_CODE.SUCCESS:raise RuntimeError(f'SVO timestamp scan failed: {status}')
            stamps.append(int(cam.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()))
        if len(stamps)!=total or any(a>=b for a,b in zip(stamps,stamps[1:])):
            raise RuntimeError('SVO frame count/timestamp sequence is inconsistent.')
        return dict(serial_number=int(info.serial_number),source_total_frames=total,
                    source_origin_ns=stamps[0],timestamps_ns=stamps)
    finally:cam.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording_root',type=Path,help='SSD folder containing recording_002 and recording_003')
    parser.add_argument('--recordings',nargs='+',choices=['002','003'],default=['002','003'])
    parser.add_argument('--models',nargs='+',default=None,help='optional model IDs from comparison_plan.json')
    parser.add_argument('--package-dir',type=Path,default=None,help='prepared comparison package (normally discovered beside shared scripts)')
    args=parser.parse_args()
    script_dir=Path(__file__).resolve().parent
    package=args.package_dir.resolve() if args.package_dir else script_dir.parent/'20261022_sdk551_body_model_comparison'
    if args.package_dir is None and not package.exists():
        package=script_dir.parent/'reports'/'20261022_sdk551_body_model_comparison'
    plan=json.loads((package/'comparison_plan.json').read_text(encoding='utf-8'))
    configurations=[r for r in plan['configurations'] if args.models is None or r['id'] in args.models]
    if args.models and set(args.models)-{r['id'] for r in configurations}:
        parser.error('Unknown model ID; see comparison_plan.json.')
    if not configurations:parser.error('No configurations selected.')
    root=args.recording_root.expanduser().resolve()
    paths={rid:root/f'recording_{rid}'/f'recording_{rid}.svo2' for rid in args.recordings}
    if str(root).startswith('\\\\') or any(not p.is_file() for p in paths.values()):
        parser.error('Specify the existing local/SSD group folder with the requested SVO2 files.')
    import pyzed.sl as sl
    version=str(sl.Camera.get_sdk_version())
    if version.split('-')[0]!='5.5.1':
        parser.error(f'This comparison expects runtime SDK 5.5.1; detected {version}.')
    probe=sl.BodyTrackingParameters()
    if not hasattr(sl,'BODY_TRACKING_MODEL_GEN') or not hasattr(probe,'model_gen'):
        parser.error('The installed pyzed binding lacks model_gen. Install the Python API matching SDK 5.5.1.')
    stamp=datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    run=package/'runs'/stamp;run.mkdir(parents=True,exist_ok=False)
    environment=os.environ.copy()
    previous_override=environment.pop('ZED_SDK_BODY_TRACKING_MODEL_GENERATION',None)
    environment.setdefault('OPENBLAS_NUM_THREADS','1')
    tools=package/'sdk_tools';exporter=tools/'analyze_svo_body_tracking.py'
    save(run/'run_start.json',dict(runtime_sdk_version=version,python=sys.executable,recordings=args.recordings,
        configurations=configurations,source_svo={k:str(p) for k,p in paths.items()},
        removed_child_process_generation_override=previous_override,
        comparison_plan_sha256=hashlib.sha256((package/'comparison_plan.json').read_bytes()).hexdigest(),
        exporter_sha256=hashlib.sha256(exporter.read_bytes()).hexdigest(),
        note='No live cameras/ROS/RViz. Each case uses a new Python/SDK process. Old outputs are read-only.'))
    try:
        gpu=subprocess.run(['nvidia-smi','--query-gpu=name,driver_version,memory.total','--format=csv,noheader'],capture_output=True,text=True,check=False)
        save(run/'gpu_environment.json',dict(returncode=gpu.returncode,stdout=gpu.stdout,stderr=gpu.stderr))
    except OSError as exc:save(run/'gpu_environment.json',dict(error=str(exc)))
    errors={};cases=[]
    for rid,svo in paths.items():
        group=run/f'recording_{rid}';group.mkdir()
        print(f'\nRecording {rid}: scanning original image timestamps without tracking.',flush=True)
        try:
            scan=scan_svo(svo);info=plan['recordings'][rid]
            if rid=='003':
                print('Export recording003 RGB/depth/IMU once; independent of model comparison.',flush=True)
                subprocess.run([sys.executable,str(tools/'export_svo_review.py'),str(svo),
                    '--output',str(group/'review'),'--every-seconds','0.2','--include-depth','--include-imu'],env=environment,check=True)
            windows={}
            for scope in info['scopes']:
                if scope=='approach' and rid=='002':
                    reference=json.loads((package/'reference_window_002.json').read_text(encoding='utf-8'))
                    a,b=int(reference['start_frame']),int(reference['end_frame_exclusive'])
                    if scan['source_total_frames']!=reference['source_total_frames'] or scan['serial_number']!=reference['serial_number'] or scan['source_origin_ns']!=reference['source_origin_ns'] or scan['timestamps_ns'][a:b]!=reference['expected_image_timestamps_ns']:
                        raise RuntimeError('Recording002 differs from the historical source/window; no relabeling applied.')
                    window=reference
                else:
                    stamps=scan['timestamps_ns']
                    chosen=list(range(len(stamps))) if scope=='full' else [i for i,t in enumerate(stamps) if int(info['approach_start_header_ns'])<=t<=int(info['approach_end_header_ns'])]
                    if not chosen:raise RuntimeError('Requested window does not overlap this SVO.')
                    a,b=chosen[0],chosen[-1]+1
                    window=dict(kind='approach_only_fresh_sdk',start_frame=a,end_frame_exclusive=b,
                        serial_number=scan['serial_number'],source_total_frames=scan['source_total_frames'],
                        source_origin_ns=scan['source_origin_ns'],expected_image_timestamps_ns=stamps[a:b],
                        scope=scope,selection_note=info.get('window_note','Full source recording'))
                window_path=group/f'window_{scope}.json';save(window_path,window);windows[scope]=window_path
            save(group/'source_scan.json',dict(source_svo=str(svo),**scan))
        except Exception as exc:
            errors[f'{rid}/preparation']=str(exc);save(group/'preparation_failed.json',dict(error=str(exc)));continue
        for scope,window_path in windows.items():
            for config in configurations:
                key=f'{rid}/{scope}/{config["id"]}';output=group/scope/config['id']
                output.parent.mkdir(parents=True,exist_ok=True)
                log=output.parent/f'{config["id"]}.log'
                print(f'\n{key}: {config["body_model"]} / {config["generation"]}, fresh SDK process.',flush=True)
                command=[sys.executable,str(exporter),str(svo),'--output',str(output),
                    '--window-manifest',str(window_path),'--body-model',config['body_model'],
                    '--model-generation',config['generation'],'--require-generation-readback',
                    '--max-depth-m','30','--confidence','35','--preview-period-s','0.2']
                try:
                    with log.open('x',encoding='utf-8') as stream:
                        child=subprocess.Popen(command,env=environment,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                            text=True,encoding='utf-8',errors='replace',bufsize=1)
                        for line in child.stdout:
                            stream.write(line);stream.flush();print(line.rstrip(),flush=True)
                        if child.wait()!=0:raise subprocess.CalledProcessError(child.returncode,command)
                    camera=json.loads((output/'camera.json').read_text(encoding='utf-8'))
                    if camera['sdk_version']!='5.5.1' or camera['body_model_generation_effective']!=config['generation']:
                        raise RuntimeError('Effective SDK/generation does not match this case.')
                    cases.append(dict(recording=rid,scope=scope,configuration=config,body_dir=str(output.relative_to(run)),window=str(window_path.relative_to(run)),status='completed'))
                except Exception as exc:
                    errors[key]=str(exc);cases.append(dict(recording=rid,scope=scope,configuration=config,status='failed',error=str(exc),log=str(log.relative_to(run))))
                    print(f'Failed {key}; see {log}',flush=True)
    save(run/'case_results.json',dict(cases=cases,errors=errors))
    subprocess.run([sys.executable,str(tools/'compare_body_models.py'),'--package-dir',str(package),'--run-dir',str(run)],env=environment,check=True)
    comparison=json.loads((run/'comparison.json').read_text(encoding='utf-8'))
    for row in comparison['cases']:
        if row['status']=='invalid':
            errors[f'{row["recording"]}/{row["scope"]}/{row["label"]}/metrics']=row['error']
    save(run/'run_complete.json',dict(status='completed_with_errors' if errors else 'all_selected_cases_completed',
        completed_cases=sum(c['status']=='completed' for c in cases),failed_cases=len(errors),
        note='Comparison metrics generated; visual target association and accuracy verdict still require review.'))
    print(f'\nComparison: {run/"index.html"}',flush=True)
    if os.name=='nt':os.startfile(str(run/'index.html'))
    if errors:raise SystemExit(1)


if __name__=='__main__':main()
