"""Preflight real pick, coupled flip, place and empty return before any pick IO.

Each candidate is checked by the actual deploy pick planner and shared place
planner. Output aliases preserve the existing compose/dispatch interface.
No controller connection is made.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
from current_profile import load_profile

HERE=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('observation','grasp','snapshot','site-urdf','output'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--scene-state',type=Path)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    profile=load_profile()
    if profile['task_mode']!='stacked14':raise ValueError('Requires stacked14 profile')
    scene=json.loads(a.scene_state.read_text()) if a.scene_state else {}
    if len(scene.get('catalog',[]))>=2:raise ValueError('Both stack14 arrivals already committed')
    grasp=json.loads(a.grasp.read_text());selected=grasp['selected']
    choices=[selected]+grasp.get('candidates',[])
    unique=[];seen=set()
    for c in choices:
        key=json.dumps(c['T_world_tcp'])
        if key not in seen and c.get('full_envelope_fits',False) and c.get('contact_plate_fits',False):
            unique.append(c);seen.add(key)
    report=dict(success=False,task_mode='stacked14',effective_profile=profile,
                selection='first complete feasible deploy pick/place/return in perception preference order',attempts=[])
    for index,candidate in enumerate(unique):
        folder=a.output/f'grasp_{index:02d}';folder.mkdir()
        chosen=folder/'grasp.json';chosen.write_text(json.dumps(dict(grasp,selected=candidate),indent=2))
        row=dict(candidate=index);report['attempts'].append(row)
        common=['--observation',str(a.observation),'--grasp',str(chosen),'--site-urdf',str(a.site_urdf)]
        if a.scene_state:common+=['--scene-state',str(a.scene_state)]
        for phase,script in (('pick','prepare_observed_pick.py'),('place','prepare_observed_place.py')):
            command=[sys.executable,str(HERE/script),*common,'--output',str(folder/phase)]
            if phase=='pick':command+=['--snapshot',str(a.snapshot)]
            else:command+=['--pick-result',str(folder/'pick/result.json')]
            with (folder/(phase+'.log')).open('w') as log:
                done=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
            result_path=folder/phase/'result.json'
            result=json.loads(result_path.read_text()) if result_path.exists() else {}
            row[phase]=dict(success=bool(result.get('success')),error=result.get('error') if result else f'No result.json; see {folder/phase}.log',exit_code=done.returncode)
            if done.returncode or not result.get('success'):break
        else:
            # Keep original absolute paths in reports valid; expose selected folders
            # through aliases rather than copying results with stale embedded paths.
            for phase in ('pick','place'):
                (a.output/phase).symlink_to((folder/phase).resolve(),target_is_directory=True)
            report.update(success=True,selected_candidate=index)
        (a.output/'preflight.json').write_text(json.dumps(report,indent=2))
        if report['success']:break
    if not unique:
        report['error']='No full-envelope grasp candidates'
    (a.output/'preflight.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report))
    if not report['success']:raise RuntimeError('No complete stack14 deploy cycle; see preflight.json and grasp_*/{pick,place}/result.json')


if __name__=='__main__':main()
