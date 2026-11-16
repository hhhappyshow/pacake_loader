"""Offline acceptance check of saved actual FK states for one or more sequential flipped boxes."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml


def audit(folder, box_number=1):
    folder=Path(folder)
    report=json.loads((folder/'result.json').read_text())
    cfg=yaml.safe_load((folder/'scene_tf.yaml').read_text())
    tf=next(t for t in cfg['static_transforms'] if t['child']=='container_link')
    cr=R.from_euler('xyz',tf['rotation_rpy']);translation=np.asarray(tf['translation'])
    door_x=cfg['container']['opening']['aperture']['corners'][0][0]
    floor=cfg['container']['inner']['floor_z']
    stack=report.get('preloaded_boxes',[])
    support_height=max((b['pos'][2]+b['dims'][2] for b in stack),default=0.)
    support_plane=floor+support_height
    with np.load(folder/'trajectory.npz') as data:
        stages=data['stage'];poses=data['body_q'];labels=list(data['body_labels'])
    tcp_index=labels.index('S20/suction_contact_frame');box_index=labels.index(f'B{box_number}')
    dimensions=np.asarray(report['source_catalog_m'][box_number-1])  # Audit truth only, not planner input.
    flip=stages==f'B{box_number}_FLIP_90_OUTSIDE';insert=stages==f'B{box_number}_HORIZONTAL_INSERT'
    release=stages==f'B{box_number}_RELEASE_ACTUAL_FROZEN';grasp=stages=='IDEAL_VACUUM_ON'
    if not all(np.any(mask) for mask in (flip,insert,release,grasp)):
        raise ValueError('Incomplete flip trajectory: required stages absent')
    grasp_indices=np.flatnonzero(grasp)
    grasp_index=grasp_indices[box_number-1]
    tcp=poses[insert,tcp_index]
    normals=R.from_quat(tcp[:,3:]).apply(np.tile([0,0,1],(len(tcp),1)))
    horizontal=np.degrees(np.arcsin(np.clip(abs(normals[:,2]),0,1)))
    direction=cr.apply([1.,0.,0.]);direction[2]=0;direction/=np.linalg.norm(direction)
    alignment=np.degrees(np.arccos(np.clip(normals@direction,-1,1)))
    first=np.flatnonzero(insert)[0]
    start=poses[first-1,tcp_index,:3];end=tcp[-1,:3];delta=end-start
    t=np.clip((tcp[:,:3]-start)@delta/(delta@delta),0,1)
    line_error=np.linalg.norm(tcp[:,:3]-(start+t[:,None]*delta),axis=1)
    outside=[]
    for pose in poses[flip,box_index]:
        extent=np.abs((cr.inv()*R.from_quat(pose[3:])).as_matrix())@dimensions
        center=cr.inv().apply(pose[:3]-translation)
        outside.append(door_x-center[0]-extent[0]/2)
    released=poses[release,box_index][-1]
    release_rot=R.from_quat(released[3:])
    extent=np.abs((cr.inv()*release_rot).as_matrix())@dimensions
    center=cr.inv().apply(released[:3]-translation)
    initial_normal=R.from_quat(poses[grasp_index,box_index,3:]).apply([0,0,1])
    final_normal=release_rot.apply([0,0,1])
    tilt=float(np.degrees(np.arccos(np.clip(initial_normal@final_normal,-1,1))))
    release_index=np.flatnonzero(release)[-1]
    frozen_drift=float(np.max(np.linalg.norm(poses[release_index:,box_index,:3]-released[:3],axis=1)))
    result=dict(frozen_position_drift_max_m=frozen_drift,completed=report['completed'],completed_cycles=report['completed_cycles'],
        insert_horizontal_error_max_deg=float(horizontal.max()),insert_inward_error_max_deg=float(alignment.max()),
        insert_line_error_max_m=float(line_error.max()),insert_distance_m=float(np.linalg.norm(delta)),
        flip_min_outside_clearance_m=float(min(outside)),actual_box_top_normal_turn_deg=tilt,
        release_lowest_corner_floor_gap_m=float(center[2]-extent[2]/2-floor),
        release_lowest_corner_support_gap_m=float(center[2]-extent[2]/2-support_plane),
        preloaded_boxes=len(stack),support_height_m=support_height,
        width_vertical_error_deg=float(np.degrees(np.arccos(np.clip(abs(release_rot.apply([0,1,0])[2]),0,1)))),
        length_horizontal_error_deg=float(np.degrees(np.arcsin(np.clip(abs(release_rot.apply([1,0,0])[2]),0,1)))),
        release_rear_face_insertion_depth_m=float(center[0]-extent[0]/2-door_x),
        returned_to_observation=bool(np.any(stages==f'B{box_number}_RETURN_OBSERVATION')),
        simulation_scope='Ideal attachment and frozen release; floor gap is geometric, not dynamic support',
        thresholds=dict(horizontal_deg=1.,line_error_m=.005,outside_clearance_m=.01,box_quarter_turn_error_deg=2.))
    if stack:
        clearances=[]
        for pose in poses[insert,box_index]:
            span=np.abs((cr.inv()*R.from_quat(pose[3:])).as_matrix())@dimensions
            local=cr.inv().apply(pose[:3]-translation)
            clearances.append(cfg['container']['inner']['ceiling_z']-local[2]-span[2]/2)
        result['insert_min_ceiling_clearance_m']=float(min(clearances))
        insert_rot=R.from_quat(poses[insert,box_index,3:])
        widths=insert_rot.apply(np.tile([0,1,0],(len(tcp),1)))
        lengths=insert_rot.apply(np.tile([1,0,0],(len(tcp),1)))
        result['insert_width_vertical_error_max_deg']=float(np.degrees(np.arccos(np.clip(abs(widths[:,2]),0,1))).max())
        result['insert_length_horizontal_error_max_deg']=float(np.degrees(np.arcsin(np.clip(abs(lengths[:,2]),0,1))).max())
    result['passed']=bool(result['completed'] and result['completed_cycles']==report['requested_cycles'] and
        horizontal.max()<=1 and alignment.max()<=1 and line_error.max()<=.005 and min(outside)>=.01 and
        abs(tilt-90)<=2 and -.001<=result['release_lowest_corner_support_gap_m']<=.02 and
        result['release_rear_face_insertion_depth_m']>0 and result['returned_to_observation'] and frozen_drift<1e-6)
    if report.get('placement_mode')=='stacked-width':
        result['passed']=bool(result['passed'] and result['width_vertical_error_deg']<=2 and
            result['length_horizontal_error_deg']<=2 and result['insert_width_vertical_error_max_deg']<=2 and
            result['insert_length_horizontal_error_max_deg']<=2 and result['insert_min_ceiling_clearance_m']>=.01)
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_folder',type=Path)
    args=parser.parse_args()
    report=json.loads((args.run_folder/'result.json').read_text())
    count=report['requested_cycles']
    rows=[dict(box=f'B{i}',**audit(args.run_folder,i)) for i in range(1,count+1)]
    result=rows[0] if count==1 else dict(passed=all(row['passed'] for row in rows),requested_cycles=count,boxes=rows)
    (args.run_folder/'flip_acceptance.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)
