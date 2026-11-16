"""Check observed-grasp approach and loaded transfer before choosing a grasp.

Runs in the isolated planning worker, with its observed attachment estimate.
The main simulation still checks every segment on the actual attachment.
"""
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R, Slerp
import newton


def check_pick_transfer(r, qstart, contact, predicted, lift_q, source_index):
    r.q=np.asarray(qstart).copy();r.w.held=None
    body=r.boxes[source_index]
    r.fixed[body]=np.r_[predicted['source_center_world_m'],predicted['source_rotation_world_xyzw']]
    r.frame(r.q,'PREFLIGHT_OBSERVATION')
    travel=0.;samples=0
    def segment(position,quat,name,speed=.30):
        nonlocal travel,samples
        tcp=r.state.body_q.numpy()[r.tcp];q=r.q.copy()
        duration=max(np.linalg.norm(position-tcp[:3])/speed,
            (R.from_quat(tcp[3:]).inv()*R.from_quat(quat)).magnitude(),.02)
        count=max(2,int(np.ceil(duration/.02)))
        rot=Slerp([0,1],R.from_quat([tcp[3:],quat]));knots=[q.copy()]
        for a in np.linspace(0,1,count+1)[1:]:
            ok,q=r.ik._solve_once(tcp[:3]*(1-a)+position*a,rot(a).as_quat(),q,check_collision=False)
            if not ok:raise RuntimeError(f'{name}: pickup IK rejected')
            knots.append(q.copy())
        curve=CubicSpline(np.linspace(0,duration,count+1),knots,bc_type=((1,np.zeros(6)),(1,np.zeros(6))))
        grid=curve(np.linspace(0,duration,count+1))
        for q in grid:
            if not r.validate(q):raise RuntimeError(f'{name}: pickup CAD rejected {r.collision.last_hits}')
        travel+=float(np.linalg.norm(np.diff(grid,axis=0),axis=1).sum());samples+=len(grid)
        r.frame(grid[-1],name)
    normal=-R.from_quat(contact[3:]).apply([0,0,1])
    segment(contact[:3]+normal*.10,contact[3:],'PREFLIGHT_APPROACH')
    segment(contact[:3],contact[3:],'PREFLIGHT_CONTACT',.15)
    r.w.held=body
    r.w.grip_offset=np.asarray(predicted['offset_m']);r.w.grip_rot=R.from_quat(predicted['rotation_xyzw'])
    r.frame(r.q,'PREFLIGHT_GRIP')
    segment(contact[:3]+[0,0,.20],contact[3:],'PREFLIGHT_LIFT')
    r.ik.model.joint_q.assign(np.asarray(lift_q))
    newton.eval_fk(r.ik.model,r.ik.model.joint_q,r.ik.model.joint_qd,r.ik.model)
    lift=r.ik.model.body_q.numpy()[r.ik.contact_body].copy()
    segment(lift[:3],lift[3:],'PREFLIGHT_TRANSFER')
    # The following placement plan starts from this exact IK branch.
    return dict(joint_travel_rad=travel,checked_samples=samples,end_q=r.q.tolist())
