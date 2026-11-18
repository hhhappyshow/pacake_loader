"""Geometric two-height contact candidates; not a rigid-body stability proof.

Clip the projected bottom face against each support top. Compute the exact
lowest bottom-face height on each clipped polygon, then solve roll so both
supports contact at the same box centre height. No support-height tolerance
or fictitious support material is introduced.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import brentq


def clip(poly, axis, value, keep_greater):
    result=[]
    if not len(poly):return np.empty((0,3))
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        da=(a[axis]-value)*(1 if keep_greater else -1)
        db=(b[axis]-value)*(1 if keep_greater else -1)
        if da>=-1e-10:result.append(a)
        if (da<0)!=(db<0):
            result.append(a+(b-a)*da/(da-db))
    return np.asarray(result).reshape(-1,3)


def bottom_face(dimensions, rotation, center_xy):
    matrix=rotation.as_matrix();axis=int(np.argmax(abs(matrix[2])))
    others=[i for i in range(3) if i!=axis]
    vertices=[]
    for a,b in ((-1,-1),(1,-1),(1,1),(-1,1)):
        v=np.zeros(3);v[axis]=-np.sign(matrix[2,axis])*dimensions[axis]/2
        v[others]=np.asarray(dimensions)[others]*[a,b]/2
        vertices.append(rotation.apply(v)+[*center_xy,0.])
    return np.asarray(vertices)


def support_height(poly, support):
    for axis in (0,1):
        poly=clip(poly,axis,support['pos'][axis],True)
        poly=clip(poly,axis,support['pos'][axis]+support['dims'][axis],False)
    if len(poly)<3:return None
    return float(support['pos'][2]+support['dims'][2]-poly[:,2].min())


def solve(dimensions, base_tool, attachment, container_rotation, center_xy, supports):
    axis=base_tool.apply([0,0,1])
    def evaluate(angle):
        tool=R.from_rotvec(axis*angle)*base_tool
        box=container_rotation.inv()*tool*attachment
        poly=bottom_face(dimensions,box,center_xy)
        heights=[support_height(poly,s) for s in supports]
        return tool,box,heights
    def residual(angle):
        heights=evaluate(angle)[2]
        if any(h is None for h in heights):raise ValueError('Bottom face does not overlap both supports')
        return heights[0]-heights[1]
    try:
        angle=brentq(residual,-np.radians(6),np.radians(6),xtol=1e-10)
    except ValueError:return None
    tool,rotation,heights=evaluate(angle)
    extent=abs(rotation.as_matrix())@dimensions
    center=np.r_[center_xy,max(heights)]
    # Center projection must lie between the two support regions laterally.
    # This is a geometric gate, not friction/COM/load/dynamic validation.
    if not supports[0]['pos'][1]<=center[1]<=supports[1]['pos'][1]+supports[1]['dims'][1]:return None
    return dict(roll_rad=float(angle),tool_xyzw=tool.as_quat().tolist(),
        dims=extent.tolist(),pos=(center-extent/2).tolist(),center=center.tolist(),
        support_ids=[s['id'] for s in supports],support_height_difference_m=abs(supports[0]['pos'][2]+supports[0]['dims'][2]-supports[1]['pos'][2]-supports[1]['dims'][2]),
        contact_height_residual_m=float(abs(heights[0]-heights[1])),
        model='Bottom-face clipped against unequal top planes; edge contact, no area-support or physical stability claim')
