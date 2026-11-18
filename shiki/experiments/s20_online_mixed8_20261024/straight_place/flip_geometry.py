"""Single-box horizontal-tool insertion geometry (simulation only)."""
import numpy as np
from scipy.spatial.transform import Rotation as R


def horizontal_tool(container_rotation, tcp_to_box_rotation, width_vertical=False):
    """TCP +Z points inward and is exactly perpendicular to world gravity.

    Begin with a box quarter turn about container Y, then minimally correct
    the TCP direction for the measured tool/box attachment and tilted ULD.
    """
    inward = container_rotation.apply([1., 0., 0.])
    inward[2] = 0.
    inward /= np.linalg.norm(inward)
    nominal = container_rotation * R.from_euler('y', -np.pi/2) * tcp_to_box_rotation.inv()
    correction, _ = R.align_vectors(inward[None, :], nominal.apply([[0., 0., 1.]]))
    result = correction * nominal
    if width_vertical:
        # Rotate about the horizontal suction normal before insertion: box Y
        # becomes vertical while box X (long edge) stays horizontal.
        result = R.from_rotvec(inward*np.pi/2)*result
    return result


def rotated_extent(dimensions, box_rotation, container_rotation):
    return np.abs((container_rotation.inv()*box_rotation).as_matrix()) @ np.asarray(dimensions)


def sample_metrics(tcp_pose, stage, offset, attachment, dimensions, container_rotation,
                   container_translation, door_x, width_vertical=False, expected_tool_quaternion=None):
    """Check actual FK, not requested pose; original rigid attachment is retained."""
    rotation=R.from_quat(np.asarray(tcp_pose)[3:])
    metrics={}
    if '_HORIZONTAL_' in stage or stage.endswith('_EMPTY_SEPARATE'):
        normal=rotation.apply([0.,0.,1.])
        metrics['horizontal_error_deg']=float(np.degrees(np.arcsin(np.clip(abs(normal[2]),0,1))))
        inward=container_rotation.apply([1.,0.,0.]);inward[2]=0;inward/=np.linalg.norm(inward)
        metrics['inward_alignment']=float(normal@inward)
        if expected_tool_quaternion is not None:
            metrics['target_orientation_error_deg']=float(np.degrees((R.from_quat(expected_tool_quaternion).inv()*rotation).magnitude()))
        elif width_vertical:
            box_rotation=rotation*attachment
            metrics['width_vertical_error_deg']=float(np.degrees(np.arccos(np.clip(abs(box_rotation.apply([0,1,0])[2]),0,1))))
            metrics['length_horizontal_error_deg']=float(np.degrees(np.arcsin(np.clip(abs(box_rotation.apply([1,0,0])[2]),0,1))))
    if stage.endswith('_FLIP_90_OUTSIDE') or '_FLIP_BLEND_' in stage or stage.endswith('_FLIP_COUPLED_OUTSIDE'):
        box_rotation=rotation*attachment
        center=np.asarray(tcp_pose)[:3]+rotation.apply(offset)
        local_center=container_rotation.inv().apply(center-container_translation)
        half=rotated_extent(dimensions,box_rotation,container_rotation)/2
        metrics['outside_clearance_m']=float(door_x-local_center[0]-half[0])
    return metrics


def constraint_error(metrics):
    if metrics.get('target_orientation_error_deg',0.)>2.:
        return 'tilted_target_orientation_error_exceeds_2deg'
    if metrics.get('width_vertical_error_deg',0.) > 2. or metrics.get('length_horizontal_error_deg',0.) > 2.:
        return 'box_width_not_vertical_or_length_not_horizontal_within_2deg'
    if metrics.get('horizontal_error_deg',0.) > 1.:
        return 'TCP_not_horizontal_within_1deg'
    if metrics.get('inward_alignment',1.) < np.cos(np.radians(1.)):
        return 'TCP_not_pointing_inward_within_1deg'
    if metrics.get('outside_clearance_m',1.) < .01:
        return 'flip_swept_box_not_fully_outside_door'
    return None
