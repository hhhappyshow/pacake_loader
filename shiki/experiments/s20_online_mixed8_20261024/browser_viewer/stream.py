"""Display-only Newton geometry and atomic live snapshots. No GL/CUDA renderer."""
import json
import os
import time
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as R


def atomic_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False))
    os.replace(temporary, path)


class BrowserStream:
    def __init__(self, folder, runtime):
        self.folder = folder
        folder.mkdir(parents=True, exist_ok=True)
        self.last = 0.
        model = runtime.model
        bodies = model.shape_body.numpy()
        kinds = model.shape_type.numpy()
        scales = model.shape_scale.numpy()
        flags = model.shape_flags.numpy()
        transforms = model.shape_transform.numpy()
        colors = model.shape_color.numpy()
        shapes, chunks, offset = [], [], 0
        palette = [[.12,.65,.65],[.95,.58,.18],[.60,.42,.78],[.32,.68,.40]]
        for i, label in enumerate(model.shape_label):
            body = int(bodies[i])
            box = label.endswith('_collision_box')
            if label.endswith('_suitcase_visual') or (not box and not (int(flags[i]) & 1)):
                continue
            scale, kind = scales[i], int(kinds[i])
            if kind in (8,10):
                source = model.shape_source[i]
                mesh = trimesh.Trimesh(np.asarray(source._vertices)*scale,
                                       np.asarray(source._indices).reshape(-1,3), process=False)
            elif kind == 7:
                mesh = trimesh.creation.box(extents=2*scale)
            elif kind == 3:
                mesh = trimesh.creation.icosphere(subdivisions=2, radius=float(scale[0]))
            elif kind == 6:
                mesh = trimesh.creation.cylinder(radius=float(scale[0]), height=float(2*scale[1]), sections=24)
            elif kind == 1:
                continue  # Browser draws the ground grid.
            else:
                raise ValueError(f'Unsupported browser shape: {label}, type={kind}')
            vertices = np.asarray(mesh.vertices, dtype='<f4').ravel()
            indices = np.asarray(mesh.faces, dtype='<u4').ravel()
            color = colors[i].tolist()
            if label.startswith('Stack_'):
                level=int(label.split('_')[1][1:])-1
                color=[[.64,.68,.73],[.58,.64,.70],[.51,.59,.67]][level%3]
            if box:
                color = palette[runtime.boxes.index(body) % len(palette)]
            shapes.append(dict(name=label, body=body, pose=transforms[i].tolist(), color=color,
                               vertex_offset=offset, vertex_count=len(vertices),
                               index_offset=offset+vertices.nbytes, index_count=len(indices)))
            chunks.extend([vertices.tobytes(), indices.tobytes()])
            offset += vertices.nbytes+indices.nbytes
        (folder/'geometry.bin').write_bytes(b''.join(chunks))
        atomic_json(folder/'scene.json', dict(bodies=list(model.body_label), shapes=shapes,
                                            tcp=runtime.tcp, boxes=runtime.boxes, units='metres', up='Z'))
        self.update(runtime, {}, force=True)

    def update(self, runtime, report, force=False):
        now = time.monotonic()
        if not force and now-self.last < .05:
            return
        self.last = now
        tcp = runtime.state.body_q.numpy()[runtime.tcp]
        x,y,z,w = tcp[3:]
        normal = np.array([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])
        box_pose=runtime.state.body_q.numpy()[runtime.boxes[min(max(report.get('current_cycle',1)-1,0),len(runtime.boxes)-1)]]
        box_rot=R.from_quat(box_pose[3:])
        length_axis=box_rot.apply([1,0,0]);width_axis=box_rot.apply([0,1,0])
        atomic_json(self.folder/'state.json', dict(
            box_pose=box_pose.tolist(),box_length_axis=length_axis.tolist(),box_width_axis=width_axis.tolist(),
            length_horizontal_error_deg=float(np.degrees(np.arcsin(np.clip(abs(length_axis[2]),0,1)))),
            width_vertical_error_deg=float(np.degrees(np.arccos(np.clip(abs(width_axis[2]),0,1)))),
            total=len(runtime.boxes), placement_mode=report.get('placement_mode','upright'),
            tcp_normal_world=normal.tolist(), tcp_horizontal_error_deg=float(np.degrees(np.arcsin(np.clip(abs(normal[2]),0,1)))),
            body_q=runtime.state.body_q.numpy().tolist(), q=runtime.q.tolist(),
            stage=report.get('computation_phase') or runtime.stages[-1], frame=len(runtime.history), timestamp=time.time(),
            cycle=report.get('current_cycle',0), completed_cycles=report.get('completed_cycles',0),
            completed=report.get('completed',False), error=report.get('error'),
            running=report.get('running_simulation',True)))
