"""Declared simulation fixture: three layers of known boxes under the site roof.

Positions are lower corners relative to (door_x, inner_y0, floor_z).
This is a configurable task prior, not a reconstruction of the user's cargo.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R


def three_layer_boxes():
    # Two depth rows and three lateral columns, repeated through three layers.
    # Adjacent boxes meet at their faces; layers have matching support footprints.
    return [dict(id=f'Stack_L{level+1}_R{row+1}_C{column+1}',
                 dims=[.55,.40,.25],
                 pos=[.08+row*.55,.08+column*.40,level*.25])
            for level in range(3) for row in range(2) for column in range(3)]


def add_static_boxes(builder, config, boxes):
    """Add visible, collidable static shapes before Newton model finalization."""
    import warp as wp
    tf=next(t for t in config['static_transforms'] if t['child']=='container_link')
    rotation=R.from_euler('xyz',tf['rotation_rpy'])
    inner=config['container']['inner']
    door_x=config['container']['opening']['aperture']['corners'][0][0]
    origin=np.array([door_x,-inner['width']/2,inner['floor_z']])
    for box in boxes:
        dims=np.asarray(box['dims'])
        center=np.asarray(tf['translation'])+rotation.apply(origin+box['pos']+dims/2)
        cfg=builder.default_shape_cfg.copy()
        cfg.density=0.
        builder.add_shape_box(-1,xform=wp.transform(wp.vec3(*center),wp.quat(*rotation.as_quat())),
            hx=float(dims[0]/2),hy=float(dims[1]/2),hz=float(dims[2]/2),
            cfg=cfg,label=box['id'])
