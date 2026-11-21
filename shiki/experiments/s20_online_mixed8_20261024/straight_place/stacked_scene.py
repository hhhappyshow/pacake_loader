"""Declared simulation fixture: three layers of known boxes under the site roof.

Positions are lower corners relative to (door_x, inner_y0, floor_z).
This is a configurable task prior, not a reconstruction of the user's cargo.
"""


# User inventory in centimetres, in the supplied order. IDs remain stable when
# sorting for assembly: Base_01 is always the user's first base box.
BASE14_CM = [(34,22,55),(37,23,58),(40,30,63),(40,26,63),
             (42,25,68),(38,22,40),(38,23,63),(47,29,74),
             (44,27,71),(47,26,72),(33,21,53),(34,21,53)]
ARRIVAL14_CM = [(35,23,54),(36,23,54)]


def twelve_box_fixture():
    """Three columns, four levels; longest edge points into the container.

    First two columns finish at 0.99 m, the third at 0.97 m. Their upper
    footprints retain full support on each column. The middle column is
    centred on its base; top lateral gaps are 85 mm and 115 mm. Geometric design
    only: cardboard stiffness, loads and suction capacity are not modelled.
    All coordinates are lower corners relative to door / inner-y0 / floor.
    """
    columns = [([3,4,1,11], .08, .40, 'right'),
               ([8,10,2,12], .50, .47, 'center'),
               ([9,5,7,6], .99, .44, 'center')]
    boxes=[]
    for column,(ids,y,width,alignment) in enumerate(columns,1):
        z=0.
        for level,number in enumerate(ids,1):
            lateral,height,depth=(v/100 for v in BASE14_CM[number-1])
            yy=y+(width-lateral)*(1. if alignment=='right' else .5 if alignment=='center' else 0.)
            boxes.append(dict(id=f'Base_{number:02d}',inventory_id=number,
                column=column,level=level,dims=[float(depth),float(lateral),float(height)],
                pos=[float(.42-depth/2),float(yy),float(round(z,8))]))
            z+=height
    return sorted(boxes,key=lambda b:(b['level'],b['column']))


def three_layer_boxes():
    # Two depth rows and three lateral columns, repeated through three layers.
    # Adjacent boxes meet at their faces; layers have matching support footprints.
    return [dict(id=f'Stack_L{level+1}_R{row+1}_C{column+1}',
                 dims=[.55,.40,.25],
                 pos=[.08+row*.55,.08+column*.40,level*.25])
            for level in range(3) for row in range(2) for column in range(3)]


def add_static_boxes(builder, config, boxes):
    """Add visible, collidable static shapes before Newton model finalization."""
    import numpy as np
    from scipy.spatial.transform import Rotation as R
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
