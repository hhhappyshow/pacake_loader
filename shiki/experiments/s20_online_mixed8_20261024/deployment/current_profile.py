"""Shared current deployment values; no devices or heavy imports."""
import json
import os
import sys
from pathlib import Path

PROFILE = Path(__file__).with_suffix('.json')


def load_profile():
    profile = json.loads(Path(os.environ.get('S20_PROFILE', PROFILE)).read_text())
    options = profile['packing']['worker_options']
    if options.get('fixture14'):
        sys.path.insert(0, str(PROFILE.parent.parent))
        from straight_place.stacked_scene import twelve_box_fixture
        options['static_boxes'] = twelve_box_fixture()
        options.update(flip_horizontal=True, width_vertical=True, blended_flip=True)
        region = os.environ.get('S20_STACK14_REGION', options.get('stack14_region', 'both'))
        if region not in ('both', 'right-middle', 'left-middle'):
            raise ValueError('Invalid stack14 region: ' + region)
        options['stack14_region'] = region
        profile['task_mode'] = 'stacked14'
        profile['handoff_waypoint'] = 'post_pick_lift'
    else:
        profile['task_mode'] = 'upright'
        profile['handoff_waypoint'] = 'container_entry'
    return profile
