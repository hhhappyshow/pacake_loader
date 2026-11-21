"""Offline deployment contract tests: configuration and full-cycle gating."""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from current_profile import load_profile
import prepare_stack14_cycle as preflight


class DeploymentTests(unittest.TestCase):
    def test_shared_fixture_and_handoff(self):
        with patch.dict(os.environ,{'S20_STACK14_REGION':'left-middle'},clear=True):
            p=load_profile()
        o=p['packing']['worker_options']
        self.assertEqual(p['handoff_waypoint'],'post_pick_lift')
        self.assertTrue(o['blended_flip'])
        self.assertEqual(o['stack14_region'],'left-middle')
        self.assertAlmostEqual(next(b for b in o['static_boxes'] if b['id']=='Base_12')['pos'][1],.565)

    def test_upright_preserves_old_handoff(self):
        with patch.dict(os.environ,{'S20_PROFILE':str(Path(__file__).with_name('upright_profile.json'))},clear=True):
            p=load_profile()
        self.assertEqual(p['handoff_waypoint'],'container_entry')
        self.assertFalse(p['packing']['worker_options'].get('fixture14',False))

    def test_failed_place_retries_complete_pick_before_accepting(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);g=root/'grasp.json';out=root/'out'
            candidates=[dict(T_world_tcp=[[i]],full_envelope_fits=True,contact_plate_fits=True) for i in (1,2)]
            g.write_text(json.dumps(dict(selected=candidates[0],candidates=candidates)))
            calls=[]
            def run(command,**kwargs):
                folder=Path(command[command.index('--output')+1]);folder.mkdir()
                calls.append((folder.parent.name,folder.name))
                success=not(folder.parent.name=='grasp_00' and folder.name=='place')
                (folder/'result.json').write_text(json.dumps(dict(success=success,error=None if success else 'unreachable')))
                return type('Result',(),{'returncode':0})()
            args=['preflight','--observation',str(root/'obs'),'--grasp',str(g),'--snapshot',str(root/'snapshot'),
                  '--site-urdf',str(root/'robot'),'--output',str(out)]
            with patch.object(sys,'argv',args),patch.object(preflight.subprocess,'run',side_effect=run),patch('builtins.print'):
                preflight.main()
            self.assertEqual(calls,[('grasp_00','pick'),('grasp_00','place'),('grasp_01','pick'),('grasp_01','place')])
            self.assertEqual((out/'pick').resolve(),out/'grasp_01/pick')
            self.assertTrue(json.loads((out/'preflight.json').read_text())['success'])


if __name__=='__main__':unittest.main()
