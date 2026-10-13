import sys
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from forest_geometry import extract_geometry,geometry_model


class ForestGeometry(unittest.TestCase):
    def cylinder(self,x,y,r):
        theta,z=np.meshgrid(np.linspace(0,2*np.pi,48,endpoint=False),np.arange(0,3,.15))
        return np.column_stack([x+r*np.cos(theta.ravel()),y+r*np.sin(theta.ravel()),z.ravel()])

    def test_separated_trunks_recover_supported_geometry(self):
        p=np.vstack([self.cylinder(0,0,.2),self.cylinder(2,0,.15)])
        g=extract_geometry(p)
        trunks=sorted([c for c in g["colliders"] if c["kind"]=="trunk"],key=lambda c:c["a"][0])
        self.assertEqual(len(trunks),2)
        for c,x,r in zip(trunks,[0,2],[.2,.15]):
            np.testing.assert_allclose(c["a"][:2],[x,0],atol=.04)
            self.assertAlmostEqual(c["radius"],r,delta=.03)
            self.assertGreater(c["b"][2]-c["a"][2],2)

    def test_ground_plane_not_classified_as_tree(self):
        x,y=np.meshgrid(np.arange(-1,1,.07),np.arange(-1,1,.07))
        g=extract_geometry(np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)]))
        self.assertEqual(len(g["colliders"]),0)

    def test_short_vertical_tube_is_marked_as_candidate_stem(self):
        theta,z=np.meshgrid(np.linspace(0,2*np.pi,32,endpoint=False),np.linspace(.05,.9,30))
        p=np.column_stack([1.3+.06*np.cos(theta.ravel()),.3+.06*np.sin(theta.ravel()),z.ravel()])
        g=extract_geometry(p)
        self.assertEqual(g["counts"]["trunk"],0)
        self.assertGreater(g["counts"]["stem"],0)

    def test_world_translation_and_branch_axis_match_local_geometry(self):
        g={"colliders":[{"kind":"branch","a":[1,2,3],"b":[2,3,4],"radius":.1}]}
        m=geometry_model(g,[10,20,30]);self.assertEqual(m.findtext("pose"),"10 20 30 0 0 0")
        pose=np.fromstring(m.findtext("link/pose"),sep=" ")
        np.testing.assert_allclose(pose[:3],[1.5,2.5,3.5])
        direction=Rotation.from_euler("xyz",pose[3:]).apply([0,0,1])
        np.testing.assert_allclose(direction,np.ones(3)/np.sqrt(3),atol=1e-10)
        self.assertIsNotNone(m.find("link/collision"));self.assertIsNotNone(m.find("link/visual"))

    def test_invalid_geometry_and_budget_fail_explicitly(self):
        with self.assertRaises(ValueError):extract_geometry([[np.nan,0,0]])
        with self.assertRaises(ValueError):extract_geometry(np.vstack([self.cylinder(0,0,.2),self.cylinder(2,0,.2)]),{"max_colliders":1})
        with self.assertRaises(ValueError):geometry_model({"colliders":[{"kind":"trunk","a":[0,0,0],"b":[0,0,0],"radius":.2}]})
