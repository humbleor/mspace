import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from geometry_diagnostics import surface_support,retained_baseline_trees,compare_geometry


class GeometryDiagnostics(unittest.TestCase):
    def geometry(self,x=0):
        return {"colliders":[{"kind":"trunk","tree_id":1,"a":[x,0,0],"b":[x,0,2],"radius":1.}]}

    def test_support_is_finite_surface_union_and_respects_mask(self):
        p=np.array([[1,0,0],[1,0,1],[1,0,2],[0,0,1],[3,0,1],[1.2,0,1],[1,0,3]])
        g=self.geometry()
        expected=[True,True,True,False,False,False,False]
        np.testing.assert_array_equal(surface_support(p,g,tolerance=.05),expected)
        g["colliders"]*=2
        np.testing.assert_array_equal(surface_support(p,g,tolerance=.05),expected)
        eligible=np.array([True,False,True,True,True,True,True])
        np.testing.assert_array_equal(surface_support(p,g,eligible,.05),[True,False,True,False,False,False,False])

    def test_baseline_axis_matching_requires_spatial_coverage(self):
        old=self.geometry()
        self.assertEqual(retained_baseline_trees(old,self.geometry(.05)),{"1":1.})
        self.assertEqual(retained_baseline_trees(old,self.geometry(.5)),{"1":0.})
        new=self.geometry();new["colliders"][0]["b"][2]=.5
        self.assertLess(retained_baseline_trees(old,new)["1"],.5)

    def test_different_maps_cannot_be_compared(self):
        with self.assertRaises(ValueError):
            compare_geometry(np.zeros((1,3)),{"local_map_sha256":"a"},{"local_map_sha256":"b"},".")

