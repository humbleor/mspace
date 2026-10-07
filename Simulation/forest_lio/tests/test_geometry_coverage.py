#!/usr/bin/env python3
"""Coverage semantics, sampling support and independently checked mixed geometry."""
from pathlib import Path
import sys,tempfile
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
import unittest
import numpy as np
from coverage_map import CoverageBuilder,CoverageMap
from bvh_renderer import BVHRenderer,exhaustive_ranges
from surface_model import adaptive_surfaces

class CoverageTests(unittest.TestCase):
    def test_observed_ray_surface_and_unknown(self):
        b=CoverageBuilder(.5);b.update([0,0,0],[[2.1,0,0]],blind=.5,max_range=3)
        m=b.export();f=m.classify([[.75,0,0],[2.1,0,0],[2.75,0,0],[0,1,0],[.1,0,0]])
        self.assertEqual(f[0],1);self.assertTrue(f[1]&2)
        self.assertEqual(f[2],0);self.assertEqual(f[3],0);self.assertEqual(f[4],4)
    def test_clipped_ray_does_not_invent_surface(self):
        b=CoverageBuilder(.5);b.update([0,0,0],[[100,0,0]],blind=.5,max_range=3)
        m=b.export();self.assertFalse(np.any(m.flags&2))
        self.assertTrue(m.classify([[2.5,0,0]])[0]&1)
        self.assertEqual(m.classify([[3.75,0,0]])[0],0)
    def test_corner_ray_negative_indices_and_roundtrip(self):
        b=CoverageBuilder(.5)
        b.update([0,0,0],[[2.1,2.1,2.1],[-2.1,0,0]],blind=0,max_range=10)
        m=b.export()
        self.assertEqual(m.classify([[.75,.75,.75]])[0],1)
        self.assertEqual(m.classify([[.75,.25,.25]])[0],0)
        self.assertTrue(m.classify([[-.75,0,0]])[0]&1)
        with tempfile.TemporaryDirectory() as d:
            m.save(Path(d)/"coverage.npz");loaded=CoverageMap.load(Path(d)/"coverage.npz")
            np.testing.assert_array_equal(loaded.indices,m.indices);np.testing.assert_array_equal(loaded.flags,m.flags)

class AdaptiveGeometryTests(unittest.TestCase):
    def test_plane_density_preserves_known_hits(self):
        rng=np.random.default_rng(44);targets=rng.uniform(-.6,.6,(80,2))
        rays=np.column_stack([np.full(len(targets),3),targets]);expected=np.linalg.norm(rays,axis=1)
        for spacing in (.025,.05,.1):
            y,z=np.meshgrid(np.arange(-1,1.001,spacing),np.arange(-1,1.001,spacing))
            points=np.column_stack([np.full(y.size,3.),y.ravel(),z.ravel()])
            r=BVHRenderer(points,.2,10,threads=1,surface_model="adaptive")
            np.testing.assert_allclose(r.scan_world(rays,[0,0,0],.5),expected,atol=1e-8)
            self.assertGreater(np.mean(r.kinds==0),.9)
    def test_thin_strip_does_not_become_wide_plane(self):
        z=np.linspace(-1,1,81);y=np.linspace(-.02,.02,5);Y,Z=np.meshgrid(y,z)
        points=np.column_stack([np.full(Y.size,3.),Y.ravel(),Z.ravel()])
        normals,radii,kinds,_=adaptive_surfaces(points)
        self.assertLess(np.median(radii),.04)
        r=BVHRenderer(points,.2,10,surface_model="adaptive",threads=1)
        # A ray 10 cm outside a 4 cm wide known strip must not be filled by a 20 cm disk.
        np.testing.assert_array_equal(r.scan_world([[3,.12,0]],[0,0,0],.5),[0])
        self.assertGreater(r.scan_world([[1,0,0]],[0,0,0],.5)[0],0)
    def test_collinear_normals_are_uncertain(self):
        points=np.column_stack([np.full(30,3.),np.zeros(30),np.linspace(-1,1,30)])
        _,radii,kinds,_=adaptive_surfaces(points)
        self.assertTrue(np.all(kinds==1));self.assertTrue(np.all(radii<=.04))
    def test_mixed_disks_spheres_and_radii_match_oracle(self):
        rng=np.random.default_rng(7);points=rng.uniform([-1,-1,-1],[4,1,1],(500,3))
        normals=rng.normal(size=(500,3));radii=rng.uniform(.02,.18,500);kinds=rng.integers(0,2,500)
        r=BVHRenderer(points,.2,10,normals=normals,radii=radii,kinds=kinds,threads=4)
        rays=rng.normal(size=(100,3));origins=rng.uniform(-.2,.2,(100,3))
        reference=exhaustive_ranges(points,normals,radii,rays,origins,.2,10,kinds=kinds)
        np.testing.assert_allclose(r.scan_world(rays,origins,.2),reference,atol=1e-10,rtol=1e-10)
if __name__=="__main__":unittest.main()
