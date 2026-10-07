#!/usr/bin/env python3
"""Known geometry and independent exhaustive oracle checks for ray occlusion."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from bvh_renderer import BVHRenderer,exhaustive_ranges
from renderer import PointRenderer

class BVHTests(unittest.TestCase):
    def test_plane_occludes_back_plane_with_hole(self):
        y,z=np.meshgrid(np.arange(-1,1.01,.1),np.arange(-1,1.01,.1))
        front=np.column_stack([np.full(y.size,3.),y.ravel(),z.ravel()])
        front=front[np.linalg.norm(front[:,1:],axis=1)>.31]
        back=np.column_stack([np.full(y.size,6.),y.ravel(),z.ravel()])
        xyz=np.vstack([front,back]);normals=np.tile([1.,0,0],(len(xyz),1))
        renderer=BVHRenderer(xyz,.075,10,normals=normals,threads=1)
        rays=np.array([[1.,0,0],[3,.7,0],[-1,0,0],[0,1,0]])
        out=renderer.scan(rays,np.zeros(3),np.eye(3),.5)
        self.assertAlmostEqual(out[0],6)
        self.assertAlmostEqual(out[1],np.linalg.norm([3,.7,0]))
        np.testing.assert_array_equal(out[2:],0)
    def test_valid_farther_candidate_not_hidden_by_nearer_miss(self):
        xyz=np.array([[2.,.15,0],[4.,0,0]])
        renderer=BVHRenderer(xyz,.05,10,normals=np.tile([1,0,0],(2,1)),threads=1)
        np.testing.assert_allclose(renderer.scan([[1,0,0]],[0,0,0],np.eye(3),.5),[4.])
        # Same two modeled disks, repeated for PCA initialization. Exact normals held equal.
        old=PointRenderer(np.repeat(xyz,8,axis=0),10,.05,10)
        old.normals[:]=[1,0,0]
        np.testing.assert_array_equal(old.scan([[1,0,0]],[0,0,0],np.eye(3),.5),[0.])
    def test_per_ray_pose_and_rotation(self):
        xyz=np.array([[3.,0,0]]);r=BVHRenderer(xyz,.5,10,normals=[[1,0,0]],threads=1)
        origins=np.column_stack([np.linspace(0,.2,31),np.zeros((31,2))])
        out=r.scan_world(np.tile([1,0,0],(31,1)),origins,.5)
        np.testing.assert_allclose(out,3-origins[:,0],atol=1e-12)
        rot=Rotation.from_euler("z",90,degrees=True).as_matrix()
        np.testing.assert_allclose(r.scan([[0,-1,0]],[0,0,0],rot,.5),[3],atol=1e-12)
    def test_blind_range_and_rear_surface(self):
        r=BVHRenderer([[.1,0,0],[2,0,0],[4,0,0]],.05,3,normals=np.tile([1,0,0],(3,1)),threads=1)
        np.testing.assert_allclose(r.scan([[1,0,0]],[0,0,0],np.eye(3),.5),[0])
        # Software output range filtering never makes an opaque near obstacle transparent.
        r2=BVHRenderer([[2,0,0],[4,0,0]],.05,3,normals=np.tile([1,0,0],(2,1)),threads=1)
        np.testing.assert_allclose(r2.scan([[1,0,0]],[0,0,0],np.eye(3),.5),[2])
        np.testing.assert_allclose(r.scan([[1,0,0]],[0,0,0],np.eye(3),2.5),[0])
    def test_randomized_exact_oracle_and_threads(self):
        rng=np.random.default_rng(19)
        points=rng.uniform([-2,-2,-2],[5,2,2],(1500,3))
        normals=rng.normal(size=(1500,3));normals/=np.linalg.norm(normals,axis=1)[:,None]
        rays=rng.normal(size=(128,3));origins=rng.uniform(-.3,.3,(128,3))
        reference=exhaustive_ranges(points,normals,.18,rays,origins,.2,10)
        for threads in (1,4):
            r=BVHRenderer(points,.18,10,normals=normals,threads=threads)
            np.testing.assert_allclose(r.scan_world(rays,origins,.2),reference,rtol=1e-10,atol=1e-10)
    def test_thin_cylinder_analytic(self):
        theta=np.linspace(0,2*np.pi,121,endpoint=False);height=np.linspace(-.5,.5,51)
        a,z=np.meshgrid(theta,height);radius=.08
        xyz=np.column_stack([3+radius*np.cos(a.ravel()),radius*np.sin(a.ravel()),z.ravel()])
        normals=np.column_stack([np.cos(a.ravel()),np.sin(a.ravel()),np.zeros(a.size)])
        r=BVHRenderer(xyz,.016,10,normals=normals,threads=1)
        y=np.linspace(-.06,.06,21);rays=np.column_stack([np.full(21,3.),y,np.zeros(21)])
        rays/=np.linalg.norm(rays,axis=1)[:,None]
        # Intersections with infinite cylinder (x-3)^2+y^2=radius^2 at z=0.
        b=-6*rays[:,0];c=9-radius**2
        truth=(-b-np.sqrt(b*b-4*c))/2
        actual=r.scan_world(rays,[0,0,0],.5)
        self.assertTrue(np.all(actual>0))
        self.assertLess(np.max(np.abs(actual-truth)),.002)
    def test_no_old_pose_visibility_cache(self):
        r=BVHRenderer([[3,0,0]],.1,10,normals=[[1,0,0]],threads=1)
        np.testing.assert_allclose(r.scan([[1,0,0]],[0,0,0],np.eye(3),.5),[3])
        np.testing.assert_array_equal(r.scan([[1,0,0]],[0,.3,0],np.eye(3),.5),[0])
        np.testing.assert_allclose(r.scan([[1,0,0]],[.2,0,0],np.eye(3),.5),[2.8])
if __name__=="__main__":unittest.main()
