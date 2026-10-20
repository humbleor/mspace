import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from forest_geometry import extract_geometry,geometry_model


class OrientedTrees(unittest.TestCase):
    def tube(self,a,b,r=.18,arc=2*np.pi):
        a,b=np.array(a,dtype=float),np.array(b,dtype=float)
        d=b-a;length=np.linalg.norm(d);axis=d/length
        u=np.cross(axis,[0.,1,0]);u/=np.linalg.norm(u);v=np.cross(axis,u)
        theta,t=np.meshgrid(np.linspace(0,arc,48,endpoint=False),np.arange(0,length,.065))
        return a+t.ravel()[:,None]*axis+r*(np.cos(theta.ravel())[:,None]*u+np.sin(theta.ravel())[:,None]*v)

    def ground(self,slope=0):
        x,y=np.meshgrid(np.arange(-2,6,.12),np.arange(-3,4,.12))
        return np.column_stack([x.ravel(),y.ravel(),slope*x.ravel()])

    def trunks(self,g):
        return [c for c in g["colliders"] if c["kind"]=="trunk"]

    def test_tilt_and_radius_with_missing_base_and_sloping_ground(self):
        p=np.vstack([self.tube([.2,0,.8],[1.2,0,4],.2),self.ground(.1)])
        g=extract_geometry(p);cs=self.trunks(g)
        self.assertEqual(g["counts"]["trees"],1);self.assertEqual(len(cs),1)
        c=cs[0];a,b=np.array(c["a"]),np.array(c["b"]);axis=(b-a)/np.linalg.norm(b-a)
        expected=np.array([1.,0,3.2]);expected/=np.linalg.norm(expected)
        self.assertLess(np.rad2deg(np.arccos(np.clip(axis@expected,-1,1))),2)
        self.assertAlmostEqual(c["radius"],.2,delta=.02)
        self.assertAlmostEqual(a[2],.1*a[0],delta=.03)
        self.assertGreater(c["ground_extension_m"],.5)
        m=geometry_model(g);self.assertEqual(len(m.findall(".//visual")),len(g["colliders"]))

    def test_curved_trunk_has_shared_segment_endpoints(self):
        theta,z=np.meshgrid(np.linspace(0,2*np.pi,48,endpoint=False),np.arange(.2,4,.065))
        p=np.column_stack([.06*z.ravel()**2+.16*np.cos(theta.ravel()),
                           .16*np.sin(theta.ravel()),z.ravel()])
        g=extract_geometry(np.vstack([p,self.ground()]))
        cs=self.trunks(g);self.assertEqual(g["counts"]["trees"],1);self.assertGreater(len(cs),1)
        for left,right in zip(cs[:-1],cs[1:]):
            np.testing.assert_allclose(left["b"],right["a"],atol=1e-10)
        self.assertLess(max(c["fit_median_error_m"] for c in cs),.03)

    def test_nearby_trunks_do_not_merge(self):
        p=np.vstack([self.tube([0,0,.1],[0,0,3],.15),
                     self.tube([.8,0,.1],[.9,0,3],.15),self.ground()])
        self.assertEqual(extract_geometry(p)["counts"]["trees"],2)

    def test_small_observation_gap_connected_without_fragments(self):
        p=self.tube([0,0,.1],[.4,0,4],.18)
        p=p[(p[:,2]<1.4)|(p[:,2]>1.8)]
        g=extract_geometry(np.vstack([p,self.ground()]))
        self.assertEqual(g["counts"]["trees"],1)
        self.assertEqual(len(self.trunks(g)),1)

    def test_suspended_tube_not_extended_unboundedly(self):
        g=extract_geometry(np.vstack([self.tube([0,0,4],[0,0,7]),self.ground()]))
        self.assertEqual(g["counts"]["trees"],1)
        c=self.trunks(g)[0]
        self.assertEqual(c["ground_status"],"extension_limit")
        self.assertEqual(c["ground_extension_m"],0)
        self.assertGreaterEqual(c["a"][2],4)

    def test_no_ground_evidence_does_not_invent_ground(self):
        g=extract_geometry(self.tube([0,0,1],[0,0,4]))
        self.assertEqual(g["counts"]["trees"],1)
        c=self.trunks(g)[0]
        self.assertIsNone(c["ground"])
        self.assertEqual(c["ground_status"],"unobserved")
        self.assertEqual(c["ground_extension_m"],0)
        self.assertGreaterEqual(c["a"][2],1)
        m=geometry_model(g)
        self.assertEqual(len(m.findall(".//collision")),len(g["colliders"]))

    def test_foliage_contamination_does_not_inflate_trunk_radius(self):
        rng=np.random.default_rng(2)
        leaves=rng.uniform([-.65,-.65,.5],[.65,.65,3.5],(1600,3))
        p=np.vstack([self.tube([0,0,.1],[0,0,4],.2),leaves,self.ground()])
        g=extract_geometry(p)
        self.assertEqual(g["counts"]["trees"],1)
        self.assertAlmostEqual(self.trunks(g)[0]["radius"],.2,delta=.03)

    def test_detached_branch_rejected(self):
        g=extract_geometry(np.vstack([self.tube([0,0,.1],[0,0,4]),
             self.tube([0,3,2],[3,3,2],.09),self.ground()]))
        self.assertEqual(g["counts"]["trees"],1)
        self.assertEqual(g["counts"]["branch"],0)

    def test_attached_branch_merged_across_cells(self):
        g=extract_geometry(np.vstack([self.tube([0,0,.1],[0,0,4]),
             self.tube([.12,0,2],[3,0,2],.09),self.ground()]))
        branches=[c for c in g["colliders"] if c["kind"]=="branch"]
        self.assertEqual(len(branches),1)
        self.assertGreater(np.linalg.norm(np.array(branches[0]["b"])-branches[0]["a"]),2.4)
        self.assertEqual(branches[0]["tree_id"],self.trunks(g)[0]["tree_id"])

    def test_partial_arc_with_sufficient_support(self):
        g=extract_geometry(np.vstack([self.tube([0,0,.1],[.3,0,3],.2,arc=np.pi),self.ground()]))
        self.assertEqual(g["counts"]["trees"],1)
        self.assertAlmostEqual(self.trunks(g)[0]["radius"],.2,delta=.025)



    def test_long_gap_not_connected_into_a_single_tree(self):
        p=self.tube([0,0,.1],[0,0,6],.18)
        p=p[(p[:,2]<2.5)|(p[:,2]>4)]
        g=extract_geometry(np.vstack([p,self.ground()]))
        self.assertGreaterEqual(g["counts"]["trees"],1)
        self.assertTrue(all(not (c["a"][2]<2.5 and c["b"][2]>4) for c in self.trunks(g)))

    def test_vertical_wall_is_not_a_trunk(self):
        y,z=np.meshgrid(np.arange(-1,1,.06),np.arange(.1,4,.06))
        wall=np.column_stack([np.zeros(y.size),y.ravel(),z.ravel()])
        g=extract_geometry(np.vstack([wall,self.ground()]))
        self.assertEqual(g["counts"]["trees"],0)

    def test_invalid_fit_settings_rejected(self):
        p=self.tube([0,0,0],[0,0,3])
        for cfg in ({"normal_neighbors":1},{"normal_neighbors":3.5},
                    {"min_radius":.7},{"max_tilt_deg":90},{"normal_agreement":2},
                    {"unknown_setting":1}):
            with self.assertRaises(ValueError):extract_geometry(p,cfg)


    def test_sparse_normal_rescue_and_cross_scale_deduplication(self):
        cloud=self.tube([0,0,.1],[.2,0,4],.16)
        rng=np.random.default_rng(5)
        cloud=cloud[rng.choice(len(cloud),len(cloud)//12,replace=False)]
        g=extract_geometry(cloud)
        self.assertEqual(g["counts"]["trees"],1)
        self.assertEqual(self.trunks(g)[0]["extraction_pass"],"sparse")
        self.assertAlmostEqual(self.trunks(g)[0]["radius"],.16,delta=.025)
        stats=g["extraction_statistics"]
        self.assertGreater(stats["sparse_added_surface_points"],0)
        self.assertEqual(len(g["candidate_diagnostics"]),stats["associated_tracks"])
        self.assertEqual(sum(c["status"]=="accepted" for c in g["candidate_diagnostics"]),1)

    def test_dense_tree_not_duplicated_by_rescue_pass(self):
        g=extract_geometry(np.vstack([self.tube([0,0,.1],[0,0,4]),self.ground()]))
        self.assertEqual(g["counts"]["trees"],1)
        self.assertGreater(g["extraction_statistics"]["track_rejections"].get("duplicate_observation",0),0)

    def test_foliage_only_not_promoted_to_trunks(self):
        cloud=np.random.default_rng(44).uniform([-1,-1,.3],[1,1,4],(1800,3))
        self.assertEqual(extract_geometry(cloud)["counts"]["trees"],0)
