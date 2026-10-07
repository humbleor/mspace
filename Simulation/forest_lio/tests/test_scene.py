import sys
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from scene import local_frame,to_local,to_world,footprint_evidence,planner_config_for_route
from coverage_map import CoverageMap

class SceneTests(unittest.TestCase):
    def test_yaw_frame_preserves_gravity_and_extrinsic(self):
        q=Rotation.from_euler("xyz",[.1,-.2,1.3]).as_quat()
        origin,R=local_frame([4,5,2],q)
        np.testing.assert_allclose(R[:,2],[0,0,1],atol=1e-12)
        points=np.array([[0,1,2],[3,-1,7]])
        np.testing.assert_allclose(to_local(to_world(points,origin,R),origin,R),points,atol=1e-12)
        T=np.array([-.011,-.02329,.04412])
        np.testing.assert_allclose(to_local(origin+R@T,origin,R),T,atol=1e-12)
    def test_center_evidence_does_not_cover_body(self):
        c=CoverageMap([[0,0,0]],[4],.3)
        rows,report=footprint_evidence(c,[[.15,.15,.15]],.25)
        self.assertEqual(rows[0,0],0)
        self.assertGreater(rows[0,2],0)
        self.assertEqual(report["footprint_has_unknown_samples"],1)
    def test_sphere_voxel_overlap_excludes_diagonal(self):
        c=CoverageMap([[0,0,0]],[2],.3)
        rows,_=footprint_evidence(c,[[.15,.15,.15]],.25)
        self.assertEqual(rows[0,1],19)
        self.assertEqual(rows[0,3],1)
        self.assertEqual(rows[0,2],18)
    def test_boundary_sphere_touches_eight_cells(self):
        c=CoverageMap([[0,0,0]],[1],.3)
        rows,_=footprint_evidence(c,[[0,0,0]],.01)
        self.assertEqual(rows[0,1],8)
        self.assertEqual(rows[0,2],7)

    def test_route_map_extent_includes_local_update_margin(self):
        cfg={"grid_map/map_size_x":20,"grid_map/map_size_y":20,
             "grid_map/local_update_range_x":6.5,"grid_map/local_update_range_y":6.5}
        grown=planner_config_for_route(cfg,[[11.3,-2,0]])
        self.assertGreater(grown["grid_map/map_size_x"]/2,11.3+6.5)
        self.assertEqual(grown["grid_map/map_size_y"],20)
        self.assertEqual(cfg["grid_map/map_size_x"],20)
        import yaml
        self.assertEqual(yaml.safe_load(yaml.safe_dump(grown)),grown)
