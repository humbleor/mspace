import json,shutil,subprocess,sys,tempfile,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from waypoints import generate_wps

def config(bounds,direction=0):
    return dict(waypointDistriFlag=0,grid_direction=direction,
                **{k:dict(zip(("min","max","step"),v)) for k,v in zip(("x","y","z"),bounds)})

class GridWaypoints(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which("g++"):raise unittest.SkipTest("Native generator comparison requires g++")
        cls.tmp=tempfile.TemporaryDirectory()
        root=Path(__file__).resolve().parents[3]
        src=(root/"Modules/ego_planner_swarm/plan_manage/src/ego_replan_fsm.cpp").read_text()
        body=src[src.index("  int EGOReplanFSM::generateGridWaypoints"):src.index("  int EGOReplanFSM::generateSinWaypoints")]
        body=body.replace("EGOReplanFSM::","")
        native='#include <iostream>\n#include <iomanip>\n#include <cstdlib>\ndouble waypoints_[300][3]; int grid_direction_;\n'+body
        native+='\nint main(int argc,char** argv) { grid_direction_=atoi(argv[10]); int n=generateGridWaypoints(atof(argv[1]),atof(argv[2]),atof(argv[3]),atof(argv[4]),atof(argv[5]),atof(argv[6]),atof(argv[7]),atof(argv[8]),atof(argv[9])); std::cout<<std::setprecision(17); for(int i=0;i<n;i++) std::cout<<waypoints_[i][0]<<" "<<waypoints_[i][1]<<" "<<waypoints_[i][2]<<"\\n"; }'
        p=Path(cls.tmp.name);(p/"generator.cpp").write_text(native);cls.binary=p/"generator"
        subprocess.run(["g++","-std=c++11",str(p/"generator.cpp"),"-o",str(cls.binary)],check=True,capture_output=True)
    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()
    def compare(self,c):
        args=[c[k][v] for k in ("x","y","z") for v in ("min","max")]+[c[k]["step"] for k in ("x","y","z")]+[c["grid_direction"]]
        output=subprocess.check_output([str(self.binary)]+list(map(str,args)),text=True)
        original=np.array([list(map(float,line.split())) for line in output.splitlines()])
        np.testing.assert_allclose(generate_wps(c),original,atol=1e-12)
    def test_requested_route_matches_native(self):
        c=config([(0,32,8),(0,16,8),(2,2,2)])
        self.compare(c);self.assertEqual(len(generate_wps(c)),15)
        np.testing.assert_allclose(generate_wps(c)[[0,4,5,9,10,14]],[[0,0,2],[32,0,2],[32,8,2],[0,8,2],[0,16,2],[32,16,2]])
    def test_layers_directions_and_fractional_ranges_match_native(self):
        for direction in (0,1):
            for bounds in [[(0,3,1),(0,2,1),(1,2,.5)],[(0,2.3,.7),(-1,1.4,.8),(1,1.7,.3)],[(0,0,1),(0,2,1),(2,2,1)]]:
                with self.subTest(direction=direction,bounds=bounds):self.compare(config(bounds,direction))
    def test_invalid_bounds_and_steps_rejected(self):
        for bounds in [[(0,2,0),(0,2,1),(2,2,1)],[(3,2,1),(0,2,1),(2,2,1)],[(0,float("nan"),1),(0,2,1),(2,2,1)]]:
            with self.assertRaises(ValueError):generate_wps(config(bounds))
if __name__=="__main__":unittest.main()
