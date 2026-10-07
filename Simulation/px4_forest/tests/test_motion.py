import sys,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from motion import lidar_motion

class MovingLidar(unittest.TestCase):
    def test_quarter_turn_and_lever_arm(self):
        q=Rotation.from_euler("z",[0,90],degrees=True).as_quat()
        h=np.column_stack([[0,1],[2,3],[0,0],[0,0],q])
        origins,directions=lidar_motion(h,np.array([0,.5,1]),np.array([2,0,0]),np.array([.1,0,0]),np.eye(3),np.tile([1,0,0],(3,1)))
        expected=np.array([[1,0,0],[2**-.5,2**-.5,0],[0,1,0]])
        np.testing.assert_allclose(directions,expected,atol=1e-12)
        np.testing.assert_allclose(origins,np.array([[0,0,0],[.5,0,0],[1,0,0]])+.1*expected,atol=1e-12)
    def test_quaternion_sign_flip_same_motion(self):
        h=np.array([[0,0,0,0,0,0,0,1],[1,0,0,0,0,0,0,-1]],dtype=float)
        _,d=lidar_motion(h,[.5],[0,0,0],[0,0,0],np.eye(3),np.array([[0,1,0]]))
        np.testing.assert_allclose(d,[[0,1,0]])
    def test_out_of_history_rejected(self):
        h=np.array([[0,0,0,0,0,0,0,1],[1,0,0,0,0,0,0,1]],dtype=float)
        with self.assertRaises(ValueError):lidar_motion(h,[1.1],[0,0,0],[0,0,0],np.eye(3),np.array([[1,0,0]]))

if __name__=="__main__":unittest.main()
