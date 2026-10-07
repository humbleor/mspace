import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from visualize_sitl import make_cloud,local_pose
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
class VisualizationFrames(unittest.TestCase):
    def test_cloud_ros_layout_is_xyz_in_world(self):
        points=np.array([[1,-2,3],[4,5,.5]],dtype=float)
        m=make_cloud(points)
        self.assertEqual(m.header.frame_id,"world")
        np.testing.assert_allclose(list(point_cloud2.read_points(m,field_names=("x","y","z"))),points)
    def test_truth_origin_does_not_modify_input_message(self):
        m=Odometry();m.header.frame_id="gazebo_world"
        m.pose.pose.position.x=6;m.pose.pose.position.y=4;m.pose.pose.position.z=2
        m.pose.pose.orientation.w=1
        p=local_pose(m,[5,3,.1])
        self.assertEqual(p.header.frame_id,"world")
        np.testing.assert_allclose([p.pose.position.x,p.pose.position.y,p.pose.position.z],[1,1,1.9])
        self.assertEqual(m.pose.pose.position.x,6)
        self.assertEqual(m.header.frame_id,"gazebo_world")
if __name__=="__main__":unittest.main()
