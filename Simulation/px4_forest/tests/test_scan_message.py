import io,struct,sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/"Simulation/forest_lio/scripts"))
from scan_message import pack_scan
from common import POINT_DTYPE
from livox_ros_driver2.msg import CustomMsg
from rospy.msg import serialize_message
class LivoxTransport(unittest.TestCase):
    def test_ros_round_trip_point_timing_and_stamp_rollover(self):
        p=np.zeros(3,dtype=POINT_DTYPE)
        p["offset_time"]=[0,50000000,99999999]
        p["xyz"]=[[1,2,3],[-1,.5,6],[4,-2,8]]
        p["line"]=[0,1,2];p["reflectivity"]=[10,20,30]
        b=io.BytesIO();serialize_message(b,11,pack_scan(7,2.9999999999,p))
        wire=b.getvalue();self.assertEqual(struct.unpack_from("<I",wire)[0],len(wire)-4)
        m=CustomMsg().deserialize(wire[4:])
        self.assertEqual((m.header.seq,m.header.stamp.secs,m.header.stamp.nsecs),(11,3,0))
        self.assertEqual(m.timebase,3000000000)
        self.assertEqual(m.header.frame_id,"livox_frame")
        self.assertEqual(m.point_num,3)
        np.testing.assert_allclose([[x.x,x.y,x.z] for x in m.points],p["xyz"])
        self.assertEqual([x.offset_time for x in m.points],p["offset_time"].tolist())
        self.assertEqual([x.line for x in m.points],[0,1,2])
if __name__=="__main__":unittest.main()
