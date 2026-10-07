"""Publish the Livox wire format directly, without unpacking/repacking each point."""
import struct
from livox_ros_driver2.msg import CustomMsg

class PackedScan(CustomMsg):
    def __init__(self, data):
        super().__init__()
        self.wire_data=data
        self.header.seq,self.header.stamp.secs,self.header.stamp.nsecs=struct.unpack_from("<III",data)
        self.header.frame_id="livox_frame"
    def serialize(self, buff):
        buff.write(struct.pack("<I",self.header.seq))
        buff.write(memoryview(self.wire_data)[4:])

def pack_scan(seq,stamp,points):
    sec=int(stamp);ns=int(round((stamp-sec)*1e9))
    if ns==1000000000:sec+=1;ns=0
    frame=b"livox_frame"
    data=struct.pack("<IIII",seq,sec,ns,len(frame))+frame
    data+=struct.pack("<QIB3sI",sec*1000000000+ns,len(points),0,b"\0"*3,len(points))
    return PackedScan(data+points.tobytes())
