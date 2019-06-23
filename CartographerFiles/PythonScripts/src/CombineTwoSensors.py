#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
import numpy as np
import copy

AnglesAll = np.arange(-2.0716333668668598,0.486305713654,0.00325437542051)
AnglesPico = np.arange(-0.499194979668-np.pi/2,0.534045875072-np.pi/2,0.00463336659595)
AnglesStruct = np.arange(-0.551840126514,0.486305713654,0.00325437542051)
AnglesPicoAll = np.arange(-2.0716333668668598,-1.03674198314468,0.00325437542051)
IndPico = np.round(np.interp(AnglesPicoAll,AnglesAll,range(len(AnglesAll)))).astype(int)
IndStruct = np.round(np.interp(AnglesStruct,AnglesAll,range(len(AnglesAll)))).astype(int)
ScanCombine0 = LaserScan(angle_min=-2.0716333668668598,angle_max=0.486305713654,angle_increment=0.00325437542051,time_increment=0.0001,scan_time=0.0786,range_min=0.3,range_max=8)
ScanCombine0.header.frame_id = 'camera_depth_frame'
ScanCombine0.ranges = np.repeat(np.nan,len(AnglesAll))
Seq = 0

ScanCombine = copy.deepcopy(ScanCombine0)

def InsertData(data,type):
    global ScanCombine, AnglesPicoAll, AnglesPico, IndPico, Seq, IndStruct
    if type == 0:
        ScanCombine.ranges[IndPico] = np.interp(AnglesPicoAll, AnglesPico, data.ranges)

    elif type == 1:
        ScanCombine.header = copy.deepcopy(data.header)
        ScanCombine.header.seq = Seq
        Seq += 1
        ScanCombine.ranges[IndStruct] = data.ranges

def SendMsg(data):
    global ScanCombine, ScanCombine0
    DeltaTime = 0.0
    if ScanCombine.header.stamp.to_sec()>0.0:
        DeltaTime = data.clock.to_sec()-ScanCombine.header.stamp.to_sec()
    if DeltaTime > 0.033:
        LaserScan_pub.publish(ScanCombine)
        ScanCombine = copy.deepcopy(ScanCombine0)

if __name__ == '__main__':
    rospy.init_node('CombineScan')
    LaserScan_pub = rospy.Publisher('scan', LaserScan, queue_size=10)

    rospy.Subscriber("/Pico_Scan", LaserScan, InsertData,0)
    rospy.Subscriber("/Structure_Scan", LaserScan, InsertData,1)
    rospy.Subscriber("/clock",Clock,SendMsg)
    rospy.spin()
