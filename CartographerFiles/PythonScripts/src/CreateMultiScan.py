#!/usr/bin/python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import copy

ScanDelta = 30.0*np.pi/180.0
ScanCenter = np.array([0.0, 120.0, -120.0, 180.0])*np.pi/180.0
RangeMax = 5.0

def SingleScan(data,Center0,Center1,Delta,Frame):
    angles = np.arange(data.angle_min,data.angle_max,data.angle_increment)+Center0
    while np.any(angles > np.pi):
        angles[angles > np.pi] -= 2*np.pi
    while np.any(angles < -np.pi):    
        angles[angles < -np.pi] += 2*np.pi
    ind = np.argsort(angles)
    ranges = np.array(data.ranges)[ind]
    intensities = np.array(data.intensities)[ind]
    angles = angles[ind]
    ind1 = np.argwhere((angles>Center1-ScanDelta) & (angles<Center1+ScanDelta))
    angles1 = angles[ind1]-Center1
    data1 = copy.deepcopy(data)
    data1.header.frame_id = Frame
    data1. angle_min = min(angles1)
    data1.angle_max = max(angles1)
    data1.scan_time = data1.scan_time*(ScanDelta/np.pi)
    data1.range_max = RangeMax
    data1.ranges = ranges[ind1]
    data1.intensities = intensities[ind1]
    return data1
    
def CreateScans(data):
    LaserScan1.publish(SingleScan(data,2.70526,ScanCenter[0],ScanDelta,"Scan_1_link"))
    LaserScan2.publish(SingleScan(data,2.70526,ScanCenter[1],ScanDelta,"Scan_2_link"))
    LaserScan3.publish(SingleScan(data,2.70526,ScanCenter[2],ScanDelta,"Scan_3_link"))

if __name__ == '__main__':
    rospy.init_node('CreateMultiScan')
    LaserScan1 = rospy.Publisher('scan_1', LaserScan, queue_size=10)
    LaserScan2= rospy.Publisher('scan_2', LaserScan, queue_size=10)
    LaserScan3 = rospy.Publisher('scan_3', LaserScan, queue_size=10)
    rospy.Subscriber("/RP_scan", LaserScan,CreateScans)

    rospy.spin()
