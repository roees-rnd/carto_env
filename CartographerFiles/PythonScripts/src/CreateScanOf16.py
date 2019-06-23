#!/usr/bin/python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import copy

ScanDelta = 30.0*np.pi/180.0
ScanPosition = np.arange(-157.5,180,22.5)*np.pi/180.0-2.70526-22.5/2.0*np.pi/180.0
while np.any(ScanPosition > np.pi):
    ScanPosition[ScanPosition > np.pi] -= 2*np.pi
while np.any(ScanPosition < -np.pi):    
    ScanPosition[ScanPosition < -np.pi] += 2*np.pi
ScanPosition = np.sort(ScanPosition)

RangeMax = 4.0

def SingleScan(data):
    angles = np.arange(data.angle_min,data.angle_max,data.angle_increment)
    ind = np.around((ScanPosition-data.angle_min)/data.angle_increment).astype('uint')
    ranges = np.array(data.ranges)[ind]
    intensities = np.array(data.intensities)[ind]
    angles = angles[ind]
    data. angle_min = min(angles)
    data.angle_max = max(angles)
    data.angle_increment = 22.5*np.pi/180.0
    data.range_max = RangeMax
    data.ranges = ranges
    data.intensities = intensities
    return data
    
def CreateScans(data):
    LaserScan1.publish(SingleScan(data))


if __name__ == '__main__':
    rospy.init_node('CreateScanOf16')
    LaserScan1 = rospy.Publisher('scan', LaserScan, queue_size=10)
    rospy.Subscriber("/RP_scan", LaserScan,CreateScans)

    rospy.spin()
