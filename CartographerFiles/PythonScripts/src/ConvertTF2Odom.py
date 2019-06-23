#!/usr/bin/env python
import rospy

from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import numpy as np
import tf

PosOld = np.array([0.0, 0.0, 0.0])
QuatOld = np.array([0.0, 0.0, 0.0, 1.0])
TimeOld = 0.0



def ShowTfData(data):
    global PosOld, QuatOld, TimeOld
    odom = Odometry()
    odom.header = data.transforms[0].header
    odom.child_frame_id = data.transforms[0].child_frame_id

    if (data.transforms[0].child_frame_id=='base_link') and (data.transforms[0].header.frame_id=='map'):
        Pos = np.array([data.transforms[0].transform.translation.x,
                        data.transforms[0].transform.translation.y,
                        data.transforms[0].transform.translation.z])
        Quat = np.array([data.transforms[0].transform.rotation.x,
                         data.transforms[0].transform.rotation.y,
                         data.transforms[0].transform.rotation.z,
                         data.transforms[0].transform.rotation.w])
        Time = np.array(data.transforms[0].header.stamp.to_time())

        T = tf.transformations.quaternion_matrix(Quat)[:3, :3]
        T_Old = tf.transformations.quaternion_matrix(QuatOld)[:3, :3]
        dt = Time - TimeOld
 
        odom.pose.pose.position = Point(x=Pos[0], y=Pos[1], z=Pos[2])
        odom.pose.pose.orientation = data.transforms[0].transform.rotation

        if (dt > 0) and (TimeOld > 0):
            v = np.matmul(T, Pos - PosOld) / dt
            a = (np.array(tf.transformations.euler_from_quaternion(Quat))-np.array(tf.transformations.euler_from_quaternion(QuatOld)))/dt
            odom.twist.twist.linear.x = v[0]
            odom.twist.twist.linear.y = v[1]
            odom.twist.twist.linear.z = v[2]
            odom.twist.twist.angular.x = a[0]
            odom.twist.twist.angular.y = a[1]
            odom.twist.twist.angular.z = a[0] 
            PosOld = Pos.copy()
            QuatOld = Quat.copy()
            TimeOld = Time
            OdomPub.publish(odom)
        if TimeOld==0:
            PosOld = Pos.copy()
            QuatOld = Quat.copy()
            TimeOld = Time



if __name__ == '__main__':
    rospy.init_node('ConvertTF2Odom')
    rospy.Subscriber("/tf_static", TFMessage, ShowTfData)
    rospy.Subscriber("/tf", TFMessage, ShowTfData)
    OdomPub = rospy.Publisher('odom', Odometry, queue_size=10)
    rospy.spin()
