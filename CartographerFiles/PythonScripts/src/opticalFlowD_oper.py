#!/usr/bin/env python
import numpy as np
from math import sqrt
import matplotlib.pyplot as plt
import time
import rosbag
import rospy
import os
from cv_bridge import CvBridge
import cv2 as cv
import csv

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf


removeLP = 1
LK_flag = 1
removeOutlierByBestBuddies = 1
bestBuddiesSqrErrTh = 4
removeOutliersRansac = 1
ransacErrTh = 2
useZ = False

figMask = False
figViz = False
figVizFeat = False
figDepth = False
figDisp = False
figDiff = False
figTraj = False

MIN_FEATURES_ALLOWED = 10
# Create some random colors
color = np.random.randint(0, 255, (500, 3))

class OdomCalculator():

    def __init__(self):
        # params for ShiTomasi corner detection
        self.feature_params = dict( maxCorners = 500,
                               qualityLevel = 0.05,
                               minDistance = 10,
                               blockSize = 25 )

        # Parameters for lucas kanade optical flow
        self.lk_params = dict( winSize  = (15,15),
                          maxLevel = 5,
                          criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

        #params for masking
        self.MASK_FRAME_X = 0.025 #pct
        self.MASK_FRAME_Y = 0.025 #pct
        self.MASK_FRAME_W = 0.95 #pct
        self.MASK_FRAME_H = 0.95 #pct
        self.MASK_THRESH = 250
        self.MASK_ERODE_SIZE = 25



        # params for pico
        self.camProps = {}
        self.camProps['fovw'] = 62 #deg
        self.camProps['fovh'] = 45 #deg
        self.camProps['pixw'] = 224 #pixles
        self.camProps['pixh'] = 171 #pixles
        self.camProps['depthScale'] = 1
        self.camProps['fx'] = 0
        self.camProps['fy'] = 0
        self.camProps['cx'] = 0
        self.camProps['cy'] = 0

        self.flagsGrayDepth = [False, False]
        self.cnt = 0
        self.bridge = CvBridge()
        self.calcOdomBusyFlag = 0
        self.dZ = 0

    def pico_makeMask(self,frame):
        y = int(self.MASK_FRAME_Y * frame.shape[0])
        x = int(self.MASK_FRAME_X * frame.shape[1])
        h = int(self.MASK_FRAME_H * frame.shape[0])
        w = int(self.MASK_FRAME_W * frame.shape[1])
        mask = np.zeros(frame.shape, np.uint8)
        mask[y:y + h, x:x + w] = 255
        mask[np.where((frame > self.MASK_THRESH))] = 0
        return cv.erode(mask,np.ones((self.MASK_ERODE_SIZE,self.MASK_ERODE_SIZE)))




    def rigid_transform_3D(self,A, B, fullCalc=True): #fullCalc = False --> optimize X,Y,YAW only
        assert len(A) == len(B)

        N = A.shape[0];  # total points

        if fullCalc:
            centroid_A = np.mean(A, axis=0)
            centroid_B = np.mean(B, axis=0)

            # centre the points
            AA = A - np.tile(centroid_A, (N, 1))
            BB = B - np.tile(centroid_B, (N, 1))

            # dot is matrix multiplication for array
            #H = np.transpose(AA) * BB
            H = np.dot(np.transpose(BB), AA)

            U, S, Vt = np.linalg.svd(H)

            M = np.diag([1, 1, np.linalg.det(Vt) * np.linalg.det(U)])

            R = np.dot(np.dot(U,M),Vt)

            # special reflection case
            if np.linalg.det(R) < 0:
                print("Reflection detected")
                Vt[2, :] *= -1
                R = np.dot(np.dot(U,M),Vt)

            t = np.reshape( ( np.dot(-R , centroid_A.T) + centroid_B.T) , (-1,3))

            #print(t)
        else:
            A_2D = A[:,:2]
            B_2D = B[:,:2]
            centroid_A = np.mean(A_2D, axis=0)
            centroid_B = np.mean(B_2D, axis=0)

            # centre the points
            AA = A_2D - np.tile(centroid_A, (N, 1))
            BB = B_2D - np.tile(centroid_B, (N, 1))

            # dot is matrix multiplication for array
            H = np.dot(np.transpose(BB), AA)

            U, S, Vt = np.linalg.svd(H)

            R = np.dot(U, Vt)

            # special reflection case
            if np.linalg.det(R) < 0:
                print("!!!!!!!!!!!!!!!!!  CHECK  !!!!!!!!!!! ")
                Vt[2, :] *= -1
                R = np.dot(np.dot(U, M), Vt)

            t = np.reshape((np.dot(-R, centroid_A.T) + centroid_B.T), (-1, 2))

            ### Make 2D to 3D, TODO: add pitch,roll from Z plane
            A_meanZ = np.mean(A[:,2], axis=0)
            B_meanZ = np.mean(B[:,2], axis=0)
            dZ = B_meanZ - A_meanZ
            R = np.pad(R,(0,1),'constant')
            R[2,2] = 1.0
            t = np.pad(t,((0,0),(0,1)),'constant')
            if np.abs(dZ) > 0.005:
                t[0,2] = dZ
                self.dZ += dZ
                print(self.dZ,dZ)


        return(R, t)


    def getDepth(self,x,y,frame):
        if x == int(x) and y == int(y):
            if frame[int(y),int(x)] > 0:
                return frame[int(y),int(x)]
            else:
                return np.nan
        else:
            #if x >
            cx = int(np.ceil(x))
            fx = int(np.floor(x))
            cy = int(np.ceil(y))
            fy = int(np.floor(y))
            if cx > camProps['pixw']-1 or cy > camProps['pixh']-1 :
                return frame[np.min((fy,camProps['pixh']-1)), np.min((fx,camProps['pixw']-1))] # todo: better interpolation
            wcx = cx - x
            wfx = x - fx
            wcy = cy - y
            wfy = y - fy
            dcxcy = frame[cy,cx]
            dfxcy = frame[fy, cx]
            dcxfy = frame[cy,fx]
            dfxfy = frame[fy, fx]
            allDepth = np.array( (dcxcy, dfxcy, dcxfy, dfxfy ))
            if (np.max(allDepth) - np.min(allDepth)) > 0.5:
                #print("!!!!!!!!!!!!==========> CHECK")
                return np.median(allDepth[allDepth!=0]) # todo: ignore "0"
            if np.all(allDepth > 0):
                return (dcxcy * wcx * wcy + dfxcy * wfx * wcy + dcxfy * wcx * wfy + dfxfy * wfx * wfy)
            else:
                1
                return np.nan

    def PixlesToMeters(self,xx,yy,depthFrame,defaultDepth=-1):
        Z = self.getDepth(xx, yy, depthFrame)
        if np.isnan(Z):
            if defaultDepth > 0:
                Z = defaultDepth
            else:
                return np.nan,np.nan,np.nan
        X = (xx - self.camProps['cx']) * Z / (self.camProps['fx'])
        Y = (yy - self.camProps['cy']) * Z / (self.camProps['fy'])
        return np.hstack((-X,-Y,np.array(Z)))


    def AddDepth(self,pnts,frame,camProps):
        pnts_3D = np.nan*np.zeros((pnts.shape[0],3))
        if camProps['depthScale'] is not 1:
            frame *= camProps.depthScale

        pnt_rnd = np.round(pnts).astype('int')
        pnt_rnd[np.where(pnt_rnd[:, 0] > 223), 0] = 223
        pnt_rnd[np.where(pnt_rnd[:, 1] > 170), 1] = 170
        Z = frame[pnt_rnd[:, 1], pnt_rnd[:, 0]]
        pnts_3D[:, 0] = (pnts[:, 0] - camProps['cx']) * Z / (camProps['fx'])
        pnts_3D[:, 1] = (pnts[:, 1] - camProps['cy']) * Z / (camProps['fy'])
        pnts_3D[:, 2] = Z

        pnts_3D[:, 0:2] = - pnts_3D[:, 0:2]
        return pnts_3D


    def ShowFigs(self,good_new,good_old):
        frame = self.frame_gray
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            # mask = cv.line(mask, (a,b),(c,d), color[i].tolist(), 2)
            frame = cv.circle(frame, (a, b), 5, color[i].tolist(), -1)
        mask = np.zeros_like(frame)
        #for i, pos_i in enumerate(np.flip(pos, axis=0)):
        #    pos_i_rnd = pos_i.astype('uint8')
        #    cv.circle(mask, (pos_i_rnd[0], pos_i_rnd[1]), 2, (0, 255, 0), -1)
        #    if i > 15:
        #        break
        img = cv.add(frame, mask)

        if figViz:
            cv.imshow('frame', self.frame_gray)
            cv.moveWindow('frame', 20, 0)
        if figVizFeat:
            cv.imshow('frameFeat', img)
            cv.moveWindow('frameFeat', 20, 300)
        if figDepth:
            cv.imshow('frameDepth', self.frameD)
            cv.moveWindow('frameDepth', 20, 600)

        if figTraj:
            plt.ion()
            fig = plt.figure('position', figsize=(10, 10))
            # fig.canvas.manager.window.move(20,900)
            plt.scatter(self.posFromStart.T[-1][0], self.posFromStart.T[-1][1])
            # plt.clf()
            # plt.scatter(pos[0], pos[1])
            fig.canvas.draw()

        if figMask:
            cv.imshow('mask1', self.old_gray_mask)
            cv.moveWindow('mask1', 20, 900)
        k = cv.waitKey(1) & 0xff
        if k == 27:
            return

    def publishOdom(self,rotMtx,transVec,badOdom=False):
        if badOdom:
            return False

        matrix4 = np.pad(self.currToStartMtx_rot,(0,1),'constant')
        matrix4[3,3] = 1
        matrix4[0:3,3] = self.currToStartMtx_tran.T

        '''
        matrix4 = np.pad(self.startToCurrMtx_rot,(0,1),'constant')
        matrix4[3,3] = 1
        matrix4[0:3,3] = self.startToCurrMtx_tran.T
        '''
        odom = Odometry()
        odom.header = self.msgD.header
        #odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'royale_camera_optical_frame'
        '''
        odom.pose.pose.position.x = self.startToCurrMtx_tran[0]
        odom.pose.pose.position.y = self.startToCurrMtx_tran[1]
        odom.pose.pose.position.z = self.startToCurrMtx_tran[2]
        '''
        odom.pose.pose.position.x = self.currToStartMtx_tran[0]#self.posFromStart[0]
        odom.pose.pose.position.y = self.currToStartMtx_tran[1]#self.posFromStart[1]
        odom.pose.pose.position.z = self.currToStartMtx_tran[2]#self.posFromStart[2]

        q = tf.transformations.quaternion_from_matrix(matrix4)
        odom.pose.pose.orientation.x =  q[0]
        odom.pose.pose.orientation.y =  q[1]
        odom.pose.pose.orientation.z =  q[2]
        odom.pose.pose.orientation.w =  q[3]
        odom.twist.twist.linear.x = transVec[0][0]
        odom.twist.twist.linear.y = transVec[0][1]
        odom.twist.twist.linear.z = transVec[0][2]
        x,y,z = tf.transformations.euler_from_matrix(rotMtx, axes='sxyz')
        odom.twist.twist.angular.x = x
        odom.twist.twist.angular.y = y
        odom.twist.twist.angular.z = z

        #odom.pose.covariance =
        #odom.twist.covariance =

        OdomPub.publish(odom)
        return True

    def calcOdom(self,img,imgD):

        self.cnt += 1
        if removeLP:
            imgB = cv.blur(img,(31,31))
            img = np.int16(img) - np.int16(imgB) + 127
            img[img<0]=0
            img[img>255] = 255
            img = np.uint8(img)


        if self.cnt == 1:
            # Take first frame and find corners in it
            self.old_frameD = imgD.copy()
            self.old_gray = img.copy() #cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)
            old_frameD = self.old_frameD
            old_gray = self.old_gray

            self.p0 = cv.goodFeaturesToTrack(old_gray, mask=None, **self.feature_params)
            # Create a mask image for drawing purposes
            self.old_gray_mask = self.pico_makeMask(old_gray)
            self.height = old_gray.shape[0]
            self.width = old_gray.shape[1]

            self.centerPoint = (np.zeros((1, 3)) + np.array((np.round(self.height / 2.0), np.round(self.width / 2.0), np.array(0.0)))).T
            self.currToStartMtx_rot = np.eye(3)
            self.currToStartMtx_tran = np.zeros((3,1))

            self.startToCurrMtx_rot = np.eye(3)
            self.startToCurrMtx_tran = np.zeros((3,1))

            self.lastDepth = 1.0

            self.posFromStart = np.zeros((3,1))
        else:
            ticAll = time.time()

            self.frameD = imgD.copy()
            self.frame_gray = img#cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            frameD = self.frameD
            frame_gray = self.frame_gray
            old_frameD = self.old_frameD
            old_gray = self.old_gray
            p0 = self.p0

            # calculate optical flow
            if self.p0 is None or len(self.p0) < MIN_FEATURES_ALLOWED:
                self.old_gray = frame_gray.copy()
                self.old_frameD = frameD.copy()
                self.p0 = cv.goodFeaturesToTrack(old_gray, mask=None, **self.feature_params)
                self.publishOdom(0,0,badOdom = True)
                return


            if LK_flag:
                ticOF = time.time()
                p1, st, err = cv.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **self.lk_params)
                tocOF = time.time()
                affMtx = np.eye(3)

                # Select good points
                good_new = p1[st == 1]
                good_old = p0[st == 1]

                if removeOutlierByBestBuddies:
                    backToOld, stBack, err = cv.calcOpticalFlowPyrLK(frame_gray, old_gray, good_new, None, **self.lk_params)
                    stBack = stBack.squeeze()
                    good_new = good_new[stBack==1,:]
                    backToOld = backToOld[stBack==1,:]
                    good_old = good_old[stBack==1,:]

                    bestBuddiesSqrErr = np.sum((backToOld - good_old)**2,axis=1)
                    goodBestBuddies = bestBuddiesSqrErr < bestBuddiesSqrErrTh
                    good_new = good_new[goodBestBuddies==1,:]
                    good_old = 0.5*(good_old[goodBestBuddies==1,:]+backToOld[goodBestBuddies==1,:])



                if removeOutliersRansac:
                    H, ransacMask = cv.findHomography(good_old, good_new, cv.RANSAC, ransacReprojThreshold=ransacErrTh)
                    good_new = good_new[ransacMask.squeeze()==1]
                    good_old = good_old[ransacMask.squeeze()==1]

                    #old2New = cv.warpPerspective(old_gray, H, (self.width,self.height))

                    #dif = 5*(np.int16(old2New) - np.int16(frame_gray)) + 127
                    #dif[dif<0]=0
                    #dif[dif > 255] = 255
                    #if figDiff:
                    #    cv.imshow('dif' ,np.uint8(dif))
                    #print(cnt)
                    #cv.waitKey()

                #if cnt==314:
                #    kkk=0

                '''
                displayPairs=0
                if displayPairs:
                    height,width =  old_gray.shape
                    dispImg = np.concatenate((old_gray,frame_gray),1)
                    for k in range(0,good_new.shape[0],4):
                        cv.line(dispImg,tuple(np.int32(good_old[k])),tuple(np.int32(good_new[k]+[width,0])),(0,255,0),lineType=cv.LINE_AA)

                    if figDisp:
                        cv.imshow('',dispImg)
                        cv.waitKey()
                '''

                good_old_3D = self.AddDepth(good_old, old_frameD, self.camProps)
                good_new_3D = self.AddDepth(good_new, frameD, self.camProps)

                ### FILTER POINTS WITH NANS

                validVec = (good_new_3D[:, 2] != 0.0) & (good_old_3D[:, 2] != 0.0)
                good_old_3D_filt = good_old_3D[validVec, :]
                good_new_3D_filt = good_new_3D[validVec, :]
                '''
                validVec = np.ones((good_old_3D.shape[0], 1))
                cntTmp = 0
                for ooo, nnn in zip(good_old_3D, good_new_3D):
                    if np.isnan(ooo[0]) or np.isnan(nnn[0]):
                        validVec[cntTmp] = 0
                    cntTmp += 1
                
                good_old_3D_filt = good_old_3D[np.where(validVec == 1), :][0]
                good_new_3D_filt = good_new_3D[np.where(validVec == 1), :][0]
                '''
                ### CALC R AND T
                rotMtx, tranVec = self.rigid_transform_3D(good_old_3D_filt, good_new_3D_filt, fullCalc=useZ)

                if len(good_old_3D_filt) < MIN_FEATURES_ALLOWED:
                    print(len(good_old_3D_filt))
                    # Now update the previous frame and previous points
                    self.old_gray = frame_gray.copy()
                    self.old_frameD = frameD.copy()
                    self.p0 = cv.goodFeaturesToTrack(old_gray, mask=None, **self.feature_params)
                    self.publishOdom(0,0,badOdom=True)
                    return

            else:
                #p1_extracted = cv.goodFeaturesToTrack(frame_gray, mask=None, **self.feature_params)
                p1, st, err = cv.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **self.lk_params)
                len_0=len(p0)
                len_1=len(p1)
                H, mask = cv.findHomography(p0[st==1],p1[st==1] , cv.RANSAC , 5)
                rotMtx = H.copy()
                rotMtx[0][2] = 0
                rotMtx[1][2] = 0
                rotMtx[2][2] = 1
                rotMtx[2][1] = 0
                rotMtx[2][0] = 0
                tranVec = np.array([H[0][2], H[1][2], 0])

            '''
            pos = (np.dot(rotMtx,pos) + tranVec.T)
            pos = np.append(pos, centerPoint,axis=1)
    
            pos = pos.T
            pos[-1] = PixlesToMeters(pos[-1][0], pos[-1][1], old_frameD, defaultDepth=lastDepth)
            pos = pos.T
            '''

            posLocal = self.PixlesToMeters(self.centerPoint[0], self.centerPoint[1], imgD, defaultDepth=self.lastDepth)

            self.startToCurrMtx_rot = np.dot(rotMtx,self.startToCurrMtx_rot)
            self.startToCurrMtx_tran = np.dot(rotMtx,self.startToCurrMtx_tran) + tranVec.T


            self.currToStartMtx_rot = np.dot(self.currToStartMtx_rot,np.linalg.inv(rotMtx))
            self.currToStartMtx_tran -= np.dot(self.currToStartMtx_rot, tranVec.T)
            self.posFromStart = np.dot(self.currToStartMtx_rot, np.reshape(posLocal.T,(-1,3)).T) + self.currToStartMtx_tran

            tocAll = time.time()
            #print('Time(OF) - ', str((tocOF - ticOF)*1000.0) ,'ms   ,   Time(All) - ', str((tocAll - ticAll)*1000.0) , 'ms')

            #print(self.posFromStart)
            #print(self.currToStartMtx_tran)
            # draw the stracks

            self.publishOdom(rotMtx,tranVec)

            #self.ShowFigs(good_new,good_old)

            # Now update the previous frame and previous points
            self.old_gray = frame_gray.copy()
            self.old_frameD = frameD.copy()
            self.old_gray_mask = self.pico_makeMask(old_gray)
            self.p0 = cv.goodFeaturesToTrack(old_gray, mask=self.old_gray_mask, **self.feature_params)

            #tocAllBag = time.time()
            #print('ALL WITH BAG -->   '+ str((tocAllBag - ticAllBag)*1000.0) )


    def CheckDataForOdom(self):
        if not all(self.flagsGrayDepth):
            return
        deltaT = self.tG_org - self.tD_org
        if np.abs(deltaT > 0.01):
            print('ERROR: SYNC PROBLEM ==> FIXING', deltaT)
            if deltaT > 0:
                self.flagsGrayDepth[1] = False
            else:
                self.flagsGrayDepth[0] = False
            return
        self.flagsGrayDepth = [False,False]
        img = np.asarray(self.bridge.imgmsg_to_cv2(self.msgG, 'mono16')).astype('uint8')
        imgD = self.bridge.imgmsg_to_cv2(self.msgD)
        #print("Start Calc")
        if self.calcOdomBusyFlag == 0:
            self.calcOdomBusyFlag = 1
            self.calcOdom(img,imgD)
            self.calcOdomBusyFlag = 0
        else:
            print('*'*40 + 'error' + '*'*40 )

        #print("Good")

    def GrayCallBack(self,msg):
        #print("recieve Gray")
        self.flagsGrayDepth[0] = True
        self.msgG = msg
        self.tG_org = msg.header.stamp.to_sec()
        self.CheckDataForOdom()

    def DepthCallBack(self,msg):
        #print("recieve Depth")
        self.flagsGrayDepth[1] = True
        self.msgD = msg
        self.tD_org = msg.header.stamp.to_sec()
        self.CheckDataForOdom()

    def CamInfoCallBack(self,msg):
        if self.camProps['fx'] == 0:
            self.camProps['fx'] = msg.K[0]
            self.camProps['fy'] = msg.K[4]
            self.camProps['cx'] = msg.K[2]
            self.camProps['cy'] = msg.K[5]
            print('UPDATED CAMERA PROPERTIES')
        else:
            return

    def IMUCallBack(self,msg):
        return


if __name__ == '__main__':
    OC = OdomCalculator()
    rospy.init_node('Calc_Odom')
    rospy.Subscriber("/royale_camera_driver/gray_image", Image, OC.GrayCallBack)
    rospy.Subscriber("/royale_camera_driver/depth_image", Image, OC.DepthCallBack)
    rospy.Subscriber("/royale_camera_driver/camera_info", CameraInfo, OC.CamInfoCallBack)
    rospy.Subscriber("/imu/data", Imu, OC.IMUCallBack)
    OdomPub = rospy.Publisher('odom', Odometry, queue_size=10)
    print("======= Calculate Odom Node Initialized =======")
    rospy.spin()
    cv.destroyAllWindows()
